#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>

#define CMD_I2C_WRITE 0x01
#define CMD_I2C_READ  0x03

typedef struct {
    unsigned char dev_addr;
    unsigned int reg_addr;
    unsigned int addr_byte_num;
    unsigned int data;
    unsigned int data_byte_num;
} I2C_DATA_S;

typedef struct {
    uint16_t reg;
    uint8_t expected;
} REG_EXPECTED;

static int fd = -1;

static unsigned int devaddr = 0x30;
static unsigned int addr_bytes = 2;

static unsigned int read_ok = 0;
static unsigned int read_errors = 0;
static unsigned int write_ok = 0;
static unsigned int write_errors = 0;

static volatile sig_atomic_t stop_requested = 0;


/*
 * SC5332 initialization table from the OpenXiongmai X50 sensor library.
 *
 * Registers that occur multiple times are compared against their final
 * initialization state.
 *
 * Exposure and gain registers can legitimately differ while the sensor
 * is running and are therefore marked as dynamic during comparison.
 */
static const REG_EXPECTED sc5332_init[] = {
    {0x0103, 0x01},
    {0x0100, 0x00},

    {0x36E9, 0x80},
    {0x36F9, 0x80},

    {0x301F, 0x06},

    {0x3200, 0x00},
    {0x3201, 0x00},
    {0x3202, 0x00},
    {0x3203, 0x00},
    {0x3204, 0x0A},
    {0x3205, 0x2B},
    {0x3206, 0x07},
    {0x3207, 0xA3},
    {0x3208, 0x0A},
    {0x3209, 0x28},
    {0x320A, 0x07},
    {0x320B, 0xA0},
    {0x320C, 0x07},
    {0x320D, 0x53},

    {0x3210, 0x00},
    {0x3211, 0x02},
    {0x3212, 0x00},
    {0x3213, 0x02},

    {0x3301, 0x08},
    {0x3302, 0x10},
    {0x3303, 0x08},
    {0x3304, 0x40},
    {0x3306, 0x40},
    {0x3308, 0x18},
    {0x3309, 0x88},
    {0x330B, 0xAA},
    {0x330D, 0x20},
    {0x330E, 0x20},
    {0x330F, 0x08},

    {0x3310, 0x04},
    {0x331C, 0x02},
    {0x331E, 0x31},
    {0x331F, 0x79},

    {0x3333, 0x10},
    {0x3356, 0x39},
    {0x3364, 0x17},

    {0x3390, 0x0C},
    {0x3391, 0x1C},
    {0x3392, 0x3C},
    {0x3393, 0x0F},
    {0x3394, 0x19},
    {0x3395, 0x20},

    {0x33AF, 0x4A},
    {0x33B5, 0x10},

    {0x3622, 0x16},

    {0x3630, 0xB0},
    {0x3631, 0x80},
    {0x3633, 0x73},
    {0x3634, 0x43},
    {0x3637, 0x12},
    {0x3638, 0x08},
    {0x363B, 0x00},

    {0x3670, 0x0A},
    {0x3674, 0xB0},
    {0x3675, 0xBA},
    {0x3676, 0xBF},
    {0x367C, 0x0C},
    {0x367D, 0x3C},

    {0x3690, 0x43},
    {0x3691, 0x53},
    {0x3692, 0x53},
    {0x369C, 0x1C},
    {0x369D, 0x3C},

    {0x36EA, 0x35},
    {0x36EB, 0x0C},
    {0x36EC, 0x0C},
    {0x36ED, 0x24},

    {0x36FA, 0x35},
    {0x36FC, 0x01},
    {0x36FD, 0x27},

    {0x3E01, 0xF7},
    {0x3E02, 0x00},

    {0x3F09, 0x47},

    {0x4505, 0x09},
    {0x4509, 0x10},

    {0x4819, 0x09},
    {0x481B, 0x05},
    {0x481D, 0x14},
    {0x481F, 0x04},
    {0x4821, 0x0A},
    {0x4823, 0x05},
    {0x4825, 0x04},
    {0x4827, 0x05},
    {0x4829, 0x08},

    {0x5787, 0x10},
    {0x5788, 0x06},
    {0x578A, 0x10},
    {0x578B, 0x06},

    {0x5790, 0x10},
    {0x5791, 0x10},
    {0x5792, 0x00},
    {0x5793, 0x10},
    {0x5794, 0x10},
    {0x5795, 0x00},
    {0x5799, 0x00},

    {0x57C7, 0x10},
    {0x57C8, 0x06},
    {0x57CA, 0x10},
    {0x57CB, 0x06},

    {0x57D0, 0x10},
    {0x57D1, 0x10},
    {0x57D2, 0x00},
    {0x57D3, 0x10},
    {0x57D4, 0x10},
    {0x57D5, 0x00},
    {0x57D9, 0x00},

    {0x59E0, 0x60},
    {0x59E1, 0x08},
    {0x59E2, 0x3F},
    {0x59E3, 0x18},
    {0x59E4, 0x18},
    {0x59E5, 0x3F},
    {0x59E6, 0x06},
    {0x59E7, 0x02},
    {0x59E8, 0x38},
    {0x59E9, 0x10},
    {0x59EA, 0x0C},
    {0x59EB, 0x10},
    {0x59EC, 0x04},
    {0x59ED, 0x02},

    /*
     * Final values after initialization.
     */
    {0x36E9, 0x20},
    {0x36F9, 0x20},
    {0x0100, 0x01}
};


/* -------------------------------------------------------------------------
 * Generic helpers
 * ------------------------------------------------------------------------- */

static void signal_handler(int signum)
{
    (void)signum;
    stop_requested = 1;
}


static int parse_uint(const char *text, unsigned int *value)
{
    char *end = NULL;
    unsigned long parsed;

    if (text == NULL || value == NULL || *text == '\0')
        return -1;

    errno = 0;
    parsed = strtoul(text, &end, 0);

    if (errno != 0 || end == text || *end != '\0')
        return -1;

    if (parsed > 0xFFFFFFFFUL)
        return -1;

    *value = (unsigned int)parsed;
    return 0;
}


static void sleep_ms(unsigned int milliseconds)
{
    struct timespec request;
    struct timespec remaining;

    request.tv_sec = milliseconds / 1000;
    request.tv_nsec =
        (long)(milliseconds % 1000) * 1000000L;

    while (nanosleep(&request, &remaining) != 0) {
        if (errno != EINTR)
            break;

        if (stop_requested)
            break;

        request = remaining;
    }
}


/* -------------------------------------------------------------------------
 * I2C access
 * ------------------------------------------------------------------------- */

static int read_reg(unsigned int reg, unsigned int *value)
{
    I2C_DATA_S d;
    int ret;

    if (value == NULL)
        return -1;

    memset(&d, 0, sizeof(d));

    d.dev_addr = (unsigned char)devaddr;
    d.reg_addr = reg;
    d.addr_byte_num = addr_bytes;
    d.data = 0;
    d.data_byte_num = 1;

    errno = 0;
    ret = ioctl(fd, CMD_I2C_READ, &d);

    if (ret != 0) {
        read_errors++;
        return -1;
    }

    *value = d.data & 0xFF;
    read_ok++;

    return 0;
}


static int write_reg(unsigned int reg, unsigned int value)
{
    I2C_DATA_S d;
    int ret;

    memset(&d, 0, sizeof(d));

    d.dev_addr = (unsigned char)devaddr;
    d.reg_addr = reg;
    d.addr_byte_num = addr_bytes;
    d.data = value & 0xFF;
    d.data_byte_num = 1;

    errno = 0;
    ret = ioctl(fd, CMD_I2C_WRITE, &d);

    if (ret != 0) {
        write_errors++;
        return -1;
    }

    write_ok++;
    return 0;
}


/* -------------------------------------------------------------------------
 * Register classification and safety
 * ------------------------------------------------------------------------- */

static int dynamic_register(unsigned int reg)
{
    switch (reg) {
    case 0x3E00:
    case 0x3E01:
    case 0x3E02:
    case 0x3E06:
    case 0x3E07:
    case 0x3E08:
    case 0x3E09:
        return 1;

    default:
        return 0;
    }
}


static int protected_register(unsigned int reg)
{
    /*
     * Software reset and streaming control.
     */
    if (reg == 0x0100 || reg == 0x0103)
        return 1;

    /*
     * Sensor mode/clock selector.
     */
    if (reg == 0x301F)
        return 1;

    /*
     * Internal clock/analog timing area.
     */
    if (reg >= 0x36E9 && reg <= 0x36FD)
        return 1;

    /*
     * MIPI control and timing registers.
     */
    if (reg >= 0x4800 && reg <= 0x483F)
        return 1;

    return 0;
}


/* -------------------------------------------------------------------------
 * Output helpers
 * ------------------------------------------------------------------------- */

static void print_reg(const char *name, unsigned int reg)
{
    unsigned int value;

    if (read_reg(reg, &value) == 0) {
        printf("0x%04X = 0x%02X", reg, value);

        if (name != NULL && *name != '\0')
            printf("  %s", name);

        printf("\n");
    } else {
        printf("0x%04X = ERROR", reg);

        if (name != NULL && *name != '\0')
            printf("  %s", name);

        printf(" (%s)\n", strerror(errno));
    }
}


static int dump_range(unsigned int start, unsigned int end)
{
    unsigned int reg;
    unsigned int value;

    if (start > 0xFFFF || end > 0xFFFF) {
        printf("ERROR: register addresses must be 0x0000..0xFFFF\n");
        return -1;
    }

    if (start > end) {
        printf("ERROR: range start must not exceed range end\n");
        return -1;
    }

    /*
     * Prevent an accidental dump of nearly the entire register map.
     */
    if ((end - start) > 0x1000) {
        printf("ERROR: maximum range size is 4097 registers\n");
        return -1;
    }

    printf("\n--- RANGE 0x%04X - 0x%04X ---\n",
           start, end);

    reg = start;

    for (;;) {
        if (read_reg(reg, &value) == 0) {
            printf("0x%04X = 0x%02X\n",
                   reg, value);
        } else {
            printf("0x%04X = ERROR (%s)\n",
                   reg, strerror(errno));
        }

        if (reg == end)
            break;

        reg++;
    }

    return 0;
}


/* -------------------------------------------------------------------------
 * Basic SC5332 diagnostics
 * ------------------------------------------------------------------------- */

static void basic_diagnostics(void)
{
    unsigned int idh, idl;
    unsigned int wh, wl;
    unsigned int hh, hl;
    unsigned int htsh, htsl;
    unsigned int vtsh, vtsl;
    unsigned int stream;

    printf("\n========================================\n");
    printf(" BASIC SENSOR INFORMATION\n");
    printf("========================================\n");

    print_reg("Streaming", 0x0100);
    print_reg("Software reset/status", 0x0103);

    if (read_reg(0x3107, &idh) == 0 &&
        read_reg(0x3108, &idl) == 0) {
        unsigned int id = (idh << 8) | idl;

        printf("\nChip ID : 0x%04X", id);

        if (id == 0xCE1A)
            printf("  [SC5332 MATCH]\n");
        else
            printf("  [UNEXPECTED]\n");
    }

    if (read_reg(0x3208, &wh) == 0 &&
        read_reg(0x3209, &wl) == 0 &&
        read_reg(0x320A, &hh) == 0 &&
        read_reg(0x320B, &hl) == 0) {
        printf("Output  : %u x %u\n",
               (wh << 8) | wl,
               (hh << 8) | hl);
    }

    if (read_reg(0x320C, &htsh) == 0 &&
        read_reg(0x320D, &htsl) == 0) {
        unsigned int hts = (htsh << 8) | htsl;

        printf("HTS     : 0x%04X (%u)\n",
               hts, hts);
    }

    if (read_reg(0x320E, &vtsh) == 0 &&
        read_reg(0x320F, &vtsl) == 0) {
        unsigned int vts = (vtsh << 8) | vtsl;

        printf("VTS     : 0x%04X (%u)\n",
               vts, vts);
    }

    if (read_reg(0x0100, &stream) == 0) {
        printf("Stream  : %s (0x%02X)\n",
               (stream & 1) ? "ON" : "OFF",
               stream);
    }

    printf("\nOther useful registers:\n");

    print_reg("Orientation", 0x3221);

    print_reg("Exposure H", 0x3E00);
    print_reg("Exposure M", 0x3E01);
    print_reg("Exposure L", 0x3E02);

    print_reg("Gain 3E06", 0x3E06);
    print_reg("Gain 3E07", 0x3E07);
    print_reg("Gain 3E08", 0x3E08);
    print_reg("Gain 3E09", 0x3E09);

    print_reg("Analog 36E9", 0x36E9);
    print_reg("Analog 36F9", 0x36F9);

    print_reg("MIPI 4800", 0x4800);
    print_reg("MIPI 4801", 0x4801);
    print_reg("MIPI 483C", 0x483C);

    print_reg("Control 5000", 0x5000);
    print_reg("Control 5001", 0x5001);
}


static void timing_diagnostics(void)
{
    printf("\n========================================\n");
    printf(" SC5332 CLOCK / MIPI TIMING\n");
    printf("========================================\n");

    print_reg("Mode/clock 301F", 0x301F);

    printf("\nInternal clock related:\n");

    print_reg("Clock 36EA", 0x36EA);
    print_reg("Clock 36EB", 0x36EB);
    print_reg("Clock 36EC", 0x36EC);
    print_reg("Clock 36ED", 0x36ED);

    print_reg("Clock 36FA", 0x36FA);
    print_reg("Clock 36FC", 0x36FC);
    print_reg("Clock 36FD", 0x36FD);

    printf("\nMIPI timing:\n");

    print_reg("MIPI timing 4819", 0x4819);
    print_reg("MIPI timing 481B", 0x481B);
    print_reg("MIPI timing 481D", 0x481D);
    print_reg("MIPI timing 481F", 0x481F);
    print_reg("MIPI timing 4821", 0x4821);
    print_reg("MIPI timing 4823", 0x4823);
    print_reg("MIPI timing 4825", 0x4825);
    print_reg("MIPI timing 4827", 0x4827);
    print_reg("MIPI timing 4829", 0x4829);
}


/* -------------------------------------------------------------------------
 * Init table comparison
 * ------------------------------------------------------------------------- */

static void compare_init_table(void)
{
    unsigned int value;

    unsigned int matches = 0;
    unsigned int mismatches = 0;
    unsigned int dynamic = 0;
    unsigned int errors = 0;

    size_t count =
        sizeof(sc5332_init) / sizeof(sc5332_init[0]);

    size_t i;
    size_t j;

    printf("\n========================================\n");
    printf(" SC5332 INIT TABLE VS LIVE SENSOR\n");
    printf("========================================\n");

    printf("Register   Expected   Live       Result\n");
    printf("----------------------------------------\n");

    for (i = 0; i < count; i++) {
        unsigned int reg = sc5332_init[i].reg;
        unsigned int expected = sc5332_init[i].expected;

        int later_duplicate = 0;

        /*
         * Compare only the final expected state when a register occurs
         * more than once in the initialization sequence.
         */
        for (j = i + 1; j < count; j++) {
            if (sc5332_init[j].reg == reg) {
                later_duplicate = 1;
                break;
            }
        }

        if (later_duplicate)
            continue;

        if (read_reg(reg, &value) != 0) {
            printf("0x%04X      0x%02X       ERROR      READ ERROR\n",
                   reg, expected);
            errors++;
            continue;
        }

        if (dynamic_register(reg)) {
            printf("0x%04X      0x%02X       0x%02X       DYNAMIC\n",
                   reg, expected, value);
            dynamic++;
            continue;
        }

        if (value == expected) {
            printf("0x%04X      0x%02X       0x%02X       MATCH\n",
                   reg, expected, value);
            matches++;
        } else {
            printf("0x%04X      0x%02X       0x%02X       MISMATCH\n",
                   reg, expected, value);
            mismatches++;
        }
    }

    printf("\nComparison summary:\n");
    printf("Matches    : %u\n", matches);
    printf("Mismatches : %u\n", mismatches);
    printf("Dynamic    : %u\n", dynamic);
    printf("Errors     : %u\n", errors);
}


/* -------------------------------------------------------------------------
 * Predefined full register dump
 * ------------------------------------------------------------------------- */

static void focused_dump(void)
{
    printf("\n========================================\n");
    printf(" SC5332 RELEVANT REGISTER DUMP\n");
    printf("========================================\n");

    dump_range(0x3000, 0x303F);
    dump_range(0x3100, 0x313F);

    dump_range(0x3200, 0x323F);
    dump_range(0x3250, 0x327F);

    dump_range(0x3300, 0x33BF);
    dump_range(0x3600, 0x36FF);

    dump_range(0x3900, 0x397F);

    dump_range(0x3E00, 0x3E3F);
    dump_range(0x3F00, 0x3F1F);

    dump_range(0x4500, 0x451F);
    dump_range(0x4800, 0x483F);
    dump_range(0x5000, 0x503F);

    dump_range(0x5780, 0x579F);
    dump_range(0x57C0, 0x57DF);
    dump_range(0x59E0, 0x59EF);
}


/* -------------------------------------------------------------------------
 * Verified generic register write
 * ------------------------------------------------------------------------- */

static int verified_write_reg(unsigned int reg,
                              unsigned int value,
                              int force)
{
    unsigned int old_value;
    unsigned int new_value;

    if (reg > 0xFFFF) {
        printf("ERROR: register must be 0x0000..0xFFFF\n");
        return -1;
    }

    if (value > 0xFF) {
        printf("ERROR: value must be 0x00..0xFF\n");
        return -1;
    }

    if (protected_register(reg) && !force) {
        printf("ERROR: register 0x%04X is protected.\n", reg);
        printf("Use --force only if the consequences are understood.\n");
        return -1;
    }

    if (read_reg(reg, &old_value) != 0) {
        printf("ERROR: cannot read register 0x%04X before write: %s\n",
               reg, strerror(errno));
        return -1;
    }

    printf("\n========================================\n");
    printf(" VERIFIED REGISTER WRITE\n");
    printf("========================================\n");

    printf("Register : 0x%04X\n", reg);
    printf("Old      : 0x%02X\n", old_value);
    printf("Requested: 0x%02X\n", value);

    if (force)
        printf("Protection: OVERRIDDEN WITH --force\n");

    if (old_value == value) {
        printf("Result   : no change required\n");
        return 0;
    }

    if (write_reg(reg, value) != 0) {
        printf("WRITE FAILED: %s\n", strerror(errno));
        return -1;
    }

    if (read_reg(reg, &new_value) != 0) {
        printf("WRITE completed, but readback failed: %s\n",
               strerror(errno));
        return -1;
    }

    printf("Readback : 0x%02X\n", new_value);

    if (new_value != value) {
        printf("VERIFY FAILED\n");
        return -1;
    }

    printf("VERIFY OK\n");

    printf("\nRestore command:\n");
    printf("  ./xm_i2c_test 0x%02X %u write 0x%04X 0x%02X",
           devaddr,
           addr_bytes,
           reg,
           old_value);

    if (protected_register(reg))
        printf(" --force");

    printf("\n");

    return 0;
}


/* -------------------------------------------------------------------------
 * Safe 16-bit VTS operation
 * ------------------------------------------------------------------------- */

static int set_vts(unsigned int new_vts)
{
    unsigned int old_h;
    unsigned int old_l;

    unsigned int new_h;
    unsigned int new_l;

    unsigned int verify_h;
    unsigned int verify_l;

    unsigned int old_vts;
    unsigned int verify_vts;

    if (new_vts < 16 || new_vts > 0xFFFF) {
        printf("ERROR: VTS must be 16..65535\n");
        return -1;
    }

    if (read_reg(0x320E, &old_h) != 0 ||
        read_reg(0x320F, &old_l) != 0) {
        printf("ERROR: cannot read current VTS\n");
        return -1;
    }

    old_vts = (old_h << 8) | old_l;

    new_h = (new_vts >> 8) & 0xFF;
    new_l = new_vts & 0xFF;

    printf("\n========================================\n");
    printf(" SC5332 LIVE VTS TEST\n");
    printf("========================================\n");

    printf("Old VTS : %u (0x%04X)\n",
           old_vts, old_vts);

    printf("New VTS : %u (0x%04X)\n",
           new_vts, new_vts);

    printf("\nRegisters:\n");

    printf("0x320E: 0x%02X -> 0x%02X\n",
           old_h, new_h);

    printf("0x320F: 0x%02X -> 0x%02X\n",
           old_l, new_l);

    if (old_vts == new_vts) {
        printf("\nResult: VTS already has the requested value.\n");
        return 0;
    }

    /*
     * Write the low byte first and the high byte immediately afterwards.
     * This minimizes the period during which the register pair contains
     * a mixed old/new value.
     */
    if (write_reg(0x320F, new_l) != 0) {
        printf("ERROR writing VTS low byte: %s\n",
               strerror(errno));
        return -1;
    }

    if (write_reg(0x320E, new_h) != 0) {
        printf("ERROR writing VTS high byte: %s\n",
               strerror(errno));

        printf("Attempting low-byte rollback...\n");
        write_reg(0x320F, old_l);

        return -1;
    }

    if (read_reg(0x320E, &verify_h) != 0 ||
        read_reg(0x320F, &verify_l) != 0) {
        printf("ERROR: VTS readback failed\n");
        return -1;
    }

    verify_vts = (verify_h << 8) | verify_l;

    printf("\nReadback VTS: %u (0x%04X)\n",
           verify_vts, verify_vts);

    if (verify_vts != new_vts) {
        printf("VERIFY FAILED\n");
        return -1;
    }

    printf("VERIFY OK\n");

    printf("\nRestore command:\n");

    printf("  ./xm_i2c_test 0x%02X %u vts %u\n",
           devaddr,
           addr_bytes,
           old_vts);

    printf("\nCheck VI timing with:\n");
    printf("  cat /proc/umap/vi | tail -10\n");

    return 0;
}


/* -------------------------------------------------------------------------
 * Watch mode
 * ------------------------------------------------------------------------- */

static int watch_registers(unsigned int interval_ms,
                           int register_count,
                           char **register_arguments)
{
    unsigned int *registers;
    unsigned int *values;

    unsigned int iteration = 0;

    int i;

    if (register_count <= 0) {
        printf("ERROR: watch requires at least one register\n");
        return -1;
    }

    if (interval_ms < 10) {
        printf("ERROR: minimum watch interval is 10 ms\n");
        return -1;
    }

    if (interval_ms > 3600000) {
        printf("ERROR: maximum watch interval is 3600000 ms\n");
        return -1;
    }

    registers = calloc((size_t)register_count,
                       sizeof(*registers));

    values = calloc((size_t)register_count,
                    sizeof(*values));

    if (registers == NULL || values == NULL) {
        printf("ERROR: cannot allocate watch buffers\n");

        free(registers);
        free(values);

        return -1;
    }

    for (i = 0; i < register_count; i++) {
        if (parse_uint(register_arguments[i],
                       &registers[i]) != 0 ||
            registers[i] > 0xFFFF) {
            printf("ERROR: invalid register: %s\n",
                   register_arguments[i]);

            free(registers);
            free(values);

            return -1;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("\n========================================\n");
    printf(" REGISTER WATCH\n");
    printf("========================================\n");

    printf("Interval: %u ms\n", interval_ms);
    printf("Stop with Ctrl+C\n\n");

    while (!stop_requested) {
        iteration++;

        printf("[%06u]", iteration);

        for (i = 0; i < register_count; i++) {
            if (read_reg(registers[i], &values[i]) == 0) {
                printf(" %04X=%02X",
                       registers[i],
                       values[i]);
            } else {
                printf(" %04X=ERR",
                       registers[i]);
            }
        }

        printf("\n");
        fflush(stdout);

        if (!stop_requested)
            sleep_ms(interval_ms);
    }

    printf("\nWatch stopped.\n");

    free(registers);
    free(values);

    return 0;
}


/* -------------------------------------------------------------------------
 * Command handlers
 * ------------------------------------------------------------------------- */

static int command_read(int argc, char **argv)
{
    int i;

    if (argc < 5) {
        printf("ERROR: read requires at least one register\n");
        return -1;
    }

    for (i = 4; i < argc; i++) {
        unsigned int reg;

        if (parse_uint(argv[i], &reg) != 0 ||
            reg > 0xFFFF) {
            printf("ERROR: invalid register: %s\n",
                   argv[i]);
            return -1;
        }

        print_reg("User requested register", reg);
    }

    return 0;
}


static int command_range(int argc, char **argv)
{
    unsigned int start;
    unsigned int end;

    if (argc != 6) {
        printf("ERROR: range requires start and end\n");
        return -1;
    }

    if (parse_uint(argv[4], &start) != 0 ||
        parse_uint(argv[5], &end) != 0) {
        printf("ERROR: invalid range\n");
        return -1;
    }

    return dump_range(start, end);
}


static int command_write(int argc, char **argv)
{
    unsigned int reg;
    unsigned int value;

    int force = 0;

    if (argc != 6 && argc != 7) {
        printf("ERROR: write requires register and value\n");
        return -1;
    }

    if (parse_uint(argv[4], &reg) != 0 ||
        parse_uint(argv[5], &value) != 0) {
        printf("ERROR: invalid register or value\n");
        return -1;
    }

    if (argc == 7) {
        if (strcmp(argv[6], "--force") != 0) {
            printf("ERROR: unknown write option: %s\n",
                   argv[6]);
            return -1;
        }

        force = 1;
    }

    return verified_write_reg(reg, value, force);
}


static int command_vts(int argc, char **argv)
{
    unsigned int value;

    if (argc != 5) {
        printf("ERROR: vts requires one value\n");
        return -1;
    }

    if (parse_uint(argv[4], &value) != 0) {
        printf("ERROR: invalid VTS value\n");
        return -1;
    }

    return set_vts(value);
}


static int command_watch(int argc, char **argv)
{
    unsigned int interval_ms;

    if (argc < 6) {
        printf("ERROR: watch requires interval and registers\n");
        return -1;
    }

    if (parse_uint(argv[4], &interval_ms) != 0) {
        printf("ERROR: invalid watch interval\n");
        return -1;
    }

    return watch_registers(interval_ms,
                           argc - 5,
                           &argv[5]);
}


/* -------------------------------------------------------------------------
 * Usage
 * ------------------------------------------------------------------------- */

static void print_usage(const char *program)
{
    printf("\nUsage:\n");

    printf("  %s [i2c-address] [address-bytes] "
           "[command] [arguments]\n\n",
           program);

    printf("Commands:\n");

    printf("  diag\n");
    printf("      Basic diagnostics, clock/MIPI timing and init comparison\n\n");

    printf("  dump\n");
    printf("      Diagnostics plus predefined full register dump\n\n");

    printf("  read <reg> [reg ...]\n");
    printf("      Read one or more arbitrary registers\n\n");

    printf("  range <start> <end>\n");
    printf("      Read an arbitrary continuous register range\n\n");

    printf("  watch <interval-ms> <reg> [reg ...]\n");
    printf("      Continuously watch arbitrary registers\n\n");

    printf("  write <reg> <value> [--force]\n");
    printf("      Write one byte, read it back and verify it\n\n");

    printf("  vts <value>\n");
    printf("      Set the 16-bit VTS value in 0x320E/0x320F\n\n");

    printf("Examples:\n");

    printf("  %s 0x30 2\n", program);
    printf("  %s 0x30 2 diag\n", program);
    printf("  %s 0x30 2 dump\n", program);

    printf("  %s 0x30 2 read 0x36EA\n",
           program);

    printf("  %s 0x30 2 read "
           "0x36EA 0x36EB 0x36EC 0x36ED\n",
           program);

    printf("  %s 0x30 2 range 0x4800 0x483F\n",
           program);

    printf("  %s 0x30 2 watch 250 "
           "0x3E00 0x3E01 0x3E02 "
           "0x3E06 0x3E07 0x3E08 0x3E09\n",
           program);

    printf("  %s 0x30 2 write 0x320F 0x60\n",
           program);

    printf("  %s 0x30 2 write 0x301F 0x06 --force\n",
           program);

    printf("  %s 0x30 2 vts 2800\n",
           program);

    printf("\nWARNING:\n");

    printf("  diag, dump, read, range and watch are READ ONLY.\n");
    printf("  write and vts modify the live sensor.\n");
    printf("  Changes remain active until overwritten, reinitialized or rebooted.\n");
}


/* -------------------------------------------------------------------------
 * Main
 * ------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    const char *dev = "/dev/xm_i2c";
    const char *command = "diag";

    int result = 0;
    int write_mode = 0;

    if (argc > 1) {
        if (!strcmp(argv[1], "-h") ||
            !strcmp(argv[1], "--help")) {
            print_usage(argv[0]);
            return 0;
        }

        if (parse_uint(argv[1], &devaddr) != 0 ||
            devaddr > 0x7F) {
            fprintf(stderr,
                    "ERROR: invalid I2C address\n");
            return 1;
        }
    }

    if (argc > 2) {
        if (parse_uint(argv[2], &addr_bytes) != 0) {
            fprintf(stderr,
                    "ERROR: invalid address-byte count\n");
            return 1;
        }
    }

    if (argc > 3)
        command = argv[3];

    if (addr_bytes != 1 && addr_bytes != 2) {
        fprintf(stderr,
                "ERROR: address-bytes must be 1 or 2\n");
        return 1;
    }

    if (!strcmp(command, "write") ||
        !strcmp(command, "vts")) {
        write_mode = 1;
    }

    printf("========================================\n");
    printf(" XM530 / SC5332 UNIVERSAL TOOL\n");
    printf("========================================\n");

    printf("Device       : %s\n", dev);
    printf("I2C address  : 0x%02X\n", devaddr);
    printf("Address bytes: %u\n", addr_bytes);
    printf("Data bytes   : 1\n");
    printf("Command      : %s\n", command);

    printf("Access       : %s\n",
           write_mode ? "READ / WRITE" : "READ ONLY");

    fd = open(dev, O_RDWR);

    if (fd < 0) {
        fprintf(stderr,
                "Cannot open %s: %s\n",
                dev,
                strerror(errno));
        return 1;
    }

    if (!strcmp(command, "diag")) {
        basic_diagnostics();
        timing_diagnostics();
        compare_init_table();

    } else if (!strcmp(command, "dump")) {
        basic_diagnostics();
        timing_diagnostics();
        compare_init_table();
        focused_dump();

    } else if (!strcmp(command, "read")) {
        result = command_read(argc, argv);

    } else if (!strcmp(command, "range")) {
        result = command_range(argc, argv);

    } else if (!strcmp(command, "watch")) {
        result = command_watch(argc, argv);

    } else if (!strcmp(command, "write")) {
        result = command_write(argc, argv);

    } else if (!strcmp(command, "vts")) {
        result = command_vts(argc, argv);

    } else {
        fprintf(stderr,
                "ERROR: unknown command: %s\n",
                command);

        print_usage(argv[0]);
        result = -1;
    }

    close(fd);
    fd = -1;

    printf("\n========================================\n");
    printf(" I2C OPERATION SUMMARY\n");
    printf("========================================\n");

    printf("Successful reads  : %u\n", read_ok);
    printf("Read errors       : %u\n", read_errors);
    printf("Successful writes : %u\n", write_ok);
    printf("Write errors      : %u\n", write_errors);

    if (result != 0) {
        printf("\nResult: FAILED\n");
        return 1;
    }

    printf("\nResult: OK\n");
    return 0;
}
