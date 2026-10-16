#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdint.h>

#define CMD_I2C_READ 0x03

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

/*
 * SC5332 init table from OpenXiongmai X50 sensor library.
 *
 * Dynamic exposure/gain registers may legitimately differ after startup.
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

static int read_reg(unsigned int reg, unsigned int *value)
{
    I2C_DATA_S d;
    int ret;

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

    *value = d.data & 0xff;
    read_ok++;

    return 0;
}

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

static void print_reg(const char *name, unsigned int reg)
{
    unsigned int value;

    if (read_reg(reg, &value) == 0) {
        printf("0x%04X = 0x%02X  %s\n",
               reg, value, name);
    } else {
        printf("0x%04X = ERROR  %s (%s)\n",
               reg, name, strerror(errno));
    }
}

static void dump_range(unsigned int start, unsigned int end)
{
    unsigned int reg;
    unsigned int value;

    printf("\n--- RANGE 0x%04X - 0x%04X ---\n",
           start, end);

    for (reg = start; reg <= end; reg++) {
        if (read_reg(reg, &value) == 0) {
            printf("0x%04X = 0x%02X\n",
                   reg, value);
        } else {
            printf("0x%04X = ERROR (%s)\n",
                   reg, strerror(errno));
        }
    }
}

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

        printf("HTS     : 0x%04X (%u)\n",
               (htsh << 8) | htsl,
               (htsh << 8) | htsl);
    }

    if (read_reg(0x320E, &vtsh) == 0 &&
        read_reg(0x320F, &vtsl) == 0) {

        printf("VTS     : 0x%04X (%u)\n",
               (vtsh << 8) | vtsl,
               (vtsh << 8) | vtsl);
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

static void compare_init_table(void)
{
    unsigned int value;
    unsigned int matches = 0;
    unsigned int mismatches = 0;
    unsigned int dynamic = 0;
    unsigned int errors = 0;

    size_t count =
        sizeof(sc5332_init) / sizeof(sc5332_init[0]);

    printf("\n========================================\n");
    printf(" SC5332 INIT TABLE VS LIVE SENSOR\n");
    printf("========================================\n");

    printf("Register   Expected   Live       Result\n");
    printf("----------------------------------------\n");

    for (size_t i = 0; i < count; i++) {

        unsigned int reg = sc5332_init[i].reg;
        unsigned int expected = sc5332_init[i].expected;

        /*
         * If a register occurs multiple times in the initialization
         * sequence, compare only its last expected state.
         */
        int later_duplicate = 0;

        for (size_t j = i + 1; j < count; j++) {
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

    /*
     * These ranges were missing from the previous dump but are
     * explicitly programmed by sensor_init_sc5332().
     */
    dump_range(0x5780, 0x579F);
    dump_range(0x57C0, 0x57DF);
    dump_range(0x59E0, 0x59EF);
}

static void print_usage(const char *program)
{
    printf("\nUsage:\n");
    printf("  %s [i2c-address] [address-bytes] [mode]\n\n",
           program);

    printf("Examples:\n");
    printf("  %s\n", program);
    printf("  %s 0x30 2\n", program);
    printf("  %s 0x30 1\n", program);
    printf("  %s 0x30 2 dump\n\n", program);

    printf("Modes:\n");
    printf("  diag   Basic diagnostics + init comparison\n");
    printf("  dump   Diagnostics + comparison + register dump\n\n");

    printf("WARNING: This utility performs READ operations only.\n");
}

int main(int argc, char **argv)
{
    const char *dev = "/dev/xm_i2c";
    const char *mode = "diag";

    if (argc > 1) {
        if (!strcmp(argv[1], "-h") ||
            !strcmp(argv[1], "--help")) {
            print_usage(argv[0]);
            return 0;
        }

        devaddr = strtoul(argv[1], NULL, 0);
    }

    if (argc > 2)
        addr_bytes = strtoul(argv[2], NULL, 0);

    if (argc > 3)
        mode = argv[3];

    if (addr_bytes != 1 && addr_bytes != 2) {
        fprintf(stderr,
                "ERROR: address-bytes must be 1 or 2\n");
        return 1;
    }

    printf("========================================\n");
    printf(" XM530 / SC5332 DIAGNOSTIC TOOL\n");
    printf("========================================\n");

    printf("Device       : %s\n", dev);
    printf("I2C address  : 0x%02X\n", devaddr);
    printf("Address bytes: %u\n", addr_bytes);
    printf("Data bytes   : 1\n");
    printf("Mode         : %s\n", mode);
    printf("Access       : READ ONLY\n");

    fd = open(dev, O_RDWR);

    if (fd < 0) {
        fprintf(stderr,
                "Cannot open %s: %s\n",
                dev,
                strerror(errno));
        return 1;
    }

    basic_diagnostics();

    compare_init_table();

    if (!strcmp(mode, "dump"))
        focused_dump();

    close(fd);

    printf("\n========================================\n");
    printf(" I2C READ SUMMARY\n");
    printf("========================================\n");

    printf("Successful reads : %u\n", read_ok);
    printf("Read errors      : %u\n", read_errors);

    printf("\nDone.\n");

    return 0;
}
