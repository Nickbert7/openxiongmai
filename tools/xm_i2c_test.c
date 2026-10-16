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

static int fd;
static unsigned int devaddr = 0x30;

static int read_reg(unsigned int reg, unsigned int *value)
{
    I2C_DATA_S d;

    memset(&d, 0, sizeof(d));

    d.dev_addr = (unsigned char)devaddr;
    d.reg_addr = reg;
    d.addr_byte_num = 2;
    d.data = 0;
    d.data_byte_num = 1;

    errno = 0;

    if (ioctl(fd, CMD_I2C_READ, &d) < 0)
        return -1;

    *value = d.data & 0xff;

    return 0;
}

static void read_single(const char *name, unsigned int reg)
{
    unsigned int value;

    if (read_reg(reg, &value) == 0)
        printf("0x%04X = 0x%02X  # %s\n", reg, value, name);
    else
        printf("0x%04X = ERROR   # %s (%s)\n",
               reg, name, strerror(errno));
}

static void dump_range(unsigned int start, unsigned int end)
{
    unsigned int reg;
    unsigned int value;

    printf("\n--- Register range 0x%04X - 0x%04X ---\n",
           start, end);

    for (reg = start; reg <= end; reg++) {

        if (read_reg(reg, &value) == 0) {
            printf("0x%04X = 0x%02X\n", reg, value);
        } else {
            printf("0x%04X = ERROR (%s)\n",
                   reg, strerror(errno));
        }
    }
}

int main(int argc, char **argv)
{
    const char *dev = "/dev/xm_i2c";

    if (argc > 1)
        devaddr = strtoul(argv[1], NULL, 0);

    printf("========================================\n");
    printf(" XM530 / SC5332 diagnostic register dump\n");
    printf("========================================\n\n");

    printf("Device       : %s\n", dev);
    printf("I2C address  : 0x%02X\n", devaddr);
    printf("Address size : 2 bytes\n");
    printf("Data size    : 1 byte\n\n");

    fd = open(dev, O_RDWR);

    if (fd < 0) {
        fprintf(stderr,
                "Cannot open %s: %s\n",
                dev,
                strerror(errno));
        return 1;
    }

    /*
     * First read the most useful registers individually.
     */

    printf("--- Important registers ---\n");

    read_single("Streaming",             0x0100);
    read_single("Software reset",        0x0103);

    read_single("Chip ID high",          0x3107);
    read_single("Chip ID low",           0x3108);

    read_single("HTS high",              0x320C);
    read_single("HTS low",               0x320D);

    read_single("VTS high",              0x320E);
    read_single("VTS low",               0x320F);

    read_single("Image orientation",     0x3221);

    read_single("Analog control 36E9",   0x36E9);
    read_single("Analog control 36F9",   0x36F9);

    read_single("Exposure 3E00",         0x3E00);
    read_single("Exposure 3E01",         0x3E01);
    read_single("Exposure 3E02",         0x3E02);

    read_single("Gain 3E06",             0x3E06);
    read_single("Gain 3E07",             0x3E07);
    read_single("Gain 3E08",             0x3E08);
    read_single("Gain 3E09",             0x3E09);

    read_single("Control 5000",          0x5000);

    /*
     * Diagnostic register ranges.
     *
     * These deliberately read complete blocks instead of only registers
     * currently known by this tool. This lets us compare the live sensor
     * state with the SC5332 initialization code later.
     */

    dump_range(0x3000, 0x303F);
    dump_range(0x3100, 0x313F);

    dump_range(0x3200, 0x323F);
    dump_range(0x3250, 0x327F);

    dump_range(0x3300, 0x33FF);

    dump_range(0x3400, 0x341F);

    dump_range(0x3600, 0x36FF);

    dump_range(0x3700, 0x37FF);

    dump_range(0x3900, 0x39FF);

    dump_range(0x3E00, 0x3E3F);

    dump_range(0x4400, 0x443F);

    dump_range(0x4500, 0x453F);

    dump_range(0x4800, 0x483F);

    dump_range(0x5000, 0x503F);

    dump_range(0x5700, 0x573F);

    close(fd);

    printf("\n========================================\n");
    printf(" Dump complete\n");
    printf("========================================\n");

    return 0;
}
