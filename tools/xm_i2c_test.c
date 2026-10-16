#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>

#define CMD_I2C_READ 0x03

typedef struct {
    unsigned char dev_addr;
    unsigned int reg_addr;
    unsigned int addr_byte_num;
    unsigned int data;
    unsigned int data_byte_num;
} I2C_DATA_S;

static const unsigned int regs[] = {
    0x0100,

    /* Chip identification */
    0x3107,
    0x3108,

    /* Timing */
    0x320C,
    0x320D,
    0x320E,
    0x320F,

    /* Orientation */
    0x3221,

    /* Analog */
    0x36E9,
    0x36F9,

    /* Exposure / gain */
    0x3E00,
    0x3E01,
    0x3E02,
    0x3E06,
    0x3E07,
    0x3E08,
    0x3E09,

    0x5000
};

int main(int argc, char **argv)
{
    const char *dev = "/dev/xm_i2c";
    unsigned int devaddr = 0x30;

    if (argc > 1)
        devaddr = strtoul(argv[1], NULL, 0);

    printf("XM530 I2C diagnostic\n");
    printf("Device: %s\n", dev);
    printf("I2C address: 0x%02X\n\n", devaddr);

    int fd = open(dev, O_RDWR);

    if (fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n",
                dev, strerror(errno));
        return 1;
    }

    for (unsigned int i = 0;
         i < sizeof(regs) / sizeof(regs[0]);
         i++) {

        I2C_DATA_S d;
        memset(&d, 0, sizeof(d));

        d.dev_addr = (unsigned char)devaddr;
        d.reg_addr = regs[i];
        d.addr_byte_num = 2;
        d.data_byte_num = 1;

        errno = 0;

        int ret = ioctl(fd, CMD_I2C_READ, &d);

        if (ret < 0) {
            printf("0x%04X : ERROR (%s)\n",
                   regs[i],
                   strerror(errno));
        } else {
            printf("0x%04X : 0x%02X\n",
                   regs[i],
                   d.data & 0xff);
        }
    }

    close(fd);
    return 0;
}
