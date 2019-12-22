#include "gpio.h"

#include "delay.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/* --------------------- */

#define GPIO_MIN 0
#define GPIO_MAX 53

#define GPSET0 7
#define GPSET1 8
#define GPCLR0 10
#define GPCLR1 11
#define GPLEV0 13
#define GPLEV1 14
#define GPPUD 37
#define GPPUDCLK0 38
#define GPPUDCLK1 39
/* 2711 has a different mechanism for pin pull-up/down/enable  */
#define GPPUPPDN0 57 /* Pin pull-up/down for pins 15:0  */
#define GPPUPPDN1 58 /* Pin pull-up/down for pins 31:16 */
#define GPPUPPDN2 59 /* Pin pull-up/down for pins 47:32 */
#define GPPUPPDN3 60 /* Pin pull-up/down for pins 57:48 */

#define BLOCK_SIZE (4 * 1024)
#define GPIO_BASE_OFFSET 0x00200000

static unsigned int get_regs(const void* base, unsigned int reg) {
    return *((volatile unsigned int*)(((unsigned int)base + reg * 4)));
}
static void set_regs(const void* base, unsigned int reg, unsigned int value) {
    *((volatile unsigned int*)(((unsigned int)base + reg * 4))) = value;
}
static uint32_t get_hwbase(void) {
    const char* ranges_file = "/proc/device-tree/soc/ranges";
    uint8_t ranges[12];
    FILE* fd;
    uint32_t ret = 0;

    memset(ranges, 0, sizeof(ranges));

    if ((fd = fopen(ranges_file, "rb")) == NULL) {
        printf("Can't open '%s'\n", ranges_file);
    } else if (fread(ranges, 1, sizeof(ranges), fd) >= 8) {
        ret = (ranges[4] << 24) | (ranges[5] << 16) | (ranges[6] << 8) |
              (ranges[7] << 0);
        if (!ret)
            ret = (ranges[8] << 24) | (ranges[9] << 16) | (ranges[10] << 8) |
                  (ranges[11] << 0);
        if ((ranges[0] != 0x7e) || (ranges[1] != 0x00) || (ranges[2] != 0x00) ||
            (ranges[3] != 0x00) ||
            ((ret != 0x20000000) && (ret != 0x3f000000) &&
             (ret != 0xfe000000))) {
            printf(
                "Unexpected ranges data (%02x%02x%02x%02x %02x%02x%02x%02x "
                "%02x%02x%02x%02x)\n",
                ranges[0], ranges[1], ranges[2], ranges[3], ranges[4],
                ranges[5], ranges[6], ranges[7], ranges[8], ranges[9],
                ranges[10], ranges[11]);
            ret = 0;
        }
    } else {
        printf("Ranges data too short\n");
    }

    fclose(fd);

    return ret;
}

static int gpio_set_pull_(gpio* drv, int gpio, gpio_pull_mode type) {
    if (gpio < GPIO_MIN || gpio > GPIO_MAX) return -1;
    if (type < 0 || type > 2) return -1;

    void* gpio_base = drv->internal.gpio_base;
    if (drv->internal.is_2711) {
        int pullreg = GPPUPPDN0 + (gpio >> 4);
        int pullshift = (gpio & 0xf) << 1;
        unsigned int pullbits;
        unsigned int pull;

        switch (type) {
            case gpio_pull_mode_none:
                pull = 0;
                break;
            case gpio_pull_mode_up:
                pull = 1;
                break;
            case gpio_pull_mode_down:
                pull = 2;
                break;
            default:
                return 1; /* An illegal value */
        }

        pullbits = get_regs(gpio_base, pullreg);
        pullbits &= ~(3 << pullshift);
        pullbits |= (pull << pullshift);
        set_regs(gpio_base, pullreg, pullbits);
    } else {
        int clkreg = GPPUDCLK0 + (gpio >> 5);
        int clkbit = 1 << (gpio & 0x1f);

        set_regs(gpio_base, GPPUD, type);
        delay_us(10);
        set_regs(gpio_base, clkreg, clkbit);
        delay_us(10);
        set_regs(gpio_base, GPPUD, 0);
        delay_us(10);
        set_regs(gpio_base, clkreg, 0);
        delay_us(10);
    }

    return 0;
}
static void set_gpio_fsel(gpio* drv, int gpio, int fsel) {
    uint32_t reg = gpio / 10;
    uint32_t sel = gpio % 10;
    uint32_t mask;
    if (gpio < GPIO_MIN || gpio > GPIO_MAX) return;
    mask = 0x7 << (3 * sel);
    mask = ~mask;
    set_regs(drv->internal.gpio_base, reg,
             get_regs(drv->internal.gpio_base, reg) & mask);
    set_regs(
        drv->internal.gpio_base, reg,
        get_regs(drv->internal.gpio_base, reg) | ((fsel & 0x7) << (3 * sel)));
}

/* --------------------- */

int gpio_initialize(gpio* drv) {
    drv->internal.hw_base_address = 0;
    drv->internal.gpio_base = 0;
    drv->internal.is_2711 = 0;

    int fd = open("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC);
    if (fd >= 0) {
        drv->internal.gpio_base = (uint32_t*)mmap(
            0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        close(fd);
    }
    if (drv->internal.gpio_base == 0) {
        fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
        if (fd >= 0) {
            drv->internal.hw_base_address = get_hwbase();
            drv->internal.gpio_base = (uint32_t*)mmap(
                0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                GPIO_BASE_OFFSET + drv->internal.hw_base_address);
            close(fd);
        }
    }
    if (drv->internal.gpio_base == 0) {
        return -1;
    }
    drv->internal.is_2711 =
        (get_regs(drv->internal.gpio_base, GPPUPPDN3) != 0x6770696f);

    return 0;
}
int gpio_destroy(gpio* drv) {
    if (drv->internal.gpio_base != 0) {
        munmap(drv->internal.gpio_base, BLOCK_SIZE);
        drv->internal.gpio_base = 0;
    }
    return 0;
}
int gpio_set_pull(gpio* drv, int pin_no, gpio_pull_mode mode) {
    gpio_set_pull_(drv, pin_no, mode);
    return 0;
}
int gpio_set_direction(gpio* drv, int pin_no, gpio_direction dir) {
    set_gpio_fsel(drv, pin_no, dir);
    return 0;
}
int gpio_set_level(gpio* drv, int pin_no, int level_output) {
    if (pin_no < GPIO_MIN || pin_no > GPIO_MAX) return -1;
    void* gpio_base = drv->internal.gpio_base;
    if (level_output != 0) {
        if (pin_no < 32) {
            set_regs(gpio_base, GPSET0, 0x1 << pin_no);
        } else {
            pin_no -= 32;
            set_regs(gpio_base, GPSET1, 0x1 << pin_no);
        }
    } else {
        if (pin_no < 32) {
            set_regs(gpio_base, GPCLR0, 0x1 << pin_no);
        } else {
            pin_no -= 32;
            set_regs(gpio_base, GPCLR1, 0x1 << pin_no);
        }
    }
    return 0;
}
int gpio_get_level(gpio* drv, int pin_no) {
    if (pin_no < GPIO_MIN || pin_no > GPIO_MAX) return -1;
    void* gpio_base = drv->internal.gpio_base;
    if (pin_no < 32) {
        return ((get_regs(gpio_base, GPLEV0)) >> pin_no) & 0x1;
    } else {
        pin_no = pin_no - 32;
        return ((get_regs(gpio_base, GPLEV1)) >> pin_no) & 0x1;
    }
}
