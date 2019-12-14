#ifndef __PI_GPIO_H__
#define __PI_GPIO_H__

#include <stdint.h>

/**
 /dev/gpiomem
 or
 /dev/mem
 /proc/device-tree/soc/ranges

 */

typedef struct rasp_gpio_internal_ {
    unsigned int hw_base_address;
    void* gpio_base;
    int is_2711;
} rasp_gpio_internal;

typedef struct rasp_gpio_ {
    rasp_gpio_internal internal;
} rasp_gpio;

enum rasp_gpio_pull_mode {
    rasp_gpio_pull_mode_none = 0,
    rasp_gpio_pull_mode_down,
    rasp_gpio_pull_mode_up,
};
enum rasp_gpio_direction {
    rasp_gpio_direction_in = 0,
    rasp_gpio_direction_out,
};

int rasp_gpio_initialize(rasp_gpio* drv);
int rasp_gpio_destroy(rasp_gpio* drv);

int rasp_gpio_set_pull(rasp_gpio* drv, int pin_no, rasp_gpio_pull_mode mode);
int rasp_gpio_set_direction(rasp_gpio* drv, int pin_no,
                            rasp_gpio_direction dir);

int rasp_gpio_set_level(rasp_gpio* drv, int pin_no, int level_output);
int rasp_gpio_get_level(rasp_gpio* drv, int pin_no);

#endif
