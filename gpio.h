#ifndef __PI_GPIO_H__
#define __PI_GPIO_H__

#include <stdint.h>

/**
 /dev/gpiomem
 or
 /dev/mem
 /proc/device-tree/soc/ranges

 */

typedef struct gpio_internal_ {
    unsigned int hw_base_address;
    void* gpio_base;
    int is_2711;
} gpio_internal;

typedef struct gpio_ {
    gpio_internal internal;
} gpio;

typedef enum gpio_pull_mode_ {
    gpio_pull_mode_none = 0,
    gpio_pull_mode_down,
    gpio_pull_mode_up,
} gpio_pull_mode;
typedef enum gpio_direction_ {
    gpio_direction_in = 0,
    gpio_direction_out,
} gpio_direction;

int gpio_initialize(gpio* drv);
int gpio_destroy(gpio* drv);

int gpio_set_pull(gpio* drv, int pin_no, gpio_pull_mode mode);
int gpio_set_direction(gpio* drv, int pin_no, gpio_direction dir);

int gpio_set_level(gpio* drv, int pin_no, int level_output);
int gpio_get_level(gpio* drv, int pin_no);

#endif
