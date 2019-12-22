#include "dht11.h"

#include "delay.h"
#include "gpio.h"

int dht11_initialize(dht11* drv) {
    for (int i = 0; i < 4; i++) {
        drv->internal.data[i] = 0;
    }
    return 0;
}
int dht11_destroy(dht11* drv) { return 0; }

static void set_low(gpio* drv, unsigned int pin) {
    gpio_set_direction(drv, pin, gpio_direction_out);
    gpio_set_level(drv, pin, 0);
}
static void set_high(gpio* drv, unsigned int pin) {
    gpio_set_direction(drv, pin, gpio_direction_out);
    gpio_set_level(drv, pin, 1);
}

int dht11_read(dht11* drv) {
    gpio gpio;
    int err = gpio_initialize(&gpio);
    if (err) {
        return dht11_error_gpio_failed;
    }
    err = gpio_set_pull(&gpio, drv->pin_no, gpio_pull_mode_up);
    if (err) {
        gpio_destroy(&gpio);
        return dht11_error_gpio_failed;
    }

    // + line = 1 (pull up)
    // + MCU set line = 0 in 18us
    set_low(&gpio, drv->pin_no);
    delay_ms(18);
    // +  MCU set line = 1 in 40us
    set_high(&gpio, drv->pin_no);
    delay_us(40);

    gpio_set_direction(&gpio, drv->pin_no, gpio_direction_in);
    int prev_level = 1;
    int baud_cnt = 0;
    int bit_time[40] = {0};
    int bit_time_size = 0;
    err = 0;
    while (1) {
        // wait for line's state is changed
        int level;
        int timeout = 0;
        do {
            level = gpio_get_level(&gpio, drv->pin_no);
            if (level != prev_level) {
                break;
            }
            delay_us(1);
            timeout += 1;
            if (timeout > 255) break;
        } while (prev_level == level);

        if (timeout > 255) {
            err = dht11_error_timeout;
            break;
        }

        if ((baud_cnt >= 4) && (baud_cnt % 2 == 0)) {
            if (bit_time_size < 40) {
                bit_time[bit_time_size++] = timeout;
            }
        }
        if (bit_time_size >= 40) {
            break;
        }
        prev_level = level;
        ++baud_cnt;
    }

    gpio_set_direction(&gpio, drv->pin_no, gpio_direction_out);
    gpio_destroy(&gpio);

    if (err) return err;

    const int time_threshold = 15;

    uint8_t data[5] = {0};
    for (int i = 0; i < bit_time_size; i++) {
        // int bit;
        uint8_t c = data[i / 8] << 1;
        if (bit_time[i] < time_threshold) {
            // bit = 0;
        } else {
            // bit = 1;
            c |= 1;
        }
        data[i / 8] = c;

        // printf("%u @%d\r\n", bit, bit_time[i]);
    }
    uint8_t calc_sum = data[0] + data[1] + data[2] + data[3];
    if (calc_sum != data[4]) {
        return dht11_error_checksum;
    }
    // printf("RH=%d.%d T=%d.%d SUM=%02x/%02x Bit=%u\r\n", data[0], data[1],
    //       data[2], data[3], data[4], calc_sum, bit_time_size);

    for (int i = 0; i < 4; i++) drv->internal.data[i] = data[i];

    return 0;
}
/**
 * @return int Temperature (C) * 1000
 */
int dht11_get_temperature(dht11* drv) {
    return ((int)drv->internal.data[2]) * 1000 + drv->internal.data[3];
}
/**
 * @return int Humidity (RH) * 1000
 */
int dht11_get_humidity(dht11* drv) {
    return ((int)drv->internal.data[0]) * 1000 + drv->internal.data[1];
}
