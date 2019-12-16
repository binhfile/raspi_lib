#include "ds18b20.h"

#include <stdio.h>
#include "delay.h"
#include "gpio.h"

int ds18b20_initialize(ds18b20* drv) { return 0; }
int ds18b20_destroy(ds18b20* drv) { return 0; }

static int ds18b20_reset(ds18b20* drv, rasp_gpio* gpio) {
    // + MCU output = 1
    // + MCU output = 0 in 480us
    // + Slave wait for 15-60us
    // + Slave output = 0 for 60-240us

    rasp_gpio_set_level(gpio, drv->pin_no, 1);
    rasp_gpio_set_direction(gpio, drv->pin_no, rasp_gpio_direction_out);
    delay_us(1);
    rasp_gpio_set_level(gpio, drv->pin_no, 0);
    delay_us(480);
    rasp_gpio_set_direction(gpio, drv->pin_no, rasp_gpio_direction_in);
    delay_us(60);

    int timeout = 0;
    int prev_level = 1;
    int changed_state_cnt = 0;
    while (changed_state_cnt < 2) {
        timeout = 0;
        int level;
        do {
            level = rasp_gpio_get_level(gpio, drv->pin_no);
            if (level != prev_level) {
                break;
            }
            delay_us(1);
            timeout += 1;
            if (timeout > 500) break;
        } while (prev_level == level);
        if (timeout > 500) {
            break;
        }
        prev_level = level;
        ++changed_state_cnt;
    }
    if (timeout > 500) return ds18b20_error_timeout;
    delay_us(480 + 10);
    return 0;
}
static int ds18b20_write_bit(ds18b20* drv, rasp_gpio* gpio, int bit) {
    // port is already output
    rasp_gpio_set_level(gpio, drv->pin_no, 0);
    rasp_gpio_set_direction(gpio, drv->pin_no, rasp_gpio_direction_out);
    if (bit) {
        delay_us(1);
        rasp_gpio_set_level(gpio, drv->pin_no, 1);
        delay_us(64);
    } else {
        delay_us(65);
        rasp_gpio_set_level(gpio, drv->pin_no, 1);
    }
    delay_us(5);
    rasp_gpio_set_direction(gpio, drv->pin_no, rasp_gpio_direction_in);
    return 0;
}
static int ds18b20_write_byte(ds18b20* drv, rasp_gpio* gpio, uint8_t bit) {
    for (int i = 0; i < 8; i++) {
        ds18b20_write_bit(drv, gpio, (bit & 0x01));
        bit >>= 1;
    }
    return 0;
}
static int ds18b20_read_bit(ds18b20* drv, rasp_gpio* gpio, uint8_t* bit) {
    rasp_gpio_set_level(gpio, drv->pin_no, 0);
    rasp_gpio_set_direction(gpio, drv->pin_no, rasp_gpio_direction_out);
    delay_us(2);
    rasp_gpio_set_direction(gpio, drv->pin_no, rasp_gpio_direction_in);

    int prev_level = 0;
    int level = 0;
    struct timeval timeout;
    struct timeval a = time_now();
    do {
        level = rasp_gpio_get_level(gpio, drv->pin_no);
        struct timeval b = time_now();
        time_diff(&a, &b, &timeout);
        if (level != prev_level) break;
        delay_us(1);
    } while (timeout.tv_sec == 0 && timeout.tv_usec < 255);
    delay_us(70);

    // printf("timeout = %u/%u\r\n", timeout.tv_sec, timeout.tv_usec);

    if (timeout.tv_sec == 0 && timeout.tv_usec < 5) {
        *bit = 1;
        return 0;
    } else if (timeout.tv_sec > 0 || timeout.tv_usec > 255)
        return ds18b20_error_timeout;
    *bit = 0;
    return 0;
}
static uint64_t ds18b20_scan_1_device(ds18b20* drv, rasp_gpio* gpio) {
    uint64_t id = 0;

    int err = ds18b20_reset(drv, gpio);
    if (err) return 0;
    ds18b20_write_byte(drv, gpio, 0xF0);

    int id_bit_cnt = 0;
    uint8_t bit, bit_cmp;
    while (id_bit_cnt < 64) {
        err = ds18b20_read_bit(drv, gpio, &bit);
        if (err) break;
        err = ds18b20_read_bit(drv, gpio, &bit_cmp);
        if (err) break;
        if (bit == bit_cmp) {
            // printf("bit[%u]=%u/%u\r\n", id_bit_cnt, bit, bit_cmp);
            return 0;
        } else {
            // printf("bit[%u]=%u\r\n", id_bit_cnt, bit);

            if (bit)
                id |= (((uint64_t)1) << id_bit_cnt);
            else
                id &= ~(((uint64_t)1) << id_bit_cnt);
            ds18b20_write_bit(drv, gpio, bit);
        }
        ++id_bit_cnt;
    }
    if (err) return 0;
    return id;
}
static int ds18b20_initialize_gpio(ds18b20* drv, rasp_gpio* gpio) {
    int err = rasp_gpio_initialize(gpio);
    if (err) {
        return err;
    }
    err = rasp_gpio_set_pull(gpio, drv->pin_no, rasp_gpio_pull_mode_up);
    return err;
}
static int ds18b20_destroy_gpio(ds18b20* drv, rasp_gpio* gpio) {
    rasp_gpio_destroy(gpio);
    return 0;
}
static int ds18b20_read_byte(ds18b20* drv, rasp_gpio* gpio, uint8_t* result) {
    uint8_t bit;
    uint8_t ret = 0;
    for (int i = 0; i < 8; i++) {
        int err = ds18b20_read_bit(drv, gpio, &bit);
        if (err) return err;
        ret >>= 1;
        ret |= (((uint8_t)bit) << 7) & 0x80;
    }
    *result = ret;
    return 0;
}
static uint8_t ds18b20_calc_crc(uint8_t* addr, uint8_t len) {
    uint8_t crc = 0, inbyte, i, mix;

    while (len--) {
        inbyte = *addr++;
        for (i = 8; i; i--) {
            mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            inbyte >>= 1;
        }
    }
    return crc;
}

#define DS18B20_DECIMAL_STEPS_12BIT 0.0625
#define DS18B20_DECIMAL_STEPS_11BIT 0.125
#define DS18B20_DECIMAL_STEPS_10BIT 0.25
#define DS18B20_DECIMAL_STEPS_9BIT 0.5
int ds18b20_read(ds18b20* drv, uint64_t id, int* result) {
    int err = 0;

    rasp_gpio gpio;

    err = ds18b20_initialize_gpio(drv, &gpio);
    if (err) {
        return err;
    }

    // convert
    err = ds18b20_reset(drv, &gpio);
    if (err) {
        ds18b20_destroy_gpio(drv, &gpio);
        return err;
    }

    ds18b20_write_byte(drv, &gpio, 0x55);
    for (int i = 0; i < 8; i++) {
        ds18b20_write_byte(drv, &gpio, (id >> (8 * i)) & 0xFF);
    }

    ds18b20_write_byte(drv, &gpio, 0x44);

    int timeout = 0;
    do {
        uint8_t level;
        err = ds18b20_read_bit(drv, &gpio, &level);
        if (err) {
            ds18b20_destroy_gpio(drv, &gpio);
            return err;
        }
        if (level) break;
        delay_ms(100);
        timeout += 100;
    } while (timeout < 1000);
    if (timeout >= 1000) {
        ds18b20_destroy_gpio(drv, &gpio);
        return ds18b20_error_timeout;
    }
    delay_us(5);

    // read
    err = ds18b20_reset(drv, &gpio);
    if (err) {
        ds18b20_destroy_gpio(drv, &gpio);
        return err;
    }

    ds18b20_write_byte(drv, &gpio, 0x55);
    for (int i = 0; i < 8; i++) {
        ds18b20_write_byte(drv, &gpio, (id >> (8 * i)) & 0xFF);
    }

    uint8_t data[9] = {0};
    ds18b20_write_byte(drv, &gpio, 0xBE);
    for (int i = 0; i < 9; i++) {
        err = ds18b20_read_byte(drv, &gpio, &data[i]);
        if (err) break;
    }
    ds18b20_destroy_gpio(drv, &gpio);

    uint8_t crc = ds18b20_calc_crc(data, 8);
    if (crc != data[8]) return ds18b20_error_crc;

    uint8_t resolution;
    int8_t digit, minus = 0;
    float decimal;
    uint16_t temperature = ((uint16_t)data[0]) | (((uint16_t)data[1]) << 8);
    /* Check if temperature is negative */
    if (temperature & 0x8000) {
        /* Two's complement, temperature is negative */
        temperature = ~temperature + 1;
        minus = 1;
    }

    /* Get sensor resolution */
    resolution = ((data[4] & 0x60) >> 5) + 9;

    /* Store temperature integer digits and decimal digits */
    digit = temperature >> 4;
    digit |= ((temperature >> 8) & 0x7) << 4;

    /* Store decimal digits */
    switch (resolution) {
        case 9:
            decimal = (temperature >> 3) & 0x01;
            decimal *= (float)DS18B20_DECIMAL_STEPS_9BIT;
            break;
        case 10:
            decimal = (temperature >> 2) & 0x03;
            decimal *= (float)DS18B20_DECIMAL_STEPS_10BIT;
            break;
        case 11:
            decimal = (temperature >> 1) & 0x07;
            decimal *= (float)DS18B20_DECIMAL_STEPS_11BIT;
            break;
        case 12:
            decimal = temperature & 0x0F;
            decimal *= (float)DS18B20_DECIMAL_STEPS_12BIT;
            break;
        default:
            decimal = 0xFF;
            digit = 0;
    }

    /* Check for negative part */
    decimal = digit + decimal;
    if (minus) decimal = 0 - decimal;

    *result = decimal * 1000;

//    printf("%02X %02X %02X %02X %02X %02X %02X %02X error=%d\r\n", data[0],
//           data[1], data[2], data[3], data[4], data[5], data[6], data[7], err);

    return err;
}
/**
 * @return int Number of device is found in `device`
 */
int ds18b20_scan(ds18b20* drv, uint64_t* device, int device_size) {
    int size = 0;

    rasp_gpio gpio;

    int err = ds18b20_initialize_gpio(drv, &gpio);
    if (err) {
        return 0;
    }

    while (size < device_size) {
        uint64_t id = ds18b20_scan_1_device(drv, &gpio);
        if (id != 0) {
            device[size++] = id;
        } else {
            break;
        }
    }

    ds18b20_destroy_gpio(drv, &gpio);

    return size;
}
