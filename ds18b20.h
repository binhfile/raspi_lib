#ifndef __PI_DS18B20_H__
#define __PI_DS18B20_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ds18b20_internal_ {
} ds18b20_internal;

typedef enum ds18b20_error_ {
    ds18b20_error_none = 0,
    ds18b20_error_crc,
    ds18b20_error_timeout,
} ds18b20_error;

typedef enum ds18b20_resolution_ {
    ds18b20_resolution_9bit = 9,
    ds18b20_resolution_10bit,
    ds18b20_resolution_11bit,
    ds18b20_resolution_12bit
} ds18b20_resolution;

typedef struct ds18b20_ {
    int pin_no;
    ds18b20_internal internal;
} ds18b20;

int ds18b20_initialize(ds18b20* drv);
int ds18b20_destroy(ds18b20* drv);

/**
 * @return int Number of device is found in `device`
 */
int ds18b20_scan(ds18b20* drv, uint64_t* device, int device_size);

/**
 *
 * @param drv Driver instance
 * @param id Selected device's id
 * @param result  Temperature (C) * 1000
 * @param resolution Number of resolution bit(s)
 * @return int Error Code
 */
int ds18b20_read(ds18b20* drv, uint64_t id, int* result,
                 ds18b20_resolution* resolution);
int ds18b20_read_one_device(ds18b20* drv, int* result,
                            ds18b20_resolution* resolution);

#ifdef __cplusplus
};
#endif
#endif
