#ifndef __PI_DS18B20_H__
#define __PI_DS18B20_H__

#include <stdint.h>

typedef struct ds18b20_internal_ {
} ds18b20_internal;

enum ds18b20_error {
    ds18b20_error_none = 0,
    ds18b20_error_crc,
    ds18b20_error_timeout,
};

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
 * @param drv
 * @param id
 * @param result  Temperature (C) * 1000
 * @return int Error Code
 */
int ds18b20_read(ds18b20* drv, uint64_t id, int* result);

#endif
