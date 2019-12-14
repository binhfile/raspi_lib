#ifndef __PI_DHT11_H__
#define __PI_DHT11_H__

#include <stdint.h>

typedef struct dht11_internal_ {
    uint8_t data[4];
} dht11_internal;

typedef struct dht11_ {
    int pin_no;
    
    dht11_internal internal;
} dht11;

enum dht11_error {
	dht11_error_none = 0,
	dht11_error_gpio_failed = 1,
	dht11_error_timeout = 2,
	dht11_error_checksum = 3,
};

int dht11_initialize(dht11* drv);
int dht11_destroy(dht11* drv);
int dht11_read(dht11* drv);
/**
 * @return int Temperature (C) * 1000
 */
int dht11_get_temperature(dht11* drv);
/**
 * @return int Humidity (RH) * 1000
 */
int dht11_get_humidity(dht11* drv);

#endif
