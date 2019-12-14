
#include <signal.h>
#include <stdio.h>
#include "delay.h"
#include "dht11.h"

static volatile int s_term = 0;
static void int_handler(int sig) { s_term = 1; }

int main() {    
    dht11 drv;
    drv.pin_no = 23;
    
    int err = dht11_initialize(&drv);
    if(err) {
		printf("dht11 initialize with error = %d\r\n", err);
		return err;
	}
	
	while(!s_term){
		err = dht11_read(&drv);
		if(err) {
			printf("dht11 read with error = %d\r\n", err);
		}
		else{
			printf("Temperature=%.3f Humidity=%.3f\r\n", 
				dht11_get_temperature(&drv) / 1000.0f, 
				dht11_get_humidity(&drv) / 1000.0f);
		}
		
		delay_ms(2 * 1000);
	}
	
	dht11_destroy(&drv);
    return 0;
}
