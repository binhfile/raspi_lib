#include <signal.h>
#include <stdio.h>
#include "delay.h"
#include "ds18b20.h"

//static volatile int s_term = 0;
//static void int_handler(int sig) { s_term = 1; }

int main() {    
	ds18b20 drv;
    
    drv.pin_no = 23;
    int err = ds18b20_initialize(&drv);
    if(err){
		return err;
	}
	
	uint64_t device_id[3] = {0};
	int device_size = ds18b20_scan(&drv, device_id, 1);
	printf("found %u device(s)\r\n", device_size);
	for(int i = 0; i < device_size; i++) {
		printf("    %08X%08X\r\n",
			(uint32_t)((device_id[i] >> 32) & 0xFFFFFFFF),
			(uint32_t)(device_id[i] & 0xFFFFFFFF));
		int temperature = 0;
		err = ds18b20_read(&drv, device_id[i], &temperature);
		if(err){
		    printf("    read with error=%d\r\n", err);
            continue;
		}
		printf("    T=%.3f\r\n", (float)temperature / 1000.0f);
	}
    
    ds18b20_destroy(&drv); 
    return 0;
}
