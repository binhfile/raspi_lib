#include "ds18b20.h"
#include <signal.h>
#include <stdio.h>
#include "delay.h"

// static volatile int s_term = 0;
// static void int_handler(int sig) { s_term = 1; }

int main() {
    ds18b20 drv;
    int temperature = 0;
    ds18b20_resolution resolution = ds18b20_resolution_9bit;

    drv.pin_no = 23;
    int err = ds18b20_initialize(&drv);
    if (err) {
        return err;
    }

    printf("Action with ID\r\n");

    uint64_t device_id[3] = {0};
    int device_size = ds18b20_scan(&drv, device_id, 1);
    printf("found %u device(s)\r\n", device_size);
    for (int i = 0; i < device_size; i++) {
        printf("    %08X%08X\r\n",
               (uint32_t)((device_id[i] >> 32) & 0xFFFFFFFF),
               (uint32_t)(device_id[i] & 0xFFFFFFFF));
        err = ds18b20_read(&drv, device_id[i], &temperature, &resolution);
        if (err) {
            printf("    read with error=%d\r\n", err);
            continue;
        }
        printf("    T=%.3f Bit=%d\r\n", (float)temperature / 1000.0f,
               resolution);
    }

    printf("Action without ID\r\n");
    err = ds18b20_read_one_device(&drv, &temperature, &resolution);
    if (err) {
        printf("    read with error=%d\r\n", err);
    }
    if (!err) {
        printf("    T=%.3f Bit=%d\r\n", (float)temperature / 1000.0f,
               resolution);
    }

    ds18b20_destroy(&drv);
    return 0;
}
