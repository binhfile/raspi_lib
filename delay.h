#ifndef __PI_DELAY_H__
#define __PI_DELAY_H__

#include <sys/time.h>

void delay_us(unsigned int us);
void delay_ms(unsigned int ms);

struct timeval time_now();
void time_diff(const struct timeval *a, const struct timeval *b, struct timeval *result);

#endif
