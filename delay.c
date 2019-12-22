#include <sys/time.h>
#include <time.h>
#include <unistd.h>
static void delayMicrosecondsHard(unsigned int howLong) {
    struct timeval tNow, tLong, tEnd;

    gettimeofday(&tNow, NULL);
    tLong.tv_sec = howLong / 1000000;
    tLong.tv_usec = howLong % 1000000;
    timeradd(&tNow, &tLong, &tEnd);

    while (timercmp(&tNow, &tEnd, <)) gettimeofday(&tNow, NULL);
}

struct timeval time_now() {
    struct timeval tNow;
    gettimeofday(&tNow, NULL);
    return tNow;
}

void time_diff(const struct timeval *a, const struct timeval *b,
               struct timeval *result) {
    timersub(b, a, result);
}

void delay_us(unsigned int howLong) {
    struct timespec sleeper;
    unsigned int uSecs = howLong % 1000000;
    unsigned int wSecs = howLong / 1000000;

    /**/ if (howLong == 0)
        return;
    else if (howLong < 100)
        delayMicrosecondsHard(howLong);
    else {
        sleeper.tv_sec = wSecs;
        sleeper.tv_nsec = (long)(uSecs * 1000L);
        nanosleep(&sleeper, NULL);
    }
}
void delay_ms(unsigned int howLong) {
    struct timespec sleeper, dummy;

    sleeper.tv_sec = (time_t)(howLong / 1000);
    sleeper.tv_nsec = (long)(howLong % 1000) * 1000000;

    nanosleep(&sleeper, &dummy);
}
