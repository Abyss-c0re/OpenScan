#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include "os_host.h"
void os_host_sleep_ms(int ms) {
    if (ms > 0) Sleep((unsigned)ms);
}
#else
#define _POSIX_C_SOURCE 200809L
#include <time.h>
#include "os_host.h"
void os_host_sleep_ms(int ms) {
    struct timespec ts;
    if (ms < 0) ms = 0;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}
#endif
