#ifndef OS_SURF_H
#define OS_SURF_H
#include <stdint.h>

/* One window. y grows downward. Text y is the baseline.
 * rgb is 0xRRGGBB. The scan loop is the same on every host. */

typedef struct os_surf os_surf;

os_surf *os_surf_open(int w, int h);
void os_surf_close(os_surf *s);
/* One event. Returns 0 when the queue is empty.
 * A click sets mx, my. Otherwise both are -1. key is 0 when none. */
int os_surf_pump(os_surf *s, int *mx, int *my, int *key, int *quit);
void os_surf_blit_bgr(os_surf *s, const uint8_t *bgr, int w, int h, int x, int y);
void os_surf_bar(os_surf *s, int x, int y, int w, int h, unsigned rgb);
void os_surf_text(os_surf *s, int x, int y, const char *text, unsigned rgb);
void os_surf_flush(os_surf *s);

#endif
