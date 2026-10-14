#ifndef OS_STRIPE_H
#define OS_STRIPE_H

#include "openscan/openscan_calib.h"

#include <stdint.h>

/* One measured sample. Millimetres in the pattern camera:
 * X right, Y up, Z forward. plane is the projector-plane slot. */
typedef struct os_stripe_pt {
    float x, y, z;
    float tone;
    int plane;
    int ok;
    int u, v; /* pixel in the buffers passed to os_stripe_make */
} os_stripe_pt;

typedef struct os_stripe_cloud {
    os_stripe_pt *pt; /* rows * cols, row-major. Empty slots have ok == 0. */
    int rows, cols;
    int n;
    float median_z;
} os_stripe_cloud;

/* pattern and clean are width by height luminance. pattern is camera A.
 * The NIR columns are already in that frame while A is streaming.
 * flipped is 1 when those buffers are rotated 180° from the calibration.
 * hint_mm is the depth of the center column on a full camera frame.
 * The column rank fixes the plane, and the ray intersection is the relief.
 * Points outside near_mm..far_mm are dropped.
 * Returns 0. out->pt is NULL when nothing measured.
 * docs/projector.md. */
int os_stripe_make(const openscan_calib *cal,
                   const uint8_t *pattern, const uint8_t *clean,
                   int width, int height, int flipped,
                   float hint_mm, float near_mm, float far_mm, int stride,
                   os_stripe_cloud *out);

void os_stripe_free(os_stripe_cloud *cloud);

/* Draw the projector ridges where the planes meet a flat wall at wall_z. */
void os_stripe_paint_wall(uint8_t *img, const openscan_calib *cal, float wall_z);

/* Flat wall in a synthetic rig. 0 when the recovered depth matches that wall. */
int os_stripe_self_test(void);

#endif
