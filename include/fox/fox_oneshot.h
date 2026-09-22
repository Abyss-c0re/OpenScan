#ifndef FOX_ONESHOT_H
#define FOX_ONESHOT_H

#include "fox/fox_calib.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One measured dot on a projector stripe. x,y,z are millimetres in the
 * pattern camera (camera A). u,v is that dot in the pattern image. */
typedef struct fox_shot {
    float x, y, z;
    float u, v;
} fox_shot;

/* Decode the pattern image the way OneShot does: each stripe is one of the
 * calibrated light planes, and the dot is the ray-plane intersection.
 * target_z_mm picks which plane the middle stripe belongs to.
 * Writes up to cap points. Returns how many were written, or -1. */
int fox_oneshot_points(const uint8_t *pattern, int width, int height,
                       const fox_calib *cal, float target_z_mm,
                       fox_shot *out, int cap);

/* Same decode, stored as a depth image in the pattern-camera sensor frame.
 * depth_out has width*height floats, millimetres, 0 where no stripe was
 * decoded. Returns the number of measured points, or -1. */
int fox_oneshot_depth(const uint8_t *pattern, int width, int height,
                      const fox_calib *cal, float target_z_mm,
                      float *depth_out);

#ifdef __cplusplus
}
#endif

#endif
