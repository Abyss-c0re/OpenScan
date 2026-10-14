#ifndef LIZARD_COLOR_H
#define LIZARD_COLOR_H

#include <stdint.h>

#include "openscan/openscan_calib.h"

/* Project a camera-A point into camera B. y_up_mm is the stripe frame (Y up).
 * Returns 0 when the point is behind camera B. */
int lz_project_b(const openscan_pinhole *cam, float x_mm, float y_up_mm, float z_mm,
                 float *u, float *v);

/* Sample YUV 4:2:0 at a full-resolution pixel. u/v planes are half size.
 * uv_pixel_stride is 1 for planar and 2 for interleaved chroma. */
void lz_sample_yuv(const uint8_t *y, int w, int h, int y_stride,
                   const uint8_t *u, const uint8_t *v,
                   int uv_w, int uv_h, int u_stride, int v_stride, int uv_pixel_stride,
                   float pu, float pv, float *r, float *g, float *b);

#endif
