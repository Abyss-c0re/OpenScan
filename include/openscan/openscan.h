#ifndef OPENSCAN_H
#define OPENSCAN_H

/* OpenScan is one object in front of two cameras.
 *
 * A program that only wants the engine includes this header and links
 * libopenscan. The window is a separate program.
 *
 *   openscan_live *live = openscan_live_create(&cal, NULL);
 *   openscan_live_set_mode(live, OPENSCAN_MODE_SCAN);
 *   openscan_live_push(live, gray_a, gray_b, width, height, &status);
 *   openscan_live_write(live, "object.stl", &triangles);
 *   openscan_live_destroy(live);
 *
 * gray_a is the pattern camera. gray_b is the clean picture.
 * Both are one byte per pixel, row by row.
 * Mold is the solid made from confirmed projector points. Measured is the
 * sheet in front of the scanner. Cloud keeps those points, groups them into
 * shapes, and uses that matrix to find the object again.
 * A still picture does not add a second shell. A real turn does.
 */

#include "openscan/openscan_calib.h"
#include "openscan/openscan_live.h"
#include "openscan/openscan_v4l2.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

const char *openscan_version(void);
const char *openscan_host(void);

/* Latest camera frame as gray. storage holds the raw grab.
 * Returns 0, or -1 when the frame is missing or cannot be read.
 * Pattern gray is full range. Studio swing (220/256 plus 16) shrinks the
 * lattice until the stripe gate drops the frame. */
int openscan_frame_gray(openscan_cam *cam, uint8_t *storage, size_t storage_n, uint8_t *gray);

/* Column count of the projector lattice, clip percent, median, and p95.
 * 12 or more columns is a real pattern. p95 catches a washed subject when
 * the black borders keep the clip percent low. Any pointer may be null. */
void openscan_pattern_meter(const uint8_t *y, int w, int h, int *cols, int *clip, int *p50, int *p95);

/* 1 when camera A exposure should change. Does not touch gain or camera B.
 * A washed subject (clip or p95) shortens the shutter, because a long
 * exposure adds room light and flattens the strobe. A dark subject
 * lengthens it, and stops at 24. */
int openscan_pattern_rescue(int cols, int clip, int p95, int exposure, int *next);

int openscan_pattern_self_test(void);

#ifdef __cplusplus
}
#endif

#endif
