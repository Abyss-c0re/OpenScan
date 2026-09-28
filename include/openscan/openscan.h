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
 * Mold is the solid made from the outline. Measured is the flat sheet.
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
 * Returns 0, or -1 when the frame is missing or cannot be read. */
int openscan_frame_gray(openscan_cam *cam, uint8_t *storage, size_t storage_n, uint8_t *gray);

#ifdef __cplusplus
}
#endif

#endif
