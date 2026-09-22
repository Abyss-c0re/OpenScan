#ifndef FOX_SCAN_H
#define FOX_SCAN_H

#include "fox_calib.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fox_scan_opts {
    double scale;       /* image scale before matching, 0.25..1 */
    double min_mm;      /* keep points nearer than this (mm) */
    double max_mm;
    double edge_mm;     /* break triangles across jumps larger than this */
    const char *preview_png; /* optional left-rectified preview */
} fox_scan_opts;

/* Stereo match camera A (Y) against camera B (Y) using the factory calib.
 * Tries both left/right assignments and keeps the one that lands more
 * points inside [min_mm, max_mm]. Writes a binary STL. */
int fox_scan_to_stl(const uint8_t *ya, const uint8_t *yb, int width, int height,
                    const fox_calib *calib, const fox_scan_opts *opt,
                    const char *stl_path, int *triangles_out);

#ifdef __cplusplus
}
#endif

#endif
