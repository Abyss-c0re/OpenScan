#ifndef OPENSCAN_CALIB_H
#define OPENSCAN_CALIB_H

#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Per-scanner calibration text. Not included with OpenScan.
 * The owner of a unit supplies calib/<serial>.txt.
 *
 *   <width> <height>
 *   <a> <b>
 *   <pattern_w> <pattern_h>
 *   <code> <offset>
 *   <15 floats>     camera 1: fx fy cx cy k1 k2 p1 p2 k3 + 6 zeros
 *   <15 floats>     camera 2: fx fy cx cy k1 k2 p1 p2 k3 + rodrigues(3) + t(3 mm)
 *   <15 floats>     projector modelled as a third pinhole
 *   ... lookup and spatial code ...
 *   ***DevID:<sn>***CalibrateDate:...
 *
 * Camera 2's translation is in millimetres relative to camera 1.
 * On this Fox the baseline is about 32 mm along X. */

typedef struct openscan_pinhole {
    double fx, fy, cx, cy;
    double k1, k2, p1, p2, k3;
    double rvec[3];
    double tvec[3]; /* millimetres, camera-2 and projector only */
} openscan_pinhole;

/* A projector stripe. In the projector image it is the line
 * u = slope * v + offset. That line is one projector plane. */
typedef struct openscan_plane {
    int index;
    double slope;
    double offset;
} openscan_plane;

#define OPENSCAN_MAX_PLANES 160

typedef struct openscan_calib {
    int width, height;
    char serial[64];
    char date[64];
    openscan_pinhole cam[3]; /* 0 pattern camera, 1 clean camera, 2 projector */
    int nplanes;
    openscan_plane plane[OPENSCAN_MAX_PLANES];
} openscan_calib;

/* A factory file starts with the image width. Leading space is ignored,
 * the same rule as Android CalibName.looksLikeCalib. */
static inline int openscan_calib_text_ok(const char *text) {
    if (!text) return 0;
    while (*text == ' ' || *text == '\n' || *text == '\r' || *text == '\t') text++;
    if (*text < '0' || *text > '9') return 0;
    return strstr(text, "DevID:") != NULL;
}

int openscan_calib_load(const char *path, openscan_calib *out);

/* Load path only when its DevID is this serial. Other units are rejected. */
int openscan_calib_load_for(const char *path, const char *serial, openscan_calib *out);

/* Optional lookup for this serial's own calibration file.
 * Runs only when a local signing key is set (OPENSCAN_CALIB_SIGN or
 * calib/sign.local). That key is not part of this source tree.
 * The file is not uploaded. Writes dest_path. */
int openscan_calib_fetch(const char *serial, const char *dest_path);

/* Load a local file for this serial, or download one into
 * $XDG_DATA_HOME/openscan/calib/<serial>.txt when none is on disk.
 * A file is accepted only when its DevID is this serial.
 * explicit_path, when set, is the only file tried, is never downloaded,
 * and is not checked against the serial. used_path may be NULL. */
int openscan_calib_ensure(const char *serial, const char *explicit_path,
                     openscan_calib *out, char *used_path, size_t used_n);

#ifdef __cplusplus
}
#endif

#endif
