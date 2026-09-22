#ifndef FOX_CALIB_H
#define FOX_CALIB_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Factory calib.txt written by JMStudio for a Fox (JMM8) unit.
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

typedef struct fox_pinhole {
    double fx, fy, cx, cy;
    double k1, k2, p1, p2, k3;
    double rvec[3];
    double tvec[3]; /* millimetres, camera-2 and projector only */
} fox_pinhole;

/* A projector stripe. In the projector image it is the line
 * u = slope * v + offset. OneShot turns that line into a 3D plane. */
typedef struct fox_plane {
    int index;
    double slope;
    double offset;
} fox_plane;

#define FOX_MAX_PLANES 160

typedef struct fox_calib {
    int width, height;
    char serial[64];
    char date[64];
    fox_pinhole cam[3]; /* 0 pattern camera, 1 clean camera, 2 projector */
    int nplanes;
    fox_plane plane[FOX_MAX_PLANES];
} fox_calib;

int fox_calib_load(const char *path, fox_calib *out);

/* Download the factory calib.txt for this serial. The request is signed
 * from the serial alone; no account cookie is sent. Writes dest_path. */
int fox_calib_fetch(const char *serial, const char *dest_path);

/* Load a local file for this serial, or download one into
 * $XDG_DATA_HOME/fox3d/calib/<serial>.txt when none is on disk.
 * explicit_path, when set, is the only file tried and is never downloaded.
 * used_path may be NULL. */
int fox_calib_ensure(const char *serial, const char *explicit_path,
                     fox_calib *out, char *used_path, size_t used_n);

#ifdef __cplusplus
}
#endif

#endif
