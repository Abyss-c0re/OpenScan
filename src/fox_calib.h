#ifndef FOX_CALIB_H
#define FOX_CALIB_H

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

typedef struct fox_calib {
    int width, height;
    char serial[64];
    char date[64];
    fox_pinhole cam[3]; /* 0 left/reference, 1 right, 2 projector */
} fox_calib;

int fox_calib_load(const char *path, fox_calib *out);

#ifdef __cplusplus
}
#endif

#endif
