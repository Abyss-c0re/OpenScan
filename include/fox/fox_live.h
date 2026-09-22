#ifndef FOX_LIVE_H
#define FOX_LIVE_H

#include "fox/fox_calib.h"
#include "fox/fox_scan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Live scan of one object. The scanner may move around it, closer, or
 * above. The object cloud is the only thing tracked; a view that does not
 * match that object is dropped. */

typedef struct fox_live fox_live;

#define FOX_MODE_STOP  0
#define FOX_MODE_SCAN  1
#define FOX_MODE_PAUSE 2

typedef struct fox_live_status {
    int fused;          /* frames integrated into the model */
    int lost;           /* frames where tracking did not lock */
    int valid_pixels;   /* depth samples inside the working range, this frame */
    double median_mm;   /* median depth of those samples, 0 if none */
    int tracking;       /* 1 fused, 0 lost, -1 waiting for a surface */
    double match_ms;
    int mode;           /* FOX_MODE_STOP, SCAN, or PAUSE */
    int points;         /* triangles in the 3D model */
    int scanned_deg;    /* degrees of the object that have a surface */
    int detail;         /* average observations on the scanned surface */
} fox_live_status;

fox_live *fox_live_create(const fox_calib *calib, const fox_scan_opts *opt);
void fox_live_destroy(fox_live *live);

/* ya/yb are full-resolution luminance, calib->width by calib->height. */
int fox_live_push(fox_live *live, const uint8_t *ya, const uint8_t *yb,
                  int width, int height, fox_live_status *status);

/* BGR preview, valid until the next push. Returns NULL before the first push. */
const uint8_t *fox_live_preview_bgr(const fox_live *live, int *width, int *height);

void fox_live_reset(fox_live *live);

/* Headless scans start in FOX_MODE_SCAN. A window starts in FOX_MODE_STOP
 * until the user presses Start. */
void fox_live_set_mode(fox_live *live, int mode);
int fox_live_mode(const fox_live *live);

/* Assumed distance to the object, millimetres. This sets the size of the
 * model. Reset the scan after changing it. */
void fox_live_set_distance_mm(fox_live *live, float mm);
float fox_live_distance_mm(const fox_live *live);
int fox_live_points(const fox_live *live);

/* Mouse events use the OpenCV highgui event and flag values. */
void fox_live_mouse(fox_live *live, int event, int x, int y, int flags);
/* S saves, Space toggles scan/pause, Q or Esc closes. */
void fox_live_key(fox_live *live, int key);

/* Save is requested by the Save button or the S key. The flag clears on read. */
int fox_live_take_save(fox_live *live);
/* Close button, Q, or Esc. The window must exit and stay closed. */
int fox_live_quit(const fox_live *live);

/* Marching-cubes the fused model and write a binary STL in millimetres.
 * Camera frame of the first fused view: X right, Y down, Z forward. */
int fox_live_write(fox_live *live, const char *stl_path, int *triangles_out);

/* Analytic sphere. Returns 0 when the mesher is producing a closed shell. */
int fox_mesh_self_test(void);

#ifdef __cplusplus
}
#endif

#endif
