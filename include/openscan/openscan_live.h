#ifndef OPENSCAN_LIVE_H
#define OPENSCAN_LIVE_H

#include "openscan/openscan_calib.h"
#include "openscan/openscan_scan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Live scan of one object. The scanner may move around it, closer, or
 * above. The object cloud is the only thing tracked; a view that does not
 * match that object is dropped. */

typedef struct openscan_live openscan_live;

#define OPENSCAN_MODE_STOP  0
#define OPENSCAN_MODE_SCAN  1
#define OPENSCAN_MODE_PAUSE 2

/* Mold is the solid body swept from the camera outline. Measured is the
 * projector-stripe surface of the face in front of the camera. */
#define OPENSCAN_SHAPE_MOLD     0
#define OPENSCAN_SHAPE_MEASURED 1

/* Knobs that change the mesh, the same way Kinect near/far/stride change
 * the point cloud. They are read on every frame. */
typedef struct openscan_build {
    float near_mm;   /* drop geometry closer than this */
    float far_mm;    /* drop geometry farther than this */
    int stride;      /* 1..6, sample step. 1 is the full grid */
    int smooth;      /* 0..8, depth blur before the mesh is built */
    int sweep_deg;   /* mold only: how far the outline is swept, 20..140 */
    int relief;      /* mold only: 0 is a smooth solid, 100 is full shading */
    int solid;       /* 1 builds triangles, 0 keeps the points */
    int flip;        /* 1 turns both cameras 180°, matching the Fox mount */
} openscan_build;

typedef struct openscan_live_status {
    int fused;          /* frames integrated into the model */
    int lost;           /* frames where tracking did not lock */
    int valid_pixels;   /* depth samples inside the working range, this frame */
    double median_mm;   /* median depth of those samples, 0 if none */
    int tracking;       /* 1 fused, 0 lost, -1 waiting for a surface */
    double match_ms;
    int mode;           /* OPENSCAN_MODE_STOP, SCAN, or PAUSE */
    int points;         /* triangles in the 3D model */
    int scanned_deg;    /* degrees of the object that have a surface */
    int detail;         /* average observations on the scanned surface */
} openscan_live_status;

openscan_live *openscan_live_create(const openscan_calib *calib, const openscan_scan_opts *opt);
void openscan_live_destroy(openscan_live *live);

/* ya/yb are full-resolution luminance, calib->width by calib->height. */
int openscan_live_push(openscan_live *live, const uint8_t *ya, const uint8_t *yb,
                  int width, int height, openscan_live_status *status);

/* BGR preview, valid until the next push. Returns NULL before the first push. */
const uint8_t *openscan_live_preview_bgr(const openscan_live *live, int *width, int *height);

/* Separate pictures for a window that lays itself out. 0 camera, 1 model,
 * 2 pattern camera, 3 clean camera, 4 side. Valid until the next push. */
const uint8_t *openscan_live_panel_bgr(const openscan_live *live, int panel, int *width, int *height);

void openscan_live_reset(openscan_live *live);

/* Headless scans start in OPENSCAN_MODE_SCAN. A window starts in OPENSCAN_MODE_STOP
 * until the user presses Start. */
void openscan_live_set_mode(openscan_live *live, int mode);
int openscan_live_mode(const openscan_live *live);

/* OPENSCAN_SHAPE_MOLD or OPENSCAN_SHAPE_MEASURED. Switching clears the current model. */
void openscan_live_set_shape(openscan_live *live, int shape);
int openscan_live_shape(const openscan_live *live);

/* Working distance in millimetres. In Mold this is the size of the model.
 * In Measured it identifies the projector stripes. Reset after changing it. */
void openscan_live_set_distance_mm(openscan_live *live, float mm);
float openscan_live_distance_mm(const openscan_live *live);
void openscan_live_set_build(openscan_live *live, const openscan_build *build);
void openscan_live_get_build(const openscan_live *live, openscan_build *build);
int openscan_live_points(const openscan_live *live);

/* Mouse events use the OpenCV highgui event and flag values. */
void openscan_live_mouse(openscan_live *live, int event, int x, int y, int flags);
/* S saves, Space toggles scan/pause, Q or Esc closes. */
void openscan_live_key(openscan_live *live, int key);

/* Save is requested by the Save button or the S key. The flag clears on read. */
int openscan_live_take_save(openscan_live *live);
/* Close button, Q, or Esc. The window must exit and stay closed. */
int openscan_live_quit(const openscan_live *live);

/* Marching-cubes the fused model and write a binary STL in millimetres.
 * Camera frame of the first fused view: X right, Y down, Z forward. */
int openscan_live_write(openscan_live *live, const char *stl_path, int *triangles_out);

/* Analytic sphere. Returns 0 when the mesher is producing a closed shell. */
int openscan_mesh_self_test(void);

#ifdef __cplusplus
}
#endif

#endif
