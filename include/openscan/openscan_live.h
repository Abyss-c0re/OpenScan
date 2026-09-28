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
    int sweep_deg;   /* mold only: half-angle of the outline sweep, 60..140. 60 is 120° of the object */
    int relief;      /* mold only: 0 is a smooth solid, 100 is full shading */
    int solid;       /* 1 builds triangles, 0 keeps the points */
    int flip;        /* 1 turns both cameras 180°, matching the Fox mount */
} openscan_build;

/* Same starting point as the desktop sliders and Android.
 * The slider, the command line, and the engine all stay on 100..500. */
static inline int openscan_default_distance_mm(void) { return 220; }
static inline float openscan_distance_floor_mm(void) { return 100.f; }
static inline float openscan_distance_ceiling_mm(void) { return 500.f; }
static inline float openscan_distance_clamped(float mm) {
    if (mm < openscan_distance_floor_mm()) return openscan_distance_floor_mm();
    if (mm > openscan_distance_ceiling_mm()) return openscan_distance_ceiling_mm();
    return mm;
}

/* Shortest distance around a circle split into `bins` equal wedges. */
static inline int openscan_bin_delta(int a, int b, int bins) {
    int d = a - b;
    if (d < 0) d = -d;
    if (bins > 0 && d > bins / 2) d = bins - d;
    return d;
}

/* Mold commits only the silhouette in front of the camera, about ±15°.
 * The swept wings of one photo are not a scan of those other sides. */
static inline int openscan_mold_keeps_bin(int bin, int front, int bins) {
    int half;
    if (bin < 0 || front < 0 || bins <= 0) return 0;
    half = bins / 24;
    if (half < 1) half = 1;
    return openscan_bin_delta(bin, front, bins) <= half;
}

static inline openscan_build openscan_default_build(void) {
    openscan_build b;
    b.near_mm = 80.f;
    b.far_mm = 550.f;
    b.stride = 1;
    b.smooth = 5;
    b.sweep_deg = 80;
    b.relief = 40;
    b.solid = 1;
    b.flip = 1;
    return b;
}

/* No-flag recorded range for --min-mm and --max-mm. Same millimetres as the
 * default Near and Far. A saved Near or Far does not move it. Android records
 * the same range. The live model does not build a matcher from it. */
static inline double openscan_default_search_min_mm(void) {
    return (double)openscan_default_build().near_mm;
}
static inline double openscan_default_search_max_mm(void) {
    return (double)openscan_default_build().far_mm;
}

/* Snap breaks a triangle across a jump larger than this. Desktop and Android. */
static inline double openscan_default_edge_mm(void) { return 4.0; }

/* Widest Near and Far the sliders can select. Measured must not drop a
 * plane intersection inside this before those sliders run. */
static inline float openscan_near_floor_mm(void) { return 60.f; }
static inline float openscan_far_ceiling_mm(void) { return 800.f; }

/* Stripe target. Below the slider floor of 100 mm is raised to 100.
 * Above the far-slider ceiling is the projector safety stop, not a scan. */
static inline float openscan_stripe_distance_mm(float mm) {
    if (mm < openscan_distance_floor_mm()) return openscan_distance_floor_mm();
    if (mm > openscan_far_ceiling_mm()) return openscan_far_ceiling_mm();
    return mm;
}

/* Outline size. At or below 80 mm the distance was never set, so use 220.
 * A value between that and the slider floor is raised to 100, not used raw. */
static inline float openscan_outline_distance_mm(float mm) {
    if (!(mm > 80.f)) return (float)openscan_default_distance_mm();
    if (mm < openscan_distance_floor_mm()) return openscan_distance_floor_mm();
    return mm;
}

/* Pattern-camera depth a Measured scan is allowed to hand to Near and Far.
 * 90 mm is inside a Near of 80. 750 mm is inside a Far of 800.
 * Outside 60..800 the sliders cannot keep the point either. */
static inline int openscan_measured_hit_ok(float z_mm) {
    return z_mm >= openscan_near_floor_mm() && z_mm <= openscan_far_ceiling_mm();
}

/* Far stays at least 40 mm past Near, so the depth band is never empty.
 * Desktop and Android settings show this same result. */
static inline openscan_build openscan_build_clamped(const openscan_build *in) {
    openscan_build b = openscan_default_build();
    if (!in) return b;
    b = *in;
    if (b.near_mm < openscan_near_floor_mm()) b.near_mm = openscan_near_floor_mm();
    if (b.near_mm > 450.f) b.near_mm = 450.f;
    if (b.far_mm < b.near_mm + 40.f) b.far_mm = b.near_mm + 40.f;
    if (b.far_mm > openscan_far_ceiling_mm()) b.far_mm = openscan_far_ceiling_mm();
    if (b.stride < 1) b.stride = 1;
    if (b.stride > 6) b.stride = 6;
    if (b.smooth < 0) b.smooth = 0;
    if (b.smooth > 8) b.smooth = 8;
    if (b.sweep_deg < 60) b.sweep_deg = 60;
    if (b.sweep_deg > 140) b.sweep_deg = 140;
    if (b.relief < 0) b.relief = 0;
    if (b.relief > 100) b.relief = 100;
    b.solid = b.solid ? 1 : 0;
    b.flip = b.flip ? 1 : 0;
    return b;
}

/* Saved desktop keys. *_set is 0 when that key is absent.
 * Distance stays on 100..500. The build uses openscan_build_clamped.
 * Mold unless the saved shape is Measured. */
static inline int openscan_shape_resolve(int shape, int shape_set) {
    if (shape_set && shape == OPENSCAN_SHAPE_MEASURED) return OPENSCAN_SHAPE_MEASURED;
    return OPENSCAN_SHAPE_MOLD;
}

/* A command-line distance or shape wins over the saved value.
 * flag_set is 0 when that flag was omitted. Distance stays on 100..500.
 * Any shape other than Measured is Mold. */
static inline int openscan_distance_override(int resolved_mm, int flag_set, int flag_mm) {
    int d = flag_set ? flag_mm : resolved_mm;
    return (int)openscan_distance_clamped((float)d);
}

static inline int openscan_shape_override(int resolved_shape, int flag_set, int flag_shape) {
    if (!flag_set) return openscan_shape_resolve(resolved_shape, 1);
    return openscan_shape_resolve(flag_shape, 1);
}

/* Command-line knobs win over the saved build. *_set is 0 when that flag
 * was omitted. The result uses openscan_build_clamped, so Far stays 40 mm
 * past Near and each knob stays in the slider range. */
static inline void openscan_build_override(const openscan_build *resolved,
                                          int near_set, int near_mm,
                                          int far_set, int far_mm,
                                          int stride_set, int stride,
                                          int smooth_set, int smooth,
                                          int sweep_set, int sweep_deg,
                                          int relief_set, int relief,
                                          int solid_set, int solid,
                                          int flip_set, int flip,
                                          openscan_build *out) {
    openscan_build b = openscan_default_build();
    if (!out) return;
    if (resolved) b = *resolved;
    if (near_set) b.near_mm = (float)near_mm;
    if (far_set) b.far_mm = (float)far_mm;
    if (stride_set) b.stride = stride;
    if (smooth_set) b.smooth = smooth;
    if (sweep_set) b.sweep_deg = sweep_deg;
    if (relief_set) b.relief = relief;
    if (solid_set) b.solid = solid;
    if (flip_set) b.flip = flip;
    *out = openscan_build_clamped(&b);
}

static inline void openscan_build_resolve(int distance, int distance_set,
                                         int near_mm, int near_set,
                                         int far_mm, int far_set,
                                         int stride, int stride_set,
                                         int smooth, int smooth_set,
                                         int sweep_deg, int sweep_set,
                                         int relief, int relief_set,
                                         int solid, int solid_set,
                                         int flip, int flip_set,
                                         int *out_distance, openscan_build *out) {
    openscan_build b = openscan_default_build();
    int dist = openscan_default_distance_mm();
    if (!out_distance || !out) return;
    if (distance_set) dist = distance;
    if (near_set) b.near_mm = (float)near_mm;
    if (far_set) b.far_mm = (float)far_mm;
    if (stride_set) b.stride = stride;
    if (smooth_set) b.smooth = smooth;
    if (sweep_set) b.sweep_deg = sweep_deg;
    if (relief_set) b.relief = relief;
    if (solid_set) b.solid = solid;
    if (flip_set) b.flip = flip;
    *out_distance = (int)openscan_distance_clamped((float)dist);
    *out = openscan_build_clamped(&b);
}

/* Snap's keep range. A missing flag uses the saved Near or Far.
 * An explicit --max-mm is kept. A missing far is raised so it stays
 * at least 40 mm past near, the same rule as the sliders.
 * Scan records --min-mm and --max-mm separately and does not use this.
 * The live model does not build a matcher from that range. */
static inline void openscan_snap_depth(int min_set, double min_mm,
                                       int max_set, double max_mm,
                                       double near_mm, double far_mm,
                                       double *out_min, double *out_max) {
    double lo = min_set ? min_mm : near_mm;
    double hi = max_set ? max_mm : far_mm;
    if (!max_set && hi < lo + 40.0) hi = lo + 40.0;
    if (out_min) *out_min = lo;
    if (out_max) *out_max = hi;
}

/* Recorded --min-mm and --max-mm. Not the Near/Far keep band.
 * The near end stays at least 50 mm. A far end that is not past it is
 * moved 50 mm farther. The live scanner does not build a matcher from this. */
static inline void openscan_stereo_clamp(double *min_mm, double *max_mm) {
    if (!min_mm || !max_mm) return;
    if (*min_mm < 50.0) *min_mm = 50.0;
    if (*max_mm <= *min_mm) *max_mm = *min_mm + 50.0;
}

/* Pixel span for a disparity matcher that the live scanner does not build.
 * min_disparity is the near end (negative on this rig). num_disparities is a
 * multiple of 16 and stops at the far end instead of at disparity 0.
 * The near end is a little closer than min_mm, and the far end a little
 * farther than max_mm. The width stays inside 16..320. */
typedef struct openscan_stereo_search {
    int min_disparity;
    int num_disparities;
} openscan_stereo_search;

static inline openscan_stereo_search openscan_stereo_search_for(double fx, double baseline,
                                                                double min_mm, double max_mm) {
    openscan_stereo_search w;
    double near_z, far_z, px;
    int near_px, far_px;
    w.min_disparity = -64;
    w.num_disparities = 64;
    if (!(fx > 1.0) || !(baseline > 0.1)) return w;
    openscan_stereo_clamp(&min_mm, &max_mm);
    near_z = min_mm * 0.85;
    if (near_z < 40.0) near_z = 40.0;
    far_z = max_mm / 0.85;
    if (far_z < near_z + 1.0) far_z = near_z + 1.0;
    px = fx * baseline / near_z;
    near_px = px > 0.0 ? (int)(px + 0.5) : 0;
    near_px = (near_px + 15) & ~15;
    if (near_px < 64) near_px = 64;
    if (near_px > 320) near_px = 320;
    px = fx * baseline / far_z;
    far_px = px > 0.0 ? (int)(px + 0.5) : 0;
    far_px &= ~15;
    if (far_px < 0) far_px = 0;
    if (far_px > near_px - 16) far_px = near_px - 16;
    w.min_disparity = -near_px;
    w.num_disparities = near_px - far_px;
    return w;
}

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

/* Working distance in millimetres, clamped to 100..500 like the sliders.
 * In Mold this is the size of the model. In Measured it identifies the
 * projector stripes. Reset after changing it. */
void openscan_live_set_distance_mm(openscan_live *live, float mm);
float openscan_live_distance_mm(const openscan_live *live);
void openscan_live_set_build(openscan_live *live, const openscan_build *build);
void openscan_live_get_build(const openscan_live *live, openscan_build *build);
int openscan_live_points(const openscan_live *live);

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

/* Mold turn. Non-zero if a rotation is fused twice or not at all. */
int openscan_turn_self_test(void);

#ifdef __cplusplus
}
#endif

#endif
