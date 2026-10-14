#ifndef LIZARD_CAM_H
#define LIZARD_CAM_H

#include <stdint.h>

/* Fox pair on the Quest USB bus. Camera A (0c45:636a) is the lattice.
 * Camera B (0c45:636b) is colour. Both are UVC MJPEG at the calibration size. */

typedef struct LzCam LzCam;

void lz_cam_set_android(void *vm, void *activity);
LzCam *lz_cam_open(int width, int height);
void lz_cam_close(LzCam *cam);

/* Tight luminance for both cameras, and camera B chroma at half resolution.
 * ub/vb are (width/2)*(height/2). Returns 0 when a new pair is in hand,
 * 1 when both cameras are open but neither has a new frame, -1 when a frame
 * has not arrived yet, and -2 when the Fox pair is not open. */
int lz_cam_copy(LzCam *cam, uint8_t *ya, uint8_t *yb, uint8_t *ub, uint8_t *vb,
                int *seq);

int lz_cam_locked(const LzCam *cam);
int lz_cam_cols(const LzCam *cam);
/* Latest camera A meter. 0 until a frame has been scored. p95 may be null. */
int lz_cam_meter(const LzCam *cam, int *cols, int *clip, int *p50, int *p95);
/* Latest camera B meter (colour). 0 until a frame has been scored. */
int lz_cam_meter_b(const LzCam *cam, int *clip, int *p50, int *p95);
const char *lz_cam_status(const LzCam *cam);

/* User exposure and gain, the same 1..200 and 0..100 as the desktop sliders.
 * Exposure is in 100 µs units. This stops the automatic sweep from writing
 * over the sliders. Call it on the camera thread. It never restreams. */
void lz_cam_set_user(LzCam *cam, int exp_a, int gain_a, int exp_b, int gain_b);

#endif
