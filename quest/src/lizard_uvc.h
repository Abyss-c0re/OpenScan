#ifndef LIZARD_UVC_H
#define LIZARD_UVC_H

#include <stdint.h>

/* Fox cameras are USB Video Class, MJPEG at the calibration size.
 * The Quest camera service does not publish them. */
typedef struct Uvc Uvc;

int uvc_init(void *vm, void *activity);

/* max_payload is the largest isochronous microframe to commit, in bytes.
 * 0 lets the device choose. */
Uvc *uvc_open(uint16_t vid, uint16_t pid, int width, int height, int max_payload);
void uvc_close(Uvc *dev);

/* 0 when a new frame was decoded into y (and u/v when want_uv).
 * 1 when the previous frame is still current.
 * -1 when no frame has arrived. -2 when the stream died. */
int uvc_grab(Uvc *dev, uint8_t *y, uint8_t *u, uint8_t *v, int w, int h, int want_uv);
/* Exposure is in steps of 100 microseconds. Gain is the raw camera value, 0..100. */
int uvc_set_exposure_gain(Uvc *dev, int exposure_100us, int gain);
/* Stop and start camera A's stream so the sensor script latches exposure. */
void uvc_restream(Uvc *dev);
const char *uvc_status(Uvc *dev);

/* 0 ok, 1 absent, 2 permission, 3 negotiate, 4 bandwidth. */
int uvc_last_error(void);
const char *uvc_last_error_str(void);

#endif
