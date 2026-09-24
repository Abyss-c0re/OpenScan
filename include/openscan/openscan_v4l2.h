#ifndef OPENSCAN_V4L2_H
#define OPENSCAN_V4L2_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fox (3DMakerpro, serial JMM8*) shows up as two Sonix UVC cameras.
 * The kernel uvcvideo driver owns the USB interface. This is the
 * userspace half: open both sensors, lock the scan format, and pull
 * luminance frames. */

typedef struct openscan_cam openscan_cam;

/* index-0 capture nodes whose names are "KYT Camera A/B: <serial>_A/B".
 * serial_out receives the shared serial (JMM8...) without the _A/_B suffix. */
int openscan_find_cameras(char *path_a, char *path_b, size_t path_n,
                     char *serial_out, size_t serial_n);

/* mjpeg nonzero selects Motion-JPEG. Both Fox sensors sit behind one USB 2.0
 * hub, and two 1280x720 YUYV streams overrun it. MJPEG at that size fits. */
openscan_cam *openscan_cam_open(const char *path, int width, int height, int fps, int mjpeg);
void openscan_cam_close(openscan_cam *cam);

/* exposure_100us is the UVC absolute exposure (units of 100 microseconds).
 * gain is the processing-unit gain, 0..100 on this sensor. */
int openscan_cam_set_exposure(openscan_cam *cam, int exposure_100us, int gain);

int openscan_cam_start(openscan_cam *cam);
int openscan_cam_stop(openscan_cam *cam);

/* Copy the next payload. YUYV frames are unpacked to a Y plane of width*height.
 * MJPEG frames are the raw JPEG. ts_us may be NULL. */
int openscan_cam_grab(openscan_cam *cam, uint8_t *dst, size_t dst_cap, size_t *out_n, uint64_t *ts_us);

/* Same as openscan_cam_grab, then discard any further frames already queued so the
 * caller sees the newest one. Use this when processing is slower than 10 fps. */
int openscan_cam_grab_latest(openscan_cam *cam, uint8_t *dst, size_t dst_cap, size_t *out_n, uint64_t *ts_us);
int openscan_cam_mjpeg(const openscan_cam *cam);

int openscan_cam_width(const openscan_cam *cam);
int openscan_cam_height(const openscan_cam *cam);
const char *openscan_cam_path(const openscan_cam *cam);

/* Sonix extension unit (unit 3, selector 1): ASIC register access.
 * Read is safe. Write is provided for experiments; it can change GPIO. */
int openscan_asic_read(openscan_cam *cam, unsigned addr, uint8_t *value);
int openscan_asic_write(openscan_cam *cam, unsigned addr, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
