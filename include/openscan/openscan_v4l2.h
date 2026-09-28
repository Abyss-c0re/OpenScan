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

/* "KYT Camera A: <serial>_A" and the matching B name. Shared by every camera backend. */
int openscan_serial_from_camera_name(const char *name, char *out, size_t n);
int openscan_shared_camera_serial(const char *name_a, const char *name_b, char *out, size_t n);

/* index-0 capture nodes whose names are "KYT Camera A/B: <serial>_A/B".
 * serial_out receives the shared serial (JMM8...) without the _A/_B suffix.
 * Returns 0 when both names carry that same serial, -1 when the pair is
 * missing, and -2 when both cameras are present but the serials are missing,
 * unsafe, or different. A mixed pair is not opened. */
int openscan_find_cameras(char *path_a, char *path_b, size_t path_n,
                     char *serial_out, size_t serial_n);

/* mjpeg nonzero selects Motion-JPEG. Both Fox sensors sit behind one USB 2.0
 * hub, and two 1280x720 YUYV streams overrun it. MJPEG at that size fits. */
openscan_cam *openscan_cam_open(const char *path, int width, int height, int fps, int mjpeg);
void openscan_cam_close(openscan_cam *cam);

/* 0 when manual exposure and gain were written.
 * 1 when the camera stayed on auto, so the sliders were not applied.
 * -1 when exposure or gain could not be written. Streaming may continue on 1. */
static inline int openscan_exposure_result(int auto_present, int manual_now,
                                           int absolute_ok, int gain_ok) {
    if (auto_present && !manual_now) return 1;
    if (!absolute_ok || !gain_ok) return -1;
    return 0;
}

/* Same sentence as the Android idle status. */
static inline const char *openscan_exposure_auto_note(void) {
    return "Exposure stayed on auto. The sliders were not applied.";
}

/* 1 when either camera stayed on auto. A hard failure (-1) is not that case. */
static inline int openscan_exposure_auto_pair(int result_a, int result_b) {
    return result_a > 0 || result_b > 0;
}

/* 1..200, same as the desktop sliders and Android. 200 is 20 ms. */
static inline int openscan_exposure_clamp(int exposure) {
    if (exposure < 1) return 1;
    if (exposure > 200) return 200;
    return exposure;
}

/* 0..100, the value written to the camera. Same as the sliders and Android. */
static inline int openscan_gain_clamp(int gain) {
    if (gain < 0) return 0;
    if (gain > 100) return 100;
    return gain;
}

/* Starting exposure and gain. Same numbers as the desktop sliders and Android:
 * camera A 22 / 6, camera B 16 / 4. A negative flag means "not passed". */
static inline int openscan_default_exposure_a(void) { return 22; }
static inline int openscan_default_gain_a(void) { return 6; }
static inline int openscan_default_exposure_b(void) { return 16; }
static inline int openscan_default_gain_b(void) { return 4; }

/* flag, then a shared --exposure/--gain, then a saved desktop value, then fallback.
 * saved_set is 0 when that key is absent. The result is clamped to lo..hi. */
static inline int openscan_setting_pick(int flag, int shared, int saved, int saved_set,
                                        int fallback, int lo, int hi) {
    int v;
    if (flag >= 0) v = flag;
    else if (shared >= 0) v = shared;
    else if (saved_set) v = saved;
    else v = fallback;
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Headless grab, snap, and scan. A per-camera flag wins. --exposure / --gain
 * fill only a camera that was not named. Otherwise the desktop app's saved
 * value, and only then the defaults above. saved_*_set is 0 when that key
 * is absent. Does not use "one third of A" for camera B. */
static inline void openscan_exposure_resolve(int exposure, int gain,
                                            int saved_ea, int saved_ea_set,
                                            int saved_ga, int saved_ga_set,
                                            int saved_eb, int saved_eb_set,
                                            int saved_gb, int saved_gb_set,
                                            int *exp_a, int *gain_a,
                                            int *exp_b, int *gain_b) {
    if (!exp_a || !gain_a || !exp_b || !gain_b) return;
    *exp_a = openscan_setting_pick(*exp_a, exposure, saved_ea, saved_ea_set,
                                  openscan_default_exposure_a(), 1, 200);
    *exp_b = openscan_setting_pick(*exp_b, exposure, saved_eb, saved_eb_set,
                                  openscan_default_exposure_b(), 1, 200);
    *gain_a = openscan_setting_pick(*gain_a, gain, saved_ga, saved_ga_set,
                                  openscan_default_gain_a(), 0, 100);
    *gain_b = openscan_setting_pick(*gain_b, gain, saved_gb, saved_gb_set,
                                  openscan_default_gain_b(), 0, 100);
}

/* Gain to add when the shutter is already at 200 and the frame is still dark.
 * Same 1.35 ceiling as the shutter ratio, so one measurement cannot ask for
 * more than that much extra light if gain is linear. At least 2, at most 16.
 * Does not pull a higher setting down. */
static inline int openscan_iround(double x) {
    return (int)(x >= 0.0 ? x + 0.5 : x - 0.5);
}

static inline int openscan_ae_gain_up(int gain, double mean) {
    double floor_mean, ratio;
    int step;
    if (gain < 0) gain = 0;
    floor_mean = mean < 1.0 ? 1.0 : mean;
    ratio = 55.0 / floor_mean;
    if (ratio > 1.35) ratio = 1.35;
    if (ratio < 1.0) ratio = 1.0;
    step = openscan_iround((double)gain * (ratio - 1.0));
    if (step < 2) step = 2;
    if (step > 16) step = 16;
    return step;
}

/* Shutter multiplier toward mean 55. An ordinary bright frame does not
 * shrink faster than 0.72. A clipped frame, mean above 200, stays at 0.45
 * so that floor does not cancel the faster cut. */
static inline double openscan_ae_ratio(double mean) {
    double floor_mean = mean < 1.0 ? 1.0 : mean;
    double ratio = 55.0 / floor_mean;
    if (ratio < 0.72) ratio = 0.72;
    if (ratio > 1.35) ratio = 1.35;
    if (mean > 200.0) ratio = 0.45;
    return ratio;
}

/* Gain to remove when the shutter is already at 1 and the frame is still bright.
 * Uses openscan_ae_ratio, including 0.45 on a clipped frame. At least 2, at most 16. */
static inline int openscan_ae_gain_down(int gain, double mean) {
    double ratio;
    int step;
    if (gain < 0) gain = 0;
    ratio = openscan_ae_ratio(mean);
    if (ratio > 1.0) ratio = 1.0;
    step = openscan_iround((double)gain * (1.0 - ratio));
    if (step < 2) step = 2;
    if (step > 16) step = 16;
    return step;
}

/* Scan balancer target luminance, and the band where it stops. */
static inline int openscan_ae_settled(double mean) {
    double d = mean - 55.0;
    if (d < 0.0) d = -d;
    return d < 10.0;
}

/* One shutter/gain step toward mean 55.
 * The shutter moves by a clamped ratio of the miss. Gain moves only when
 * the shutter is already at a stop: down by openscan_ae_gain_down at 1 if
 * the frame is still bright, up by openscan_ae_gain_up at 200 if it is
 * still dark. Returns 1 when a value changed. */
static inline int openscan_ae_nudge(int *exp, int *gain, double mean) {
    double ratio;
    int before_e, before_g, next;
    if (!exp || !gain) return 0;
    if (openscan_ae_settled(mean)) return 0;
    ratio = openscan_ae_ratio(mean);
    next = openscan_iround((double)(*exp) * ratio);
    if (next == *exp) next += (mean < 55.0) ? 2 : -2;
    before_e = *exp;
    before_g = *gain;
    if (next < 1) {
        *exp = 1;
        if (mean > 55.0) *gain = openscan_gain_clamp(*gain - openscan_ae_gain_down(*gain, mean));
    } else if (openscan_exposure_clamp(next) < next && mean < 55.0) {
        *exp = openscan_exposure_clamp(next);
        *gain = openscan_gain_clamp(*gain + openscan_ae_gain_up(*gain, mean));
    } else {
        *exp = openscan_exposure_clamp(next);
    }
    return (*exp != before_e || *gain != before_g) ? 1 : 0;
}

/* Frames to read after a shutter or gain change.
 * grab_latest returns the newest completed frame and leaves the queue empty,
 * so that read can still be the frame that was already integrating. The next
 * read is the first one that started after the control. A third read only
 * repeats the new frame. */
static inline int openscan_ae_settle_frames(void) { return 2; }

/* How many nudge steps a fixed miss can take before both stops are hit.
 * Gain no longer moves by only 2 once the shutter is at a stop. The long
 * walk is the bright edge (mean 65) from shutter 176 / gain 100 down to
 * 1/0, 44 steps. A step of 2 on that gain used to take 75. Dark climbs
 * from the defaults fit inside this. The scan loop stops early when both
 * cameras stop moving. */
static inline int openscan_ae_step_limit(void) { return 44; }

/* One step of the scan-window exposure keys. The top stays at 200. */
static inline int openscan_exposure_step(int current, int direction) {
    int step = current / 6;
    if (step < 1) step = 1;
    if (direction < 0) return openscan_exposure_clamp(current - step);
    return openscan_exposure_clamp(current + step);
}

/* exposure_100us is the UVC absolute exposure (units of 100 microseconds).
 * gain is the processing-unit gain, 0..100 on this sensor.
 * Returns openscan_exposure_result(). */
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
