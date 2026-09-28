#include "openscan/openscan_v4l2.h"

int openscan_find_cameras(char *path_a, char *path_b, size_t path_n, char *serial_out, size_t serial_n) {
    (void)path_n;
    (void)serial_n;
    if (path_a) path_a[0] = 0;
    if (path_b) path_b[0] = 0;
    if (serial_out) serial_out[0] = 0;
    return -1;
}

openscan_cam *openscan_cam_open(const char *path, int width, int height, int fps, int mjpeg) {
    (void)path;
    (void)width;
    (void)height;
    (void)fps;
    (void)mjpeg;
    return 0;
}

void openscan_cam_close(openscan_cam *cam) { (void)cam; }

int openscan_cam_set_exposure(openscan_cam *cam, int exposure_100us, int gain) {
    (void)cam;
    (void)exposure_100us;
    (void)gain;
    return -1;
}
int openscan_cam_start(openscan_cam *cam) {
    (void)cam;
    return -1;
}
int openscan_cam_stop(openscan_cam *cam) {
    (void)cam;
    return 0;
}
int openscan_cam_grab(openscan_cam *cam, uint8_t *dst, size_t dst_cap, size_t *out_n, uint64_t *ts_us) {
    (void)cam;
    (void)dst;
    (void)dst_cap;
    (void)out_n;
    (void)ts_us;
    return -1;
}
int openscan_cam_grab_latest(openscan_cam *cam, uint8_t *dst, size_t dst_cap, size_t *out_n, uint64_t *ts_us) {
    return openscan_cam_grab(cam, dst, dst_cap, out_n, ts_us);
}
int openscan_cam_width(const openscan_cam *cam) {
    (void)cam;
    return 0;
}
int openscan_cam_height(const openscan_cam *cam) {
    (void)cam;
    return 0;
}
int openscan_cam_mjpeg(const openscan_cam *cam) {
    (void)cam;
    return 0;
}
int openscan_asic_read(openscan_cam *cam, unsigned addr, uint8_t *value) {
    (void)cam;
    (void)addr;
    (void)value;
    return -1;
}
int openscan_asic_write(openscan_cam *cam, unsigned addr, uint8_t value) {
    (void)cam;
    (void)addr;
    (void)value;
    return -1;
}
