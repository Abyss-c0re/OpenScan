#include "openscan/openscan.h"
#include "image/os_jpeg.h"

#include <string.h>

const char *openscan_version(void) { return OPENSCAN_VERSION; }
const char *openscan_host(void) { return OPENSCAN_HOST; }

int openscan_frame_gray(openscan_cam *cam, uint8_t *storage, size_t storage_n, uint8_t *gray) {
    size_t n = 0;
    int w, h;
    if (!cam || !storage || !gray || storage_n < 16) return -1;
    w = openscan_cam_width(cam);
    h = openscan_cam_height(cam);
    if (w < 2 || h < 2) return -1;
    if (openscan_cam_grab_latest(cam, storage, storage_n, &n, NULL) != 0 || n < 16) return -1;
    if (!openscan_cam_mjpeg(cam)) {
        if (n < (size_t)w * (size_t)h) return -1;
        memcpy(gray, storage, (size_t)w * (size_t)h);
        return 0;
    }
    return os_jpeg_gray(storage, n, w, h, gray);
}
