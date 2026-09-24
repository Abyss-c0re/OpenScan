#include "openscan/openscan_v4l2.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

#define OPENSCAN_BUFFERS 4

struct openscan_buf {
    void *start;
    size_t length;
};

struct openscan_cam {
    int fd;
    char path[256];
    int width;
    int height;
    int mjpeg;
    int bytesperline;
    int streaming;
    struct openscan_buf bufs[OPENSCAN_BUFFERS];
    unsigned nbufs;
};

static int xioctl(int fd, unsigned long req, void *arg) {
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r < 0 && errno == EINTR);
    return r;
}

static void trim_nl(char *s) {
    size_t n = strlen(s);
    while (n && (s[n - 1] == '\n' || s[n - 1] == '\r')) s[--n] = 0;
}

int openscan_find_cameras(char *path_a, char *path_b, size_t path_n,
                     char *serial_out, size_t serial_n) {
    DIR *d = opendir("/sys/class/video4linux");
    if (!d) return -1;
    path_a[0] = path_b[0] = 0;
    if (serial_out && serial_n) serial_out[0] = 0;

    struct dirent *de;
    while ((de = readdir(d))) {
        if (strncmp(de->d_name, "video", 5) != 0) continue;
        char ipath[512], npath[512], name[256], index_s[32];
        snprintf(ipath, sizeof ipath, "/sys/class/video4linux/%s/index", de->d_name);
        snprintf(npath, sizeof npath, "/sys/class/video4linux/%s/name", de->d_name);
        FILE *f = fopen(ipath, "r");
        if (!f) continue;
        if (!fgets(index_s, sizeof index_s, f)) { fclose(f); continue; }
        fclose(f);
        if (atoi(index_s) != 0) continue;
        f = fopen(npath, "r");
        if (!f) continue;
        if (!fgets(name, sizeof name, f)) { fclose(f); continue; }
        fclose(f);
        trim_nl(name);

        const char *tag = NULL;
        char *dst = NULL;
        if (strncmp(name, "KYT Camera A:", 13) == 0) {
            tag = "KYT Camera A:";
            dst = path_a;
        } else if (strncmp(name, "KYT Camera B:", 13) == 0) {
            tag = "KYT Camera B:";
            dst = path_b;
        } else {
            continue;
        }
        snprintf(dst, path_n, "/dev/%s", de->d_name);
        if (serial_out && serial_n && !serial_out[0]) {
            const char *sn = name + strlen(tag);
            while (*sn == ' ') sn++;
            snprintf(serial_out, serial_n, "%s", sn);
            char *us = strrchr(serial_out, '_');
            if (us && (us[1] == 'A' || us[1] == 'B') && us[2] == 0) *us = 0;
        }
        (void)tag;
    }
    closedir(d);
    if (!path_a[0] || !path_b[0]) return -1;
    return 0;
}

openscan_cam *openscan_cam_open(const char *path, int width, int height, int fps, int mjpeg) {
    openscan_cam *cam = calloc(1, sizeof *cam);
    if (!cam) return NULL;
    snprintf(cam->path, sizeof cam->path, "%s", path);
    cam->fd = open(path, O_RDWR | O_NONBLOCK);
    if (cam->fd < 0) {
        fprintf(stderr, "open %s: %s\n", path, strerror(errno));
        free(cam);
        return NULL;
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    uint32_t want = mjpeg ? V4L2_PIX_FMT_MJPEG : V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.pixelformat = want;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(cam->fd, VIDIOC_S_FMT, &fmt) < 0) {
        fprintf(stderr, "%s: set format %dx%d: %s\n", path, width, height, strerror(errno));
        openscan_cam_close(cam);
        return NULL;
    }
    if (fmt.fmt.pix.pixelformat != want ||
        (int)fmt.fmt.pix.width != width || (int)fmt.fmt.pix.height != height) {
        fprintf(stderr, "%s: camera offered %c%c%c%c %ux%u\n",
                path,
                fmt.fmt.pix.pixelformat & 255,
                (fmt.fmt.pix.pixelformat >> 8) & 255,
                (fmt.fmt.pix.pixelformat >> 16) & 255,
                (fmt.fmt.pix.pixelformat >> 24) & 255,
                fmt.fmt.pix.width, fmt.fmt.pix.height);
        openscan_cam_close(cam);
        return NULL;
    }
    cam->width = (int)fmt.fmt.pix.width;
    cam->height = (int)fmt.fmt.pix.height;
    cam->bytesperline = (int)fmt.fmt.pix.bytesperline;
    cam->mjpeg = mjpeg;

    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof parm);
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    if (xioctl(cam->fd, VIDIOC_S_PARM, &parm) < 0)
        fprintf(stderr, "%s: set %d fps: %s\n", path, fps, strerror(errno));

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof req);
    req.count = OPENSCAN_BUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(cam->fd, VIDIOC_REQBUFS, &req) < 0 || req.count < 2) {
        fprintf(stderr, "%s: reqbufs: %s\n", path, strerror(errno));
        openscan_cam_close(cam);
        return NULL;
    }
    cam->nbufs = req.count;
    for (unsigned i = 0; i < cam->nbufs; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(cam->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            fprintf(stderr, "%s: querybuf: %s\n", path, strerror(errno));
            openscan_cam_close(cam);
            return NULL;
        }
        cam->bufs[i].length = buf.length;
        cam->bufs[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, cam->fd, buf.m.offset);
        if (cam->bufs[i].start == MAP_FAILED) {
            fprintf(stderr, "%s: mmap: %s\n", path, strerror(errno));
            cam->bufs[i].start = NULL;
            openscan_cam_close(cam);
            return NULL;
        }
    }
    return cam;
}

void openscan_cam_close(openscan_cam *cam) {
    if (!cam) return;
    if (cam->streaming) openscan_cam_stop(cam);
    for (unsigned i = 0; i < cam->nbufs; i++) {
        if (cam->bufs[i].start && cam->bufs[i].start != MAP_FAILED)
            munmap(cam->bufs[i].start, cam->bufs[i].length);
    }
    if (cam->fd >= 0) close(cam->fd);
    free(cam);
}

static int set_ctrl(int fd, uint32_t id, int value) {
    struct v4l2_control c = { .id = id, .value = value };
    return xioctl(fd, VIDIOC_S_CTRL, &c);
}

int openscan_cam_set_exposure(openscan_cam *cam, int exposure_100us, int gain) {
    /* Manual mode is menu value 1 on this sensor (V4L2_EXPOSURE_MANUAL). */
    if (set_ctrl(cam->fd, V4L2_CID_EXPOSURE_AUTO, 1) < 0)
        fprintf(stderr, "%s: manual exposure: %s\n", cam->path, strerror(errno));
    if (set_ctrl(cam->fd, V4L2_CID_EXPOSURE_ABSOLUTE, exposure_100us) < 0) {
        fprintf(stderr, "%s: exposure %d: %s\n", cam->path, exposure_100us, strerror(errno));
        return -1;
    }
    if (set_ctrl(cam->fd, V4L2_CID_GAIN, gain) < 0) {
        fprintf(stderr, "%s: gain %d: %s\n", cam->path, gain, strerror(errno));
        return -1;
    }
    return 0;
}

int openscan_cam_start(openscan_cam *cam) {
    if (cam->streaming) return 0;
    for (unsigned i = 0; i < cam->nbufs; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof buf);
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (xioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "%s: qbuf: %s\n", cam->path, strerror(errno));
            return -1;
        }
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(cam->fd, VIDIOC_STREAMON, &type) < 0) {
        fprintf(stderr, "%s: streamon: %s\n", cam->path, strerror(errno));
        return -1;
    }
    cam->streaming = 1;
    return 0;
}

int openscan_cam_stop(openscan_cam *cam) {
    if (!cam->streaming) return 0;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(cam->fd, VIDIOC_STREAMOFF, &type);
    cam->streaming = 0;
    return 0;
}

static int copy_payload(openscan_cam *cam, const struct v4l2_buffer *buf,
                        uint8_t *dst, size_t dst_cap, size_t *out_n, uint64_t *ts_us) {
    if (buf->index >= cam->nbufs) return -1;
    const uint8_t *src = cam->bufs[buf->index].start;
    size_t n = 0;
    if (cam->mjpeg) {
        n = buf->bytesused;
        if (n > dst_cap) n = dst_cap;
        memcpy(dst, src, n);
    } else {
        int w = cam->width, h = cam->height;
        int stride = cam->bytesperline > 0 ? cam->bytesperline : w * 2;
        n = (size_t)w * h;
        if (n > dst_cap) {
            fprintf(stderr, "%s: Y plane does not fit\n", cam->path);
            return -1;
        }
        for (int row = 0; row < h; row++) {
            const uint8_t *line = src + (size_t)row * stride;
            for (int col = 0; col < w; col++) dst[row * w + col] = line[col * 2];
        }
    }
    if (out_n) *out_n = n;
    if (ts_us)
        *ts_us = (uint64_t)buf->timestamp.tv_sec * 1000000ull + (uint64_t)buf->timestamp.tv_usec;
    return 0;
}

static int dq_one(openscan_cam *cam, struct v4l2_buffer *buf, int block) {
    memset(buf, 0, sizeof *buf);
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;
    for (;;) {
        if (xioctl(cam->fd, VIDIOC_DQBUF, buf) == 0) return 0;
        if (errno == EAGAIN) {
            if (!block) return -1;
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(cam->fd, &fds);
            struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
            int r = select(cam->fd + 1, &fds, NULL, NULL, &tv);
            if (r == 0) {
                fprintf(stderr, "%s: frame timeout\n", cam->path);
                return -1;
            }
            if (r < 0 && errno != EINTR) {
                fprintf(stderr, "%s: select: %s\n", cam->path, strerror(errno));
                return -1;
            }
            continue;
        }
        fprintf(stderr, "%s: dqbuf: %s\n", cam->path, strerror(errno));
        return -1;
    }
}

int openscan_cam_grab(openscan_cam *cam, uint8_t *dst, size_t dst_cap, size_t *out_n, uint64_t *ts_us) {
    struct v4l2_buffer buf;
    if (dq_one(cam, &buf, 1) < 0) return -1;
    int rc = copy_payload(cam, &buf, dst, dst_cap, out_n, ts_us);
    if (xioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
        fprintf(stderr, "%s: requeue: %s\n", cam->path, strerror(errno));
        return -1;
    }
    return rc;
}

int openscan_cam_grab_latest(openscan_cam *cam, uint8_t *dst, size_t dst_cap, size_t *out_n, uint64_t *ts_us) {
    if (openscan_cam_grab(cam, dst, dst_cap, out_n, ts_us) < 0) return -1;
    for (;;) {
        struct v4l2_buffer buf;
        if (dq_one(cam, &buf, 0) < 0) {
            if (errno == EAGAIN) return 0;
            return -1;
        }
        int rc = copy_payload(cam, &buf, dst, dst_cap, out_n, ts_us);
        if (xioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
            fprintf(stderr, "%s: requeue: %s\n", cam->path, strerror(errno));
            return -1;
        }
        if (rc < 0) return -1;
    }
}

int openscan_cam_width(const openscan_cam *cam) { return cam->width; }
int openscan_cam_height(const openscan_cam *cam) { return cam->height; }
int openscan_cam_mjpeg(const openscan_cam *cam) { return cam->mjpeg; }
const char *openscan_cam_path(const openscan_cam *cam) { return cam->path; }

static int xu_query(int fd, uint8_t sel, uint8_t query, uint16_t size, uint8_t *data) {
    struct uvc_xu_control_query q = {
        .unit = 3,
        .selector = sel,
        .query = query,
        .size = size,
        .data = data,
    };
    if (xioctl(fd, UVCIOC_CTRL_QUERY, &q) < 0) return -1;
    return 0;
}

int openscan_asic_read(openscan_cam *cam, unsigned addr, uint8_t *value) {
    uint8_t b[4] = { (uint8_t)(addr & 0xff), (uint8_t)((addr >> 8) & 0xff), 0x00, 0xff };
    if (xu_query(cam->fd, 1, UVC_SET_CUR, 4, b) < 0) return -1;
    memset(b, 0, sizeof b);
    if (xu_query(cam->fd, 1, UVC_GET_CUR, 4, b) < 0) return -1;
    *value = b[2];
    return 0;
}

int openscan_asic_write(openscan_cam *cam, unsigned addr, uint8_t value) {
    uint8_t b[4] = { (uint8_t)(addr & 0xff), (uint8_t)((addr >> 8) & 0xff), value, 0x00 };
    return xu_query(cam->fd, 1, UVC_SET_CUR, 4, b);
}
