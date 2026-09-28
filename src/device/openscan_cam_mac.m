/* macOS UVC pair. AVFoundation names the two Fox cameras.
 * Frames are gray, so the window does not decode JPEG for this host. */
#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#include <os/lock.h>

#include "openscan/openscan_v4l2.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct openscan_cam {
    AVCaptureSession *session;
    AVCaptureDevice *device;
    id delegate;
    dispatch_queue_t queue;
    os_unfair_lock lock;
    uint8_t *gray;
    int width, height;
    int have;
    int mjpeg;
};

@interface OSCamGrab : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
@property(nonatomic, assign) openscan_cam *cam;
@end

@implementation OSCamGrab
- (void)captureOutput:(AVCaptureOutput *)output
    didOutputSampleBuffer:(CMSampleBufferRef)sample
           fromConnection:(AVCaptureConnection *)connection {
    (void)output;
    (void)connection;
    openscan_cam *cam = self.cam;
    if (!cam || !cam->gray) return;
    CVImageBufferRef img = CMSampleBufferGetImageBuffer(sample);
    if (!img) return;
    CVPixelBufferLockBaseAddress(img, kCVPixelBufferLock_ReadOnly);
    size_t sw = CVPixelBufferGetWidth(img);
    size_t sh = CVPixelBufferGetHeight(img);
    size_t stride = CVPixelBufferGetBytesPerRow(img);
    uint8_t *src = CVPixelBufferGetBaseAddress(img);
    if (src && sw > 0 && sh > 0) {
        os_unfair_lock_lock(&cam->lock);
        for (int y = 0; y < cam->height; y++) {
            size_t sy = (size_t)y * sh / (size_t)cam->height;
            if (sy >= sh) sy = sh - 1;
            uint8_t *row = src + sy * stride;
            for (int x = 0; x < cam->width; x++) {
                size_t sx = (size_t)x * sw / (size_t)cam->width;
                if (sx >= sw) sx = sw - 1;
                uint8_t *p = row + sx * 4;
                cam->gray[y * cam->width + x] = (uint8_t)((p[2] * 77 + p[1] * 150 + p[0] * 29) >> 8);
            }
        }
        cam->have = 1;
        os_unfair_lock_unlock(&cam->lock);
    }
    CVPixelBufferUnlockBaseAddress(img, kCVPixelBufferLock_ReadOnly);
}
@end

static const char *fox_side(const char *name, int *side) {
    const char *a = strstr(name, "KYT Camera A");
    const char *b = strstr(name, "KYT Camera B");
    if (a) {
        *side = 0;
        return a;
    }
    if (b) {
        *side = 1;
        return b;
    }
    return NULL;
}

int openscan_find_cameras(char *path_a, char *path_b, size_t path_n, char *serial_out, size_t serial_n) {
    if (!path_a || !path_b || path_n < 8) return -1;
    path_a[0] = path_b[0] = 0;
    if (serial_out && serial_n) serial_out[0] = 0;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    NSArray<AVCaptureDevice *> *devs = [AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo];
#pragma clang diagnostic pop
    char name_a[256], name_b[256];
    name_a[0] = name_b[0] = 0;
    for (AVCaptureDevice *dev in devs) {
        const char *name = dev.localizedName.UTF8String;
        const char *uid = dev.uniqueID.UTF8String;
        int side = -1;
        if (!name || !uid || !fox_side(name, &side)) continue;
        if (side == 0) {
            snprintf(path_a, path_n, "%s", uid);
            snprintf(name_a, sizeof name_a, "%s", name);
        } else {
            snprintf(path_b, path_n, "%s", uid);
            snprintf(name_b, sizeof name_b, "%s", name);
        }
    }
    if (!path_a[0] || !path_b[0]) return -1;
    if (strchr(name_a, ':') && strchr(name_b, ':')) {
        char sn[64];
        if (openscan_shared_camera_serial(name_a, name_b, sn, sizeof sn) != 0) return -2;
        if (serial_out && serial_n) snprintf(serial_out, serial_n, "%s", sn);
    }
    return 0;
}

openscan_cam *openscan_cam_open(const char *path, int width, int height, int fps, int mjpeg) {
    (void)fps;
    (void)mjpeg;
    if (!path || width < 16 || height < 16) return NULL;
    NSString *uid = [NSString stringWithUTF8String:path];
    AVCaptureDevice *dev = [AVCaptureDevice deviceWithUniqueID:uid];
    if (!dev) return NULL;
    openscan_cam *cam = calloc(1, sizeof *cam);
    if (!cam) return NULL;
    cam->width = width;
    cam->height = height;
    cam->mjpeg = 0;
    cam->lock = OS_UNFAIR_LOCK_INIT;
    cam->gray = calloc((size_t)width * (size_t)height, 1);
    cam->device = dev;
    [cam->device retain];
    cam->session = [[AVCaptureSession alloc] init];
    cam->queue = dispatch_queue_create("openscan.cam", DISPATCH_QUEUE_SERIAL);
    NSError *err = nil;
    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:dev error:&err];
    if (!cam->gray || !input || ![cam->session canAddInput:input]) {
        openscan_cam_close(cam);
        return NULL;
    }
    [cam->session addInput:input];
    if ([cam->session canSetSessionPreset:AVCaptureSessionPreset1280x720])
        cam->session.sessionPreset = AVCaptureSessionPreset1280x720;
    AVCaptureVideoDataOutput *output = [[AVCaptureVideoDataOutput alloc] init];
    output.videoSettings = @{
        (id)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA)
    };
    output.alwaysDiscardsLateVideoFrames = YES;
    OSCamGrab *grab = [OSCamGrab new];
    grab.cam = cam;
    cam->delegate = grab;
    [output setSampleBufferDelegate:grab queue:cam->queue];
    [grab release];
    if (![cam->session canAddOutput:output]) {
        openscan_cam_close(cam);
        return NULL;
    }
    [cam->session addOutput:output];
    return cam;
}

void openscan_cam_close(openscan_cam *cam) {
    if (!cam) return;
    if (cam->session.running) [cam->session stopRunning];
    [cam->session release];
    [cam->device release];
    if (cam->queue) dispatch_release(cam->queue);
    cam->session = nil;
    cam->device = nil;
    cam->delegate = nil;
    cam->queue = nil;
    free(cam->gray);
    free(cam);
}

int openscan_cam_set_exposure(openscan_cam *cam, int exposure_100us, int gain) {
    if (!cam || !cam->device) return -1;
    AVCaptureDevice *dev = cam->device;
    NSError *err = nil;
    if (![dev lockForConfiguration:&err]) return -1;
    if (exposure_100us < 1) exposure_100us = 1;
    CMTime dur = CMTimeMake(exposure_100us, 10000);
    if (CMTIME_COMPARE_INLINE(dur, <, dev.activeFormat.minExposureDuration))
        dur = dev.activeFormat.minExposureDuration;
    if (CMTIME_COMPARE_INLINE(dur, >, dev.activeFormat.maxExposureDuration))
        dur = dev.activeFormat.maxExposureDuration;
    float t = gain / 100.f;
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;
    float iso = dev.activeFormat.minISO + (dev.activeFormat.maxISO - dev.activeFormat.minISO) * t;
    if ([dev isExposureModeSupported:AVCaptureExposureModeCustom]) {
        [dev setExposureModeCustomWithDuration:dur ISO:iso completionHandler:nil];
        [dev unlockForConfiguration];
        return 0;
    }
    [dev unlockForConfiguration];
    return 1;
}

int openscan_cam_start(openscan_cam *cam) {
    if (!cam || !cam->session) return -1;
    [cam->session startRunning];
    return cam->session.running ? 0 : -1;
}

int openscan_cam_stop(openscan_cam *cam) {
    if (cam && cam->session.running) [cam->session stopRunning];
    return 0;
}

int openscan_cam_grab(openscan_cam *cam, uint8_t *dst, size_t dst_cap, size_t *out_n, uint64_t *ts_us) {
    if (ts_us) *ts_us = 0;
    if (!cam || !dst || !cam->gray) return -1;
    size_t need = (size_t)cam->width * (size_t)cam->height;
    if (need > dst_cap) return -1;
    os_unfair_lock_lock(&cam->lock);
    int have = cam->have;
    if (have) memcpy(dst, cam->gray, need);
    os_unfair_lock_unlock(&cam->lock);
    if (!have) return -1;
    if (out_n) *out_n = need;
    return 0;
}

int openscan_cam_grab_latest(openscan_cam *cam, uint8_t *dst, size_t dst_cap, size_t *out_n, uint64_t *ts_us) {
    return openscan_cam_grab(cam, dst, dst_cap, out_n, ts_us);
}

int openscan_cam_width(const openscan_cam *cam) { return cam ? cam->width : 0; }
int openscan_cam_height(const openscan_cam *cam) { return cam ? cam->height : 0; }
int openscan_cam_mjpeg(const openscan_cam *cam) { return cam ? cam->mjpeg : 0; }

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
