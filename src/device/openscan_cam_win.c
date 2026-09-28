/* Windows UVC pair. Media Foundation names the two Fox cameras.
 * Exposure units differ from the Linux driver, so the shutter is left as the
 * device set it. The frame is MJPEG when the camera offers it. */
#include "openscan/openscan_v4l2.h"

#define COBJMACROS
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct openscan_cam {
    IMFMediaSource *source;
    IMFSourceReader *reader;
    int width, height;
    int mjpeg;
    int fmt; /* 0 jpeg, 1 yuy2, 2 nv12 */
    int started;
};

static int mf_refs;

static int mf_add(void) {
    if (mf_refs++ > 0) return 0;
    HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
    if (FAILED(hr) && hr != RPC_E_CHANGED_MODE) {
        mf_refs = 0;
        return -1;
    }
    if (FAILED(MFStartup(MF_VERSION, MFSTARTUP_NOSOCKET))) {
        mf_refs = 0;
        return -1;
    }
    return 0;
}

static void mf_sub(void) {
    if (mf_refs <= 0) return;
    if (--mf_refs == 0) {
        MFShutdown();
        CoUninitialize();
    }
}

static void wide_to_utf8(const wchar_t *w, char *out, size_t n) {
    if (!out || n == 0) return;
    out[0] = 0;
    if (!w) return;
    WideCharToMultiByte(CP_UTF8, 0, w, -1, out, (int)n, NULL, NULL);
}

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
    if (mf_add() != 0) return -1;
    IMFAttributes *attr = NULL;
    IMFActivate **acts = NULL;
    UINT32 count = 0;
    int rc = -1;
    char name_a[256], name_b[256];
    name_a[0] = name_b[0] = 0;
    if (FAILED(MFCreateAttributes(&attr, 1))) goto done;
    if (FAILED(IMFAttributes_SetGUID(attr, &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                                    &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID)))
        goto done;
    if (FAILED(MFEnumDeviceSources(attr, &acts, &count))) goto done;
    for (UINT32 i = 0; i < count; i++) {
        WCHAR *wn = NULL, *wl = NULL;
        UINT32 nn = 0, nl = 0;
        char name[256], link[1024];
        int side = -1;
        if (SUCCEEDED(IMFActivate_GetAllocatedString(acts[i], &MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME, &wn, &nn)))
            wide_to_utf8(wn, name, sizeof name);
        else
            name[0] = 0;
        if (SUCCEEDED(IMFActivate_GetAllocatedString(
                acts[i], &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &wl, &nl)))
            wide_to_utf8(wl, link, sizeof link);
        else
            link[0] = 0;
        CoTaskMemFree(wn);
        CoTaskMemFree(wl);
        if (!fox_side(name, &side) || !link[0]) continue;
        if (side == 0) {
            snprintf(path_a, path_n, "%s", link);
            snprintf(name_a, sizeof name_a, "%s", name);
        } else {
            snprintf(path_b, path_n, "%s", link);
            snprintf(name_b, sizeof name_b, "%s", name);
        }
    }
    if (path_a[0] && path_b[0]) {
        char sn[64];
        int named = strchr(name_a, ':') && strchr(name_b, ':');
        if (!named) {
            rc = 0;
        } else if (openscan_shared_camera_serial(name_a, name_b, sn, sizeof sn) == 0) {
            if (serial_out && serial_n) snprintf(serial_out, serial_n, "%s", sn);
            rc = 0;
        } else {
            rc = -2;
        }
    }
done:
    if (acts) {
        for (UINT32 i = 0; i < count; i++)
            if (acts[i]) IMFActivate_Release(acts[i]);
        CoTaskMemFree(acts);
    }
    if (attr) IMFAttributes_Release(attr);
    mf_sub();
    return rc;
}

static int utf8_to_wide(const char *s, wchar_t *out, int n) {
    if (!s || !out || n < 2) return -1;
    int wr = MultiByteToWideChar(CP_UTF8, 0, s, -1, out, n);
    return wr > 0 ? 0 : -1;
}

openscan_cam *openscan_cam_open(const char *path, int width, int height, int fps, int mjpeg) {
    (void)fps;
    if (!path || width < 16 || height < 16) return NULL;
    if (mf_add() != 0) return NULL;
    openscan_cam *cam = calloc(1, sizeof *cam);
    wchar_t link[1024];
    IMFAttributes *attr = NULL;
    IMFMediaType *mt = NULL;
    if (!cam) {
        mf_sub();
        return NULL;
    }
    if (utf8_to_wide(path, link, 1024) != 0) goto fail;
    cam->width = width;
    cam->height = height;
    cam->mjpeg = mjpeg ? 1 : 0;
    if (FAILED(MFCreateAttributes(&attr, 2))) goto fail;
    if (FAILED(IMFAttributes_SetGUID(attr, &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
                                    &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID)))
        goto fail;
    if (FAILED(IMFAttributes_SetString(attr, &MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, link)))
        goto fail;
    if (FAILED(MFCreateDeviceSource(attr, &cam->source))) goto fail;
    if (FAILED(MFCreateSourceReaderFromMediaSource(cam->source, NULL, &cam->reader))) goto fail;
    IMFSourceReader_SetStreamSelection(cam->reader, MF_SOURCE_READER_FIRST_VIDEO_STREAM, TRUE);
    if (FAILED(MFCreateMediaType(&mt))) goto fail;
    IMFMediaType_SetGUID(mt, &MF_MT_MAJOR_TYPE, &MFMediaType_Video);
    IMFMediaType_SetGUID(mt, &MF_MT_SUBTYPE, mjpeg ? &MFVideoFormat_MJPG : &MFVideoFormat_YUY2);
    IMFMediaType_SetUINT64(mt, &MF_MT_FRAME_SIZE,
                           ((UINT64)(UINT32)width << 32) | (UINT32)height);
    if (FAILED(IMFSourceReader_SetCurrentMediaType(cam->reader, MF_SOURCE_READER_FIRST_VIDEO_STREAM, NULL, mt))) {
        IMFMediaType_SetGUID(mt, &MF_MT_SUBTYPE, &MFVideoFormat_YUY2);
        cam->mjpeg = 0;
        if (FAILED(IMFSourceReader_SetCurrentMediaType(cam->reader, MF_SOURCE_READER_FIRST_VIDEO_STREAM, NULL, mt)))
            goto fail;
    }
    IMFMediaType *cur = NULL;
    GUID sub;
    if (SUCCEEDED(IMFSourceReader_GetCurrentMediaType(cam->reader, MF_SOURCE_READER_FIRST_VIDEO_STREAM, &cur))) {
        if (SUCCEEDED(IMFMediaType_GetGUID(cur, &MF_MT_SUBTYPE, &sub))) {
            if (IsEqualGUID(&sub, &MFVideoFormat_MJPG)) {
                cam->fmt = 0;
                cam->mjpeg = 1;
            } else if (IsEqualGUID(&sub, &MFVideoFormat_NV12)) {
                cam->fmt = 2;
                cam->mjpeg = 0;
            } else {
                cam->fmt = 1;
                cam->mjpeg = 0;
            }
        }
        IMFMediaType_Release(cur);
    }
    IMFMediaType_Release(mt);
    IMFAttributes_Release(attr);
    return cam;
fail:
    if (mt) IMFMediaType_Release(mt);
    if (attr) IMFAttributes_Release(attr);
    openscan_cam_close(cam);
    return NULL;
}

void openscan_cam_close(openscan_cam *cam) {
    if (!cam) return;
    if (cam->reader) IMFSourceReader_Release(cam->reader);
    if (cam->source) {
        IMFMediaSource_Shutdown(cam->source);
        IMFMediaSource_Release(cam->source);
    }
    free(cam);
    mf_sub();
}

int openscan_cam_set_exposure(openscan_cam *cam, int exposure_100us, int gain) {
    (void)cam;
    (void)exposure_100us;
    (void)gain;
    return 1;
}

int openscan_cam_start(openscan_cam *cam) {
    if (!cam || !cam->reader) return -1;
    cam->started = 1;
    return 0;
}

int openscan_cam_stop(openscan_cam *cam) {
    if (cam) cam->started = 0;
    return 0;
}

static int copy_sample(openscan_cam *cam, IMFSample *sample, uint8_t *dst, size_t dst_cap, size_t *out_n) {
    IMFMediaBuffer *mb = NULL;
    if (FAILED(IMFSample_ConvertToContiguousBuffer(sample, &mb))) return -1;
    BYTE *p = NULL;
    DWORD n = 0;
    if (FAILED(IMFMediaBuffer_Lock(mb, &p, NULL, &n))) {
        IMFMediaBuffer_Release(mb);
        return -1;
    }
    int rc = -1;
    if (cam->fmt == 0) {
        if (n >= 16 && n <= dst_cap) {
            memcpy(dst, p, n);
            if (out_n) *out_n = n;
            rc = 0;
        }
    } else if (cam->fmt == 2) {
        size_t need = (size_t)cam->width * (size_t)cam->height;
        if (n >= need && need <= dst_cap) {
            memcpy(dst, p, need);
            if (out_n) *out_n = need;
            rc = 0;
        }
    } else {
        size_t need = (size_t)cam->width * (size_t)cam->height;
        if (n >= need * 2 && need <= dst_cap) {
            for (size_t i = 0; i < need; i++) dst[i] = p[i * 2];
            if (out_n) *out_n = need;
            rc = 0;
        }
    }
    IMFMediaBuffer_Unlock(mb);
    IMFMediaBuffer_Release(mb);
    return rc;
}

int openscan_cam_grab(openscan_cam *cam, uint8_t *dst, size_t dst_cap, size_t *out_n, uint64_t *ts_us) {
    if (ts_us) *ts_us = 0;
    if (!cam || !cam->reader || !cam->started || !dst) return -1;
    DWORD flags = 0;
    LONGLONG ts = 0;
    IMFSample *sample = NULL;
    HRESULT hr = IMFSourceReader_ReadSample(cam->reader, MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, NULL, &flags, &ts, &sample);
    if (FAILED(hr) || !sample) return -1;
    int rc = copy_sample(cam, sample, dst, dst_cap, out_n);
    IMFSample_Release(sample);
    return rc;
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
