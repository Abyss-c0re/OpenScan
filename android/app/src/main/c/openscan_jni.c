#include "openscan/openscan_calib.h"
#include "openscan/openscan_live.h"
#include "openscan/openscan_v4l2.h"

#include <jni.h>
#include <android/log.h>
#include <stdlib.h>
#include <string.h>

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "openscan", __VA_ARGS__)

typedef struct {
    openscan_cam *a;
    openscan_cam *b;
    int exposure_auto;
} OsCams;

JNIEXPORT jlong JNICALL
Java_dev_lab_openscan_Engine_nativeCreate(JNIEnv *env, jclass cls, jstring path) {
    (void)cls;
    if (!path) return 0;
    const char *p = (*env)->GetStringUTFChars(env, path, NULL);
    openscan_calib cal;
    if (openscan_calib_load(p, &cal) != 0) {
        (*env)->ReleaseStringUTFChars(env, path, p);
        return 0;
    }
    (*env)->ReleaseStringUTFChars(env, path, p);
    openscan_live *live = openscan_live_create(&cal, NULL);
    if (live) openscan_live_set_mode(live, OPENSCAN_MODE_STOP);
    return (jlong)live;
}

JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeDestroy(JNIEnv *env, jclass cls, jlong handle) {
    (void)env; (void)cls;
    if (handle) openscan_live_destroy((openscan_live *)handle);
}

JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeSetShape(JNIEnv *env, jclass cls, jlong handle, jint shape) {
    (void)env; (void)cls;
    if (handle) openscan_live_set_shape((openscan_live *)handle, shape);
}

JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeSetDistance(JNIEnv *env, jclass cls, jlong handle, jfloat mm) {
    (void)env; (void)cls;
    if (handle) openscan_live_set_distance_mm((openscan_live *)handle, mm);
}

JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeSetBuild(JNIEnv *env, jclass cls, jlong handle,
        jfloat near_mm, jfloat far_mm, jint stride, jint smooth, jint sweep, jint relief,
        jint solid, jint flip) {
    (void)env; (void)cls;
    if (!handle) return;
    openscan_build b;
    openscan_live_get_build((openscan_live *)handle, &b);
    b.near_mm = near_mm;
    b.far_mm = far_mm;
    b.stride = stride;
    b.smooth = smooth;
    b.sweep_deg = sweep;
    b.relief = relief;
    b.solid = solid;
    b.flip = flip;
    openscan_live_set_build((openscan_live *)handle, &b);
}

JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeSetMode(JNIEnv *env, jclass cls, jlong handle, jint mode) {
    (void)env; (void)cls;
    if (handle) openscan_live_set_mode((openscan_live *)handle, mode);
}

JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeReset(JNIEnv *env, jclass cls, jlong handle) {
    (void)env; (void)cls;
    if (!handle) return;
    openscan_live *live = (openscan_live *)handle;
    int shape = openscan_live_shape(live);
    float dist = openscan_live_distance_mm(live);
    openscan_build b;
    openscan_live_get_build(live, &b);
    openscan_live_reset(live);
    openscan_live_set_shape(live, shape);
    openscan_live_set_distance_mm(live, dist);
    openscan_live_set_build(live, &b);
    openscan_live_set_mode(live, OPENSCAN_MODE_STOP);
}

JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeMouse(JNIEnv *env, jclass cls, jlong handle, jint event, jint x, jint y, jint flags) {
    (void)env; (void)cls;
    if (handle) openscan_live_mouse((openscan_live *)handle, event, x, y, flags);
}

JNIEXPORT jint JNICALL
Java_dev_lab_openscan_Engine_nativePoints(JNIEnv *env, jclass cls, jlong handle) {
    (void)env; (void)cls;
    return handle ? openscan_live_points((openscan_live *)handle) : 0;
}

JNIEXPORT jint JNICALL
Java_dev_lab_openscan_Engine_nativePush(JNIEnv *env, jclass cls, jlong handle, jbyteArray ja, jbyteArray jb,
                                       jint width, jint height, jintArray status) {
    (void)cls;
    if (!handle || !ja || !jb) return -1;
    if ((*env)->GetArrayLength(env, ja) < width * height) return -1;
    if ((*env)->GetArrayLength(env, jb) < width * height) return -1;
    jbyte *a = (*env)->GetByteArrayElements(env, ja, NULL);
    jbyte *b = (*env)->GetByteArrayElements(env, jb, NULL);
    openscan_live_status st;
    int rc = openscan_live_push((openscan_live *)handle, (uint8_t *)a, (uint8_t *)b, width, height, &st);
    (*env)->ReleaseByteArrayElements(env, ja, a, JNI_ABORT);
    (*env)->ReleaseByteArrayElements(env, jb, b, JNI_ABORT);
    if (status && (*env)->GetArrayLength(env, status) >= 9) {
        jint out[9] = {st.fused, st.lost, st.valid_pixels, st.tracking, st.mode,
                       st.points, st.scanned_deg, st.detail, (jint)st.median_mm};
        (*env)->SetIntArrayRegion(env, status, 0, 9, out);
    }
    return rc;
}

static jintArray bgr_pixels(JNIEnv *env, const uint8_t *bgr, int w, int h) {
    if (!bgr || w < 2 || h < 2) return NULL;
    jint n = w * h;
    jint *px = malloc(((size_t)n + 2) * sizeof(jint));
    if (!px) return NULL;
    px[0] = w;
    px[1] = h;
    for (int i = 0; i < n; i++) {
        uint8_t b = bgr[i * 3], g = bgr[i * 3 + 1], r = bgr[i * 3 + 2];
        px[i + 2] = (jint)(0xff000000u | (r << 16) | (g << 8) | b);
    }
    jintArray arr = (*env)->NewIntArray(env, n + 2);
    (*env)->SetIntArrayRegion(env, arr, 0, n + 2, px);
    free(px);
    return arr;
}

JNIEXPORT jintArray JNICALL
Java_dev_lab_openscan_Engine_nativePreview(JNIEnv *env, jclass cls, jlong handle) {
    (void)cls;
    if (!handle) return NULL;
    int w = 0, h = 0;
    const uint8_t *bgr = openscan_live_preview_bgr((openscan_live *)handle, &w, &h);
    return bgr_pixels(env, bgr, w, h);
}

JNIEXPORT jintArray JNICALL
Java_dev_lab_openscan_Engine_nativePanel(JNIEnv *env, jclass cls, jlong handle, jint which) {
    (void)cls;
    if (!handle) return NULL;
    int w = 0, h = 0;
    const uint8_t *bgr = openscan_live_panel_bgr((openscan_live *)handle, which, &w, &h);
    return bgr_pixels(env, bgr, w, h);
}

JNIEXPORT jlong JNICALL
Java_dev_lab_openscan_Engine_nativeScannerStart(JNIEnv *env, jclass cls, jstring ja, jstring jb,
                                               jint ea, jint ga, jint eb, jint gb, jint width, jint height) {
    (void)cls;
    if (!ja || !jb) return 0;
    const char *pa = (*env)->GetStringUTFChars(env, ja, NULL);
    const char *pb = (*env)->GetStringUTFChars(env, jb, NULL);
    openscan_cam *a = openscan_cam_open(pa, width, height, 10, 1);
    openscan_cam *b = openscan_cam_open(pb, width, height, 10, 1);
    (*env)->ReleaseStringUTFChars(env, ja, pa);
    (*env)->ReleaseStringUTFChars(env, jb, pb);
    if (!a || !b) {
        openscan_cam_close(a);
        openscan_cam_close(b);
        return 0;
    }
    OsCams *cams = calloc(1, sizeof *cams);
    cams->a = a;
    cams->b = b;
    int ra = openscan_cam_set_exposure(a, ea, ga);
    int rb = openscan_cam_set_exposure(b, eb, gb);
    cams->exposure_auto = openscan_exposure_auto_pair(ra, rb);
    if (openscan_cam_start(a) != 0 || openscan_cam_start(b) != 0) {
        openscan_cam_close(a);
        openscan_cam_close(b);
        free(cams);
        return 0;
    }
    return (jlong)cams;
}

JNIEXPORT jint JNICALL
Java_dev_lab_openscan_Engine_nativeScannerExposure(JNIEnv *env, jclass cls, jlong handle,
                                                  jint ea, jint ga, jint eb, jint gb) {
    (void)env; (void)cls;
    if (!handle) return 0;
    OsCams *cams = (OsCams *)handle;
    int ra = openscan_cam_set_exposure(cams->a, ea, ga);
    int rb = openscan_cam_set_exposure(cams->b, eb, gb);
    cams->exposure_auto = openscan_exposure_auto_pair(ra, rb);
    return cams->exposure_auto;
}

JNIEXPORT jint JNICALL
Java_dev_lab_openscan_Engine_nativeScannerExposureAuto(JNIEnv *env, jclass cls, jlong handle) {
    (void)env; (void)cls;
    return handle ? ((OsCams *)handle)->exposure_auto : 0;
}

JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeScannerClose(JNIEnv *env, jclass cls, jlong handle) {
    (void)env; (void)cls;
    if (!handle) return;
    OsCams *cams = (OsCams *)handle;
    openscan_cam_close(cams->a);
    openscan_cam_close(cams->b);
    free(cams);
}

JNIEXPORT jint JNICALL
Java_dev_lab_openscan_Engine_nativeScannerGrab(JNIEnv *env, jclass cls, jlong handle, jbyteArray ya, jbyteArray yb) {
    (void)cls;
    (void)env; (void)handle; (void)ya; (void)yb;
    /* Phone frames arrive through nativePush. This path is the desktop cameras. */
    return -1;
}

JNIEXPORT jint JNICALL
Java_dev_lab_openscan_Engine_nativeWrite(JNIEnv *env, jclass cls, jlong handle, jstring path) {
    (void)cls;
    if (!handle || !path) return -1;
    const char *p = (*env)->GetStringUTFChars(env, path, NULL);
    int tris = 0;
    int rc = openscan_live_write((openscan_live *)handle, p, &tris);
    (*env)->ReleaseStringUTFChars(env, path, p);
    return rc == 0 ? tris : -1;
}
