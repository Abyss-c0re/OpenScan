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

/* Android has no libjpeg. The platform decoder turns the camera JPEG into gray. */
static int jpeg_gray(JNIEnv *env, const uint8_t *jpg, size_t n, int w, int h, uint8_t *dst) {
    jclass factory, bmp_cls;
    jmethodID decode, get_w, get_h, get_px, recycle;
    jbyteArray bytes;
    jobject bmp;
    jintArray pixels;
    jint *px, sw, sh;
    int y, x;
    if (!jpg || n < 16 || w < 2 || h < 2 || !dst) return -1;
    factory = (*env)->FindClass(env, "android/graphics/BitmapFactory");
    if (!factory) return -1;
    decode = (*env)->GetStaticMethodID(env, factory, "decodeByteArray", "([BII)Landroid/graphics/Bitmap;");
    if (!decode) return -1;
    bytes = (*env)->NewByteArray(env, (jsize)n);
    if (!bytes) return -1;
    (*env)->SetByteArrayRegion(env, bytes, 0, (jsize)n, (const jbyte *)jpg);
    bmp = (*env)->CallStaticObjectMethod(env, factory, decode, bytes, 0, (jint)n);
    (*env)->DeleteLocalRef(env, bytes);
    if ((*env)->ExceptionCheck(env)) {
        (*env)->ExceptionClear(env);
        return -1;
    }
    if (!bmp) return -1;
    bmp_cls = (*env)->GetObjectClass(env, bmp);
    get_w = (*env)->GetMethodID(env, bmp_cls, "getWidth", "()I");
    get_h = (*env)->GetMethodID(env, bmp_cls, "getHeight", "()I");
    get_px = (*env)->GetMethodID(env, bmp_cls, "getPixels", "([IIIIIII)V");
    recycle = (*env)->GetMethodID(env, bmp_cls, "recycle", "()V");
    sw = (*env)->CallIntMethod(env, bmp, get_w);
    sh = (*env)->CallIntMethod(env, bmp, get_h);
    if (sw < 1 || sh < 1 || !get_px) {
        if (recycle) (*env)->CallVoidMethod(env, bmp, recycle);
        (*env)->DeleteLocalRef(env, bmp);
        return -1;
    }
    pixels = (*env)->NewIntArray(env, sw * sh);
    (*env)->CallVoidMethod(env, bmp, get_px, pixels, 0, sw, 0, 0, sw, sh);
    if ((*env)->ExceptionCheck(env)) {
        (*env)->ExceptionClear(env);
        (*env)->DeleteLocalRef(env, pixels);
        if (recycle) (*env)->CallVoidMethod(env, bmp, recycle);
        (*env)->DeleteLocalRef(env, bmp);
        return -1;
    }
    px = (*env)->GetIntArrayElements(env, pixels, NULL);
    for (y = 0; y < h; y++) {
        int sy = y * sh / h;
        if (sy >= sh) sy = sh - 1;
        for (x = 0; x < w; x++) {
            int sx = x * sw / w;
            unsigned p;
            if (sx >= sw) sx = sw - 1;
            p = (unsigned)px[sy * sw + sx];
            dst[y * w + x] = (uint8_t)((((p >> 16) & 255) * 77 + ((p >> 8) & 255) * 150 + (p & 255) * 29) >> 8);
        }
    }
    (*env)->ReleaseIntArrayElements(env, pixels, px, JNI_ABORT);
    (*env)->DeleteLocalRef(env, pixels);
    if (recycle) (*env)->CallVoidMethod(env, bmp, recycle);
    (*env)->DeleteLocalRef(env, bmp);
    return 0;
}

static int grab_gray(JNIEnv *env, openscan_cam *cam, jbyteArray out) {
    int w = openscan_cam_width(cam);
    int h = openscan_cam_height(cam);
    size_t cap = (size_t)w * (size_t)h * 4;
    uint8_t *raw = NULL, *gray = NULL;
    size_t n = 0;
    int rc = -1;
    if (w < 2 || h < 2 || (*env)->GetArrayLength(env, out) < w * h) return -1;
    raw = malloc(cap);
    gray = malloc((size_t)w * (size_t)h);
    if (!raw || !gray) goto done;
    if (openscan_cam_grab_latest(cam, raw, cap, &n, NULL) != 0 || n < 16) goto done;
    if (openscan_cam_mjpeg(cam)) {
        if (jpeg_gray(env, raw, n, w, h, gray) != 0) goto done;
    } else {
        if (n < (size_t)w * (size_t)h) goto done;
        memcpy(gray, raw, (size_t)w * (size_t)h);
    }
    (*env)->SetByteArrayRegion(env, out, 0, w * h, (const jbyte *)gray);
    rc = 0;
done:
    free(raw);
    free(gray);
    return rc;
}

JNIEXPORT jint JNICALL
Java_dev_lab_openscan_Engine_nativeScannerGrab(JNIEnv *env, jclass cls, jlong handle, jbyteArray ya, jbyteArray yb) {
    OsCams *cams;
    (void)cls;
    if (!handle || !ya || !yb) return -1;
    cams = (OsCams *)handle;
    if (grab_gray(env, cams->a, ya) != 0) return -1;
    if (grab_gray(env, cams->b, yb) != 0) return -1;
    return 0;
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
