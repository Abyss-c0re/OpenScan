#include "openscan/openscan_calib.h"
#include "openscan/openscan_live.h"
#include "openscan/openscan_scan.h"
#include "openscan/openscan_v4l2.h"

#include <android/log.h>

#include <opencv2/imgcodecs.hpp>

#include <jni.h>

#include <cstring>
#include <string>
#include <vector>

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "openscan", __VA_ARGS__)

static openscan_scan_opts default_opts() {
    openscan_scan_opts o{};
    o.scale = 0.5;
    o.min_mm = 80;
    o.max_mm = 700;
    o.edge_mm = 12;
    o.preview_png = nullptr;
    return o;
}

extern "C" JNIEXPORT jlong JNICALL
Java_dev_lab_openscan_Engine_nativeCreate(JNIEnv *env, jclass, jstring path) {
    const char *p = env->GetStringUTFChars(path, nullptr);
    openscan_calib cal{};
    int rc = openscan_calib_load(p, &cal);
    env->ReleaseStringUTFChars(path, p);
    if (rc != 0) {
        LOGI("calib load failed");
        return 0;
    }
    openscan_scan_opts opt = default_opts();
    openscan_live *live = openscan_live_create(&cal, &opt);
    if (!live) return 0;
    openscan_live_set_mode(live, OPENSCAN_MODE_STOP);
    return reinterpret_cast<jlong>(live);
}

extern "C" JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeDestroy(JNIEnv *, jclass, jlong handle) {
    if (handle) openscan_live_destroy(reinterpret_cast<openscan_live *>(handle));
}

extern "C" JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeSetShape(JNIEnv *, jclass, jlong handle, jint shape) {
    if (handle) openscan_live_set_shape(reinterpret_cast<openscan_live *>(handle), shape);
}

extern "C" JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeSetDistance(JNIEnv *, jclass, jlong handle, jfloat mm) {
    if (handle) openscan_live_set_distance_mm(reinterpret_cast<openscan_live *>(handle), mm);
}

extern "C" JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeSetBuild(JNIEnv *, jclass, jlong handle, jfloat near_mm, jfloat far_mm,
                                        jint stride, jint smooth, jint sweep, jint relief, jint solid, jint flip) {
    if (!handle) return;
    openscan_build b{};
    b.near_mm = near_mm;
    b.far_mm = far_mm;
    b.stride = stride;
    b.smooth = smooth;
    b.sweep_deg = sweep;
    b.relief = relief;
    b.solid = solid;
    b.flip = flip;
    openscan_live_set_build(reinterpret_cast<openscan_live *>(handle), &b);
}

extern "C" JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeSetMode(JNIEnv *, jclass, jlong handle, jint mode) {
    if (handle) openscan_live_set_mode(reinterpret_cast<openscan_live *>(handle), mode);
}

extern "C" JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeReset(JNIEnv *, jclass, jlong handle) {
    if (!handle) return;
    openscan_live *live = reinterpret_cast<openscan_live *>(handle);
    int shape = openscan_live_shape(live);
    float dist = openscan_live_distance_mm(live);
    openscan_build b{};
    openscan_live_get_build(live, &b);
    openscan_live_reset(live);
    openscan_live_set_shape(live, shape);
    openscan_live_set_distance_mm(live, dist);
    openscan_live_set_build(live, &b);
    openscan_live_set_mode(live, OPENSCAN_MODE_STOP);
}

extern "C" JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeMouse(JNIEnv *, jclass, jlong handle, jint event, jint x, jint y, jint flags) {
    if (handle) openscan_live_mouse(reinterpret_cast<openscan_live *>(handle), event, x, y, flags);
}

extern "C" JNIEXPORT jint JNICALL
Java_dev_lab_openscan_Engine_nativePoints(JNIEnv *, jclass, jlong handle) {
    return handle ? openscan_live_points(reinterpret_cast<openscan_live *>(handle)) : 0;
}

extern "C" JNIEXPORT jint JNICALL
Java_dev_lab_openscan_Engine_nativePush(JNIEnv *env, jclass, jlong handle, jbyteArray ja, jbyteArray jb, jint width,
                                    jint height, jintArray status) {
    if (!handle || !ja || !jb) return -1;
    jsize na = env->GetArrayLength(ja);
    jsize nb = env->GetArrayLength(jb);
    if (na < width * height || nb < width * height) return -1;
    jbyte *a = env->GetByteArrayElements(ja, nullptr);
    jbyte *b = env->GetByteArrayElements(jb, nullptr);
    if (!a || !b) {
        if (a) env->ReleaseByteArrayElements(ja, a, JNI_ABORT);
        if (b) env->ReleaseByteArrayElements(jb, b, JNI_ABORT);
        return -1;
    }
    openscan_live_status st{};
    int rc = openscan_live_push(reinterpret_cast<openscan_live *>(handle), reinterpret_cast<uint8_t *>(a),
                           reinterpret_cast<uint8_t *>(b), width, height, &st);
    env->ReleaseByteArrayElements(ja, a, JNI_ABORT);
    env->ReleaseByteArrayElements(jb, b, JNI_ABORT);
    if (status && env->GetArrayLength(status) >= 9) {
        jint out[9] = {st.fused, st.lost, st.valid_pixels, st.tracking, st.mode,
                       st.points, st.scanned_deg, st.detail, (jint)st.median_mm};
        env->SetIntArrayRegion(status, 0, 9, out);
    }
    return rc;
}

static jintArray bgr_pixels(JNIEnv *env, const uint8_t *bgr, int w, int h) {
    if (!bgr || w < 2 || h < 2) return nullptr;
    jint n = w * h;
    std::vector<jint> px((size_t)n + 2);
    px[0] = w;
    px[1] = h;
    for (int i = 0; i < n; i++) {
        uint8_t b = bgr[i * 3];
        uint8_t g = bgr[i * 3 + 1];
        uint8_t r = bgr[i * 3 + 2];
        px[(size_t)i + 2] = (jint)(0xff000000u | (r << 16) | (g << 8) | b);
    }
    jintArray arr = env->NewIntArray(n + 2);
    env->SetIntArrayRegion(arr, 0, n + 2, px.data());
    return arr;
}

extern "C" JNIEXPORT jintArray JNICALL
Java_dev_lab_openscan_Engine_nativePreview(JNIEnv *env, jclass, jlong handle) {
    if (!handle) return nullptr;
    int w = 0, h = 0;
    const uint8_t *bgr = openscan_live_preview_bgr(reinterpret_cast<openscan_live *>(handle), &w, &h);
    return bgr_pixels(env, bgr, w, h);
}

extern "C" JNIEXPORT jintArray JNICALL
Java_dev_lab_openscan_Engine_nativePanel(JNIEnv *env, jclass, jlong handle, jint which) {
    if (!handle) return nullptr;
    int w = 0, h = 0;
    const uint8_t *bgr = openscan_live_panel_bgr(reinterpret_cast<openscan_live *>(handle), which, &w, &h);
    return bgr_pixels(env, bgr, w, h);
}

struct OpenScanCams {
    openscan_cam *a;
    openscan_cam *b;
};

static int jpeg_gray(const uint8_t *jpg, size_t n, int w, int h, uint8_t *y) {
    cv::Mat encoded(1, (int)n, CV_8UC1, const_cast<uint8_t *>(jpg));
    cv::Mat gray = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
    if (gray.empty() || gray.cols != w || gray.rows != h || !gray.isContinuous()) return -1;
    memcpy(y, gray.data, (size_t)w * (size_t)h);
    return 0;
}

extern "C" JNIEXPORT jlong JNICALL
Java_dev_lab_openscan_Engine_nativeScannerStart(JNIEnv *env, jclass, jstring ja, jstring jb, jint ea, jint ga, jint eb,
                                         jint gb, jint width, jint height) {
    if (!ja || !jb) return 0;
    const char *pa = env->GetStringUTFChars(ja, nullptr);
    const char *pb = env->GetStringUTFChars(jb, nullptr);
    openscan_cam *a = openscan_cam_open(pa, width, height, 10, 1);
    openscan_cam *b = openscan_cam_open(pb, width, height, 10, 1);
    if (!a || !b) {
        LOGI("scanner camera open failed A=%s (%s) B=%s (%s)", pa, a ? "ok" : "no", pb, b ? "ok" : "no");
        openscan_cam_close(a);
        openscan_cam_close(b);
        env->ReleaseStringUTFChars(ja, pa);
        env->ReleaseStringUTFChars(jb, pb);
        return 0;
    }
    openscan_cam_set_exposure(a, ea, ga);
    openscan_cam_set_exposure(b, eb, gb);
    int started = openscan_cam_start(a) == 0 && openscan_cam_start(b) == 0;
    env->ReleaseStringUTFChars(ja, pa);
    env->ReleaseStringUTFChars(jb, pb);
    if (!started) {
        LOGI("scanner camera stream failed");
        openscan_cam_close(a);
        openscan_cam_close(b);
        return 0;
    }
    return reinterpret_cast<jlong>(new OpenScanCams{a, b});
}

extern "C" JNIEXPORT void JNICALL
Java_dev_lab_openscan_Engine_nativeScannerClose(JNIEnv *, jclass, jlong handle) {
    if (!handle) return;
    OpenScanCams *cams = reinterpret_cast<OpenScanCams *>(handle);
    openscan_cam_close(cams->a);
    openscan_cam_close(cams->b);
    delete cams;
}

extern "C" JNIEXPORT jint JNICALL
Java_dev_lab_openscan_Engine_nativeScannerGrab(JNIEnv *env, jclass, jlong handle, jbyteArray ya, jbyteArray yb) {
    if (!handle || !ya || !yb) return -1;
    OpenScanCams *cams = reinterpret_cast<OpenScanCams *>(handle);
    int w = openscan_cam_width(cams->a);
    int h = openscan_cam_height(cams->a);
    if (w < 2 || h < 2) return -1;
    if (env->GetArrayLength(ya) < w * h || env->GetArrayLength(yb) < w * h) return -1;
    std::vector<uint8_t> ja(2 * 1024 * 1024), jb(2 * 1024 * 1024);
    size_t na = 0, nb = 0;
    if (openscan_cam_grab_latest(cams->a, ja.data(), ja.size(), &na, nullptr) < 0) return -1;
    if (openscan_cam_grab_latest(cams->b, jb.data(), jb.size(), &nb, nullptr) < 0) return -1;
    jbyte *a = env->GetByteArrayElements(ya, nullptr);
    jbyte *b = env->GetByteArrayElements(yb, nullptr);
    if (!a || !b) {
        if (a) env->ReleaseByteArrayElements(ya, a, JNI_ABORT);
        if (b) env->ReleaseByteArrayElements(yb, b, JNI_ABORT);
        return -1;
    }
    int rc = 0;
    if (jpeg_gray(ja.data(), na, w, h, reinterpret_cast<uint8_t *>(a)) < 0 ||
        jpeg_gray(jb.data(), nb, w, h, reinterpret_cast<uint8_t *>(b)) < 0)
        rc = -1;
    env->ReleaseByteArrayElements(ya, a, rc == 0 ? 0 : JNI_ABORT);
    env->ReleaseByteArrayElements(yb, b, rc == 0 ? 0 : JNI_ABORT);
    return rc;
}

extern "C" JNIEXPORT jint JNICALL
Java_dev_lab_openscan_Engine_nativeWrite(JNIEnv *env, jclass, jlong handle, jstring path) {
    if (!handle || !path) return -1;
    const char *p = env->GetStringUTFChars(path, nullptr);
    int tris = 0;
    int rc = openscan_live_write(reinterpret_cast<openscan_live *>(handle), p, &tris);
    env->ReleaseStringUTFChars(path, p);
    return rc == 0 ? tris : -1;
}
