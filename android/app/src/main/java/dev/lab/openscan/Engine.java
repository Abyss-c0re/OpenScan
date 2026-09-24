package dev.lab.openscan;

import android.content.Context;
import android.graphics.Bitmap;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;

/** JNI face of the desktop scan backend. Settings use the same names as the desktop app. */
public final class Engine {
    static {
        System.loadLibrary("openscanjni");
    }

    public static final int MODE_STOP = 0;
    public static final int MODE_SCAN = 1;
    public static final int MODE_PAUSE = 2;
    public static final int SHAPE_MOLD = 0;
    public static final int SHAPE_MEASURED = 1;
    public static final int PANEL_CAMERA = 0;
    public static final int PANEL_MODEL = 1;
    public static final int PANEL_A = 2;
    public static final int PANEL_B = 3;
    public static final int PANEL_SIDE = 4;

    public static final int EV_MOVE = 0;
    public static final int EV_DOWN = 1;
    public static final int EV_UP = 4;

    public long handle;
    public int frameW = 1280;
    public int frameH = 720;

    public static Engine open(Context context) {
        Engine e = new Engine();
        File dir = new File(context.getFilesDir(), "calib");
        if (!dir.exists() && !dir.mkdirs()) return null;
        File calib = new File(dir, "EXAMPLE01.txt");
        if (!calib.exists()) {
            try (InputStream in = context.getAssets().open("EXAMPLE01.txt");
                 FileOutputStream out = new FileOutputStream(calib)) {
                byte[] buf = new byte[8192];
                int n;
                while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
            } catch (Exception ex) {
                return null;
            }
        }
        try (java.io.BufferedReader r = new java.io.BufferedReader(new java.io.FileReader(calib))) {
            String line = r.readLine();
            if (line != null) {
                String[] p = line.trim().split("\\s+");
                if (p.length >= 2) {
                    e.frameW = Integer.parseInt(p[0]);
                    e.frameH = Integer.parseInt(p[1]);
                }
            }
        } catch (Exception ignored) {
        }
        e.handle = nativeCreate(calib.getAbsolutePath());
        return e.handle == 0 ? null : e;
    }

    public void close() {
        if (handle != 0) nativeDestroy(handle);
        handle = 0;
    }

    public void setShape(int shape) { nativeSetShape(handle, shape); }
    public void setDistance(float mm) { nativeSetDistance(handle, mm); }

    public void setBuild(float near, float far, int stride, int smooth, int sweep, int relief,
                         boolean solid, boolean flip) {
        nativeSetBuild(handle, near, far, stride, smooth, sweep, relief, solid ? 1 : 0, flip ? 1 : 0);
    }

    public void setMode(int mode) { nativeSetMode(handle, mode); }
    public void reset() { nativeReset(handle); }
    public int points() { return nativePoints(handle); }

    public void mouse(int event, int x, int y) { nativeMouse(handle, event, x, y, 0); }

    /** status: fused, lost, valid, tracking, mode, points, scannedDeg, detail, medianMm */
    public int push(byte[] a, byte[] b, int[] status) {
        return nativePush(handle, a, b, frameW, frameH, status);
    }

    public Bitmap preview() {
        return bitmap(nativePreview(handle));
    }

    /** Camera, model, pattern, clean view, or side. Null until a frame has been pushed. */
    public Bitmap panel(int which) {
        return bitmap(nativePanel(handle, which));
    }

    private static Bitmap bitmap(int[] px) {
        if (px == null || px.length < 2) return null;
        int w = px[0];
        int h = px[1];
        if (w < 2 || h < 2 || px.length < w * h + 2) return null;
        Bitmap bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        bmp.setPixels(px, 2, w, 0, 0, w, h);
        return bmp;
    }

    /** Writes the current model. Returns triangle count, 0 for a point cloud, or -1. */
    public int write(File file) {
        return nativeWrite(handle, file.getAbsolutePath());
    }

    /** Opens the Fox UVC pair. MJPEG, same exposure units as the desktop app. 0 on failure. */
    public long scannerStart(String pathA, String pathB, int exposureA, int gainA, int exposureB, int gainB) {
        return nativeScannerStart(pathA, pathB, exposureA, gainA, exposureB, gainB, frameW, frameH);
    }

    public void scannerClose(long cams) { nativeScannerClose(cams); }

    /** Fills both buffers with luminance. Returns 0 when a pair was decoded. */
    public int scannerGrab(long cams, byte[] a, byte[] b) { return nativeScannerGrab(cams, a, b); }

    private static native long nativeCreate(String path);
    private static native void nativeDestroy(long handle);
    private static native void nativeSetShape(long handle, int shape);
    private static native void nativeSetDistance(long handle, float mm);
    private static native void nativeSetBuild(long handle, float nearMm, float farMm, int stride,
                                              int smooth, int sweep, int relief, int solid, int flip);
    private static native void nativeSetMode(long handle, int mode);
    private static native void nativeReset(long handle);
    private static native void nativeMouse(long handle, int event, int x, int y, int flags);
    private static native int nativePoints(long handle);
    private static native int nativePush(long handle, byte[] a, byte[] b, int w, int h, int[] status);
    private static native int[] nativePreview(long handle);
    private static native int[] nativePanel(long handle, int which);
    private static native int nativeWrite(long handle, String path);
    private static native long nativeScannerStart(String pathA, String pathB, int exposureA, int gainA,
                                              int exposureB, int gainB, int width, int height);
    private static native void nativeScannerClose(long cams);
    private static native int nativeScannerGrab(long cams, byte[] a, byte[] b);
}
