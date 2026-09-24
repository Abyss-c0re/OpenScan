package dev.lab.openscan;

import android.content.Context;
import android.content.SharedPreferences;

/** Same keys the desktop app stores, so a setting means the same thing. */
public final class OpenScanPrefs {
    public final int exposureA;
    public final int gainA;
    public final int exposureB;
    public final int gainB;
    public final int distanceMm;
    public final int nearMm;
    public final int farMm;
    public final int stride;
    public final int smooth;
    public final int sweepDeg;
    public final int relief;
    public final boolean solid;
    public final boolean flip;
    public final int shape;

    private OpenScanPrefs(SharedPreferences p) {
        exposureA = clamp(p.getInt("exposure-a", 22), 1, 200);
        gainA = clamp(p.getInt("gain-a", 6), 0, 100);
        exposureB = clamp(p.getInt("exposure-b", 16), 1, 200);
        gainB = clamp(p.getInt("gain-b", 4), 0, 100);
        distanceMm = clamp(p.getInt("distance-mm", 220), 100, 500);
        nearMm = clamp(p.getInt("near-mm", 80), 60, 450);
        farMm = clamp(p.getInt("far-mm", 550), 100, 800);
        stride = clamp(p.getInt("stride", 1), 1, 6);
        smooth = clamp(p.getInt("smooth", 5), 0, 8);
        sweepDeg = clamp(p.getInt("sweep-deg", 80), 20, 140);
        relief = clamp(p.getInt("relief", 40), 0, 100);
        solid = p.getBoolean("solid", true);
        flip = p.getBoolean("flip", true);
        int sh = p.getInt("shape", Engine.SHAPE_MOLD);
        shape = sh == Engine.SHAPE_MEASURED ? Engine.SHAPE_MEASURED : Engine.SHAPE_MOLD;
    }

    public static SharedPreferences store(Context context) {
        return context.getSharedPreferences("openscan", Context.MODE_PRIVATE);
    }

    public static OpenScanPrefs load(Context context) {
        return new OpenScanPrefs(store(context));
    }

    public void apply(Engine engine) {
        if (engine == null || engine.handle == 0) return;
        engine.setDistance(distanceMm);
        engine.setBuild(nearMm, farMm, stride, smooth, sweepDeg, relief, solid, flip);
        if (engine.handle != 0) {
            /* setShape clears the model when the mode actually changes. */
        }
    }

    public static void saveShape(Context context, int shape) {
        store(context).edit().putInt("shape", shape).apply();
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
