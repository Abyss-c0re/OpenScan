package dev.lab.openscan;

import android.app.Activity;
import android.content.SharedPreferences;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.os.Bundle;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.Switch;
import android.widget.TextView;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/** Every desktop setting, with the same range and the same stored name. */
public class SettingsActivity extends Activity {
    private final Map<String, SeekBar> bars = new LinkedHashMap<>();
    private Switch solid;
    private Switch flip;
    private TextView solidLabel;
    private TextView flipLabel;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        SharedPreferences prefs = OpenScanPrefs.store(this);
        LinearLayout col = new LinearLayout(this);
        col.setOrientation(LinearLayout.VERTICAL);
        int pad = (int) (12 * getResources().getDisplayMetrics().density);
        col.setPadding(pad, pad, pad, pad);

        add(col, prefs, "Camera A exposure", "exposure-a", 1, 200, 22, true,
                "1 is 0.1 ms, 200 is 20 ms. Raise it if camera A is too dark.");
        add(col, prefs, "Camera A gain", "gain-a", 0, 100, 6, false,
                "Amplifies camera A after the exposure. Keep this low so the dots stay sharp.");
        add(col, prefs, "Camera B exposure", "exposure-b", 1, 200, 16, true,
                "Camera B is the clean view. Lower this if the object is blown out white.");
        add(col, prefs, "Camera B gain", "gain-b", 0, 100, 4, false,
                "Amplifies camera B. Usually lower than camera A.");
        add(col, prefs, "Distance", "distance-mm", 100, 500, 220, false,
                "Millimetres. Mold uses it as the size of the solid. Measured uses it to pick the stripe band.");
        add(col, prefs, "Near", "near-mm", 60, 450, 80, false,
                "Drops any 3D point closer than this.");
        add(col, prefs, "Far", "far-mm", 100, 800, 550, false,
                "Drops any 3D point farther than this.");
        add(col, prefs, "Stride", "stride", 1, 6, 1, false,
                "Sample step. 1 is the full grid. 6 keeps every sixth row.");
        add(col, prefs, "Smooth", "smooth", 0, 8, 5, false,
                "Depth blur before the triangles. 0 keeps every stripe. 8 clays the surface.");
        add(col, prefs, "Mold sweep", "sweep-deg", 20, 140, 80, false,
                "How far Mold wraps the outline. 20 is almost flat. 140 is a round body.");
        add(col, prefs, "Mold relief", "relief", 0, 100, 40, false,
                "How strongly shading pushes the Mold surface. 0 is smooth. 100 cuts in eyes and nose.");

        solidLabel = new TextView(this);
        solidLabel.setPadding(0, pad, 0, 0);
        solid = new Switch(this);
        solid.setChecked(prefs.getBoolean("solid", true));
        solidLabel.setText(solid.isChecked() ? "Solid mesh: on" : "Solid mesh: off");
        solid.setOnCheckedChangeListener((b, on) -> {
            prefs.edit().putBoolean("solid", on).apply();
            solidLabel.setText(on ? "Solid mesh: on" : "Solid mesh: off");
        });
        TextView solidTip = tip("On builds triangles. Off keeps the point cloud.");
        col.addView(solidLabel);
        col.addView(solid);
        col.addView(solidTip);

        flipLabel = new TextView(this);
        flipLabel.setPadding(0, pad, 0, 0);
        flip = new Switch(this);
        flip.setChecked(prefs.getBoolean("flip", true));
        flipLabel.setText(flip.isChecked() ? "Flip scanner: on" : "Flip scanner: off");
        flip.setOnCheckedChangeListener((b, on) -> {
            prefs.edit().putBoolean("flip", on).apply();
            flipLabel.setText(on ? "Flip scanner: on" : "Flip scanner: off");
        });
        col.addView(flipLabel);
        col.addView(flip);
        col.addView(tip("Turns both cameras upside down. Leave this on unless the picture is upside down."));

        TextView hint = new TextView(this);
        hint.setText("Near, Far, Stride, Smooth and Solid mesh change the points and the triangles on the next frame. "
                + "Sweep and Relief change the Mold solid only. Measured stays the stripe surface. "
                + "Flip scanner turns the picture and the model together. "
                + "A missing calibration file is downloaded from 3DMakerpro's servers. "
                + "Tested on the Fox. Other 3DMakerpro scanners may work.");
        hint.setPadding(0, pad, 0, pad);
        col.addView(hint);

        Button defaults = new Button(this);
        defaults.setText("Defaults");
        defaults.setOnClickListener(v -> {
            set(prefs, "exposure-a", 22);
            set(prefs, "gain-a", 6);
            set(prefs, "exposure-b", 16);
            set(prefs, "gain-b", 4);
            set(prefs, "distance-mm", 220);
            set(prefs, "near-mm", 80);
            set(prefs, "far-mm", 550);
            set(prefs, "stride", 1);
            set(prefs, "smooth", 5);
            set(prefs, "sweep-deg", 80);
            set(prefs, "relief", 40);
            solid.setChecked(true);
            flip.setChecked(true);
        });
        col.addView(defaults);

        ScrollView scroll = new ScrollView(this);
        scroll.addView(col);
        setContentView(scroll);
    }

    private void set(SharedPreferences prefs, String key, int value) {
        SeekBar bar = bars.get(key);
        if (bar != null) bar.setProgress(value);
        else prefs.edit().putInt(key, value).apply();
    }

    private void add(LinearLayout col, SharedPreferences prefs, String name, String key,
                     int min, int max, int def, boolean exposure, String caption) {
        int value = prefs.getInt(key, def);
        if (value < min) value = min;
        if (value > max) value = max;
        TextView label = new TextView(this);
        label.setPadding(0, 18, 0, 0);
        SeekBar bar = new SeekBar(this);
        bar.setMin(min);
        bar.setMax(max);
        int tint = Color.WHITE;
        bar.setProgressTintList(ColorStateList.valueOf(tint));
        bar.setThumbTintList(ColorStateList.valueOf(tint));
        Runnable show = () -> label.setText(captionFor(name, key, bar.getProgress(), exposure));
        bar.setProgress(value);
        show.run();
        bar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                prefs.edit().putInt(key, progress).apply();
                show.run();
            }
            @Override public void onStartTrackingTouch(SeekBar seekBar) {}
            @Override public void onStopTrackingTouch(SeekBar seekBar) {}
        });
        bars.put(key, bar);
        col.addView(label);
        col.addView(bar);
        col.addView(tip(caption));
    }

    private static String captionFor(String name, String key, int progress, boolean exposure) {
        if (exposure) return name + ": " + progress + "  ·  " + String.format(Locale.US, "%.1f", progress * 0.1) + " ms";
        if ("distance-mm".equals(key) || "near-mm".equals(key) || "far-mm".equals(key))
            return name + ": " + progress + " mm";
        if ("sweep-deg".equals(key)) return name + ": " + progress + "°";
        return name + ": " + progress;
    }

    private TextView tip(String caption) {
        TextView tip = new TextView(this);
        tip.setText(caption);
        tip.setTextSize(12);
        tip.setTextColor(0xFFB0B0B0);
        return tip;
    }
}
