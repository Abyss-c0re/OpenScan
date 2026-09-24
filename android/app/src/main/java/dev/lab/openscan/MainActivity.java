package dev.lab.openscan;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.content.res.ColorStateList;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.drawable.BitmapDrawable;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.media.Image;
import android.media.ImageReader;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;
import android.util.Range;
import android.util.Size;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Scan screen. Live frames come from the Fox USB cameras. A pair of pictures
 * can also be opened with DocumentsUI, and export goes back through DocumentsUI.
 * Drag the right half of the preview to orbit the model.
 */
public class MainActivity extends Activity {
    private static final int REQ_CAMERA = 1;
    private static final int REQ_FRAMES = 2;
    private static final int REQ_EXPORT = 3;
    private static final String USB_ACTION = "dev.lab.openscan.USB_PERMISSION";
    private static final int VENDOR_SONIX = 0x0c45;
    private static final int PRODUCT_A = 0x636a;
    private static final int PRODUCT_B = 0x636b;

    private Engine engine;
    private TextView modeChip;
    private TextView distanceChip;
    private TextView trackChip;
    private TextView modelCaption;
    private ImageView cameraView;
    private ImageView modelView;
    private ImageView camAView;
    private ImageView camBView;
    private ImageView sideView;
    private Button moldBtn;
    private Button measuredBtn;
    private Button scanBtn;
    private int orbitX = 120;
    private int orbitY = 80;
    private float lastOrbitX;
    private float lastOrbitY;
    private volatile int shape = Engine.SHAPE_MOLD;
    private volatile int scanMode = Engine.MODE_STOP;
    private byte[] frameA;
    private byte[] frameB;
    private byte[] raw0;
    private byte[] raw1;
    private boolean fromFiles;
    private boolean assigned;
    private boolean patternIsFirst = true;
    private boolean swapCameras;
    private String exportExt = "stl";
    private volatile String cameraNote = "";
    private volatile boolean alive = true;
    private boolean pushed;
    private boolean usbRegistered;
    private final int[] scanStatus = new int[9];
    private final Object framesLock = new Object();
    private final Object engineLock = new Object();
    private final Object camLock = new Object();
    private final AtomicBoolean frameQueued = new AtomicBoolean();
    private final Handler ui = new Handler();
    private HandlerThread scanThread;
    private Handler scanHandler;
    private HandlerThread camThread;
    private Handler camHandler;
    private int camEpoch;
    private boolean camerasRunning;
    private String running0;
    private String running1;
    private final List<CameraDevice> openDevices = new ArrayList<>();
    private final List<ImageReader> openReaders = new ArrayList<>();
    private volatile boolean scannerRun;
    private Thread scannerThread;
    private String scannerPathA;
    private String scannerPathB;

    private final BroadcastReceiver usbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            if (intent == null || !USB_ACTION.equals(intent.getAction())) return;
            if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) startCameras();
            else note("USB permission denied — use Frames");
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON
                | WindowManager.LayoutParams.FLAG_SHOW_WHEN_LOCKED
                | WindowManager.LayoutParams.FLAG_TURN_SCREEN_ON);
        setShowWhenLocked(true);
        setTurnScreenOn(true);
        setContentView(R.layout.activity_main);
        modeChip = findViewById(R.id.modeChip);
        distanceChip = findViewById(R.id.distanceChip);
        trackChip = findViewById(R.id.trackChip);
        modelCaption = findViewById(R.id.modelCaption);
        cameraView = findViewById(R.id.cameraView);
        modelView = findViewById(R.id.modelView);
        camAView = findViewById(R.id.camA);
        camBView = findViewById(R.id.camB);
        sideView = findViewById(R.id.sideView);
        moldBtn = findViewById(R.id.mold);
        measuredBtn = findViewById(R.id.measured);
        scanBtn = findViewById(R.id.scan);
        for (ImageView card : new ImageView[]{cameraView, modelView, camAView, camBView, sideView}) {
            if (card.getParent() instanceof View) ((View) card.getParent()).setClipToOutline(true);
        }
        modelView.setKeepScreenOn(true);
        swapCameras = OpenScanPrefs.store(this).getBoolean("camera-swap", false);
        engine = Engine.open(this);
        if (engine == null) {
            trackChip.setText("No calibration");
            return;
        }
        scanThread = new HandlerThread("openscan-engine");
        scanThread.start();
        scanHandler = new Handler(scanThread.getLooper());
        applyPrefs();
        moldBtn.setOnClickListener(v -> chooseShape(Engine.SHAPE_MOLD));
        measuredBtn.setOnClickListener(v -> chooseShape(Engine.SHAPE_MEASURED));
        scanBtn.setOnClickListener(v -> mode(scanMode == Engine.MODE_SCAN ? Engine.MODE_PAUSE : Engine.MODE_SCAN));
        findViewById(R.id.stop).setOnClickListener(v -> mode(Engine.MODE_STOP));
        findViewById(R.id.reset).setOnClickListener(v -> reset());
        findViewById(R.id.settings).setOnClickListener(v -> startActivity(new Intent(this, SettingsActivity.class)));
        findViewById(R.id.frames).setOnClickListener(v -> openFrames());
        findViewById(R.id.swap).setOnClickListener(v -> swapCameras());
        findViewById(R.id.export).setOnClickListener(v -> chooseExport());
        findViewById(R.id.view).setOnClickListener(v -> openViewer());
        modelView.setOnTouchListener(this::orbit);
        IntentFilter filter = new IntentFilter(USB_ACTION);
        if (Build.VERSION.SDK_INT >= 33) registerReceiver(usbReceiver, filter, Context.RECEIVER_NOT_EXPORTED);
        else registerReceiver(usbReceiver, filter);
        usbRegistered = true;
        watchCameras();
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED)
            requestPermissions(new String[]{Manifest.permission.CAMERA}, REQ_CAMERA);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (engine == null) return;
        applyPrefs();
        startCameras();
    }

    @Override
    protected void onPause() {
        stopScanner();
        stopCameras();
        super.onPause();
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        setIntent(intent);
        startCameras();
    }

    @Override
    protected void onDestroy() {
        alive = false;
        ui.removeCallbacksAndMessages(null);
        stopScanner();
        stopCameras();
        if (usbRegistered) {
            unregisterReceiver(usbReceiver);
            usbRegistered = false;
        }
        if (scanHandler != null) {
            scanHandler.post(() -> {
                synchronized (engineLock) {
                    if (engine != null) engine.close();
                }
            });
            scanThread.quitSafely();
        } else if (engine != null) {
            engine.close();
        }
        super.onDestroy();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] results) {
        super.onRequestPermissionsResult(requestCode, permissions, results);
        if (requestCode != REQ_CAMERA) return;
        if (results.length > 0 && results[0] == PackageManager.PERMISSION_GRANTED) startCameras();
        else note("Camera permission is off — use Frames");
    }

    private void applyPrefs() {
        postCommand(() -> {
            synchronized (engineLock) {
                if (engine == null) return;
                OpenScanPrefs p = OpenScanPrefs.load(this);
                p.apply(engine);
                if (p.shape != shape) {
                    shape = p.shape;
                    engine.setShape(shape);
                }
                publish();
            }
        });
    }

    private void chooseShape(int next) {
        if (engine == null || next == shape) return;
        OpenScanPrefs.saveShape(this, next);
        postCommand(() -> {
            synchronized (engineLock) {
                shape = next;
                engine.setShape(next);
                OpenScanPrefs.load(this).apply(engine);
                publish();
            }
        });
    }

    private void mode(int m) {
        postCommand(() -> {
            synchronized (engineLock) {
                if (engine == null) return;
                scanMode = m;
                engine.setMode(m);
                pushIfReady();
                publish();
            }
        });
    }

    private void reset() {
        postCommand(() -> {
            synchronized (engineLock) {
                if (engine == null) return;
                engine.reset();
                scanMode = Engine.MODE_STOP;
                OpenScanPrefs p = OpenScanPrefs.load(this);
                p.apply(engine);
                shape = p.shape;
                engine.setShape(shape);
                pushed = false;
                publish();
            }
        });
    }

    private void swapCameras() {
        swapCameras = !swapCameras;
        OpenScanPrefs.store(this).edit().putBoolean("camera-swap", swapCameras).apply();
        synchronized (framesLock) {
            assigned = false;
            if (fromFiles && frameA != null && frameB != null) {
                byte[] tmp = frameA;
                frameA = frameB;
                frameB = tmp;
                assigned = true;
            }
        }
        postFrame();
    }

    private boolean orbit(View v, MotionEvent e) {
        if (engine == null) return false;
        int action = e.getActionMasked();
        int ev = Engine.EV_MOVE;
        if (action == MotionEvent.ACTION_DOWN) {
            lastOrbitX = e.getX();
            lastOrbitY = e.getY();
            orbitX = 120;
            orbitY = 80;
            ev = Engine.EV_DOWN;
        } else if (action == MotionEvent.ACTION_MOVE) {
            orbitX += Math.round(e.getX() - lastOrbitX);
            orbitY += Math.round(e.getY() - lastOrbitY);
            lastOrbitX = e.getX();
            lastOrbitY = e.getY();
        } else if (action == MotionEvent.ACTION_UP || action == MotionEvent.ACTION_CANCEL) {
            ev = Engine.EV_UP;
        } else {
            return true;
        }
        int x = orbitX;
        int y = orbitY;
        int event = ev;
        postCommand(() -> {
            synchronized (engineLock) {
                if (engine == null) return;
                engine.mouse(event, x, y);
                if (event != Engine.EV_UP) publish();
            }
        });
        return true;
    }

    private void openFrames() {
        Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
        intent.addCategory(Intent.CATEGORY_OPENABLE);
        intent.setType("image/*");
        intent.putExtra(Intent.EXTRA_ALLOW_MULTIPLE, true);
        intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
        startActivityForResult(intent, REQ_FRAMES);
    }

    private void chooseExport() {
        postCommand(() -> {
            int n;
            synchronized (engineLock) { n = engine == null ? 0 : engine.points(); }
            ui.post(() -> {
                if (!alive) return;
                if (n < 80) {
                    Toast.makeText(this, "Nothing to export yet. Start a scan first.", Toast.LENGTH_LONG).show();
                    return;
                }
                new AlertDialog.Builder(this)
                        .setTitle("Export")
                        .setItems(new String[]{"STL", "OBJ", "PLY"}, (d, which) -> {
                            exportExt = which == 1 ? "obj" : which == 2 ? "ply" : "stl";
                            Intent intent = new Intent(Intent.ACTION_CREATE_DOCUMENT);
                            intent.addCategory(Intent.CATEGORY_OPENABLE);
                            intent.setType("application/octet-stream");
                            intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION);
                            String modeName = shape == Engine.SHAPE_MEASURED ? "measured" : "mold";
                            intent.putExtra(Intent.EXTRA_TITLE, "openscan-" + modeName + "." + exportExt);
                            startActivityForResult(intent, REQ_EXPORT);
                        })
                        .show();
            });
        });
    }

    private void openViewer() {
        postCommand(() -> {
            int n;
            File tmp = new File(getCacheDir(), "view.stl");
            int tris = -1;
            synchronized (engineLock) {
                n = engine == null ? 0 : engine.points();
                if (n >= 80) tris = engine.write(tmp);
            }
            int written = tris;
            ui.post(() -> {
                if (!alive) return;
                if (n < 80) {
                    startActivity(new Intent(this, ModelActivity.class));
                    return;
                }
                if (written < 0) {
                    Toast.makeText(this, "Could not write the model", Toast.LENGTH_LONG).show();
                    return;
                }
                Intent intent = new Intent(this, ModelActivity.class);
                intent.putExtra(ModelActivity.EXTRA_PATH, tmp.getAbsolutePath());
                startActivity(intent);
            });
        });
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (resultCode != RESULT_OK || data == null) return;
        if (requestCode == REQ_FRAMES) loadFrames(data);
        else if (requestCode == REQ_EXPORT && data.getData() != null) copyExport(data.getData());
    }

    private void loadFrames(Intent data) {
        if (engine == null) return;
        List<Uri> uris = new ArrayList<>();
        if (data.getClipData() != null) {
            for (int i = 0; i < data.getClipData().getItemCount(); i++)
                uris.add(data.getClipData().getItemAt(i).getUri());
        } else if (data.getData() != null) {
            uris.add(data.getData());
        }
        if (uris.isEmpty()) return;
        postCommand(() -> {
            try {
                byte[] a = gray(uris.get(0));
                byte[] b = uris.size() > 1 ? gray(uris.get(1)) : null;
                synchronized (framesLock) {
                    fromFiles = true;
                    assigned = true;
                    frameA = a;
                    if (b != null) frameB = b;
                }
                synchronized (engineLock) {
                    cameraNote = uris.size() > 1
                            ? "Pair loaded. First image is camera A, second is camera B."
                            : "One frame loaded. Pick the pattern and the clean view.";
                    pushIfReady();
                    publish();
                }
            } catch (Exception e) {
                ui.post(() -> {
                    if (alive) Toast.makeText(this, "Could not read the frames", Toast.LENGTH_LONG).show();
                });
            }
        });
    }

    private void copyExport(Uri uri) {
        Toast.makeText(this, "Exporting…", Toast.LENGTH_SHORT).show();
        postCommand(() -> {
            File tmp = new File(getCacheDir(), "export." + exportExt);
            int tris;
            synchronized (engineLock) {
                tris = engine == null ? -1 : engine.write(tmp);
            }
            if (tris < 0) {
                ui.post(() -> {
                    if (alive) Toast.makeText(this, "Could not build the file", Toast.LENGTH_LONG).show();
                });
                return;
            }
            try (InputStream in = new FileInputStream(tmp); OutputStream out = getContentResolver().openOutputStream(uri)) {
                if (out == null) throw new IllegalStateException("uri");
                byte[] buf = new byte[16384];
                int n;
                while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
                String what = tris == 0 ? "point cloud" : tris + " triangles";
                ui.post(() -> {
                    if (alive) Toast.makeText(this, "Saved " + what, Toast.LENGTH_LONG).show();
                });
            } catch (Exception e) {
                ui.post(() -> {
                    if (alive) Toast.makeText(this, "DocumentsUI did not accept the file", Toast.LENGTH_LONG).show();
                });
            }
        });
    }

    private void pushIfReady() {
        byte[] a;
        byte[] b;
        synchronized (framesLock) {
            a = frameA;
            b = frameB;
        }
        if (engine == null || a == null || b == null) return;
        if (a.length != engine.frameW * engine.frameH || b.length != a.length) return;
        int rc = engine.push(a, b, scanStatus);
        pushed = rc == 0;
        if (rc != 0) cameraNote = "Frame size does not match the calibration.";
    }

    private void publish() {
        if (engine == null) return;
        Bitmap camera = engine.panel(Engine.PANEL_CAMERA);
        Bitmap model = engine.panel(Engine.PANEL_MODEL);
        Bitmap a = engine.panel(Engine.PANEL_A);
        Bitmap b = engine.panel(Engine.PANEL_B);
        Bitmap side = engine.panel(Engine.PANEL_SIDE);
        int z = pushed ? scanStatus[8] : 0;
        int deg = pushed ? scanStatus[6] : 0;
        int tracking = pushed ? scanStatus[3] : -1;
        int pts = engine.points();
        ui.post(() -> paint(camera, model, a, b, side, z, deg, tracking, pts));
    }

    private void paint(Bitmap camera, Bitmap model, Bitmap a, Bitmap b, Bitmap side,
                       int z, int deg, int tracking, int pts) {
        if (!alive) {
            recycle(camera);
            recycle(model);
            recycle(a);
            recycle(b);
            recycle(side);
            return;
        }
        setPanel(cameraView, camera);
        setPanel(modelView, model);
        setPanel(camAView, a);
        setPanel(camBView, b);
        setPanel(sideView, side);
        modeChip.setText(shape == Engine.SHAPE_MEASURED ? "Measured" : "Mold");
        distanceChip.setText(z > 0 ? z + " mm" : "—");
        if (cameraNote != null && !cameraNote.isEmpty()) trackChip.setText(cameraNote);
        else if (scanMode == Engine.MODE_SCAN && tracking == 0) trackChip.setText("Tracking lost");
        else if (scanMode == Engine.MODE_SCAN) trackChip.setText(deg + "°  ·  " + pts + " tris");
        else if (scanMode == Engine.MODE_PAUSE) trackChip.setText("Paused");
        else trackChip.setText(pts > 0 ? pts + " tris" : "Ready");
        modelCaption.setText(deg > 0 ? "3D  ·  " + deg + "°  ·  drag to turn" : "3D  ·  drag to turn");
        int idle = 0xFF2A2A30;
        int on = 0xFF3D8BFD;
        moldBtn.setBackgroundTintList(ColorStateList.valueOf(shape == Engine.SHAPE_MOLD ? on : idle));
        measuredBtn.setBackgroundTintList(ColorStateList.valueOf(shape == Engine.SHAPE_MEASURED ? on : idle));
        if (scanMode == Engine.MODE_SCAN) {
            scanBtn.setText("Pause");
            scanBtn.setBackgroundTintList(ColorStateList.valueOf(0xFF2A9D6A));
        } else if (scanMode == Engine.MODE_PAUSE) {
            scanBtn.setText("Resume");
            scanBtn.setBackgroundTintList(ColorStateList.valueOf(0xFF3D8BFD));
        } else {
            scanBtn.setText("Start");
            scanBtn.setBackgroundTintList(ColorStateList.valueOf(0xFF3D8BFD));
        }
    }

    private void setPanel(ImageView view, Bitmap bmp) {
        if (bmp == null) return;
        Bitmap old = null;
        if (view.getDrawable() instanceof BitmapDrawable)
            old = ((BitmapDrawable) view.getDrawable()).getBitmap();
        view.setImageBitmap(bmp);
        if (old != null && old != bmp) old.recycle();
    }

    private static void recycle(Bitmap bmp) {
        if (bmp != null) bmp.recycle();
    }

    private void note(String text) {
        cameraNote = text;
        postCommand(() -> {
            synchronized (engineLock) { publish(); }
        });
    }

    private void postCommand(Runnable r) {
        if (!alive) return;
        if (scanHandler == null) r.run();
        else scanHandler.post(r);
    }

    private void postFrame() {
        if (!alive || scanHandler == null) return;
        if (!frameQueued.compareAndSet(false, true)) return;
        scanHandler.post(() -> {
            frameQueued.set(false);
            synchronized (engineLock) {
                pushIfReady();
                publish();
            }
        });
    }

    /** Capture nodes are sysfs index 0. The kernel names them "KYT Camera A/B: serial". */
    private String[] findScannerNodes() {
        File dir = new File("/sys/class/video4linux");
        File[] kids = dir.listFiles();
        if (kids == null) return null;
        String a = null, b = null;
        for (File kid : kids) {
            if (!kid.getName().startsWith("video")) continue;
            String index = readLine(new File(kid, "index"));
            if (!"0".equals(index)) continue;
            String name = readLine(new File(kid, "name"));
            if (name.startsWith("KYT Camera A:")) a = "/dev/" + kid.getName();
            else if (name.startsWith("KYT Camera B:")) b = "/dev/" + kid.getName();
        }
        if (a == null || b == null) return null;
        return new String[]{a, b};
    }

    private static String readLine(File file) {
        try (BufferedReader r = new BufferedReader(new FileReader(file))) {
            String line = r.readLine();
            return line == null ? "" : line.trim();
        } catch (Exception e) {
            return "";
        }
    }

    private void startScanner(String pathA, String pathB) {
        stopScanner();
        scannerPathA = pathA;
        scannerPathB = pathB;
        scannerRun = true;
        note("Opening the scanner…");
        OpenScanPrefs prefs = OpenScanPrefs.load(this);
        int pixels = engine.frameW * engine.frameH;
        Thread thread = new Thread(() -> {
            long cams = engine.scannerStart(pathA, pathB, prefs.exposureA, prefs.gainA, prefs.exposureB, prefs.gainB);
            if (cams == 0 || !scannerRun) {
                if (cams != 0) engine.scannerClose(cams);
                scannerRun = false;
                note("The scanner is plugged in, but Android blocked the camera devices.");
                return;
            }
            byte[] a = new byte[pixels];
            byte[] b = new byte[pixels];
            int misses = 0;
            int frames = 0;
            while (scannerRun) {
                int rc = engine.scannerGrab(cams, a, b);
                if (!scannerRun) break;
                if (rc != 0) {
                    Log.i("openscan", "scanner grab failed");
                    if (++misses == 3) {
                        ui.post(() -> {
                            if (alive) trackChip.setText("No picture");
                        });
                    }
                    continue;
                }
                misses = 0;
                frames++;
                byte[] ca = a.clone();
                byte[] cb = b.clone();
                synchronized (framesLock) {
                    fromFiles = false;
                    assigned = true;
                    patternIsFirst = true;
                    frameA = ca;
                    frameB = cb;
                }
                if (frames == 1) cameraNote = "";
                postFrame();
            }
            engine.scannerClose(cams);
        }, "openscan-v4l");
        scannerThread = thread;
        thread.start();
    }

    private void stopScanner() {
        scannerRun = false;
        Thread thread = scannerThread;
        scannerThread = null;
        scannerPathA = null;
        scannerPathB = null;
        if (thread != null) {
            try { thread.join(4000); } catch (InterruptedException ignored) {}
        }
    }

    private void askUsb() {
        UsbManager usb = (UsbManager) getSystemService(USB_SERVICE);
        if (usb == null) return;
        int flags = PendingIntent.FLAG_UPDATE_CURRENT;
        if (Build.VERSION.SDK_INT >= 31) flags |= PendingIntent.FLAG_MUTABLE;
        int req = 1;
        for (UsbDevice d : usb.getDeviceList().values()) {
            if (!isScanner(d) || usb.hasPermission(d)) continue;
            Intent intent = new Intent(USB_ACTION);
            intent.setPackage(getPackageName());
            PendingIntent pi = PendingIntent.getBroadcast(this, req++, intent, flags);
            usb.requestPermission(d, pi);
        }
    }

    private boolean scannerPlugged() {
        UsbManager usb = (UsbManager) getSystemService(USB_SERVICE);
        if (usb == null) return false;
        for (UsbDevice d : usb.getDeviceList().values()) if (isScanner(d)) return true;
        return false;
    }

    private static boolean isScanner(UsbDevice d) {
        return d.getVendorId() == VENDOR_SONIX && (d.getProductId() == PRODUCT_A || d.getProductId() == PRODUCT_B);
    }

    private void watchCameras() {
        CameraManager mgr = (CameraManager) getSystemService(CAMERA_SERVICE);
        if (mgr == null) return;
        mgr.registerAvailabilityCallback(new CameraManager.AvailabilityCallback() {
            @Override
            public void onCameraAvailable(String cameraId) {
                if (checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED)
                    startCameras();
            }
        }, ui);
    }

    private void startCameras() {
        if (engine == null || !alive) return;
        String[] nodes = findScannerNodes();
        if (nodes != null) {
            if (scannerRun && nodes[0].equals(scannerPathA) && nodes[1].equals(scannerPathB)) return;
            stopCameras();
            startScanner(nodes[0], nodes[1]);
            return;
        }
        stopScanner();
        if (checkSelfPermission(Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            note("Camera permission is off — use Frames");
            return;
        }
        CameraManager mgr = (CameraManager) getSystemService(CAMERA_SERVICE);
        if (mgr == null) return;
        String[] ext;
        try {
            ext = externalIds(mgr);
        } catch (CameraAccessException e) {
            return;
        }
        if (ext.length < 2) {
            if (camerasRunning) stopCameras();
            note(ext.length == 1
                    ? "Only one external camera is available — use Frames for the pair."
                    : scannerPlugged()
                    ? "Allow USB access to the scanner, or use Frames."
                    : "No scanner — use Frames to open the pattern and the clean view.");
            return;
        }
        if (camerasRunning && ext[0].equals(running0) && ext[1].equals(running1)) return;
        stopCameras();
        final int epoch;
        synchronized (camLock) { epoch = camEpoch; }
        camThread = new HandlerThread("openscan-cam");
        camThread.start();
        camHandler = new Handler(camThread.getLooper());
        running0 = ext[0];
        running1 = ext[1];
        camerasRunning = true;
        cameraNote = "";
        try {
            openOne(mgr, ext[0], true, epoch);
            openOne(mgr, ext[1], false, epoch);
        } catch (CameraAccessException ignored) {
            note("The scanner cameras did not open — use Frames.");
        } catch (SecurityException ignored) {
            note("Camera permission is off — use Frames");
        }
    }

    private static String[] externalIds(CameraManager mgr) throws CameraAccessException {
        List<String> ids = new ArrayList<>();
        for (String id : mgr.getCameraIdList()) {
            Integer facing = mgr.getCameraCharacteristics(id).get(CameraCharacteristics.LENS_FACING);
            if (facing != null && facing == CameraCharacteristics.LENS_FACING_EXTERNAL) ids.add(id);
        }
        return ids.toArray(new String[0]);
    }

    private void openOne(CameraManager mgr, String id, boolean first, int epoch) throws CameraAccessException {
        CameraCharacteristics chars = mgr.getCameraCharacteristics(id);
        Size size = pickSize(chars, engine.frameW, engine.frameH);
        mgr.openCamera(id, new CameraDevice.StateCallback() {
            @Override
            public void onOpened(CameraDevice camera) {
                synchronized (camLock) {
                    if (epoch != camEpoch) {
                        camera.close();
                        return;
                    }
                    openDevices.add(camera);
                }
                ImageReader reader = ImageReader.newInstance(size.getWidth(), size.getHeight(), ImageFormat.YUV_420_888, 2);
                synchronized (camLock) {
                    if (epoch != camEpoch) {
                        openDevices.remove(camera);
                        reader.close();
                        camera.close();
                        return;
                    }
                    openReaders.add(reader);
                }
                reader.setOnImageAvailableListener(r -> onFrame(r, first, epoch), camHandler);
                try {
                    camera.createCaptureSession(Collections.singletonList(reader.getSurface()),
                            new CameraCaptureSession.StateCallback() {
                                @Override
                                public void onConfigured(CameraCaptureSession session) {
                                    if (epoch != camEpoch) return;
                                    try {
                                        applyExposure(camera, chars, session, reader.getSurface(), first);
                                    } catch (Exception e) {
                                        cameraNote = "The scanner camera did not start streaming.";
                                    }
                                }
                                @Override
                                public void onConfigureFailed(CameraCaptureSession session) {
                                    cameraNote = "The scanner camera rejected " + size.getWidth() + "×" + size.getHeight() + ".";
                                }
                            }, camHandler);
                } catch (CameraAccessException e) {
                    cameraNote = "The scanner camera did not start streaming.";
                }
            }
            @Override public void onDisconnected(CameraDevice camera) { camera.close(); }
            @Override public void onError(CameraDevice camera, int error) { camera.close(); }
        }, camHandler);
    }

    private void applyExposure(CameraDevice camera, CameraCharacteristics chars, CameraCaptureSession session,
                               android.view.Surface surface, boolean first) throws CameraAccessException {
        OpenScanPrefs p = OpenScanPrefs.load(this);
        long ns = (first ? p.exposureA : p.exposureB) * 100_000L;
        int iso = 100 + (first ? p.gainA : p.gainB) * 8;
        Range<Long> exp = chars.get(CameraCharacteristics.SENSOR_INFO_EXPOSURE_TIME_RANGE);
        if (exp != null) ns = Math.max(exp.getLower(), Math.min(exp.getUpper(), ns));
        Range<Integer> sens = chars.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
        if (sens != null) iso = Math.max(sens.getLower(), Math.min(sens.getUpper(), iso));
        boolean manual = false;
        int[] modes = chars.get(CameraCharacteristics.CONTROL_AE_AVAILABLE_MODES);
        if (modes != null) {
            for (int m : modes) if (m == CaptureRequest.CONTROL_AE_MODE_OFF) manual = true;
        }
        if (manual) {
            try {
                CaptureRequest.Builder req = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
                req.addTarget(surface);
                req.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
                req.set(CaptureRequest.SENSOR_EXPOSURE_TIME, ns);
                req.set(CaptureRequest.SENSOR_SENSITIVITY, iso);
                session.setRepeatingRequest(req.build(), null, camHandler);
                return;
            } catch (Exception ignored) {
            }
        }
        CaptureRequest.Builder auto = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);
        auto.addTarget(surface);
        auto.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
        session.setRepeatingRequest(auto.build(), null, camHandler);
    }

    private void onFrame(ImageReader reader, boolean first, int epoch) {
        if (epoch != camEpoch) return;
        Image image = reader.acquireLatestImage();
        if (image == null) return;
        byte[] y;
        try {
            y = scale(yPlane(image), image.getWidth(), image.getHeight(), engine.frameW, engine.frameH);
        } finally {
            image.close();
        }
        if (y.length != engine.frameW * engine.frameH) return;
        synchronized (framesLock) {
            if (fromFiles) {
                fromFiles = false;
                assigned = false;
            }
            if (first) raw0 = y;
            else raw1 = y;
            if (raw0 != null && raw1 != null) {
                if (!assigned) {
                    patternIsFirst = contrast(raw0, engine.frameW) >= contrast(raw1, engine.frameW);
                    if (swapCameras) patternIsFirst = !patternIsFirst;
                    assigned = true;
                }
                frameA = patternIsFirst ? raw0 : raw1;
                frameB = patternIsFirst ? raw1 : raw0;
            }
        }
        postFrame();
    }

    private static long contrast(byte[] y, int w) {
        if (w < 16 || y.length < w * 16) return 0;
        int h = y.length / w;
        long s = 0;
        for (int row = h / 4; row < h * 3 / 4; row += 16) {
            int base = row * w;
            for (int x = 16; x < w; x += 8) s += Math.abs((y[base + x] & 255) - (y[base + x - 1] & 255));
        }
        return s;
    }

    private static Size pickSize(CameraCharacteristics chars, int dw, int dh) {
        StreamConfigurationMap map = chars.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
        Size[] sizes = map == null ? null : map.getOutputSizes(ImageFormat.YUV_420_888);
        if (sizes == null || sizes.length == 0) return new Size(dw, dh);
        Size best = sizes[0];
        int bestScore = Integer.MAX_VALUE;
        for (Size s : sizes) {
            if (s.getWidth() == dw && s.getHeight() == dh) return s;
            int score = Math.abs(s.getWidth() - dw) + Math.abs(s.getHeight() - dh);
            if (score < bestScore) {
                bestScore = score;
                best = s;
            }
        }
        return best;
    }

    private static byte[] yPlane(Image image) {
        Image.Plane plane = image.getPlanes()[0];
        ByteBuffer buf = plane.getBuffer();
        int w = image.getWidth();
        int h = image.getHeight();
        int row = plane.getRowStride();
        int pix = plane.getPixelStride();
        byte[] out = new byte[w * h];
        int pos = buf.position();
        for (int y = 0; y < h; y++) {
            int base = pos + y * row;
            for (int x = 0; x < w; x++) {
                int i = base + x * pix;
                if (i >= 0 && i < buf.limit()) out[y * w + x] = buf.get(i);
            }
        }
        return out;
    }

    private void stopCameras() {
        List<CameraDevice> cams;
        List<ImageReader> readers;
        synchronized (camLock) {
            camEpoch++;
            cams = new ArrayList<>(openDevices);
            readers = new ArrayList<>(openReaders);
            openDevices.clear();
            openReaders.clear();
        }
        camerasRunning = false;
        running0 = null;
        running1 = null;
        Handler h = camHandler;
        HandlerThread t = camThread;
        camHandler = null;
        camThread = null;
        if (h != null) {
            h.post(() -> {
                for (CameraDevice c : cams) {
                    try { c.close(); } catch (Exception ignored) {}
                }
                for (ImageReader r : readers) {
                    try { r.close(); } catch (Exception ignored) {}
                }
                if (t != null) t.quitSafely();
            });
        }
    }

    private byte[] gray(Uri uri) throws Exception {
        byte[] data;
        try (InputStream in = getContentResolver().openInputStream(uri)) {
            if (in == null) throw new IllegalArgumentException("uri");
            data = readAll(in);
        }
        Gray pgm = decodePgm(data);
        if (pgm != null) return scale(pgm.y, pgm.w, pgm.h, engine.frameW, engine.frameH);
        BitmapFactory.Options opt = new BitmapFactory.Options();
        opt.inScaled = false;
        Bitmap src = BitmapFactory.decodeByteArray(data, 0, data.length, opt);
        if (src == null) throw new IllegalArgumentException("image");
        Bitmap b = src;
        if (src.getWidth() != engine.frameW || src.getHeight() != engine.frameH) {
            b = Bitmap.createScaledBitmap(src, engine.frameW, engine.frameH, true);
            if (b != src) src.recycle();
        }
        int n = engine.frameW * engine.frameH;
        int[] px = new int[n];
        b.getPixels(px, 0, engine.frameW, 0, 0, engine.frameW, engine.frameH);
        b.recycle();
        byte[] y = new byte[n];
        for (int i = 0; i < n; i++) {
            int c = px[i];
            y[i] = (byte) ((((c >> 16) & 255) * 77 + ((c >> 8) & 255) * 150 + (c & 255) * 29) >> 8);
        }
        return y;
    }

    private static byte[] readAll(InputStream in) throws Exception {
        ByteArrayOutputStream bos = new ByteArrayOutputStream();
        byte[] buf = new byte[16384];
        int n;
        while ((n = in.read(buf)) > 0) bos.write(buf, 0, n);
        return bos.toByteArray();
    }

    private static Gray decodePgm(byte[] data) {
        if (data.length < 3 || data[0] != 'P' || (data[1] != '5' && data[1] != '2')) return null;
        boolean binary = data[1] == '5';
        int i = 2;
        int[] wh = new int[3];
        int got = 0;
        while (got < 3 && i < data.length) {
            while (i < data.length && (data[i] == ' ' || data[i] == '\n' || data[i] == '\r' || data[i] == '\t')) i++;
            if (i < data.length && data[i] == '#') {
                while (i < data.length && data[i] != '\n') i++;
                continue;
            }
            int start = i;
            int v = 0;
            while (i < data.length && data[i] >= '0' && data[i] <= '9') {
                v = v * 10 + (data[i] - '0');
                i++;
            }
            if (i == start) return null;
            wh[got++] = v;
        }
        if (got < 3 || wh[0] < 2 || wh[1] < 2 || wh[2] <= 0 || wh[2] > 255) return null;
        if (i < data.length && (data[i] == ' ' || data[i] == '\n' || data[i] == '\r' || data[i] == '\t')) i++;
        int w = wh[0], h = wh[1];
        byte[] y = new byte[w * h];
        if (binary) {
            if (data.length < i + y.length) return null;
            System.arraycopy(data, i, y, 0, y.length);
        } else {
            int n = 0;
            while (n < y.length && i < data.length) {
                while (i < data.length && (data[i] == ' ' || data[i] == '\n' || data[i] == '\r' || data[i] == '\t')) i++;
                int start = i;
                int v = 0;
                while (i < data.length && data[i] >= '0' && data[i] <= '9') {
                    v = v * 10 + (data[i] - '0');
                    i++;
                }
                if (i == start) return null;
                y[n++] = (byte) v;
            }
            if (n != y.length) return null;
        }
        Gray g = new Gray();
        g.y = y;
        g.w = w;
        g.h = h;
        return g;
    }

    private static byte[] scale(byte[] src, int sw, int sh, int dw, int dh) {
        if (sw == dw && sh == dh) return src;
        if (sw < 2 || sh < 2 || src.length < sw * sh) return src;
        byte[] dst = new byte[dw * dh];
        for (int y = 0; y < dh; y++) {
            int sy = Math.min(sh - 1, y * sh / dh);
            for (int x = 0; x < dw; x++) {
                int sx = Math.min(sw - 1, x * sw / dw);
                dst[y * dw + x] = src[sy * sw + sx];
            }
        }
        return dst;
    }

    private static final class Gray {
        byte[] y;
        int w, h;
    }
}
