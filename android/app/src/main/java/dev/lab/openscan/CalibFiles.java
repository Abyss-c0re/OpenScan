package dev.lab.openscan;

import android.content.Context;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;

/** Local calib/<serial>.txt, then the shipped asset, then the factory servers. */
final class CalibFiles {
    private CalibFiles() {}

    private static final Object IO = new Object();

    private static final String[] HOSTS = {
            "https://sw.3dyunzhan.com/v2/swift/calibration/find",
            "https://swcn.3dyunzhan.com/v2/swift/calibration/find",
    };

    /** File already on the phone. Does not use the network. */
    static File local(Context context, String serial) {
        synchronized (IO) {
            return localLocked(context, serial);
        }
    }

    private static File localLocked(Context context, String serial) {
        serial = CalibName.safeSerial(serial);
        if (serial.isEmpty()) return null;
        File dest = destination(context, serial);
        if (dest == null) return null;
        if (valid(dest, serial)) return dest;
        if (!copyAsset(context, serial, dest)) return null;
        return valid(dest, serial) ? dest : null;
    }

    /**
     * Local file, shipped asset, or a download for this serial only.
     * Returns null when this unit has no factory file. Call off the UI thread.
     */
    static File ensure(Context context, String serial) {
        synchronized (IO) {
            File have = localLocked(context, serial);
            if (have != null) return have;
            serial = CalibName.safeSerial(serial);
            if (serial.isEmpty()) return null;
            File dest = destination(context, serial);
            if (dest == null) return null;
            String text = download(serial);
            if (!CalibName.matchesSerial(text, serial)) return null;
            File part = new File(dest.getParentFile(), serial + ".txt.part");
            try (FileOutputStream out = new FileOutputStream(part)) {
                out.write(text.getBytes(StandardCharsets.UTF_8));
            } catch (Exception e) {
                part.delete();
                return null;
            }
            if (!part.renameTo(dest)) {
                part.delete();
                return null;
            }
            return valid(dest, serial) ? dest : null;
        }
    }

    private static File destination(Context context, String serial) {
        File dir = new File(context.getFilesDir(), "calib");
        if (!dir.exists() && !dir.mkdirs()) return null;
        return new File(dir, serial + ".txt");
    }

    private static boolean copyAsset(Context context, String serial, File dest) {
        try (InputStream in = context.getAssets().open(serial + ".txt");
             FileOutputStream out = new FileOutputStream(dest)) {
            byte[] buf = new byte[8192];
            int n;
            while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
            return true;
        } catch (Exception e) {
            dest.delete();
            return false;
        }
    }

    private static boolean valid(File file, String serial) {
        if (file == null || !file.isFile() || file.length() < 32 || file.length() > 2 * 1024 * 1024) return false;
        try (InputStream in = new FileInputStream(file)) {
            return CalibName.matchesSerial(readLimited(in), serial);
        } catch (Exception e) {
            return false;
        }
    }

    /** No vendor key is shipped. A missing file stays missing. */
    private static String download(String serial) {
        String sign = CalibName.sign(serial, "");
        if (sign.isEmpty()) return null;
        byte[] body = ("{\"sn\":\"" + serial + "\"}").getBytes(StandardCharsets.UTF_8);
        for (String host : HOSTS) {
            String json = post(host, body, serial, sign);
            String url = CalibName.uploadUrl(json);
            if (url == null) continue;
            String text = get(url);
            if (CalibName.matchesSerial(text, serial)) return text;
        }
        return null;
    }

    private static String post(String host, byte[] body, String serial, String sign) {
        HttpURLConnection conn = null;
        try {
            conn = (HttpURLConnection) new URL(host).openConnection();
            conn.setConnectTimeout(20000);
            conn.setReadTimeout(20000);
            conn.setRequestMethod("POST");
            conn.setDoOutput(true);
            conn.setRequestProperty("Content-Type", "application/json;charset=utf-8");
            conn.setRequestProperty("language", "en");
            conn.setRequestProperty("sn", serial);
            conn.setRequestProperty("sign", sign);
            conn.setRequestProperty("User-Agent", "openscan");
            try (java.io.OutputStream out = conn.getOutputStream()) {
                out.write(body);
            }
            int status = conn.getResponseCode();
            InputStream in = status >= 200 && status < 300 ? conn.getInputStream() : conn.getErrorStream();
            if (in == null) return null;
            try (InputStream bodyIn = in) {
                return status >= 200 && status < 300 ? readLimited(bodyIn) : null;
            }
        } catch (Exception e) {
            return null;
        } finally {
            if (conn != null) conn.disconnect();
        }
    }

    private static String get(String url) {
        HttpURLConnection conn = null;
        try {
            conn = (HttpURLConnection) new URL(url).openConnection();
            conn.setConnectTimeout(20000);
            conn.setReadTimeout(20000);
            conn.setInstanceFollowRedirects(true);
            conn.setRequestProperty("User-Agent", "openscan");
            int status = conn.getResponseCode();
            if (status < 200 || status >= 300) return null;
            try (InputStream in = conn.getInputStream()) {
                return readLimited(in);
            }
        } catch (Exception e) {
            return null;
        } finally {
            if (conn != null) conn.disconnect();
        }
    }

    private static String readLimited(InputStream in) throws java.io.IOException {
        ByteArrayOutputStream buf = new ByteArrayOutputStream();
        byte[] tmp = new byte[4096];
        int n;
        int total = 0;
        while ((n = in.read(tmp)) > 0) {
            total += n;
            if (total > 2 * 1024 * 1024) throw new java.io.IOException("calibration response is too large");
            buf.write(tmp, 0, n);
        }
        return buf.toString(StandardCharsets.UTF_8);
    }
}
