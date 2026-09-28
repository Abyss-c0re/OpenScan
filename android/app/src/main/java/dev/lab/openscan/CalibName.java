package dev.lab.openscan;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;

/** Serial helpers. A vendor lookup key is never stored in the app. */
final class CalibName {
    private CalibName() {}

    /** "KYT Camera A: <serial>_A" -> that serial. Empty when the name is not a scanner. */
    static String serialFromCameraName(String name) {
        if (name == null) return "";
        String tag = null;
        if (name.startsWith("KYT Camera A:")) tag = "KYT Camera A:";
        else if (name.startsWith("KYT Camera B:")) tag = "KYT Camera B:";
        else return "";
        String sn = name.substring(tag.length()).trim();
        if (sn.endsWith("_A") || sn.endsWith("_B")) sn = sn.substring(0, sn.length() - 2);
        return safeSerial(sn);
    }

    /** Serials are file names. 63 fits calib.serial with a NUL, same cap as Linux. */
    static String safeSerial(String serial) {
        if (serial == null || serial.isEmpty() || serial.length() > 63) return "";
        for (int i = 0; i < serial.length(); i++) {
            char c = serial.charAt(i);
            boolean ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')
                    || (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
            if (!ok) return "";
        }
        return serial;
    }

    static boolean looksLikeCalib(String text) {
        if (text == null) return false;
        int i = 0;
        while (i < text.length()) {
            char c = text.charAt(i);
            if (c != ' ' && c != '\n' && c != '\r' && c != '\t') break;
            i++;
        }
        if (i >= text.length() || !Character.isDigit(text.charAt(i))) return false;
        return text.contains("DevID:");
    }

    /** DevID from a factory file, or empty when it is missing or not a serial. */
    static String devId(String text) {
        if (text == null) return "";
        int i = text.indexOf("DevID:");
        if (i < 0) return "";
        i += 6;
        int end = i;
        while (end < text.length()) {
            char c = text.charAt(end);
            if (c == '*' || c == ' ' || c == '\n' || c == '\r' || c == '\t') break;
            end++;
        }
        return safeSerial(text.substring(i, end));
    }

    /** True only when the file is a calib and its DevID is this serial. */
    static boolean matchesSerial(String text, String serial) {
        serial = safeSerial(serial);
        if (serial.isEmpty() || !looksLikeCalib(text)) return false;
        return serial.equals(devId(text));
    }

    /**
     * One serial for a camera pair. Empty unless both sides have a serial
     * and they are the same unit. One labeled camera must not stand in for the other.
     */
    static String sharedSerial(String serialA, String serialB) {
        serialA = safeSerial(serialA);
        serialB = safeSerial(serialB);
        if (serialA.isEmpty() || !serialA.equals(serialB)) return "";
        return serialA;
    }

    /** USB iSerial. Unlike the V4L name, this is already the unit serial. */
    static String usbSerial(String raw) {
        if (raw == null) return "";
        return safeSerial(raw.trim());
    }

    /** Both USB serials, or empty when either side is missing or they differ. */
    static String sharedUsbSerial(String serialA, String serialB) {
        serialA = usbSerial(serialA);
        serialB = usbSerial(serialB);
        if (serialA.isEmpty() || serialB.isEmpty()) return "";
        return sharedSerial(serialA, serialB);
    }

    /** File URL from a calibration/find body. Accepts the same http(s) form as the desktop lookup. */
    static String uploadUrl(String json) {
        if (json == null || !json.contains("\"code\":\"200\"")) return null;
        int key = json.indexOf("\"uploadUrl\"");
        if (key < 0) return null;
        int https = json.indexOf("https://", key);
        int http = json.indexOf("http://", key);
        int at;
        if (https < 0) at = http;
        else if (http < 0) at = https;
        else at = Math.min(https, http);
        if (at < 0) return null;
        int end = json.indexOf('"', at);
        if (end < 0) return null;
        return json.substring(at, end);
    }

    /** MD5(MD5(serial + secret)), lowercase hex. Empty when no local key was supplied. */
    static String sign(String serial, String secret) {
        if (serial == null || secret == null || secret.isEmpty()) return "";
        return md5Hex(md5Hex(serial + secret));
    }

    private static String md5Hex(String s) {
        try {
            byte[] dig = MessageDigest.getInstance("MD5").digest(s.getBytes(StandardCharsets.UTF_8));
            StringBuilder sb = new StringBuilder(dig.length * 2);
            for (byte b : dig) sb.append(String.format("%02x", b & 0xff));
            return sb.toString();
        } catch (Exception e) {
            return "";
        }
    }
}
