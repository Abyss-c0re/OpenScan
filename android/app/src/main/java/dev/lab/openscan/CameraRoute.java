package dev.lab.openscan;

/** Capture path after a V4L open has already failed for this pair. */
final class CameraRoute {
    private CameraRoute() {}

    /**
     * True when these video nodes are the pair that failed to open and the
     * engine already has that unit's serial. Camera2 may then use the same file.
     */
    static boolean useExternal(String serial, String pathA, String pathB,
                               String blockedSerial, String blockedA, String blockedB) {
        if (serial == null || serial.isEmpty() || pathA == null || pathB == null) return false;
        if (blockedSerial == null || blockedA == null || blockedB == null) return false;
        return serial.equals(blockedSerial) && pathA.equals(blockedA) && pathB.equals(blockedB);
    }

    /** Fox camera A is the pattern camera unless the user swapped the pair. */
    static boolean hardwareAisPattern(boolean swapCameras) {
        return !swapCameras;
    }

    /** Button label. "Swapped" stays visible after the saved flag is loaded again. */
    static String swapLabel(boolean swapCameras) {
        return swapCameras ? "Swapped" : "Swap";
    }

    /**
     * Pattern then clean for files opened together.
     * One image has no clean view and must not be scanned with a leftover frame.
     * Swap exchanges the two files. A missing first file is empty.
     */
    static byte[][] filePair(byte[] first, byte[] second, boolean swapped) {
        if (first == null) return new byte[][]{null, null};
        if (second == null) return new byte[][]{first, null};
        if (swapped) return new byte[][]{second, first};
        return new byte[][]{first, second};
    }

    /**
     * Pictures are measured only with a calibration read from a connected scanner.
     * The file opened at launch is for the empty screen, not for someone else's frames.
     */
    static boolean measureFrames(boolean calibFromScanner, int count) {
        return calibFromScanner && count >= 2;
    }

    /**
     * Status line after Frames. A pair names the calibration it will be measured with.
     * One image is not a pair to pick roles from. The shipped file is not a connected unit.
     */
    static String frameLoadNote(int count, boolean swapped, String serial, boolean calibFromScanner) {
        if (count < 2)
            return "One image is not a pair. Open the pattern and the clean view together.";
        if (!calibFromScanner)
            return "These pictures were not measured. Connect the scanner. The shipped file is only for the empty screen.";
        String unit = CalibName.safeSerial(serial);
        if (unit.isEmpty()) unit = "an unknown calibration";
        if (swapped)
            return "Pair loaded with " + unit + ". Swap is on, so the first image is camera B and the second is camera A.";
        return "Pair loaded with " + unit + ". First image is camera A, second is camera B.";
    }

    /**
     * Linux writes this gain, 0..100, to V4L2_CID_GAIN.
     * Camera2 only has ISO. When that range covers 0..100, ask for the same number.
     * Otherwise 0 is the bottom of the ISO range and 100 is the top.
     * A range that merely contains 100, such as 100..1600, is an ISO scale:
     * gain 100 must not become the bottom. No reported range asks for the gain
     * number itself, not a made-up ISO.
     */
    static int sensitivityForGain(int gain, int isoMin, int isoMax, boolean rangeKnown) {
        if (gain < 0) gain = 0;
        if (gain > 100) gain = 100;
        if (!rangeKnown) return gain;
        long lo = Math.min(isoMin, isoMax);
        long hi = Math.max(isoMin, isoMax);
        if (lo <= 0 && hi >= 100) return gain;
        return (int) (lo + (hi - lo) * gain / 100);
    }

    /**
     * True when the camera lists manual exposure (AE off).
     * Without it the saved exposure and gain cannot be applied.
     */
    static boolean manualExposure(int[] modes, int offMode) {
        if (modes == null) return false;
        for (int mode : modes) if (mode == offMode) return true;
        return false;
    }

    /**
     * True when slot 0 of a picked pair is hardware camera A and slot 1 is B.
     * An opaque pair is false: those ids do not say which sensor is which.
     */
    static boolean hardwareOrdered(String[] pair, String pathA, String pathB) {
        if (pair == null || pair.length < 2) return false;
        String numA = videoNumber(pathA);
        String numB = videoNumber(pathB);
        if (numA == null || numB == null || numA.equals(numB)) return false;
        return namesNode(pair[0], pathA, numA) && namesNode(pair[1], pathB, numB);
    }

    /**
     * The first buffer is the pattern image.
     * A known A-then-B pair uses camera A, unless swapped, same as the video devices.
     * An opaque pair uses contrast, then the same swap.
     */
    static boolean patternIsFirst(boolean hardwareOrdered, boolean swapCameras, boolean contrastSaysFirst) {
        if (hardwareOrdered) return hardwareAisPattern(swapCameras);
        return swapCameras ? !contrastSaysFirst : contrastSaysFirst;
    }

    /** A frame is the calibration image only when it is exactly that size. */
    static boolean sameCalibSize(int width, int height, int calibW, int calibH) {
        return calibW >= 2 && calibH >= 2 && width == calibW && height == calibH;
    }

    /**
     * The luminance plane when it is already the calibration size.
     * A different size is rejected. It is never resampled onto the intrinsics.
     */
    static byte[] planeAtCalib(byte[] y, int width, int height, int calibW, int calibH) {
        if (y == null || !sameCalibSize(width, height, calibW, calibH) || y.length != width * height)
            return null;
        return y;
    }

    /**
     * True when the camera lists the calibration size.
     * A nearby size is not usable: stretching it would move the factory intrinsics.
     */
    static boolean offersCalibSize(int[] width, int[] height, int calibW, int calibH) {
        if (width == null || height == null || width.length != height.length) return false;
        for (int i = 0; i < width.length; i++) {
            if (sameCalibSize(width[i], height[i], calibW, calibH)) return true;
        }
        return false;
    }

    /**
     * True when a V4L pair stayed on auto. Same rule as openscan_exposure_auto_pair:
     * 1 means the sliders were not applied. 0 and -1 do not.
     */
    static boolean exposureStayedAuto(int pairResult) {
        return pairResult > 0;
    }

    /**
     * Scan or pause status. When exposure stayed on auto, the sentence is the
     * same one Linux puts on the status line, not a shorter prefix.
     */
    static String trackLine(boolean exposureHeldAuto, String body) {
        if (!exposureHeldAuto) return body;
        return body + "   Exposure stayed on auto. The sliders were not applied.";
    }

    /**
     * Distance chip. Desktop prints this setting as "220 mm away".
     * The frame median is a different number and must not replace it.
     */
    static String distanceLabel(int settingMm) {
        if (settingMm < 100) settingMm = 100;
        if (settingMm > 500) settingMm = 500;
        return settingMm + " mm away";
    }

    /**
     * Near and Far after the same clamp as openscan_build_clamped.
     * Far is at least 40 mm past Near. Returns {near, far}.
     */
    static int[] depthBand(int near, int far) {
        if (near < 60) near = 60;
        if (near > 450) near = 450;
        if (far < near + 40) far = near + 40;
        if (far > 800) far = 800;
        return new int[]{near, far};
    }

    /** Camera2 may open only when both Fox cameras share the serial already loaded. */
    static boolean openExternal(boolean foxPair, String foxSerial, String engineSerial) {
        if (!foxPair || foxSerial == null || foxSerial.isEmpty() || engineSerial == null) return false;
        return foxSerial.equals(engineSerial);
    }

    /**
     * A non-Fox USB device that can appear as a camera.
     * Class 14 is video. Vendor, misc, or per-interface class counts only when
     * the interface has an isochronous IN endpoint, which is how those cameras stream.
     */
    static boolean usbCameraLike(int deviceClass, int interfaceClass, boolean isochronousIn) {
        if (deviceClass == 14 || interfaceClass == 14) return true;
        if (!isochronousIn) return false;
        return interfaceClass == 255 || interfaceClass == 239 || interfaceClass == 0;
    }

    /**
     * Camera ids for Fox A then B.
     * A named video node is used even when another camera is present.
     * Opaque ids are used only when they are the only pair and no other USB
     * camera is plugged in, or they are exactly the two Fox video numbers.
     * Otherwise null: do not guess.
     */
    static String[] pickFoxPair(String[] ids, String pathA, String pathB, boolean extraCamera) {
        if (ids == null || ids.length < 2) return null;
        String numA = videoNumber(pathA);
        String numB = videoNumber(pathB);
        if (numA != null && numB != null && !numA.equals(numB)) {
            String idA = namedVideo(ids, pathA, numA);
            String idB = namedVideo(ids, pathB, numB);
            if (idA != null && idB != null && !idA.equals(idB)) return new String[]{idA, idB};
            if (ids.length == 2 && numA.equals(ids[0]) && numB.equals(ids[1])) return new String[]{numA, numB};
            if (ids.length == 2 && numA.equals(ids[1]) && numB.equals(ids[0])) return new String[]{numA, numB};
            if (!extraCamera && ids.length == 2 && !namesVideo(ids[0]) && !namesVideo(ids[1]))
                return new String[]{ids[0], ids[1]};
            return null;
        }
        if (!extraCamera && ids.length == 2) return new String[]{ids[0], ids[1]};
        return null;
    }

    private static String videoNumber(String path) {
        if (path == null) return null;
        int slash = path.lastIndexOf('/');
        String base = slash >= 0 ? path.substring(slash + 1) : path;
        if (!base.startsWith("video")) return null;
        String n = base.substring("video".length());
        if (n.isEmpty()) return null;
        for (int i = 0; i < n.length(); i++) {
            if (!Character.isDigit(n.charAt(i))) return null;
        }
        return n;
    }

    /** The one id that names this video node, or null when none or several do. */
    private static String namedVideo(String[] ids, String path, String number) {
        String node = "video" + number;
        String hit = null;
        for (String id : ids) {
            if (id == null) continue;
            boolean named = id.equals(path) || id.equals(node) || id.endsWith("/" + node);
            if (!named) continue;
            if (hit != null) return null;
            hit = id;
        }
        return hit;
    }

    private static boolean namesVideo(String id) {
        return id != null && (id.contains("video") || id.contains("/dev/"));
    }

    private static boolean namesNode(String id, String path, String number) {
        if (id == null) return false;
        String node = "video" + number;
        return id.equals(path) || id.equals(node) || id.endsWith("/" + node) || id.equals(number);
    }
}
