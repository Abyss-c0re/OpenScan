#!/usr/bin/env bash
# Build LizardTech and install it on the Quest 3.
# OpenXR headers are the system 1.1.60. The Android loader is the Meta one
# already used by quest-depth (meta-openxr-sdk v78).
# The factory calibration is packed into the apk from the local calib file.
# It is not committed.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
QUEST="$(cd "$(dirname "$0")" && pwd)"
SDK="${ANDROID_SDK_ROOT:-${ANDROID_HOME:-$HOME/Android/Sdk}}"
NDK="${ANDROID_NDK_HOME:-$SDK/ndk/28.2.13676358}"
[[ -d "$NDK" ]] || NDK="$SDK/ndk/$(ls "$SDK/ndk" | tail -1)"
BT="$SDK/build-tools/36.1.0"
[[ -x "$BT/aapt2" ]] || BT="$SDK/build-tools/$(ls "$SDK/build-tools" | tail -1)"
PLAT="$SDK/platforms/android-32/android.jar"
[[ -f "$PLAT" ]] || PLAT="$SDK/platforms/$(ls "$SDK/platforms" | tail -1)/android.jar"
LOADER="${LIZARD_OPENXR_LOADER:-$HOME/Dev/VR/gvrmod-quest-host/third_party/oculus_openxr_mobile/OpenXR/Libs/Android/arm64-v8a/Release/libopenxr_loader.so}"
CALIB="${LIZARD_CALIB:-$HOME/.local/share/openscan/calib/JMM8006941.txt}"
S="${SERIAL:-192.168.8.186:5555}"

echo "host self-test"
cmake -S "$QUEST" -B "$QUEST/build-host"
cmake --build "$QUEST/build-host" -j"$(nproc)"
"$QUEST/build-host/lizard_selftest"

[[ -f "$LOADER" ]] || { echo "missing OpenXR loader $LOADER"; exit 1; }
[[ -f "$CALIB" ]] || { echo "missing calibration $CALIB"; exit 1; }

B="$QUEST/build-ndk"
cmake -S "$QUEST/android" -B "$B" \
  -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" \
  -DANDROID_ABI=arm64-v8a \
  -DANDROID_PLATFORM=android-29 \
  -DANDROID_STL=c++_shared \
  -DLIZARD_OPENXR_LOADER="$LOADER"
cmake --build "$B" -j"$(nproc)"
SO="$(find "$B" -name 'liblizard_oxr.so' | head -1)"
[[ -n "$SO" && -f "$SO" ]] || { echo "no liblizard_oxr.so"; exit 1; }

STAGE="$QUEST/build-apk"
rm -rf "$STAGE"
mkdir -p "$STAGE/lib/arm64-v8a" "$STAGE/assets"
cp "$SO" "$STAGE/lib/arm64-v8a/liblizard_oxr.so"
cp "$LOADER" "$STAGE/lib/arm64-v8a/libopenxr_loader.so"
CXX="$NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so"
[[ -f "$CXX" ]] && cp "$CXX" "$STAGE/lib/arm64-v8a/" || true
cp "$CALIB" "$STAGE/assets/calib.txt"

CLASS_DIR="$STAGE/classes"
mkdir -p "$CLASS_DIR"
JAVAC="${JAVAC:-}"
if [[ -z "$JAVAC" ]]; then
  if [[ -x /usr/lib/jvm/java-11-openjdk/bin/javac ]]; then JAVAC=/usr/lib/jvm/java-11-openjdk/bin/javac
  elif [[ -x "$HOME/Android/jdk-21.0.12.1+1/bin/javac" ]]; then JAVAC="$HOME/Android/jdk-21.0.12.1+1/bin/javac"
  else JAVAC="$(command -v javac || true)"; fi
fi
[[ -n "$JAVAC" && -x "$JAVAC" ]] || { echo "javac not found"; exit 1; }
"$JAVAC" --release 8 -classpath "$PLAT" -d "$CLASS_DIR" \
  "$QUEST/android/java/dev/lab/lizard/ExportActivity.java"
mapfile -t CLASS_FILES < <(find "$CLASS_DIR" -name '*.class')
"$BT/d8" --min-api 29 --lib "$PLAT" --output "$STAGE" "${CLASS_FILES[@]}"
[[ -f "$STAGE/classes.dex" ]] || { echo "no classes.dex"; exit 1; }

OUT="$QUEST/build/LizardTech.apk"
mkdir -p "$QUEST/build"
UNALIGNED="$QUEST/build/LizardTech-unaligned.apk"
"$BT/aapt2" link -o "$UNALIGNED" -I "$PLAT" --manifest "$QUEST/android/AndroidManifest.xml"
( cd "$STAGE" && zip -q -r "$UNALIGNED" lib assets classes.dex )
"$BT/zipalign" -f 4 "$UNALIGNED" "$OUT"
"$BT/apksigner" sign --ks "$HOME/.android/debug.keystore" \
  --ks-pass pass:android --key-pass pass:android \
  --ks-key-alias androiddebugkey "$OUT"
echo "APK $OUT"

if [[ "${LIZARD_INSTALL:-1}" == "1" ]]; then
  adb connect "$S" >/dev/null
  adb -s "$S" install -r "$OUT"
  adb -s "$S" shell pm grant dev.lab.lizard android.permission.CAMERA || true
  adb -s "$S" shell pm grant dev.lab.lizard com.oculus.permission.HAND_TRACKING || true
  adb -s "$S" shell pm grant dev.lab.lizard horizonos.permission.USB_CAMERA || true
  adb -s "$S" shell pm grant dev.lab.lizard horizonos.permission.HEADSET_CAMERA || true
  adb -s "$S" shell pm grant dev.lab.lizard android.permission.READ_EXTERNAL_STORAGE || true
  adb -s "$S" shell pm grant dev.lab.lizard android.permission.WRITE_EXTERNAL_STORAGE || true
  adb -s "$S" shell cmd usb grant-device-permission dev.lab.lizard 3141:25450 || true
  adb -s "$S" shell cmd usb grant-device-permission dev.lab.lizard 3141:25451 || true
  adb -s "$S" shell am force-stop dev.lab.lizard || true
  adb -s "$S" shell am start -n dev.lab.lizard/android.app.NativeActivity
  echo "started on $S"
fi
