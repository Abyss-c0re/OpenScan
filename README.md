# OpenScan

OpenScan is a scanner program, version 0.6.13. It shows both cameras, follows one object as you move the scanner around it, and exports that mesh.

It is tested on the 3DMakerpro Fox (serial prefix JMM8). Other 3DMakerpro scanners may work when they expose the same pair of USB cameras. Calibration for a serial that is not already on disk is downloaded from 3DMakerpro's servers. Those servers are proprietary. OpenScan only downloads that file. It does not upload a scan or an account.

`openscan --version` prints the version. Releases are git tags `vX.Y.Z`. The notes are in [CHANGELOG.md](CHANGELOG.md).

The cameras are ordinary USB Video Class devices. The kernel driver is `uvcvideo`. This project is the userspace program. It does not install a kernel module.

The software is free of charge and provided as is. See [LICENSE](LICENSE).

## Dependencies

Tested on CachyOS with the package versions below. The same libraries are required on other distributions; package names differ.

### Build tools

| Package | Tested | Role |
| --- | --- | --- |
| CMake | 4.4.3 | Generate the build. Minimum 3.16. |
| GCC | 16.2.1 | C11 for the camera and calibration code, C++17 for the rest. |

### Libraries

| Library | Tested | Role |
| --- | --- | --- |
| Qt 6 Widgets (`qt6-base`) | 6.11.2 | The window: Start, Pause, Stop, Reset, Export. |
| OpenCV 5 `core` | 5.0.0 | Images, matrices. |
| OpenCV 5 `imgproc` | 5.0.0 | Filters, contours, the silhouette. |
| OpenCV 5 `imgcodecs` | 5.0.0 | JPEG decode and preview images. |
| OpenCV 5 `highgui` | 5.0.0 | Headless preview windows when a command asks for one. |
| OpenCV 5 `calib` | 5.0.0 | Camera matrices and stereo rectification. |
| OpenCV 5 `stereo` | 5.0.0 | Stereo matching helpers. |
| OpenCV 5 `geometry` | 5.0.0 | Rodrigues rotations from the calibration file. |
| OpenCV 5 `rgbd` | 5.0.0 | Linked for the depth utilities. The live mesh does not use KinFu. |
| libcurl | 8.22.0 | Download a missing factory calibration file. |
| OpenSSL `libcrypto` | 3.6.4 | Sign that calibration lookup. |

OpenCV's `viz` module is not used. Do not add it: on this machine it pulls VTK libraries that are not installed.

### System

| Requirement | Role |
| --- | --- |
| Linux `uvcvideo` | Both Fox cameras already bind to this kernel driver. |
| Video4Linux2 (`linux/videodev2.h`) | Capture. Headers come with the C library. |
| Membership of group `video` | Open `/dev/video*` for capture. |
| udev rule in `udev/99-openscan.rules` | Optional. Names the nodes `/dev/openscan-a` and `/dev/openscan-b` and lets the `video` group open them. |

### Arch and CachyOS

```bash
sudo pacman -S cmake gcc qt6-base opencv curl openssl
```

### What is not required

Wine, the official JMStudio installer, an account, and a browser cookie are not required to build or run openscan.

## Build

`build/` and `dist/` are generated. They are listed in `.gitignore` and are not part of the source tree.

### Linux program

From this directory:

```bash
cmake -S . -B build
cmake --build build
```

The program is `build/openscan`.

```bash
./build/openscan mesh-test
./build/openscan devices
```

`mesh-test` checks the mesher and that STL, OBJ, and PLY output is geometry only. `devices` prints the connected cameras.

### AppImage

The AppImage is the same Linux program with Qt, OpenCV, curl, and the shipped calibration files packed inside. It still needs a Linux desktop with working OpenGL. Camera access still needs the `video` group and, if you want the stable names, the udev rule above.

Install `qmake6` (from `qt6-base`). Build the program, then pack it:

```bash
cmake -S . -B build
cmake --build build
./packaging/appimage.sh
```

The result is `dist/OpenScan-0.6.13-x86_64.AppImage`. The name follows the version in `CMakeLists.txt`. That file is gitignored.

`packaging/appimage.sh` downloads linuxdeploy into `~/.cache/openscan/` on first use. Two details on a current Arch-family system:

- Qt's `imageformats` directory also holds KDE plugins (`kimg_*.so`). linuxdeploy packs every file in that directory and stops when a plugin's library is not installed. The script points qmake at a plugin tree that keeps Qt's own `libq*.so` image plugins, and it packs the Wayland platform plugin along with `libqxcb.so`.
- The `strip` program shipped inside linuxdeploy cannot read RELR (`.relr.dyn`) sections in current system libraries, so the script sets `NO_STRIP=1`.

Check the packed program without opening the window:

```bash
./dist/OpenScan-0.6.13-x86_64.AppImage --appimage-extract-and-run mesh-test
```

### Android APK

The Android app is `android/`. It compiles the C++ in this tree. It does not contain a second copy of the scanner.

You need JDK 17, the Android SDK, NDK `28.2.13676358`, CMake `3.22.1`, and the OpenCV 4.10 Android SDK. Point `OPENCV_ANDROID_SDK` at the unpacked SDK, or unpack it at `~/Android/OpenCV-android-sdk`. `android/local.properties` is not committed. It only needs:

```text
sdk.dir=/path/to/Android/Sdk
```

From `android/`:

```bash
export JAVA_HOME=/path/to/jdk-17
export OPENCV_ANDROID_SDK=/path/to/OpenCV-android-sdk
./gradlew :app:assembleDebug
```

The APK is `android/app/build/outputs/apk/debug/app-debug.apk`. Copy it to `dist/OpenScan-0.6.13.apk` if you want it next to the AppImage. That directory is gitignored.

Install on a phone with `adb install -r dist/OpenScan-0.6.13.apk`. The application id is `dev.lab.openscan`.

## Use

Plug in the Fox and start:

```bash
./build/openscan
```

The window opens idle.

| Control | Effect |
| --- | --- |
| Start scan | Keep new surface while the turn can be followed. |
| Pause | Hold the model. The turn is still followed so a later Start lands in the right place. |
| Stop | Stop adding, and keep the model so it can be exported. |
| Reset | Drop the model and return to idle. The next Start is a new scan. |
| Export… | Write STL, OBJ, or PLY. The file stays on this computer. |
| Mold | The scanned shape made 8 mm thick, so it is a solid of the measurement. |
| Measured | Builds the model from the projector stripes. Each new view is aligned to what is already scanned and added. A view that does not fit is dropped. |
| Settings | Shows or hides the camera and distance row. That row is open at launch: camera A and B exposure and gain, and distance (100–500 mm). Distance identifies the stripes for both modes. Saved for the next launch. Switching mode clears the model. |

Move around the object, closer, or above it. The tracker follows that object's point cloud and ignores the background. The status line shows how much of it is scanned and **detail xN**. Going over a side again raises that count and updates the surface there. If the line says **TRACKING LOST**, you left the object or moved too fast. That frame is not added.

Closing the window quits. It does not reopen.

### Command line

```text
openscan
openscan devices
openscan grab -o DIR [--exposure-a N --exposure-b N --gain-a N --gain-b N]
openscan scan [--no-window -o FILE --seconds N] [--calib PATH] [--no-ae]
openscan snap -o FILE.stl [--calib PATH]
openscan asic-read HEXADDR
openscan mesh-test
```

`scan -o` and Export choose the format from the file name: `.stl`, `.obj`, or `.ply`.

Exposure is the UVC absolute exposure in units of 100 microseconds. Gain is 0..100. A useful starting point on this Fox is camera A exposure 22 gain 6, camera B exposure 16 gain 4.

## Calibration

Each Fox has its own factory file, `calib/<serial>.txt`. The serial is read from the camera.

Search order:

1. A path passed with `--calib`
2. `calib/<serial>.txt` in the current directory
3. The `calib/` directory shipped with this source tree
4. `$XDG_DATA_HOME/openscan/calib/<serial>.txt`, or `~/.local/share/openscan/calib/<serial>.txt`

If none of those exist, OpenScan asks 3DMakerpro's calibration service for that unit's file and saves the download under the XDG path above. The request goes to `https://sw.3dyunzhan.com/v2/swift/calibration/find`, then `https://swcn.3dyunzhan.com/v2/swift/calibration/find`. Those servers are proprietary. The calibration file is only downloaded. It is never uploaded.

Exported meshes do not contain the serial number or the calibration text.

## USB names

Install the udev rule if you want stable device names:

```bash
sudo install -m 0644 udev/99-openscan.rules /etc/udev/rules.d/99-openscan.rules
sudo udevadm control --reload
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=0c45
sudo udevadm trigger --subsystem-match=video4linux
```

After that, the capture nodes are `/dev/openscan-a` and `/dev/openscan-b`. Your user must be in group `video`.

## Layout

| Path | Contents |
| --- | --- |
| `include/openscan/` | Public headers: camera, calibration, scan, live session, window. |
| `src/device/` | V4L2 capture. |
| `src/calib/` | Parse the factory file, and download it when it is missing. |
| `src/scan/` | Single-pair stereo snapshot. |
| `src/live/` | Turn tracking, the mesh, and the preview. |
| `src/app/` | Qt window. |
| `android/` | Android app. The scan engine is the C++ in this tree, not a second copy. |
| `calib/` | Factory files kept with the source tree. |
| `udev/` | Device naming rule. |

More of the daily operation is in [docs/usage.md](docs/usage.md).
