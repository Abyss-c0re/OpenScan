# fox3d

Linux program for the 3DMakerpro Fox scanner (JMM8). It shows both cameras, builds a mesh while you turn the object, and exports that mesh.

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
| udev rule in `udev/99-fox3d.rules` | Optional. Names the nodes `/dev/fox3d-a` and `/dev/fox3d-b` and lets the `video` group open them. |

### Arch and CachyOS

```bash
sudo pacman -S cmake gcc qt6-base opencv curl openssl
```

### What is not required

Wine, the official JMStudio installer, an account, and a browser cookie are not required to build or run fox3d.

## Build

From this directory:

```bash
cmake -S . -B build
cmake --build build
```

The program is `build/fox3d`.

```bash
./build/fox3d mesh-test
./build/fox3d devices
```

`mesh-test` checks the mesher and that STL, OBJ, and PLY output is geometry only. `devices` prints the connected Fox cameras.

## Use

Plug in the Fox and start:

```bash
./build/fox3d
```

The window opens idle.

| Control | Effect |
| --- | --- |
| Start scan | Keep new surface while the turn can be followed. |
| Pause | Hold the model. The turn is still followed so a later Start lands in the right place. |
| Stop | Stop adding, and keep the model so it can be exported. |
| Reset | Drop the model and return to idle. The next Start is a new scan. |
| Export… | Write STL, OBJ, or PLY. The file stays on this computer. |

Turn the object steadily. The status line shows how many degrees are scanned, how many are still open, and **detail xN**. Going over a side again raises that count and updates the surface there instead of replacing it. If the line says **TRACKING LOST**, slow the turn. That frame is not added.

Closing the window quits. It does not reopen.

### Command line

```text
fox3d
fox3d devices
fox3d grab -o DIR [--exposure-a N --exposure-b N --gain-a N --gain-b N]
fox3d scan [--no-window -o FILE --seconds N] [--calib PATH] [--no-ae]
fox3d snap -o FILE.stl [--calib PATH]
fox3d asic-read HEXADDR
fox3d mesh-test
```

`scan -o` and Export choose the format from the file name: `.stl`, `.obj`, or `.ply`.

Exposure is the UVC absolute exposure in units of 100 microseconds. Gain is 0..100. A useful starting point on this Fox is camera A exposure 22 gain 6, camera B exposure 16 gain 4.

## Calibration

Each Fox has its own factory file, `calib/<serial>.txt`. The serial is read from the camera.

Search order:

1. A path passed with `--calib`
2. `calib/<serial>.txt` in the current directory
3. The `calib/` directory shipped with this source tree
4. `$XDG_DATA_HOME/fox3d/calib/<serial>.txt`, or `~/.local/share/fox3d/calib/<serial>.txt`

If none of those exist, fox3d asks the factory server for that unit's file and saves the download under the XDG path above. The calibration file is only downloaded. It is never uploaded.

Exported meshes do not contain the serial number or the calibration text.

## USB names

Install the udev rule if you want stable device names:

```bash
sudo install -m 0644 udev/99-fox3d.rules /etc/udev/rules.d/99-fox3d.rules
sudo udevadm control --reload
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=0c45
sudo udevadm trigger --subsystem-match=video4linux
```

After that, the capture nodes are `/dev/fox3d-a` and `/dev/fox3d-b`. Your user must be in group `video`.

## Layout

| Path | Contents |
| --- | --- |
| `include/fox/` | Public headers: camera, calibration, scan, live session, window. |
| `src/device/` | V4L2 capture. |
| `src/calib/` | Parse the factory file, and download it when it is missing. |
| `src/scan/` | Single-pair stereo snapshot. |
| `src/live/` | Turn tracking, the mesh, and the preview. |
| `src/app/` | Qt window. |
| `calib/` | Factory files kept with the source tree. |
| `udev/` | Device naming rule. |

More of the daily operation is in [docs/usage.md](docs/usage.md).
