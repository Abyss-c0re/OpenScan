# Changelog

## 0.6.14

OpenScan is a small C program for one handheld scanner. It is a personal tool, not a maintained product.

- Mold sweeps the camera outline into a solid. Sweep 60 covers 120° from the front. Sweep 80 covers 160°.
- A real shift of the outline turns that solid. A still frame stays where it is.
- Linux draws with X11 and captures with Video4Linux. Windows uses Win32 and Media Foundation. macOS uses Cocoa and AVFoundation.
- No factory calibration is included. Bring your own `calib/<serial>.txt`.
- The window has the settings row, the camera, the 3D view, and the side cameras.
- Export opens the system save dialog. The file name chooses STL, OBJ, or ASCII PLY. Quit still writes `openscan-last.stl`, `.obj`, and `.ply`. `scan -o` follows the file suffix.
- Linux download is one file, `OpenScan-0.6.14-linux-x86_64`. Mark it executable and run it. `OpenScan-0.6.14-x86_64.AppImage` is the same program. Windows download is `OpenScan-0.6.14-windows-x86_64.exe`. Android is `OpenScan-0.6.14.apk`.
- The engine is `libopenscan`. Include `openscan/openscan.h`.
- A still frame does not add a second shell. A turn uses the top of the outline.
