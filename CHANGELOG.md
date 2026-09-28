# Changelog

## 0.6.14

OpenScan is a small C program for one handheld scanner. It is a personal tool, not a maintained product.

- Mold sweeps the camera outline into a solid. Sweep 60 covers 120° from the front. Sweep 80 covers 160°.
- A real shift of the outline turns that solid. A still frame stays where it is.
- Linux draws with X11 and captures with Video4Linux. Windows uses Win32 and Media Foundation. macOS uses Cocoa and AVFoundation.
- No factory calibration is included. Bring your own `calib/<serial>.txt`.
