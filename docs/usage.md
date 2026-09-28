# Using OpenScan

OpenScan scans one object. The window starts idle so you can frame it before anything is kept.

Personal tool, not a maintained product. Tested on one handheld scanner. Bring your own calibration file. None is shipped.

## A scan

1. Plug in the scanner. `./build/openscan devices` prints the serial and the two camera paths when both names belong to one unit.
2. Start `./build/openscan`. The window has the settings row, the camera on the left, the 3D view in the middle, and stereo A, stereo B, and the side view on the right.
3. Drag a slider to set exposure, gain, distance, near, far, stride, smooth, mold sweep, or mold relief. **Mold** and **Measured** switch the solid. **Auto calibrate** picks an exposure. Drag the 3D view to turn it.
4. Press **Start**. Turn the object. The status line shows triangle count and degrees.
5. Press **Stop** to hold the model. **Pause** holds it and keeps the turn. **Reset** drops it. **Export** writes `openscan-last.stl`.
6. Quit with `q` or Esc. A model is written to `openscan-last.stl` in the current directory.

A front view covers twice the mold sweep. The default sweep is 80, so that front shell is 160° of the object. A later turn is added when the outline moves.

## Command line

```text
openscan
openscan help
openscan devices
openscan grab -o DIR
openscan scan -o FILE.stl --seconds N --shape mold --distance 220 --sweep 80
openscan turn-test
```

Exposure is in units of 100 microseconds. Gain is 0..100. A useful starting point on the scanner used here is camera A exposure 22 gain 6, and camera B exposure 16 gain 4.

`scan -o` writes `.stl`, `.obj`, or `.ply` from the file name. The window's Export button writes all three as `openscan-last` in the current directory and shows that path.

## Calibration

The file for a unit is `calib/<serial>.txt`. Search order:

1. `calib/<serial>.txt` in the current directory, or next to the sources
2. `$XDG_DATA_HOME/openscan/calib/<serial>.txt`, or `~/.local/share/openscan/calib/<serial>.txt`
3. On Windows, `%APPDATA%/openscan/calib/<serial>.txt`
4. On macOS, `~/Library/Application Support/openscan/calib/<serial>.txt`

Both cameras have to carry the same serial. A vendor lookup runs only when you set `OPENSCAN_CALIB_SIGN` or `calib/sign.local` yourself. That key is not in the source tree. The saved mesh does not contain the serial or the calibration text.

## Capture

Both cameras share one USB 2 hub, so the Linux capture is MJPEG at 1280×720. Two uncompressed 720p streams do not fit.

The user running OpenScan must be able to open the camera devices. Group `video` is enough on Linux. The rule in `udev/99-openscan.rules` also creates `/dev/openscan-a` and `/dev/openscan-b`. Install steps are in the [README](../README.md).
