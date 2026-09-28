# Using OpenScan

OpenScan scans one object. The window starts idle so you can frame it before anything is kept.

Personal tool, not a maintained product. Tested on one handheld scanner. Bring your own calibration file. None is shipped.

## A scan

1. Plug in the scanner. `./build/openscan devices` prints the serial and the two camera paths when both names belong to one unit.
2. Start `./build/openscan`.
3. Press **Start**. Turn the object. The status line shows triangle count and degrees.
4. Press **Stop** to hold the model. **Reset** drops it.
5. Quit with `q` or Esc. A model is written to `openscan-last.stl` in the current directory.

A front view covers twice the mold sweep. The default sweep is 80, so that front shell is 160° of the object. A later turn is added when the outline moves.

## Command line

```text
openscan
openscan devices
openscan grab -o DIR [--exposure-a N --exposure-b N --gain-a N --gain-b N]
openscan scan --no-window -o FILE.stl --seconds N
openscan turn-test
openscan mesh-test
```

Exposure is in units of 100 microseconds. Gain is 0..100. A useful starting point on the scanner used here is camera A exposure 22 gain 6, and camera B exposure 16 gain 4.

`scan --no-window` writes an STL for the number of seconds you pass.

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
