# Calibration

Nothing in this directory is shipped as a factory file.

Put your own scanner's calibration at `calib/<serial>.txt`. OpenScan looks
in the current directory, next to the sources, and in
`~/.local/share/openscan/calib/`. It does not include someone else's
calibration, and it does not include a key for a vendor download service.

A lookup key, if you are allowed to use one, stays in `calib/sign.local` or
in the `OPENSCAN_CALIB_SIGN` environment variable. Both are gitignored.
