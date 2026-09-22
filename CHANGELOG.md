# Changelog

Versions follow [semantic versioning](https://semver.org). The number in the window title, `fox3d --version`, and the git tag `vX.Y.Z` are the same version. Nothing before 0.5.0 was tagged.

## 0.5.0

- The pictures are undistorted with the factory calibration. The side panels are the rectified stereo pair, so a feature sits on the same row in both cameras.
- The 3D view uses the camera's gray instead of a tinted clay color, so the texture can be compared with the picture.
- Distance scales the model on screen and in the exported file. Settings for exposure, gain, and distance stay open and are saved.
- Turn tracking follows the face both ways. A frame that does not match is dropped.

## Earlier development

The commits before this version added the USB cameras, the live window, silhouette meshing, turn tracking, factory calibration download, STL/OBJ/PLY export, and the CMake build. See `git log` for that history.
