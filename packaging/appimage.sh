#!/bin/sh
# Pack dist/OpenScan-<version>-x86_64.AppImage from build/openscan.
# Qt, OpenCV, curl, and calib/ are copied into the image.
set -eu
root=$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)
cd "$root"

ver=$(sed -n 's/^project(openscan VERSION \([0-9.]*\).*/\1/p' CMakeLists.txt)
if [ -z "$ver" ]; then
    echo "Could not read the version from CMakeLists.txt" >&2
    exit 1
fi
if [ ! -x build/openscan ]; then
    echo "Build the program first: cmake -S . -B build && cmake --build build" >&2
    exit 1
fi
qmake6=$(command -v qmake6 || true)
if [ -z "$qmake6" ]; then
    echo "qmake6 is required (package qt6-base)" >&2
    exit 1
fi

cache=${XDG_CACHE_HOME:-$HOME/.cache}/openscan
mkdir -p "$cache" dist
deploy="$cache/linuxdeploy-x86_64.AppImage"
plugin="$cache/linuxdeploy-plugin-qt-x86_64.AppImage"
if [ ! -x "$deploy" ] || [ ! -x "$plugin" ]; then
    curl -fsSL -o "$deploy" \
        https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage
    curl -fsSL -o "$plugin" \
        https://github.com/linuxdeploy/linuxdeploy-plugin-qt/releases/download/continuous/linuxdeploy-plugin-qt-x86_64.AppImage
    chmod +x "$deploy" "$plugin"
fi

# KDE installs kimg_*.so into Qt's imageformats directory. linuxdeploy packs
# every file there and stops when a plugin's library is missing. Qt's own
# image plugins are the libq*.so files.
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/plugins/imageformats"
qt_plugins=$("$qmake6" -query QT_INSTALL_PLUGINS | tr -d '\n')
for d in "$qt_plugins"/*; do
    [ -e "$d" ] || continue
    name=$(basename "$d")
    if [ "$name" = imageformats ]; then
        find "$d" -maxdepth 1 -name 'libq*.so' -exec ln -s {} "$work/plugins/imageformats/" \;
    else
        ln -s "$d" "$work/plugins/$name"
    fi
done
cat > "$work/qmake" << EOF
#!/bin/sh
if [ "\$1" = "-query" ]; then
    "$qmake6" -query | sed "s|^QT_INSTALL_PLUGINS:.*|QT_INSTALL_PLUGINS:$work/plugins|"
    exit \$?
fi
exec "$qmake6" "\$@"
EOF
chmod +x "$work/qmake"

rm -rf build/AppDir
mkdir -p build/AppDir/usr/share/openscan/calib
cp -a calib/. build/AppDir/usr/share/openscan/calib/

out="$root/dist/OpenScan-${ver}-x86_64.AppImage"
rm -f "$out"
# linuxdeploy's bundled strip cannot read RELR (.relr.dyn) in current
# system libraries. Those libraries are already stripped.
PATH="$cache:$PATH" \
QMAKE="$work/qmake" \
EXTRA_PLATFORM_PLUGINS=libqwayland.so \
NO_STRIP=1 \
ARCH=x86_64 \
LDAI_OUTPUT="$out" \
OUTPUT="$out" \
"$deploy" --appimage-extract-and-run \
    --appdir build/AppDir \
    --executable build/openscan \
    --desktop-file packaging/openscan.desktop \
    --icon-file packaging/openscan.png \
    --plugin qt \
    --output appimage

echo "$out"
