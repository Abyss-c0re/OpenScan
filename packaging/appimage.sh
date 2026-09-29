#!/bin/sh
# Write the two Linux downloads:
#   dist/OpenScan-<version>-linux-x86_64
#   dist/OpenScan-<version>-x86_64.AppImage
# The single file has JPEG built in when libjpeg.a is available.
# The AppImage also carries libX11, so it does not need that library installed.
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
if [ ! -f build/CMakeFiles/openscan.dir/link.txt ]; then
    echo "No link line in build/. Build the program first." >&2
    exit 1
fi

jpeg_a=${OPENSCAN_LIBJPEG_A:-}
if [ -n "$jpeg_a" ] && [ ! -f "$jpeg_a" ]; then
    echo "OPENSCAN_LIBJPEG_A is not a file: $jpeg_a" >&2
    exit 1
fi
if [ -z "$jpeg_a" ]; then
    for c in /usr/lib/libjpeg.a /usr/lib64/libjpeg.a /usr/lib/x86_64-linux-gnu/libjpeg.a; do
        if [ -f "$c" ]; then
            jpeg_a=$c
            break
        fi
    done
fi

mkdir -p dist
one="$root/dist/OpenScan-${ver}-linux-x86_64"
rm -f "$root/dist/OpenScan-${ver}-x86_64.tar.gz"

if [ -n "$jpeg_a" ]; then
    link=$(sed 's/-Wl,--dependency-file=[^ ]*//' build/CMakeFiles/openscan.dir/link.txt)
    link=$(printf '%s\n' "$link" | sed "s|-o openscan|-o $one|")
    link=$(printf '%s\n' "$link" | sed -e 's|[^ ]*libjpeg\.so[^ ]*|'"$jpeg_a"'|g' -e 's|-ljpeg|'"$jpeg_a"'|g')
    case "$link" in
        *"$jpeg_a"*) ;;
        *) link="$link $jpeg_a" ;;
    esac
    case "$link" in
        *" -lm"|*" -lm "*) ;;
        *) link="$link -lm" ;;
    esac
    (
        cd "$root/build"
        # Paths in the link line are relative to build/.
        # shellcheck disable=SC2086
        $link
    )
    strip -s "$one" 2>/dev/null || true
    if readelf -d "$one" | grep -q 'libjpeg\.so'; then
        echo "The single file still needs libjpeg.so. The static archive was not used." >&2
        exit 1
    fi
else
    cp -a build/openscan "$one"
    echo "No libjpeg.a found. The single file still needs libjpeg.so.8." >&2
    echo "Set OPENSCAN_LIBJPEG_A to bake JPEG into the file." >&2
fi
chmod +x "$one"

ver_out=$("$one" version)
printf '%s\n' "$ver_out"
case "$ver_out" in
    "OpenScan ${ver} (linux)") ;;
    *)
        echo "Version check failed: $ver_out" >&2
        exit 1
        ;;
esac
"$one" turn-test

rm -rf build/AppDir
mkdir -p build/AppDir/usr/bin build/AppDir/usr/lib \
    build/AppDir/usr/share/applications \
    build/AppDir/usr/share/icons/hicolor/256x256/apps
cp -a "$one" build/AppDir/usr/bin/openscan

if command -v magick >/dev/null 2>&1; then
    magick packaging/openscan.png -depth 8 PNG32:build/AppDir/openscan.png
elif command -v convert >/dev/null 2>&1; then
    convert packaging/openscan.png -depth 8 PNG32:build/AppDir/openscan.png
else
    cp packaging/openscan.png build/AppDir/openscan.png
fi
cp build/AppDir/openscan.png build/AppDir/.DirIcon
cp build/AppDir/openscan.png build/AppDir/usr/share/icons/hicolor/256x256/apps/openscan.png
cp packaging/openscan.desktop build/AppDir/openscan.desktop
printf 'X-AppImage-Version=%s\n' "$ver" >> build/AppDir/openscan.desktop
cp build/AppDir/openscan.desktop build/AppDir/usr/share/applications/openscan.desktop

skip_lib() {
    case $(basename "$1") in
        libc.so*|libm.so*|libdl.so*|libpthread.so*|librt.so*|libresolv.so*|ld-linux*)
            return 0 ;;
    esac
    return 1
}

copy_lib() {
    src=$1
    [ -f "$src" ] || return 0
    skip_lib "$src" && return 0
    base=$(basename "$src")
    dest="build/AppDir/usr/lib/$base"
    if [ -e "$dest" ]; then
        return 0
    fi
    cp -L "$src" "$dest"
    echo "$dest"
}

pass=0
new=1
while [ "$new" -eq 1 ] && [ "$pass" -lt 6 ]; do
    pass=$((pass + 1))
    new=0
    set -- build/AppDir/usr/bin/openscan build/AppDir/usr/lib/*
    for bin in "$@"; do
        [ -f "$bin" ] || continue
        for lib in $(ldd "$bin" | sed -n 's/.*=> \([^ ]*\).*/\1/p; s/^[\t ]\(\/[^ ]*\).*/\1/p'); do
            added=$(copy_lib "$lib" || true)
            if [ -n "$added" ]; then
                new=1
            fi
        done
    done
done

cat > build/AppDir/AppRun << 'EOF'
#!/bin/sh
here=$(CDPATH= cd -- "$(dirname "$0")" && pwd)
export LD_LIBRARY_PATH="$here/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$here/usr/bin/openscan" "$@"
EOF
chmod +x build/AppDir/AppRun

tool=${APPIMAGETOOL:-}
if [ -z "$tool" ]; then
    tool=$(command -v appimagetool || true)
fi
if [ -z "$tool" ] || [ ! -x "$tool" ]; then
    echo "appimagetool was not found. The single file is ready:" >&2
    echo "$one" >&2
    echo "Put appimagetool on PATH, or set APPIMAGETOOL, and run this again." >&2
    exit 1
fi

image="$root/dist/OpenScan-${ver}-x86_64.AppImage"
part="$root/dist/.OpenScan-${ver}-x86_64.AppImage.part"
rm -f "$part"
ARCH=x86_64 APPIMAGE_EXTRACT_AND_RUN=1 "$tool" --no-appstream "$root/build/AppDir" "$part"
chmod +x "$part"
img_out=$(APPIMAGE_EXTRACT_AND_RUN=1 "$part" version || true)
case "$img_out" in
    "OpenScan ${ver} (linux)") ;;
    *)
        echo "AppImage version check failed: $img_out" >&2
        exit 1
        ;;
esac
APPIMAGE_EXTRACT_AND_RUN=1 "$part" turn-test
mv "$part" "$image"
echo "$one"
echo "$image"
du -h "$one" "$image"
