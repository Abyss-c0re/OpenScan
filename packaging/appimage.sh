#!/bin/sh
# Pack dist/OpenScan-<version>-x86_64.tar.gz
# The C binary and the shared libraries it actually links. Nothing else.
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

rm -rf build/AppDir
mkdir -p build/AppDir/usr/bin build/AppDir/usr/lib build/AppDir/usr/share/openscan/calib
cp -a build/openscan build/AppDir/usr/bin/openscan
if [ -f calib/README.md ]; then
    cp calib/README.md build/AppDir/usr/share/openscan/calib/README.md
fi
cp packaging/openscan.desktop build/AppDir/openscan.desktop
cp packaging/openscan.png build/AppDir/openscan.png

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

mkdir -p dist
out="$root/dist/OpenScan-${ver}-x86_64.tar.gz"
tar -C build/AppDir -czf "$out" .
echo "$out"
du -h "$out" | awk '{print $1}'
