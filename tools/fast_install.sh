#!/bin/sh
# Rebuild and copy only the plugin and XPHFBW binaries (and the app's settings
# page); tools/install.sh does the full install with FlyByWire's instruments.
set -e
AIRCRAFT="${1:-/d/Steam Games/steamapps/common/X-Plane 12/Aircraft/A380 for X-Plane 12}"
TARGET="${CARGO_TARGET_DIR:-/d/A380/fbw-build/target}"
APP_OUT="$TARGET/x86_64-pc-windows-msvc/release"
cd "$(dirname "$0")/.."
CARGO_TARGET_DIR="$TARGET" cargo +stable-x86_64-pc-windows-gnu build --release --features js
( cd app && CEF_PATH="${CEF_PATH:-/d/A380/fbw-build/cef}" CARGO_TARGET_DIR="$TARGET" cargo +stable-x86_64-pc-windows-msvc build --release --target x86_64-pc-windows-msvc )
cp "$TARGET/release/fbw_a380_systems.dll" "$AIRCRAFT/plugins/fbw_a380_systems/64/win.xpl"
cp "$TARGET/release/fbw_a380_systems_server.exe" "$AIRCRAFT/plugins/fbw_a380_systems/64/fbw_a380_systems_server.exe"
cp "$APP_OUT/XPHFBW.exe" "$AIRCRAFT/plugins/fbw_a380_systems/XPHFBW/"
cp -r app/ui/* "$AIRCRAFT/plugins/fbw_a380_systems/XPHFBW/ui/"
echo "fast install into $AIRCRAFT"
