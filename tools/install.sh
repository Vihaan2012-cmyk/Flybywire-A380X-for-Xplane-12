#!/bin/sh
# Install the plugin, XPHFBW (the companion app) and FlyByWire's instruments
# into the converted aircraft.
# X-Plane must be closed (it locks win.xpl and XPHFBW.exe).
#   tools/install.sh [aircraft folder]
set -e
AIRCRAFT="${1:-/d/Steam Games/steamapps/common/X-Plane 12/Aircraft/FlyByWire A380X}"
TARGET="${CARGO_TARGET_DIR:-/d/fbw-build/target-main}"
TARGET_APP="${CARGO_TARGET_DIR_APP:-/d/fbw-build/target-app}"
CEF_PATH="${CEF_PATH:-/d/fbw-build/cef}"
BUILD=/d/fbw-aircraft/fbw-a380x/out/flybywire-aircraft-a380-842/html_ui
PANEL="/d/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/Community/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842/panel"

cd "$(dirname "$0")/.."
/d/fbw-build/backup-plugin.sh
CARGO_TARGET_DIR="$TARGET" cargo +stable-x86_64-pc-windows-gnu build --release --features js
# XPHFBW: the companion app (docs/briefs/xphfbw-app.md), MSVC + CEF, its own
# target dir (a different host toolchain from the plugin above).
( cd app && CEF_PATH="$CEF_PATH" CARGO_TARGET_DIR="$TARGET_APP" cargo +stable-x86_64-pc-windows-msvc build --release )
mkdir -p "$AIRCRAFT/plugins/fbw_a380_systems/64" "$AIRCRAFT/panel" "$AIRCRAFT/html_ui"
cp "$TARGET/release/fbw_a380_systems.dll" "$AIRCRAFT/plugins/fbw_a380_systems/64/win.xpl"
# FlyByWire's systems in their own process (src/remote); the plugin falls
# back to running them itself without it.
cp "$TARGET/release/fbw_a380_systems_server.exe" "$AIRCRAFT/plugins/fbw_a380_systems/64/fbw_a380_systems_server.exe"
# XPHFBW.exe, its CEF runtime and settings UI, next to the plugin so it can
# find it (src/lib.rs start_systems -> js_bridge::plugin_dir()/XPHFBW), the
# plugin's first choice before fbw_a380_systems_server.exe and running the
# systems in-process.
XPHFBW="$AIRCRAFT/plugins/fbw_a380_systems/XPHFBW"
mkdir -p "$XPHFBW/ui" "$XPHFBW/locales" "$XPHFBW/js"
cp "$TARGET_APP/release/XPHFBW.exe" "$XPHFBW/"
cp "$TARGET_APP/release/"*.dll "$XPHFBW/"
cp "$TARGET_APP/release/"*.pak "$XPHFBW/"
cp "$TARGET_APP/release/"*.bin "$XPHFBW/"
cp "$TARGET_APP/release/"*.dat "$XPHFBW/"
cp "$TARGET_APP/release/vk_swiftshader_icd.json" "$XPHFBW/"
cp "$TARGET_APP/release/locales/"*.pak "$XPHFBW/locales/"
cp -r app/ui/* "$XPHFBW/ui/"
# Agent F's ported MSFS runtime (app/js), served by the app's own
# xphfbw://runtime/ scheme handler (app/src/scheme.rs) from XPHFBW/js next to
# the exe.
( cd app/js && find . -type f -print ) | while read -r f; do
  mkdir -p "$XPHFBW/js/$(dirname "$f")"
  cp "app/js/$f" "$XPHFBW/js/$f"
done
cp "$PANEL/panel.cfg" "$PANEL/panel.xml" "$AIRCRAFT/panel/"
# FlyByWire's built instruments (GPL-3.0), without the EFB, OITs and popup.
( cd "$BUILD" && find . -type f \
    ! -path "./Pages/VCockpit/Instruments/A380X/EFB/*" \
    ! -path "./Pages/VCockpit/Instruments/A380X/OIT/*" \
    ! -path "./Pages/VCockpit/Instruments/A380X/OITlegacy/*" \
    ! -path "./Pages/VCockpit/Instruments/A380X/popup/*" \
    ! -path "./Images/fbw-a380x/oit/*" \
    ! -path "./Fonts/fbw-a380x/EFB/*" -print ) | while read -r f; do
  mkdir -p "$AIRCRAFT/html_ui/$(dirname "$f")"
  cp "$BUILD/$f" "$AIRCRAFT/html_ui/$f"
done
# FBW reads its build info from the package root (/VFS/a380x_build_info.json).
for j in "$BUILD"/../*.json; do
  [ -f "$j" ] && cp "$j" "$AIRCRAFT/"
done
echo "installed into $AIRCRAFT (XPHFBW: $XPHFBW)"
