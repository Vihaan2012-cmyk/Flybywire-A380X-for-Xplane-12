#!/bin/sh
# Install the plugin, XPHFBW (the companion app) and FlyByWire's instruments
# into the converted aircraft.
# X-Plane must be closed (it locks win.xpl and XPHFBW.exe).
#   tools/install.sh [aircraft folder]
set -e
AIRCRAFT="${1:-/d/Steam Games/steamapps/common/X-Plane 12/Aircraft/FlyByWire A380X}"
# One shared build folder for every build (the user's preference,
# 2026-09-27). The msvc app is built for an explicit --target, so its outputs
# land in their own subfolder: without it, the app's msvc build of the
# plugin library (a dependency of the app) overwrote release/
# fbw_a380_systems.dll -- the gnu plugin with the js feature -- and the
# wrong 11.8 MB DLL was installed as win.xpl (2026-09-27).
TARGET="${CARGO_TARGET_DIR:-/d/A380/fbw-build/target}"
TARGET_APP="${CARGO_TARGET_DIR_APP:-$TARGET}"
APP_OUT="$TARGET_APP/x86_64-pc-windows-msvc/release"
CEF_PATH="${CEF_PATH:-/d/A380/fbw-build/cef}"
# FlyByWire's built instruments. The full build (fonts, images, shared JS and
# every page) comes from the D: checkout; the sibling checkout the plugin's
# own `../fbw-aircraft` path dependency uses -- the integration worktree
# (through its junction) next to E:/fbw-int/plugin -- is laid over it, since
# it rebuilds only the pages it changes. A fixed D: path alone put the older
# D: build back over the integration one on every install (2026-09-27).
BASE_BUILD="${FBW_BASE_HTML_UI:-/d/fbw-aircraft/fbw-a380x/out/flybywire-aircraft-a380-842/html_ui}"
BUILD="${FBW_HTML_UI:-$(cd "$(dirname "$0")/../.." && pwd)/fbw-aircraft/fbw-a380x/out/flybywire-aircraft-a380-842/html_ui}"
PANEL="/d/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/Community/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842/panel"

cd "$(dirname "$0")/.."
/d/A380/fbw-build/backup-plugin.sh
CARGO_TARGET_DIR="$TARGET" cargo +stable-x86_64-pc-windows-gnu build --release --features js
# XPHFBW: the companion app (docs/briefs/xphfbw-app.md), MSVC + CEF, its own
# target dir (a different host toolchain from the plugin above).
( cd app && CEF_PATH="$CEF_PATH" CARGO_TARGET_DIR="$TARGET_APP" cargo +stable-x86_64-pc-windows-msvc build --release --target x86_64-pc-windows-msvc )
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
cp "$APP_OUT/XPHFBW.exe" "$XPHFBW/"
for dll in "$APP_OUT/"*.dll; do
  # The shared build folder also holds the plugin's own DLL; XPHFBW only
  # needs CEF's.
  [ "$(basename "$dll")" = "fbw_a380_systems.dll" ] || cp "$dll" "$XPHFBW/"
done
cp "$APP_OUT/"*.pak "$XPHFBW/"
cp "$APP_OUT/"*.bin "$XPHFBW/"
cp "$APP_OUT/"*.dat "$XPHFBW/"
cp "$APP_OUT/vk_swiftshader_icd.json" "$XPHFBW/"
cp "$APP_OUT/locales/"*.pak "$XPHFBW/locales/"
cp -r app/ui/* "$XPHFBW/ui/"
# Agent F's ported MSFS runtime (app/js), served by the app's own
# xphfbw://runtime/ scheme handler (app/src/scheme.rs) from XPHFBW/js next to
# the exe.
( cd app/js && find . -type f -print ) | while read -r f; do
  mkdir -p "$XPHFBW/js/$(dirname "$f")"
  cp "app/js/$f" "$XPHFBW/js/$f"
done
cp "$PANEL/panel.cfg" "$PANEL/panel.xml" "$AIRCRAFT/panel/"
if [ ! -d "$BASE_BUILD" ]; then
  echo "install.sh: FAIL: FlyByWire's built instruments not found at $BASE_BUILD -- build them first (docs/js-build.md: npm run build-a380x:instruments) before installing." >&2
  exit 1
fi
# FlyByWire's built instruments (GPL-3.0), without the EFB, the OITs'
# superseded legacy page and the popup. The OIT itself (A380X/OIT and
# Images/fbw-a380x/oit) is installed: it is drawn now (docs/oit.md).
# Fonts/fbw-a380x/EFB is installed too, EFB or not: the OIT's own "OIT",
# Inter, Manrope and JetBrains Mono faces load from it (oit.css), and
# without it the OIT fell back to a default font (2026-09-27).
# The base build first, then the sibling build over it (see BUILD above).
for SRC in "$BASE_BUILD" "$BUILD"; do
  [ -d "$SRC" ] || continue
  ( cd "$SRC" && find . -type f \
      ! -path "./Pages/VCockpit/Instruments/A380X/EFB/*" \
      ! -path "./Pages/VCockpit/Instruments/A380X/OITlegacy/*" \
      ! -path "./Pages/VCockpit/Instruments/A380X/popup/*" -print ) | while read -r f; do
    mkdir -p "$AIRCRAFT/html_ui/$(dirname "$f")"
    cp "$SRC/$f" "$AIRCRAFT/html_ui/$f"
  done
  echo "install.sh: FlyByWire instruments from $SRC"
done
# FBW reads its build info from the package root (/VFS/a380x_build_info.json).
for j in "$BASE_BUILD"/../*.json "$BUILD"/../*.json; do
  [ -f "$j" ] && cp "$j" "$AIRCRAFT/"
done
# Verify the merge above actually landed what panel.cfg's htmlgauge entries
# need, instead of trusting a silent copy (2026-09-25: a manual install that
# skipped this whole step left html_ui/Pages and html_ui/JS absent, and
# nothing here said so -- every cockpit screen just went black).
PFD_HTML="$AIRCRAFT/html_ui/Pages/VCockpit/Instruments/A380X/PFD/pfd.html"
if [ ! -f "$PFD_HTML" ]; then
  echo "install.sh: FAIL: $PFD_HTML is missing after the html_ui merge -- every cockpit screen will be BLACK. Is $BUILD a real, current build (not stale/empty)? See docs/js-build.md." >&2
  exit 1
fi
echo "installed into $AIRCRAFT (XPHFBW: $XPHFBW)"
