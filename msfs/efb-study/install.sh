#!/bin/sh
# Install the Study overlay into an MSFS A380X package.
#
# Purely additive by design: FlyByWire's efb.js is never touched. We add two
# files of our own (study.js, catalogue.json) and append exactly one
# import-script line to efb.html. A FlyByWire update replaces efb.js and
# nothing of ours breaks.
#
#   ./install.sh [package dir]            install
#   ./install.sh [package dir] --remove   remove
set -e
PKG="${1:-/d/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/Community/flybywire-aircraft-a380-842}"
EFB="$PKG/html_ui/Pages/VCockpit/Instruments/A380X/EFB"
HERE="$(cd "$(dirname "$0")" && pwd)"
LINE='<script type="text/html" import-script="/Pages/VCockpit/Instruments/A380X/EFB/study.js" import-async="false"></script>'

[ -d "$EFB" ] || { echo "not an A380X package: $PKG" >&2; exit 1; }

if [ "$2" = "--remove" ]; then
  rm -f "$EFB/study.js" "$EFB/catalogue.json"
  grep -v 'study\.js' "$EFB/efb.html" > "$EFB/efb.html.tmp" && mv "$EFB/efb.html.tmp" "$EFB/efb.html"
  echo "removed"
  exit 0
fi

cp "$HERE/study.js" "$EFB/study.js"
[ -f "$HERE/catalogue.json" ] && cp "$HERE/catalogue.json" "$EFB/catalogue.json"
grep -q 'study\.js' "$EFB/efb.html" || printf '%s\n' "$LINE" >> "$EFB/efb.html"
echo "installed into $EFB"
echo "NOTE: layout.json must list study.js and catalogue.json or MSFS will not serve them."
