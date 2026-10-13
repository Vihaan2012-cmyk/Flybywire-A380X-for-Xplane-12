#!/bin/sh
set -e
WHICH="${1:?usage: build-fbw.sh golden|<our dist dir> <out-dir> [instruments|all]}"
OUT="${2:?usage: build-fbw.sh golden|<our dist dir> <out-dir> [instruments|all]}"
WHAT="${3:-all}"
WT=/e/fbw-int/fbw-xpsdk
LINK='E:\fbw-int\fbw-xpsdk\node_modules\@microsoft\msfs-sdk'
case "$WHICH" in
  golden) TARGET='D:\fbw-aircraft\node_modules\@microsoft\msfs-sdk' ;;
  *) TARGET="$(cygpath -w "$WHICH")" ;;
esac
powershell -NoProfile -Command "\$l='$LINK'; if (Test-Path \$l) { (Get-Item \$l).Delete() }; New-Item -ItemType Junction -Path \$l -Target '$TARGET' | Out-Null; 'build-fbw: link -> ' + (Get-Item \$l).Target"

cd "$WT"
RUN="node /e/fbw-int/xp-avionics/tools/run-clean.cjs"
export NODE_OPTIONS=--max-old-space-size=8192
if [ "$WHAT" = all ]; then
  $RUN fbw-a380x/src/systems/systems-host/build.js || echo "build-fbw: systems-host failed"
  $RUN fbw-a380x/src/systems/extras-host/build.js || echo "build-fbw: extras-host failed"
fi
$RUN node_modules/@synaptic-simulations/mach/dist/index.js build --config fbw-a380x/mach.config.js --work-in-config-dir || echo "build-fbw: mach reported failures (see above)"

rm -rf "$OUT"
mkdir -p "$OUT"
cp -r fbw-a380x/out/flybywire-aircraft-a380-842/html_ui "$OUT/"
echo "build-fbw: bundles in $OUT/html_ui"
