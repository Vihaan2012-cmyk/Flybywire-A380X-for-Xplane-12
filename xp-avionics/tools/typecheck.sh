#!/bin/sh
TAG="${1:?usage: typecheck.sh <tag>}"
OUT=/e/fbw-int/xpsdk-runs/typecheck-$TAG.txt
SNAP=/e/fbw-int/xpsdk-snap-$TAG
LINK='E:\fbw-int\fbw-xpsdk\node_modules\@microsoft\msfs-sdk'
. /e/fbw-int/xp-avionics/tools/lock.sh
acquire_lock "typecheck-$TAG"
sh /e/fbw-int/xp-avionics/tools/build.sh "$SNAP" | tail -3
powershell -NoProfile -Command "\$l='$LINK'; if (Test-Path \$l) { (Get-Item \$l).Delete() }; New-Item -ItemType Junction -Path \$l -Target '$(cygpath -w "$SNAP")' | Out-Null"
cd /e/fbw-int/fbw-xpsdk
: > "$OUT"
for p in fbw-a380x/src/systems/instruments fbw-a380x/src/systems/systems-host fbw-a380x/src/systems/extras-host; do
  echo "== $p" >> "$OUT"
  node --max-old-space-size=8192 node_modules/typescript/bin/tsc --noEmit -p "$p/tsconfig.json" >> "$OUT" 2>&1
done
release_lock
python /e/fbw-int/xp-avionics/tools/tsc_summary.py "$OUT"
