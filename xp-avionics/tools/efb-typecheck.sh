#!/bin/sh
WHICH="${1:?usage: efb-typecheck.sh golden|<lib dir> <out>}"
OUT="${2:?usage: efb-typecheck.sh golden|<lib dir> <out>}"
LINK='E:\fbw-int\fbw-xpsdk\node_modules\@microsoft\msfs-sdk'
case "$WHICH" in
  golden) TARGET='D:\fbw-aircraft\node_modules\@microsoft\msfs-sdk' ;;
  *) TARGET="$(cygpath -w "$WHICH")" ;;
esac
. /e/fbw-int/xp-avionics/tools/lock.sh
acquire_lock efb-typecheck
powershell -NoProfile -Command "\$l='$LINK'; if (Test-Path \$l) { (Get-Item \$l).Delete() }; New-Item -ItemType Junction -Path \$l -Target '$TARGET' | Out-Null"
cd /e/fbw-int/fbw-xpsdk
node --max-old-space-size=8192 node_modules/typescript/bin/tsc --noEmit -p fbw-common/src/systems/instruments/src/EFB/tsconfig.json > "$OUT.all" 2>&1
release_lock
grep -E "EFB/(Navigation|Store)/" "$OUT.all" | sed -E 's/\([0-9]+,[0-9]+\)//' | sort -u > "$OUT"
echo "efb-typecheck: $(wc -l < "$OUT") errors in chart users, $(grep -c 'error TS' "$OUT.all") overall"
