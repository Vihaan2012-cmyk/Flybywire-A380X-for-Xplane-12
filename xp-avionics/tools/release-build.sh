#!/bin/sh
set -e
OUT="${1:-/e/fbw-int/xpsdk-release}"
. /e/fbw-int/xp-avionics/tools/lock.sh
acquire_lock release
sh /e/fbw-int/xp-avionics/tools/build.sh "$OUT/lib" 2>&1 | tail -3
XP_RELEASE=1 sh /e/fbw-int/xp-avionics/tools/build-fbw.sh "$OUT/lib" "$OUT/ours" all > "$OUT.ours.log" 2>&1
XP_RELEASE=1 sh /e/fbw-int/xp-avionics/tools/build-fbw.sh golden "$OUT/golden" all > "$OUT.golden.log" 2>&1
release_lock
grep -h "Built [0-9]* instruments" "$OUT.ours.log" "$OUT.golden.log"
echo "release-build: $OUT/ours/html_ui and $OUT/golden/html_ui"
