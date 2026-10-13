#!/bin/sh
TAG="${1:?usage: cycle.sh <tag> [screens] [states]   e.g. cycle.sh sub PFD_L,ND_L cold-dark-powered}"
SCREENS="${2:-MFD,EWD,SD,PFD_L,PFD_R,ND_L,ND_R,FCU,ISIS,CLOCK,RTPI,BAT,RMP_1,RMP_2,RMP_3,SYSTEMS_HOST,EXTRAS_HOST}"
STATES="${3:-all}"
OUT=/e/fbw-int/xpsdk-runs/$TAG
SNAP=/e/fbw-int/xpsdk-snap-$TAG
BUNDLES=/e/fbw-int/xpsdk-ours-$TAG
. /e/fbw-int/xp-avionics/tools/lock.sh

acquire_lock "cycle-$TAG"
sh /e/fbw-int/xp-avionics/tools/build.sh "$SNAP" 2>&1 | tail -20
sh /e/fbw-int/xp-avionics/tools/build-fbw.sh "$SNAP" "$BUNDLES" all > "$OUT.build.log" 2>&1
release_lock
grep -E "error|Error|ERROR|Built [0-9]+|failed" "$OUT.build.log" | grep -v "Study/Study\|EFB\] \|OITlegacy\] " | head -20

sh /e/fbw-int/xp-avionics/tools/harness.sh "$BUNDLES/html_ui" "$OUT" "$SCREENS" "$STATES" > "$OUT.harness.log" 2>&1
grep -E "FAILED|[1-9][0-9]* errors," "$OUT.harness.log" | sed 's/^ *//' | cut -c1-220 | head -40
python /e/fbw-int/xp-avionics/tools/compare.py /e/fbw-int/xpsdk-runs/golden-a "$OUT" "$OUT.report.json"
echo "cycle: report $OUT.report.json; renders and logs in $OUT (each screen: .png, .dom.html, .access.json, .console.log); build log $OUT.build.log; harness log $OUT.harness.log"
