#!/bin/bash
OUT="${CAMPAIGN_DIR:?}"
EXE="${CAMPAIGN_EXE:?}"
STOP_AT="${STOP_AT:?}"
REPLAY_END_AT="${REPLAY_END_AT:?}"
STOP_S=$(date -d "$STOP_AT" +%s)
SUITE="D:/A380/msfs-a380/suite"
export CAMPAIGN_DIR="$OUT"

mkdir -p "$OUT/bin"
cp -f "$SUITE/index.html" "$OUT/index.html"
status() { printf '{"stage":"%s","detail":"%s","time":"%s"}' "$1" "$2" "$(date '+%Y-%m-%d %H:%M:%S')" > "$OUT/replay-status.json"; }
log() { echo "[$(date '+%H:%M:%S')] $*" >> "$OUT/run.log"; }
running() { tasklist //FI "IMAGENAME eq campaign.exe" //NH 2>/dev/null | grep -qi campaign.exe; }

powershell -NoProfile -Command '$t = Add-Type -MemberDefinition "[DllImport(\"kernel32.dll\")] public static extern uint SetThreadExecutionState(uint f);" -Name K -Namespace W -PassThru; while ($true) { $t::SetThreadExecutionState(2147483649) | Out-Null; Start-Sleep 30 }' &
AWAKE=$!
trap 'kill $AWAKE 2>/dev/null' EXIT

[ -f "$OUT/bin/campaign.exe" ] || cp -f "$EXE" "$OUT/bin/campaign.exe"
log "driver 5: $OUT, campaign until $STOP_AT, replay until $REPLAY_END_AT"
while running; do sleep 10; done

SEGMENT=0
while [ ! -f "$OUT/STOP" ] && [ ! -f "$OUT/COMPLETE" ]; do
  LEFT=$(( STOP_S - $(date +%s) ))
  [ "$LEFT" -lt 120 ] && break
  HOURS=$(awk -v l="$LEFT" 'BEGIN { h = l / 3600; if (h > 0.5) h = 0.5; printf "%.4f", h }')
  SEGMENT=$((SEGMENT + 1))
  status "campaign" "segment $SEGMENT"
  log "segment $SEGMENT start ($HOURS h)"
  SUITE_HOURS="$HOURS" SUITE_THREADS="${SUITE_THREADS:-30}" "$OUT/bin/campaign.exe" --ignored --exact deep_systems::tests_suite::failure_campaign --nocapture >> "$OUT/run.log" 2>&1 &
  PID=$!
  sleep 20
  powershell -NoProfile -Command "Get-Process campaign -ErrorAction SilentlyContinue | ForEach-Object { \$_.PriorityClass = 'AboveNormal' }"
  wait $PID
  log "segment $SEGMENT exit $?"
done
log "campaign finished (time, STOP or COMPLETE)"

python - "$OUT" <<'EOF'
import sys
out = sys.argv[1]
rw = [l.strip() for l in open(f'{out}/no-effect-runway.txt', encoding='utf-8') if l.strip()]
ap = set(l.strip() for l in open(f'{out}/no-effect-apron.txt', encoding='utf-8') if l.strip())
open(f'{out}/dead-both.txt', 'w', encoding='utf-8').write('\n'.join(l for l in rw if l in ap) + '\n')
EOF
for STAGE in cruise approach; do
  status "dead recheck" "$STAGE: $(wc -l < "$OUT/dead-both.txt") singles inert on runway and apron"
  log "dead recheck at $STAGE start"
  STAGE="$STAGE" TAG="$STAGE" ITEMS_FILE="$OUT/dead-both.txt" SUITE_THREADS="${SUITE_THREADS:-30}" "$OUT/bin/campaign.exe" --ignored --exact deep_systems::tests_suite::stage_singles --nocapture >> "$OUT/run.log" 2>&1
  log "dead recheck at $STAGE exit $?"
done

status "ecam replay" "starting"
python "$SUITE/stage_tar.py" "E:/fbw-replay-refresh.tar" fbw-a380x/src/systems fbw-common/src >> "$OUT/run.log" 2>&1
MSYS_NO_PATHCONV=1 timeout 900 docker run --rm -v fbw-replay-src:/external -v "E:/fbw-replay-refresh.tar:/stage.tar:ro" "ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c" bash -c 'tar -xf /stage.tar -C /external' >> "$OUT/run.log" 2>&1
log "replay volume refreshed (exit $?)"
REPLAY_PHASES=pilot,families python "$SUITE/replay_queue2.py" "$REPLAY_END_AT" 11 6 >> "$OUT/run.log" 2>&1

status "analysis" "scan, combos, extrapolation"
for TAG in runway apron; do python "$SUITE/scan_singles.py" "$OUT" "$TAG" >> "$OUT/run.log" 2>&1; done
python "$SUITE/classify_combos.py" "$OUT" >> "$OUT/run.log" 2>&1
python "$SUITE/extrapolate.py" "$OUT" > /dev/null 2>> "$OUT/run.log"
status "done" "see SUMMARY.md, ECAM.md, EXTRAPOLATION.md, SCAN-*.md"
log "driver 5 all done"
