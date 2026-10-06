#!/bin/bash
OUT="E:/test1results"
REPO="D:/A380/fbw-xp-worktrees/fs2020-672384b"
IMAGE="ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c"
SEGMENT_HOURS="${SEGMENT_HOURS:-0.5}"
SHARDS=8

mkdir -p "$OUT"
if [ "${FRESH:-0}" = "1" ]; then rm -rf "$OUT"/run.log "$OUT"/replay-*.log "$OUT"/fws-runway "$OUT"/fws-apron "$OUT"/*.jsonl "$OUT"/SUMMARY.md "$OUT"/ECAM.md "$OUT"/findings.log "$OUT"/no-effect-*.txt "$OUT"/cpu.log "$OUT"/COMPLETE; fi
rm -f "$OUT"/progress.json "$OUT"/replay-status.json
cp -f "D:/A380/msfs-a380/suite/index.html" "$OUT/index.html"
status() { printf '{"stage":"%s","detail":"%s","time":"%s"}' "$1" "$2" "$(date '+%Y-%m-%d %H:%M:%S')" > "$OUT/replay-status.json"; }
log() { echo "[$(date '+%H:%M:%S')] $*" >> "$OUT/run.log"; }

powershell -NoProfile -Command '$t = Add-Type -MemberDefinition "[DllImport(\"kernel32.dll\")] public static extern uint SetThreadExecutionState(uint f);" -Name K -Namespace W -PassThru; while ($true) { $t::SetThreadExecutionState(2147483649) | Out-Null; Start-Sleep 30 }' &
AWAKE=$!
trap 'kill $AWAKE 2>/dev/null' EXIT

EXE="${CAMPAIGN_EXE:-$(ls -t D:/A380/fbw-build/target/release/deps/a380_systems-*.exe | head -1)}"
mkdir -p "$OUT/bin"
cp -f "$EXE" "$OUT/bin/campaign.exe"
log "segmented campaign from $EXE, $SEGMENT_HOURS h segments"

SEGMENT=0
while [ ! -f "$OUT/STOP" ] && [ ! -f "$OUT/COMPLETE" ]; do
  SEGMENT=$((SEGMENT + 1))
  status "campaign" "segment $SEGMENT"
  log "segment $SEGMENT start"
  SUITE_HOURS="$SEGMENT_HOURS" SUITE_THREADS="${SUITE_THREADS:-30}" "$OUT/bin/campaign.exe" --ignored --exact deep_systems::tests_suite::failure_campaign --nocapture >> "$OUT/run.log" 2>&1 &
  PID=$!
  sleep 20
  powershell -NoProfile -Command "Get-Process campaign -ErrorAction SilentlyContinue | ForEach-Object { \$_.PriorityClass = 'AboveNormal' }"
  wait $PID
  log "segment $SEGMENT exit $?"
done
log "campaign finished (STOP or COMPLETE)"

for TAG in runway apron; do
  DIR="$OUT/fws-$TAG"
  [ -f "$DIR/runs.jsonl" ] || continue
  status "ecam replay" "$TAG: $(wc -l < "$DIR/runs.jsonl") runs on $SHARDS containers"
  log "replay $TAG start"
  for K in $(seq 0 $((SHARDS - 1))); do
    MSYS_NO_PATHCONV=1 docker run --rm -v "$REPO:/external" -v "$DIR:/replay" -e FWS_REPLAY_DIR=/replay -e FWS_SHARD="$K/$SHARDS" -e CLIENT_ID= -e CLIENT_SECRET= "$IMAGE" \
      bash -c "cd /external && npx vitest run fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/__tests__/FwsReplay.test.ts" >> "$OUT/replay-$TAG-$K.log" 2>&1 &
  done
  wait $(jobs -p | grep -v "^$AWAKE$") 2>/dev/null
  log "replay $TAG done"
done

status "merging" "ECAM.md"
python "D:/A380/msfs-a380/suite/merge_ecam.py" "$OUT" >> "$OUT/run.log" 2>&1
status "done" "see SUMMARY.md and ECAM.md"
log "all done"
