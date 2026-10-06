#!/bin/bash
OUT="E:/test1results"
REPO="D:/A380/fbw-xp-worktrees/fs2020-672384b"
IMAGE="ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c"
STOP_AT="${STOP_AT:?}"
END_AT="${END_AT:?}"
SHARDS="${SHARDS:-24}"
STOP_S=$(date -d "$STOP_AT" +%s)
END_S=$(date -d "$END_AT" +%s)

status() { printf '{"stage":"%s","detail":"%s","time":"%s"}' "$1" "$2" "$(date '+%Y-%m-%d %H:%M:%S')" > "$OUT/replay-status.json"; }
log() { echo "[$(date '+%H:%M:%S')] $*" >> "$OUT/run.log"; }
running() { tasklist //FI "IMAGENAME eq campaign.exe" //NH 2>/dev/null | grep -qi campaign.exe; }

powershell -NoProfile -Command '$t = Add-Type -MemberDefinition "[DllImport(\"kernel32.dll\")] public static extern uint SetThreadExecutionState(uint f);" -Name K -Namespace W -PassThru; while ($true) { $t::SetThreadExecutionState(2147483649) | Out-Null; Start-Sleep 30 }' &
AWAKE=$!
trap 'kill $AWAKE 2>/dev/null' EXIT

log "driver 3: campaign until $STOP_AT, ECAM replay on $SHARDS containers until $END_AT"
while running; do
  [ "$(date +%s)" -ge "$STOP_S" ] && touch "$OUT/STOP"
  sleep 10
done

SEGMENT=0
while [ ! -f "$OUT/STOP" ] && [ ! -f "$OUT/COMPLETE" ]; do
  LEFT=$(( STOP_S - $(date +%s) ))
  [ "$LEFT" -lt 120 ] && break
  HOURS=$(awk -v l="$LEFT" 'BEGIN { h = l / 3600; if (h > 0.5) h = 0.5; printf "%.4f", h }')
  SEGMENT=$((SEGMENT + 1))
  status "campaign" "driver 3 segment $SEGMENT"
  log "driver 3 segment $SEGMENT start ($HOURS h)"
  SUITE_HOURS="$HOURS" SUITE_THREADS="${SUITE_THREADS:-30}" "$OUT/bin/campaign.exe" --ignored --exact deep_systems::tests_suite::failure_campaign --nocapture >> "$OUT/run.log" 2>&1 &
  PID=$!
  sleep 20
  powershell -NoProfile -Command "Get-Process campaign -ErrorAction SilentlyContinue | ForEach-Object { \$_.PriorityClass = 'AboveNormal' }"
  wait $PID
  log "driver 3 segment $SEGMENT exit $?"
done
log "campaign finished (time, STOP or COMPLETE)"

R=$(wc -l < "$OUT/fws-runway/runs.jsonl")
A=$(wc -l < "$OUT/fws-apron/runs.jsonl")
SR=$(awk -v r="$R" -v a="$A" -v s="$SHARDS" 'BEGIN { n = int(s * r / (r + a) + 0.5); if (n < 1) n = 1; if (n > s - 1) n = s - 1; print n }')
SA=$((SHARDS - SR))
status "ecam replay" "runway $R runs on $SR containers, apron $A runs on $SA containers"
log "replay start: runway $R on $SR, apron $A on $SA"
for PAIR in "runway:$SR" "apron:$SA"; do
  TAG="${PAIR%%:*}"
  N="${PAIR##*:}"
  DIR="$OUT/fws-$TAG"
  rm -f "$DIR"/fws-*.jsonl
  for K in $(seq 0 $((N - 1))); do
    MSYS_NO_PATHCONV=1 docker run --rm --name "fws-$TAG-$K" -v "$REPO:/external" -v "$DIR:/replay" -e FWS_REPLAY_DIR=/replay -e FWS_SHARD="$K/$N" -e CLIENT_ID= -e CLIENT_SECRET= "$IMAGE" \
      bash -c "cd /external && npx vitest run fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/__tests__/FwsReplay.test.ts" >> "$OUT/replay-$TAG-$K.log" 2>&1 &
  done
done

while [ -n "$(docker ps -q --filter name=fws-runway- --filter name=fws-apron-)" ]; do
  if [ "$(date +%s)" -ge $((END_S - 90)) ]; then
    log "replay time budget reached, stopping containers"
    docker ps -q --filter name=fws-runway- --filter name=fws-apron- | xargs -r docker kill > /dev/null
    break
  fi
  DONE=$(cat "$OUT"/fws-runway/fws-*.jsonl "$OUT"/fws-apron/fws-*.jsonl 2>/dev/null | wc -l)
  status "ecam replay" "$DONE of $((R + A)) runs replayed"
  sleep 15
done
sleep 5

status "merging" "ECAM.md"
python "D:/A380/msfs-a380/suite/merge_ecam.py" "$OUT" >> "$OUT/run.log" 2>&1
DONE=$(cat "$OUT"/fws-runway/fws-*.jsonl "$OUT"/fws-apron/fws-*.jsonl 2>/dev/null | wc -l)
status "done" "$DONE of $((R + A)) runs replayed; see SUMMARY.md and ECAM.md"
log "all done: $DONE of $((R + A)) runs replayed"
