#!/bin/bash
OUT="E:/test1results"
IMAGE="ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c"
END_AT="${END_AT:?}"
SHARDS="${SHARDS:-30}"
END_S=$(date -d "$END_AT" +%s)

status() { printf '{"stage":"%s","detail":"%s","time":"%s"}' "$1" "$2" "$(date '+%Y-%m-%d %H:%M:%S')" > "$OUT/replay-status.json"; }
log() { echo "[$(date '+%H:%M:%S')] $*" >> "$OUT/run.log"; }

R=$(wc -l < "$OUT/fws-runway/runs.jsonl")
A=$(wc -l < "$OUT/fws-apron/runs.jsonl")
SR=$(awk -v r="$R" -v a="$A" -v s="$SHARDS" 'BEGIN { n = int(s * r / (r + a) + 0.5); if (n < 1) n = 1; if (n > s - 1) n = s - 1; print n }')
SA=$((SHARDS - SR))
log "detached replay: runway $R on $SR, apron $A on $SA, until $END_AT"
for PAIR in "runway:$SR" "apron:$SA"; do
  TAG="${PAIR%%:*}"
  N="${PAIR##*:}"
  DIR="$OUT/fws-$TAG"
  rm -f "$DIR"/fws-*.jsonl "$DIR"/healthy.json
  for K in $(seq 0 $((N - 1))); do
    docker rm -f "fws-$TAG-$K" > /dev/null 2>&1
    MSYS_NO_PATHCONV=1 docker run -d --rm --name "fws-$TAG-$K" -v fbw-replay-src:/external -v "$DIR:/replay" -e FWS_REPLAY_DIR=/replay -e FWS_SHARD="$K/$N" -e CLIENT_ID= -e CLIENT_SECRET= "$IMAGE" \
      bash -c "cd /external && node node_modules/vitest/vitest.mjs run fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/__tests__/FwsReplay.test.ts" >> "$OUT/run.log" 2>&1
  done
done
sleep 20
log "containers running: $(docker ps -q --filter name=fws-runway- --filter name=fws-apron- | wc -l)"

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
log "replay all done: $DONE of $((R + A)) runs replayed"
