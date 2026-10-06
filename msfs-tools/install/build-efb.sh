#!/bin/bash
set -e
CHECKOUT="D:/A380/fbw-xp-worktrees/fs2020-672384b"
LOCKS="D:/A380/msfs-a380/locks"
OUT="D:/A380/msfs-a380/install/out"
IMAGE="ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c"
HTML="$CHECKOUT/fbw-a380x/out/flybywire-aircraft-a380-842/html_ui/Pages/VCockpit/Instruments/A380X"
until mkdir "$LOCKS/docker-mach" 2>/dev/null; do echo "waiting for docker-mach"; sleep 20; done
trap 'rmdir "$LOCKS/docker-mach" 2>/dev/null' EXIT
ENVS="-e NODE_ENV=production -e AIRCRAFT_PROJECT_PREFIX=a380x -e AIRCRAFT_VARIANT=a380-842 -e VITE_BUILD=false -e CLIENT_ID= -e CLIENT_SECRET="
echo "=== tsc EFB"
MSYS_NO_PATHCONV=1 docker run --rm -v "$CHECKOUT:/external" "$IMAGE" bash -c "cd /external && npx tsc --noEmit -p fbw-common/src/systems/instruments/src/EFB/tsconfig.json" 2>&1 | grep -E 'error TS' | grep -v 'vite.config.ts' || echo "no errors outside vite.config.ts"
echo "=== mach EFB"
MSYS_NO_PATHCONV=1 docker run --rm -v "$CHECKOUT:/external" $ENVS "$IMAGE" bash -c "cd /external && npx mach build --config fbw-a380x/mach.config.js --work-in-config-dir --filter '^EFB\$'" 2>&1 | grep -v 'npm notice' | tail -6
mkdir -p "$OUT/EFB"; cp -f "$HTML/EFB/efb.js" "$HTML/EFB/efb.css" "$OUT/EFB/"
ls -la "$OUT/EFB/"
