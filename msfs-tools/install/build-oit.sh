#!/bin/bash
set -e
CHECKOUT="D:/A380/fbw-xp-worktrees/fs2020-672384b"
LOCKS="D:/A380/msfs-a380/locks"
OUT="D:/A380/msfs-a380/install/out"
IMAGE="ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c"
HTML="$CHECKOUT/fbw-a380x/out/flybywire-aircraft-a380-842/html_ui/Pages/VCockpit/Instruments/A380X"
until mkdir "$LOCKS/docker-mach" 2>/dev/null; do echo "waiting for docker-mach"; sleep 15; done
trap 'rmdir "$LOCKS/docker-mach" 2>/dev/null' EXIT
echo "=== tsc OIT"
MSYS_NO_PATHCONV=1 docker run --rm -v "$CHECKOUT:/external" "$IMAGE" bash -c "cd /external && npx tsc --noEmit -p fbw-a380x/src/systems/instruments/src/OIT/tsconfig.json" 2>&1 | grep 'error TS' || echo "tsc: no errors"
echo "=== mach OIT"
MSYS_NO_PATHCONV=1 docker run --rm -v "$CHECKOUT:/external" -e NODE_ENV=production -e AIRCRAFT_PROJECT_PREFIX=a380x -e AIRCRAFT_VARIANT=a380-842 -e VITE_BUILD=false -e CLIENT_ID= -e CLIENT_SECRET= "$IMAGE" bash -c "cd /external && npx mach build --config fbw-a380x/mach.config.js --work-in-config-dir --filter '^OIT\$'" 2>&1 | grep -v 'npm notice' | tail -4
mkdir -p "$OUT/OIT"; cp -f "$HTML/OIT"/* "$OUT/OIT/"; ls -la "$OUT/OIT/"
