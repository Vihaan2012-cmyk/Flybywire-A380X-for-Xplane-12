#!/bin/bash
REPO="D:/A380/fbw-xp-worktrees/fs2020-672384b"
IMAGE="ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c"
F="fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/__tests__/FwsReplay.test.ts"
MSYS_NO_PATHCONV=1 docker run -i --rm -v fbw-replay-src:/external "$IMAGE" bash -c "cat > /external/$F && md5sum /external/$F" < "$REPO/$F"
md5sum "$REPO/$F"
