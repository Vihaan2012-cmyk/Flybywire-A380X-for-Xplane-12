#!/bin/bash
set -e

CHECKOUT="D:/A380/fbw-xp-worktrees/fs2020-672384b"
LOCKS="D:/A380/msfs-a380/locks"
OUT="D:/A380/msfs-a380/install/out"
IMAGE="ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c"
TARGET="D:/A380/fbw-build/target"   # the one shared CARGO_TARGET_DIR (see memory: one-build-folder)
A380X_PANEL_OUT="$CHECKOUT/fbw-a380x/out/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842/panel"
A380X_HTML_OUT="$CHECKOUT/fbw-a380x/out/flybywire-aircraft-a380-842/html_ui/Pages/VCockpit/Instruments/A380X"

mkdir -p "$OUT"

take_lock() {
    local name="$1"
    local tries=0
    while ! mkdir "$LOCKS/$name" 2>/dev/null; do
        tries=$((tries+1))
        if [ "$tries" -ge 40 ]; then
            echo "could not take lock $name after 40 tries (~20 min) -- another agent still has it" >&2
            return 1
        fi
        echo "lock $name held, waiting..."
        sleep 30
    done
}
drop_lock() {
    rmdir "$LOCKS/$1" 2>/dev/null || true
}

build_systems() {
    take_lock "docker-cargo-systems" || return 1
    echo "=== building systems.wasm ==="
    MSYS_NO_PATHCONV=1 docker run --rm \
        -v "$CHECKOUT:/external" \
        -v "$TARGET:/wasmtarget" \
        -v "$OUT:/wasmout" \
        -e CARGO_TARGET_DIR=/wasmtarget \
        "$IMAGE" bash -c "cd /external && \
          cargo build -p a380_systems_wasm --target wasm32-wasip1 --release && \
          wasm-opt -O1 --signext-lowering --enable-bulk-memory --enable-nontrapping-float-to-int \
            -o /wasmout/systems.wasm /wasmtarget/wasm32-wasip1/release/a380_systems_wasm.wasm"         || { drop_lock "docker-cargo-systems"; return 1; }
    drop_lock "docker-cargo-systems"
    echo "systems.wasm -> $OUT/systems.wasm"
}

build_fbw() {
    take_lock "docker-cpp-fbw" || return 1
    echo "=== building fbw_a380 (fbw.wasm) ==="
    MSYS_NO_PATHCONV=1 docker run --rm -v "$CHECKOUT:/external" "$IMAGE" \
        bash -c "cd /external && mkdir -p fbw-a380x/out/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842/panel && npm run build-a380x:fbw" || { drop_lock "docker-cpp-fbw"; return 1; }
    cp "$A380X_PANEL_OUT/fbw.wasm" "$OUT/fbw.wasm"
    drop_lock "docker-cpp-fbw"
    echo "fbw.wasm -> $OUT/fbw.wasm"
}

build_fadec() {
    take_lock "docker-cpp-fadec" || return 1
    echo "=== building fadec_a380x (fadec-a380x.wasm) ==="
    MSYS_NO_PATHCONV=1 docker run --rm -v "$CHECKOUT:/external" "$IMAGE" \
        bash -c "cd /external && scripts/build-cmake.sh"
    cp "$A380X_PANEL_OUT/fadec-a380x.wasm" "$OUT/fadec-a380x.wasm"
    cp "$A380X_PANEL_OUT/extra-backend-a380x.wasm" "$OUT/extra-backend-a380x.wasm"
    drop_lock "docker-cpp-fadec"
    echo "fadec-a380x.wasm, extra-backend-a380x.wasm -> $OUT/"
}

build_instruments() {
    take_lock "docker-mach" || return 1
    for INSTR in EWD SD EFB; do
        echo "=== building instrument $INSTR ==="
        MSYS_NO_PATHCONV=1 docker run --rm -v "$CHECKOUT:/external" "$IMAGE" \
            bash -c "cd /external && npx mach build --config fbw-a380x/mach.config.js --work-in-config-dir --filter $INSTR"
    done
    drop_lock "docker-mach"

    mkdir -p "$OUT/EWD" "$OUT/SD" "$OUT/EFB"
    cp "$A380X_HTML_OUT/EWD/ewd.js" "$A380X_HTML_OUT/EWD/ewd.css" "$A380X_HTML_OUT/EWD/ewd.html" "$OUT/EWD/" 2>/dev/null || true
    cp "$A380X_HTML_OUT/SD/sd.js" "$A380X_HTML_OUT/SD/sd.css" "$A380X_HTML_OUT/SD/sd.html" "$OUT/SD/" 2>/dev/null || true
    cp "$A380X_HTML_OUT/EFB/efb.js" "$A380X_HTML_OUT/EFB/efb.css" "$A380X_HTML_OUT/EFB/efb.html" "$OUT/EFB/" 2>/dev/null || true
    echo "EWD/SD/EFB bundles -> $OUT/{EWD,SD,EFB}/"
}

WHAT="${1:-all}"
case "$WHAT" in
    systems)     build_systems ;;
    fbw)         build_fbw ;;
    fadec)       build_fadec ;;
    instruments) build_instruments ;;
    all)
        build_systems
        build_fbw
        build_fadec
        build_instruments
        ;;
    *) echo "usage: $0 [systems|fbw|fadec|instruments|all]" >&2; exit 2 ;;
esac

echo ""
echo "=== out/ contents ==="
find "$OUT" -type f | sort
