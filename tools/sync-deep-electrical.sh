#!/bin/sh
# Vendor the deep_electrical crate into a FlyByWire checkout.
#
# The canonical crate (crates/deep_electrical) compiles this plugin's own
# src/deep files through #[path] attributes. The vendored copy holds real
# files instead, at the paths those attributes point to, so FlyByWire's tree
# builds on its own (in their Docker image, with no plugin checkout beside
# it). Its tests/golden.rs replays the digests this plugin recorded from its
# own areas, so the copy proves it still computes what X-Plane does.
#
#   tools/sync-deep-electrical.sh [flybywire checkout]
set -e
PLUGIN="$(cd "$(dirname "$0")/.." && pwd)"
CRATE="$PLUGIN/crates/deep_electrical"
DEEP="$PLUGIN/src/deep"
FBW="${1:-/d/A380/fbw-xp-worktrees/fs2020-672384b}"
DST="$FBW/fbw-a380x/src/wasm/systems/deep_electrical"

rm -rf "$DST"
mkdir -p "$DST/src/deep/apu" "$DST/tests/golden"
cp "$CRATE/Cargo.toml" "$DST/"
cp "$CRATE/src/lib.rs" "$CRATE/src/facade.rs" "$CRATE/src/gates.rs" "$CRATE/src/golden.rs" "$DST/src/"
cp "$CRATE/src/deep/live.rs" "$DST/src/deep/"
grep -v '^#\[path' "$CRATE/src/deep/mod.rs" > "$DST/src/deep/mod.rs"
grep -v '^#\[path' "$CRATE/src/deep/apu/mod.rs" > "$DST/src/deep/apu/mod.rs"
cp "$DEEP/api.rs" "$DEEP/frame.rs" "$DST/src/deep/"
cp "$DEEP/apu/oil.rs" "$DEEP/apu/params.rs" "$DST/src/deep/apu/"
for area in breakers electrical wiring; do
    mkdir -p "$DST/src/deep/$area"
    cp "$DEEP/$area"/*.rs "$DST/src/deep/$area/"
done
cp "$CRATE/tests/golden.rs" "$CRATE/tests/facade.rs" "$DST/tests/"
cp "$CRATE/tests/golden/digests.txt" "$DST/tests/golden/"

# Provenance: a hash of every vendored file, and where they came from.
HASH=$(cd "$DST" && find . -type f ! -name SOURCE_HASH | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -c1-64)
{
    echo "$HASH"
    echo "synced from $PLUGIN at $(date -u +%Y-%m-%dT%H:%M:%SZ)"
} > "$DST/SOURCE_HASH"
echo "vendored deep_electrical into $DST ($HASH)"
