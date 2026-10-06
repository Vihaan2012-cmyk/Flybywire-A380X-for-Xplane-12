#!/bin/sh
# Vendor the deep_systems crate into a FlyByWire checkout.
#
# The canonical crate (crates/deep_systems) compiles this plugin's own
# src/deep files (and the few host-neutral plugin files they name) through
# #[path] attributes. The vendored copy holds real files instead, at the
# paths those attributes point to, so FlyByWire's tree builds on its own
# (in their Docker image, with no plugin checkout beside it).
#
#   tools/sync-deep-systems.sh [flybywire checkout]
set -e
PLUGIN="$(cd "$(dirname "$0")/.." && pwd)"
CRATE="$PLUGIN/crates/deep_systems"
DEEP="$PLUGIN/src/deep"
FBW="${1:-/d/A380/fbw-xp-worktrees/fs2020-672384b}"
DST="$FBW/fbw-a380x/src/wasm/systems/deep_systems"

rm -rf "$DST"
mkdir -p "$DST/src/deep/integration" "$DST/src/physics"
cp "$CRATE/Cargo.toml" "$DST/"
cp "$CRATE/src/facade.rs" "$CRATE/src/gates.rs" "$CRATE/src/gates_cpp.rs" "$CRATE/src/gates_js.rs" "$CRATE/src/gates_msfs.rs" "$DST/src/"

# lib.rs: the plugin's mass_balance.rs becomes src/weight_balance.rs, and its
# flight_model.cfg is FlyByWire's own copy in the same checkout.
grep -v '^#\[path' "$CRATE/src/lib.rs" > "$DST/src/lib.rs"
sed 's#"../../fbw-aircraft/fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380X/common/config/flight_model.cfg"#"../../../../base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842/flight_model.cfg"#' \
    "$PLUGIN/src/mass_balance.rs" > "$DST/src/weight_balance.rs"
# physics.rs: fluids.rs becomes src/physics/fluids.rs.
grep -v '^#\[path' "$CRATE/src/physics.rs" > "$DST/src/physics.rs"
cp "$PLUGIN/src/physics/fluids.rs" "$DST/src/physics/"
cp "$PLUGIN/src/physics/gas.rs" "$DST/src/physics/"
cp "$PLUGIN/src/physics/tyre_model.rs" "$DST/src/physics/tyre.rs"

# deep/: every module the crate names, at its conventional path.
grep -v '^#\[path' "$CRATE/src/deep/mod.rs" > "$DST/src/deep/mod.rs"
grep -v '^#\[path' "$CRATE/src/deep/integration/mod.rs" > "$DST/src/deep/integration/mod.rs"
for m in api frame live lvar_bridge msfs_inputs weather all_registry; do
    cp "$DEEP/$m.rs" "$DST/src/deep/"
done
for area in apu autoflight avionics_network breakers cabin communications ecam electrical engine_accessories environment fire_ice flight_controls fuel gear_structure hydraulics oxygen pneumatic_ducts sensors thermal_zones wiring; do
    mkdir -p "$DST/src/deep/$area"
    (cd "$DEEP/$area" && find . \( -name '*.rs' -o -name '*.js' -o -name '*.json' \) | while read -r f; do mkdir -p "$DST/src/deep/$area/$(dirname "$f")"; cp "$f" "$DST/src/deep/$area/$f"; done)
done
for m in fire_ice_adapter sensors_adapter thermal_zones_adapter environment_events_adapter registry weather_model failure_audit; do
    cp "$DEEP/integration/$m.rs" "$DST/src/deep/integration/"
done

# Provenance: a hash of every vendored file, and where they came from.
HASH=$(cd "$DST" && find . -type f ! -name SOURCE_HASH | LC_ALL=C sort | xargs sha256sum | sha256sum | cut -c1-64)
{
    echo "$HASH"
    echo "synced from $PLUGIN at $(date -u +%Y-%m-%dT%H:%M:%SZ)"
} > "$DST/SOURCE_HASH"
echo "vendored deep_systems into $DST ($HASH)"
