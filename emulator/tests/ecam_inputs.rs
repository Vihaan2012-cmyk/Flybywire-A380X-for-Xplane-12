//! Regression guard for docs/physics/ecam-inputs.md: every FWS input name
//! that doc classifies PROVIDED must actually exist in the registered var
//! set (`Emulator::snapshot_all`) once the aircraft is powered and running.
//! This does not re-derive the classification (that's the doc's job); it
//! only catches the doc going stale -- a name marked PROVIDED that stops
//! being registered (a rename, a dropped `get_identifier` call) fails here
//! instead of silently defaulting to 0 in the cockpit (js_bridge.rs's
//! "first read of unwritten L: var" is the runtime-only version of this
//! same check).

use fbw_a380_emulator::presets;

/// FWS input names (`FwsCore.ts` / `FwsAutoCallouts.ts`, `L:` vars, no
/// prefix here -- `snapshot_all()` returns the prefixed name the plugin's
/// `Vars::get()` actually registered, see src/lib.rs's `impl
/// VariableRegistry for Vars`) that docs/physics/ecam-inputs.md classifies
/// PROVIDED with a cited `a380_systems`/`a380_systems`-adjacent
/// `get_identifier` call.
const EXPECTED_PROVIDED: &[&str] = &[
    // FwsAutoCallouts.ts:16 <- a380_systems/hydraulic/autobrakes.rs:666
    "A32NX_ROW_ROP_WORD_1",
    // FwsAbnormalSensed.ts (fire section) <- a380_systems/fire_and_smoke_protection.rs:658,660
    "A32NX_FIRE_SQUIB_1_ENG_1_IS_ARMED",
    "A32NX_FIRE_SQUIB_1_ENG_1_IS_DISCHARGED",
    "A32NX_FIRE_SQUIB_2_ENG_1_IS_ARMED",
    "A32NX_FIRE_SQUIB_2_ENG_1_IS_DISCHARGED",
    "A32NX_FIRE_SQUIB_1_APU_1_IS_ARMED",
    "A32NX_FIRE_SQUIB_1_APU_1_IS_DISCHARGED",
    // FwsCore.ts landing-gear reads <- fbw-common landing_gear/mod.rs:383-392
    // (get_identifier, not get_unprefixed_identifier, so Vars::get() adds
    // the A32NX_ prefix -- see docs/physics/ecam-inputs.md's LGCIU section
    // for the extra_backend_fcdc.rs counter-example this test resolves).
    "A32NX_LGCIU_1_LEFT_GEAR_COMPRESSED",
    "A32NX_LGCIU_1_RIGHT_GEAR_COMPRESSED",
    "A32NX_LGCIU_1_NOSE_GEAR_COMPRESSED",
    "A32NX_LGCIU_2_LEFT_GEAR_COMPRESSED",
    "A32NX_LGCIU_1_DISCRETE_WORD_1",
    "A32NX_LGCIU_1_DISCRETE_WORD_2",
    "A32NX_LGCIU_1_DISCRETE_WORD_4",
    // FwsCore.ts ADIRS reads <- adirs.rs / a380_systems air_data
    "A32NX_ADIRS_ADR_1_ALTITUDE",
    "A32NX_ADIRS_ADR_1_COMPUTED_AIRSPEED",
    "A32NX_ADIRS_ADR_1_MACH",
    "A32NX_ADIRS_ADR_1_DISCRETE_WORD_1",
    "A32NX_ADIRS_IR_1_MAINT_WORD",
    "A32NX_ADIRS_IR_1_PITCH",
    "A32NX_ADIRS_REMAINING_IR_ALIGNMENT_TIME",
];

#[test]
fn provided_fws_inputs_are_registered_when_running() {
    let mut e = presets::engines_running();
    for _ in 0..100 {
        e.tick(0.05);
    }
    let snapshot = e.snapshot_all();
    let names: std::collections::HashSet<&str> = snapshot.iter().map(|(n, _)| n.as_str()).collect();

    let mut missing = Vec::new();
    for expected in EXPECTED_PROVIDED {
        if !names.contains(expected) {
            missing.push(*expected);
        }
    }
    assert!(
        missing.is_empty(),
        "docs/physics/ecam-inputs.md classifies these PROVIDED, but they are not in \
         Emulator::snapshot_all() after 5s of engines_running() -- either the doc is \
         stale or a `get_identifier` call regressed: {missing:?}"
    );
}
