//! Audit harness: does every registered failure actually change anything a
//! pilot could see?
//!
//! The catalogue (`deep::registry()`) carries thousands of `FailureDef`s,
//! each claiming an `effect` on a `component` through a named
//! `model_field`. The live areas (`deep::live::all_areas()`) publish
//! variables. Nothing, until this file, ever checked that the first
//! reaches the second.
//!
//! The check is differential and entirely black-box: build the areas cold,
//! run N frames from a chosen aircraft state with nothing armed and record
//! everything published; build them cold again, run the *same* N frames
//! from the *same* state with exactly one failure armed at a real
//! magnitude, and compare. A failure that never moves a single published
//! number, in any state this harness can set up, cannot be seen by the
//! crew, by the ECAM, or by the EFB -- whatever its catalogue entry says.
//!
//! This proves one direction only. A failure that changes something is
//! wired; a failure that changes nothing is *either*
//!
//! * (a) not wired -- no area ever reads its id,
//! * (b) wired to a model field nothing downstream consumes,
//! * (c) gated behind an input (a `Truth` field, a cockpit command, another
//!   area's published value) that is still missing or still constant, or
//! * (d) genuinely only observable in a state this harness does not reach.
//!
//! Only reading the area's source separates those four, which is why the
//! sweep reports the dead list grouped by area / component / model field
//! rather than pretending to a verdict. (d) is a real and honest category:
//! the harness holds one state fixed for a whole run and cannot, for
//! instance, fly an approach.
//!
//! ## Why positional comparison is sound
//!
//! `Area::publish` is a pure read of the area's own state that walks the
//! same code path every frame, so the sequence of names is stable. The
//! harness records the healthy run's name sequence once and, on the armed
//! run, checks each name against the recorded one as it arrives. A
//! differing name or a differing count is itself a difference (an area
//! that publishes conditionally), so nothing is missed -- and the armed
//! run allocates nothing for the comparison, which is what makes a
//! five-thousand-failure sweep tractable.
//!
//! ## Determinism
//!
//! Both runs start from `all_areas()` (every area constructed cold) and
//! see an identical `Truth` on every frame, so any difference at all is
//! caused by the armed failure. No epsilon is applied: a bit-level
//! difference is a real difference, and a comparison that ignored small
//! ones would hide exactly the marginal, near-threshold effects this audit
//! exists to find.
//!
//! ## Two things that made the first run of this sweep meaningless
//!
//! The first version of this harness reported all 5166 failures as live,
//! which is the answer a broken audit gives. Two causes, both worth
//! knowing about beyond this file:
//!
//! 1. **The electrical board is process state, not `Deep` state.**
//!    `deep::electrical::live::board` is a `thread_local!` carrying the
//!    one-frame-late channel between electrical, breakers and wiring
//!    (breaker currents, bus voltages, harness damage). Two `all_areas()`
//!    built one after another in the same thread share it, so the second
//!    starts on the first one's last frame and two runs of the *identical
//!    healthy state* come out 135 variables apart. [`fresh_areas`] calls
//!    the `board::clear()` the electrical area already provides.
//!
//! 2. **Three areas branch on `faults.any()`.** `breakers`, `electrical`
//!    and `wiring` take a cheaper path when *nothing at all* is armed --
//!    wiring returns before solving the harness. So both runs arm
//!    [`SENTINEL_ID`], an id no area owns, and the armed run adds the
//!    failure under test on top: `faults.any()` is then true in both, and
//!    no area can tell the two runs apart by the shape of the fault set.
//!    (With the board cleared this turns out not to change any published
//!    value on its own -- the test below reports whether that is still so
//!    -- but the sweep must not depend on that staying true.)
//!
//! The negative control below -- an unowned id, armed on top of the
//! sentinel, moving nothing -- is what keeps both of these honest.

use std::collections::BTreeMap;

use crate::deep::api::{Area as RegArea, Cond, EcamAlert, FailureDef};
use crate::deep::live::{all_areas, CommandedSurfaces, Controls, Faults, Truth};

use super::weather_truth::EnvironmentTruth;

// ---------------------------------------------------------------------------
// Aircraft states the sweep runs each failure in.
// ---------------------------------------------------------------------------

/// One aircraft state, held fixed for the whole run, plus how long to run.
///
/// Fixed rather than flown: the harness asks "does arming this change
/// anything at all", and holding `Truth` constant means the only
/// difference between the two runs is the failure. The states are chosen
/// to switch on whole branches of the models -- an engine turning, a bus
/// live, a duct pressurised, a wheel on the ground with a real sink speed,
/// a fire pushbutton out -- because a failure of an unpowered,
/// unpressurised, stationary part is *correctly* silent.
///
/// `dt_s` lives in the `Truth`, so a profile can buy a minute of simulated
/// time in a few dozen frames. Both runs use the same `dt_s`, so even a
/// model that would integrate badly at that step size is compared against
/// itself.
pub struct Profile {
    pub name: &'static str,
    pub truth: fn() -> Truth,
    pub frames: usize,
}

fn flying_surfaces() -> CommandedSurfaces {
    // Nothing neutral: every panel is asked for a real, distinct
    // deflection, so a jam, a float or a blow-back has something to differ
    // from. A harness that commanded neutral everywhere is exactly how the
    // 36 flight-control jams came to look healthy.
    CommandedSurfaces {
        ailerons_deg: [[4.0, 3.0, 2.0], [-4.0, -3.0, -2.0]],
        elevators_deg: [[-3.0, -2.5], [-3.5, -3.0]],
        rudders_deg: [2.0, 1.5],
        spoilers_deg: [[5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]],
        ths_deg: -1.5,
    }
}

/// Everything cold and dark: the plugin's own before-the-first-frame
/// state, run long enough for a soak or a slow drift to show.
fn cold_dark() -> Truth {
    Truth { dt_s: 0.5, ..Truth::default() }
}

/// Four engines at cruise power at FL370: both hydraulic systems at 5000
/// psi, all four AC buses live, bleeds and packs on, the aircraft flying
/// and every surface commanded somewhere.
fn cruise() -> Truth {
    Truth {
        dt_s: 0.1,
        altitude_ft: 37_000.0,
        on_ground: false,
        engine_n1_frac: [0.88; 4],
        engine_running: [true; 4],
        // A healthy engine does not bypass its oil filter; a clogged
        // one is what the failure under test has to cause.
        engine_oil_filter_bypassed: [false; 4],
        // Ordinary customer-bleed conditions: about 30 psia / 250 C at the
        // IP8 tap and 120 psia / 430 C at HP6, both well above the ambient
        // a leak discharges into at this altitude.
        engine_bleed_pressure_pa: [207_000.0; 4],
        engine_bleed_temp_k: [523.0; 4],
        engine_hp_port_pressure_pa: [827_000.0; 4],
        engine_hp_port_temp_k: [703.0; 4],
        engine_n2_frac: [0.90; 4],
        engine_n3_frac: [0.93; 4],
        engine_fuel_flow_kg_s: [0.9; 4],
        tyre_pressure_pa: [1_550_000.0; crate::physics::tyre::WHEELS],
        engine_oil_pressure_pa: [3.1e5; 4],
        engine_oil_temp_c: [85.0; 4],
        // Tanks serviced full; nothing has consumed a measurable
        // fraction of 20 L in one sector.
        engine_oil_quantity_fraction: [1.0; 4],
        // Cruise TGT for a Trent-class engine, and station 2.5 behind
        // an IP compressor working on this profile's own ram air --
        // both well below the T3 of 703 K this fixture already states.
        engine_tgt_c: [700.0; 4],
        engine_t25_c: [140.0; 4],
        // Shut up and pressurised.
        door_open_fraction: [0.0; crate::deep::live::DOOR_NAMES.len()],
        apu_running: false,
        apu_bleed_pressure_pa: 21_662.0,
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        // 5000 psi, the A380's own system pressure.
        hydraulic_pressure_pa: [34_474_000.0; 2],
        gpu_plugged_in: false,
        aircraft_mass_kg: 450_000.0,
        pitch_deg: 2.5,
        groundspeed_m_s: 250.0,
        angle_of_attack_deg: 2.5,
        radio_height_ft: 37_000.0,
        leg_on_ground: [false; 5],
        leg_touchdown_sink_speed_ms: [0.0; 5],
        cabin_pressure_pa: 75_000.0,
        cabin_temp_k: 297.0,
        sun_elevation_deg: 40.0,
        environment: EnvironmentTruth { sat_c: -56.5, leading_edge_c: -20.0, ambient_pressure_pa: 21_662.0, tas_ms: 250.0, precipitation_on_aircraft_ratio: 0.0, weather: None },
        commanded_surfaces: flying_surfaces(),
        controls: Controls { parking_brake_on: false, gear_lever_down: false, engine_master_on: [true; 4], ..Controls::default() },
        published: Default::default(),
    }
}

/// The same cruise state stepped a second at a time for a minute: the
/// reach of anything with a thermal, wear or fluid-loss time constant far
/// longer than a frame.
fn cruise_soak() -> Truth {
    Truth { dt_s: 1.0, ..cruise() }
}

/// On stand with the APU running and supplying bleed and both generators,
/// ground power plugged in, engines off, packs on: the state most ground
/// failures are found in.
fn ground_apu() -> Truth {
    Truth {
        dt_s: 0.2,
        altitude_ft: 0.0,
        on_ground: true,
        engine_running: [false; 4],
        apu_running: true,
        // The PW980 delivers bleed at roughly 45 psia, 200 C at the valve.
        apu_bleed_pressure_pa: 310_000.0,
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [0.0; 2],
        gpu_plugged_in: true,
        aircraft_mass_kg: 380_000.0,
        cabin_pressure_pa: 101_325.0,
        cabin_temp_k: 300.0,
        sun_elevation_deg: 55.0,
        leg_on_ground: [true; 5],
        controls: Controls { apu_master_sw_on: true, apu_start_pb_on: true, apu_bleed_pb_on: true, apu_gen_pb_on: [true; 2], cross_bleed_selector: 2.0, parking_brake_on: true, ..Controls::default() },
        ..Truth::default()
    }
}

/// Engine start on the ground: starters engaged, cross-bleed open, cores
/// spinning up, APU feeding the start ducts.
fn engine_start() -> Truth {
    Truth {
        dt_s: 0.2,
        on_ground: true,
        engine_running: [false; 4],
        engine_n1_frac: [0.08; 4],
        engine_n2_frac: [0.25; 4],
        engine_n3_frac: [0.22; 4],
        engine_fuel_flow_kg_s: [0.05; 4],
        apu_running: true,
        apu_bleed_pressure_pa: 310_000.0,
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        gpu_plugged_in: true,
        controls: Controls { apu_master_sw_on: true, apu_bleed_pb_on: true, starter_engaged: [true; 4], engine_master_on: [true; 4], cross_bleed_selector: 2.0, ..Controls::default() },
        ..Truth::default()
    }
}

/// Take-off roll: four engines at full thrust, all five legs loaded, the
/// aircraft accelerating through 100 kt with the ground spoilers armed.
fn takeoff_roll() -> Truth {
    Truth {
        dt_s: 0.1,
        on_ground: true,
        engine_n1_frac: [1.0; 4],
        engine_running: [true; 4],
        engine_bleed_pressure_pa: [345_000.0; 4],
        engine_bleed_temp_k: [573.0; 4],
        engine_hp_port_pressure_pa: [1_380_000.0; 4],
        engine_hp_port_temp_k: [773.0; 4],
        engine_n2_frac: [0.98; 4],
        engine_n3_frac: [1.0; 4],
        engine_fuel_flow_kg_s: [3.2; 4],
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [34_474_000.0; 2],
        aircraft_mass_kg: 560_000.0,
        groundspeed_m_s: 51.0,
        pitch_deg: 0.0,
        radio_height_ft: 0.0,
        leg_on_ground: [true; 5],
        cabin_pressure_pa: 101_325.0,
        cabin_temp_k: 298.0,
        environment: EnvironmentTruth { sat_c: 30.0, leading_edge_c: 30.0, ambient_pressure_pa: 101_325.0, tas_ms: 51.0, precipitation_on_aircraft_ratio: 0.0, weather: None },
        commanded_surfaces: CommandedSurfaces { ths_deg: -3.0, ..flying_surfaces() },
        controls: Controls { engine_master_on: [true; 4], parking_brake_on: false, ground_spoiler_lever_armed: true, gear_lever_down: true, ..Controls::default() },
        ..Truth::default()
    }
}

/// The landing itself: a firm touchdown (2.5 m/s, a real but unremarkable
/// sink rate) with the body gear a tenth of a second behind the wing gear,
/// full brake pedal and the spoilers up. `leg_touchdown_sink_speed_ms` is
/// the field a hard-landing model is most sensitive to, and it is zero in
/// every other profile.
fn touchdown() -> Truth {
    Truth {
        dt_s: 0.05,
        on_ground: true,
        engine_n1_frac: [0.25; 4],
        engine_running: [true; 4],
        engine_bleed_pressure_pa: [172_000.0; 4],
        engine_bleed_temp_k: [473.0; 4],
        engine_n2_frac: [0.65; 4],
        engine_n3_frac: [0.60; 4],
        engine_fuel_flow_kg_s: [0.3; 4],
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [34_474_000.0; 2],
        aircraft_mass_kg: 390_000.0,
        groundspeed_m_s: 70.0,
        pitch_deg: 4.0,
        angle_of_attack_deg: 6.0,
        radio_height_ft: 0.0,
        leg_on_ground: [true; 5],
        leg_touchdown_sink_speed_ms: [2.0, 2.5, 2.5, 2.2, 2.2],
        cabin_pressure_pa: 101_325.0,
        cabin_temp_k: 297.0,
        environment: EnvironmentTruth { sat_c: 12.0, leading_edge_c: 12.0, ambient_pressure_pa: 101_325.0, tas_ms: 70.0, precipitation_on_aircraft_ratio: 0.3, weather: None },
        commanded_surfaces: CommandedSurfaces { spoilers_deg: [[50.0; 8], [50.0; 8]], ..flying_surfaces() },
        controls: Controls { engine_master_on: [true; 4], gear_lever_down: true, parking_brake_on: false, brake_pedal_pos: [1.0, 1.0], ground_spoiler_lever_armed: true, gear_door_commanded_open: [0.0; 3], ..Controls::default() },
        ..Truth::default()
    }
}

/// Climbing through freezing drizzle with both anti-ice systems selected:
/// the only profile where ice accretes, where the nacelle and wing
/// anti-ice valves are commanded open, and where there is water on the
/// airframe.
fn icing_climb() -> Truth {
    Truth {
        dt_s: 0.5,
        altitude_ft: 12_000.0,
        on_ground: false,
        engine_n1_frac: [0.92; 4],
        engine_running: [true; 4],
        engine_bleed_pressure_pa: [276_000.0; 4],
        engine_bleed_temp_k: [548.0; 4],
        engine_hp_port_pressure_pa: [1_034_000.0; 4],
        engine_hp_port_temp_k: [733.0; 4],
        engine_n2_frac: [0.94; 4],
        engine_n3_frac: [0.96; 4],
        engine_fuel_flow_kg_s: [2.0; 4],
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [34_474_000.0; 2],
        aircraft_mass_kg: 520_000.0,
        pitch_deg: 8.0,
        groundspeed_m_s: 160.0,
        angle_of_attack_deg: 5.0,
        radio_height_ft: 12_000.0,
        leg_on_ground: [false; 5],
        cabin_pressure_pa: 95_000.0,
        cabin_temp_k: 295.0,
        sun_elevation_deg: -5.0,
        environment: EnvironmentTruth { sat_c: -8.0, leading_edge_c: -6.0, ambient_pressure_pa: 64_400.0, tas_ms: 170.0, precipitation_on_aircraft_ratio: 1.0, weather: None },
        commanded_surfaces: flying_surfaces(),
        controls: Controls { engine_master_on: [true; 4], wing_anti_ice_selected: true, nacelle_anti_ice_selected: [true; 4], rain_removal_selected: [true; 2], parking_brake_on: false, gear_lever_down: false, ..Controls::default() },
        ..Truth::default()
    }
}

/// Every crew *command* that any other profile leaves at rest, held on at
/// once: all four engine fire pushbuttons out plus the APU's, every fire
/// agent pushbutton pressed (both shots, all four engines, and the APU),
/// the gear doors commanded open, both batteries and every generator
/// pushbutton off, the packs and bleeds off, the cross-bleed shut.
///
/// This is not one flight state, and it is not claimed to be. It exists
/// because a whole class of failures -- a squib that cannot fire because
/// nothing ever commands a squib, a shutoff valve that is never asked to
/// move -- is invisible in every realistic state purely for want of the
/// command, and an audit that never pressed the button would report them
/// dead for the wrong reason. Each switch position here is individually
/// real and individually reachable from the flight deck.
fn all_commands_exercised() -> Truth {
    Truth {
        dt_s: 0.5,
        on_ground: false,
        altitude_ft: 20_000.0,
        engine_n1_frac: [0.60; 4],
        engine_running: [true; 4],
        engine_bleed_pressure_pa: [241_000.0; 4],
        engine_bleed_temp_k: [533.0; 4],
        engine_hp_port_pressure_pa: [896_000.0; 4],
        engine_hp_port_temp_k: [713.0; 4],
        engine_n2_frac: [0.85; 4],
        engine_n3_frac: [0.88; 4],
        engine_fuel_flow_kg_s: [1.2; 4],
        apu_running: true,
        apu_bleed_pressure_pa: 200_000.0,
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [34_474_000.0; 2],
        gpu_plugged_in: true,
        aircraft_mass_kg: 480_000.0,
        groundspeed_m_s: 180.0,
        angle_of_attack_deg: 4.0,
        radio_height_ft: 20_000.0,
        leg_on_ground: [false; 5],
        cabin_pressure_pa: 85_000.0,
        cabin_temp_k: 296.0,
        environment: EnvironmentTruth { sat_c: -25.0, leading_edge_c: -15.0, ambient_pressure_pa: 46_600.0, tas_ms: 200.0, precipitation_on_aircraft_ratio: 0.5, weather: None },
        commanded_surfaces: flying_surfaces(),
        controls: Controls {
            fire_pb_released: [true; 4],
            fire_pb_apu_released: true,
            fire_agent_pb_pressed: [[true; 2]; 4],
            fire_agent_pb_apu_pressed: true,
            wing_anti_ice_selected: true,
            nacelle_anti_ice_selected: [true; 4],
            engine_bleed_pb_auto: [false; 4],
            apu_bleed_pb_on: true,
            cross_bleed_selector: 0.0,
            pack_pb_on: [false; 2],
            starter_engaged: [true; 4],
            rain_removal_selected: [true; 2],
            steering_command_deg: [20.0, 5.0, -5.0],
            jettison_armed: true,
            jettison_valve_selected: [true; 2],
            crossfeed_valve_selected: [true; 4],
            cargo_door_commanded_open: [1.0; 3],
            water_demand_l_s: [0.05, 0.02],
            gear_door_commanded_open: [1.0; 3],
            gear_lever_down: true,
            parking_brake_on: true,
            brake_pedal_pos: [0.5, 0.5],
            engine_master_on: [false; 4],
            eng_gen_pb_on: [false; 4],
            apu_gen_pb_on: [false; 2],
            bat_pb_auto: [false; 2],
            ground_spoiler_lever_armed: true,
            apu_master_sw_on: true,
            apu_start_pb_on: true,
            reverser_deploy_commanded: [true; 2],
        },
        ..Truth::default()
    }
}

/// Two minutes on stand with the APU MASTER SW on and START pressed,
/// nothing else selected.
///
/// The APU's own start sequence -- starter motor, igniter, fuel control,
/// governor, generators coming on line -- takes the better part of a
/// minute, so every other profile here is far too short for a starter,
/// igniter or fuel-control fault to have anything to act on. This one runs
/// 120 s of simulated time in 120 frames. `deep::apu` drives its machine
/// from the real `apu_master_sw_on`/`apu_start_pb_on` pushbuttons, not
/// from `Truth::apu_running`, which is why those two are what matter here.
fn apu_start_soak() -> Truth {
    Truth {
        dt_s: 1.0,
        on_ground: true,
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        gpu_plugged_in: true,
        apu_running: true,
        apu_bleed_pressure_pa: 310_000.0,
        cabin_pressure_pa: 101_325.0,
        cabin_temp_k: 300.0,
        controls: Controls { apu_master_sw_on: true, apu_start_pb_on: true, apu_gen_pb_on: [true; 2], apu_bleed_pb_on: true, ..Controls::default() },
        ..Truth::default()
    }
}

/// Airborne with the gear commanded up, for 20 s.
///
/// Every other airborne profile holds the gear lever where it already is,
/// so nothing in the retraction chain ever moves and an uplock hook jam or
/// a downlock spring failure has no motion to jam. This one commands the
/// retraction and gives it long enough to complete.
fn gear_cycle() -> Truth {
    Truth {
        dt_s: 0.5,
        altitude_ft: 3_000.0,
        radio_height_ft: 3_000.0,
        on_ground: false,
        leg_on_ground: [false; 5],
        engine_n1_frac: [0.85; 4],
        engine_running: [true; 4],
        engine_n2_frac: [0.9; 4],
        engine_n3_frac: [0.92; 4],
        engine_fuel_flow_kg_s: [1.5; 4],
        ac_bus_volts: [115.0; 4],
        dc_bus_volts: [28.0; 2],
        hydraulic_pressure_pa: [34_474_000.0; 2],
        aircraft_mass_kg: 500_000.0,
        groundspeed_m_s: 90.0,
        angle_of_attack_deg: 6.0,
        environment: EnvironmentTruth { sat_c: 10.0, leading_edge_c: 10.0, ambient_pressure_pa: 90_800.0, tas_ms: 95.0, precipitation_on_aircraft_ratio: 0.0, weather: None },
        commanded_surfaces: flying_surfaces(),
        controls: Controls { engine_master_on: [true; 4], gear_lever_down: false, gear_door_commanded_open: [1.0; 3], parking_brake_on: false, ..Controls::default() },
        ..Truth::default()
    }
}

/// Every profile, cheapest and most-likely-to-move first, so that a
/// failure that is alive at cruise costs exactly one eight-frame run.
pub fn profiles() -> Vec<Profile> {
    vec![
        Profile { name: "cruise", truth: cruise, frames: 8 },
        Profile { name: "all_commands_exercised", truth: all_commands_exercised, frames: 16 },
        Profile { name: "ground_apu", truth: ground_apu, frames: 16 },
        Profile { name: "touchdown", truth: touchdown, frames: 16 },
        Profile { name: "takeoff_roll", truth: takeoff_roll, frames: 12 },
        Profile { name: "engine_start", truth: engine_start, frames: 16 },
        Profile { name: "icing_climb", truth: icing_climb, frames: 24 },
        Profile { name: "cold_dark", truth: cold_dark, frames: 20 },
        Profile { name: "gear_cycle", truth: gear_cycle, frames: 40 },
        // Last and longest: 60 s of simulated cruise, for anything whose
        // time constant is minutes (a reservoir draining, a bearing
        // heating, a bay soaking).
        Profile { name: "cruise_soak", truth: cruise_soak, frames: 60 },
        Profile { name: "apu_start_soak", truth: apu_start_soak, frames: 120 },
    ]
}

// ---------------------------------------------------------------------------
// The differential run.
// ---------------------------------------------------------------------------

/// An id no area owns and `deep::registry()` never issues, armed in
/// *both* runs so that the three areas with a `faults.any()` shortcut take
/// the same branch in each. See this module's "The sentinel" note.
///
/// `failure_id` packs an area code into the top of the id and the highest
/// area code in `api::Area` is 19, so 999 is out of reach of any area that
/// could ever be added, and the pre-existing catalogue in `failures.rs`
/// numbers below 100 000.
pub const SENTINEL_ID: u64 = 999_999_999;

/// The reference fault set: nothing broken, but not literally empty.
pub fn reference_faults() -> Faults {
    Faults::from_pairs([(SENTINEL_ID, 1.0)])
}

/// The reference fault set plus exactly one real failure.
pub fn armed_with(id: u64, magnitude: f64) -> Faults {
    Faults::from_pairs([(SENTINEL_ID, 1.0), (id, magnitude)])
}

/// `all_areas()`, plus the one piece of state that `Deep::new` does not
/// reset.
///
/// `deep::electrical::live::board` is a **thread-local** carrying the
/// one-frame-late channel between electrical, breakers and wiring
/// (breaker currents, bus voltages, harness damage). It belongs to the
/// process, not to the `Deep`, so two `all_areas()` built one after the
/// other in the same thread share it: the second starts with the first's
/// last frame on the board, and two runs of the identical healthy state
/// come out 135 variables apart. `board::clear()` is the reset the
/// electrical area already provides for exactly this. Nothing else under
/// `deep/` holds cross-run state (`breakers::catalog`'s `OnceLock` and
/// `electrical::live`'s `TOPOLOGY` are immutable tables, and
/// `gear_structure::registry`'s counter resets itself).
pub fn fresh_areas() -> crate::deep::live::Deep {
    crate::deep::electrical::live::board::clear();
    all_areas()
}

/// A healthy run's whole published trace: the name sequence (recorded
/// once, checked on every later frame) and each frame's values in that
/// same order.
pub struct Baseline {
    pub names: Vec<String>,
    pub frames: Vec<Vec<f64>>,
}

/// Run `frames` ticks from `truth` with `faults` armed, recording
/// everything published.
pub fn baseline(truth: &Truth, faults: &Faults, frames: usize) -> Baseline {
    let mut deep = fresh_areas();
    let mut names: Vec<String> = Vec::new();
    let mut out_frames = Vec::with_capacity(frames);
    for _ in 0..frames {
        let mut values = Vec::with_capacity(names.len());
        let mut seen: Vec<String> = Vec::new();
        let first = names.is_empty();
        deep.tick(truth.clone(), faults, &mut |name, value| {
            values.push(value);
            if first {
                seen.push(name.to_owned());
            }
        });
        if first {
            names = seen;
        }
        out_frames.push(values);
    }
    Baseline { names, frames: out_frames }
}

/// What one armed run differed from the healthy baseline in.
#[derive(Clone, Debug, Default)]
pub struct Diff {
    /// Indices into `Baseline::names` whose value ever differed.
    pub changed: Vec<usize>,
    /// The first frame (0-based) on which anything differed.
    pub first_frame: Option<usize>,
    /// The areas published a different name sequence -- itself a change.
    pub shape_changed: bool,
}

impl Diff {
    pub fn is_empty(&self) -> bool {
        self.changed.is_empty() && !self.shape_changed
    }
}

/// Run the same frames with `faults` armed and report what differed.
pub fn diff_against(base: &Baseline, truth: &Truth, faults: &Faults) -> Diff {
    let mut deep = fresh_areas();
    let mut diff = Diff::default();
    let mut changed = vec![false; base.names.len()];
    for (frame, expected) in base.frames.iter().enumerate() {
        let mut i = 0usize;
        let mut shape = false;
        let mut differed = false;
        deep.tick(truth.clone(), faults, &mut |name, value| {
            if i >= base.names.len() || base.names[i] != name {
                shape = true;
            } else if !same(expected[i], value) {
                changed[i] = true;
                differed = true;
            }
            i += 1;
        });
        if i != base.names.len() {
            shape = true;
        }
        if shape {
            diff.shape_changed = true;
        }
        if (differed || shape) && diff.first_frame.is_none() {
            diff.first_frame = Some(frame);
        }
    }
    diff.changed = changed.iter().enumerate().filter(|(_, &c)| c).map(|(i, _)| i).collect();
    diff
}

/// Bit-level equality, with NaN equal to NaN so that a value that is NaN
/// in both runs is not reported as a difference. (A NaN is its own bug;
/// `deep::live`'s own first-frame test already guards finiteness.)
fn same(a: f64, b: f64) -> bool {
    a == b || (a.is_nan() && b.is_nan())
}

// ---------------------------------------------------------------------------
// The sweep.
// ---------------------------------------------------------------------------

/// One failure's verdict.
#[derive(Clone, Debug)]
pub struct Verdict {
    pub id: u64,
    pub area: RegArea,
    pub ata: u16,
    pub name: String,
    pub component: String,
    pub model_field: String,
    /// The first profile in which it moved anything, and at what magnitude.
    pub alive_in: Option<(&'static str, f64)>,
    /// The first few variables it moved there, for the report.
    pub moved: Vec<String>,
    /// How many published variables it moved in total.
    pub moved_count: usize,
}

impl Verdict {
    pub fn is_live(&self) -> bool {
        self.alive_in.is_some()
    }
}

/// The magnitudes each failure is armed at, in order.
///
/// Fully failed first, because a failure that does anything at all
/// normally does it at 1.0. A middling magnitude second, for the rarer
/// shape where full failure saturates into the same place as healthy (a
/// valve driven so hard it parks where it started) -- the audit must not
/// call that dead.
pub const MAGNITUDES: [f64; 2] = [1.0, 0.35];

/// One failure against one set of already-built baselines.
fn verdict_for(f: &FailureDef, profiles: &[Profile], baselines: &[Baseline], truths: &[Truth]) -> Verdict {
    let mut v = Verdict { id: f.id, area: f.area, ata: f.ata, name: f.name.clone(), component: f.component.clone(), model_field: f.model_field.clone(), alive_in: None, moved: Vec::new(), moved_count: 0 };
    'search: for (p, profile) in profiles.iter().enumerate() {
        for m in MAGNITUDES {
            let d = diff_against(&baselines[p], &truths[p], &armed_with(f.id, m));
            if !d.is_empty() {
                v.alive_in = Some((profile.name, m));
                v.moved_count = d.changed.len();
                v.moved = d.changed.iter().take(6).map(|&i| baselines[p].names[i].clone()).collect();
                if d.shape_changed {
                    v.moved.push("<published name set changed>".into());
                }
                break 'search;
            }
        }
    }
    v
}

/// How many worker threads the sweep uses.
///
/// Capped well below the machine's parallelism on purpose: the electrical
/// board is a `thread_local!`, so each worker gets its own aircraft for
/// free, but the sweep is not the only thing running on this machine.
pub fn worker_threads() -> usize {
    std::thread::available_parallelism().map(|n| n.get().min(8)).unwrap_or(4).max(1)
}

/// Sweep every failure through every profile, stopping on the first
/// profile and magnitude that moves anything.
///
/// Parallel across failures. Each worker builds its own `Deep`s and its
/// own baselines, and the one piece of shared mutable state in the areas
/// (`electrical::live::board`) is thread-local, so the workers cannot see
/// each other at all. A failure that is alive in the first profile costs
/// one short run; only a dead one pays for the whole profile set, which is
/// why the cheap, most-likely profiles come first.
pub fn sweep(failures: &[FailureDef], progress: &mut (dyn FnMut(usize, usize, usize) + Send)) -> Vec<Verdict> {
    sweep_over(&profiles(), failures, progress)
}

/// [`sweep`] over a chosen profile list -- a short one for the fast,
/// always-on test, the full one for the sweep.
pub fn sweep_over(profiles: &[Profile], failures: &[FailureDef], progress: &mut (dyn FnMut(usize, usize, usize) + Send)) -> Vec<Verdict> {
    use std::sync::atomic::{AtomicUsize, Ordering};

    let threads = worker_threads();
    let done = AtomicUsize::new(0);
    let dead = AtomicUsize::new(0);
    let total = failures.len();
    let chunk = total.div_ceil(threads).max(1);

    let progress = std::sync::Mutex::new(progress);
    let mut out: Vec<Verdict> = std::thread::scope(|scope| {
        let handles: Vec<_> = failures
            .chunks(chunk)
            .map(|slice| {
                let done = &done;
                let dead = &dead;
                let progress = &progress;
                scope.spawn(move || {
                    let baselines: Vec<Baseline> = profiles.iter().map(|p| baseline(&(p.truth)(), &reference_faults(), p.frames)).collect();
                    let truths: Vec<Truth> = profiles.iter().map(|p| (p.truth)()).collect();
                    let mut mine = Vec::with_capacity(slice.len());
                    for f in slice {
                        let v = verdict_for(f, profiles, &baselines, &truths);
                        if !v.is_live() {
                            dead.fetch_add(1, Ordering::Relaxed);
                        }
                        let n = done.fetch_add(1, Ordering::Relaxed) + 1;
                        if n % 200 == 0 {
                            if let Ok(mut p) = progress.lock() {
                                p(n, total, dead.load(Ordering::Relaxed));
                            }
                        }
                        mine.push(v);
                    }
                    mine
                })
            })
            .collect();
        handles.into_iter().flat_map(|h| h.join().expect("a sweep worker panicked")).collect()
    });
    out.sort_by_key(|v| v.id);
    out
}

// ---------------------------------------------------------------------------
// The same class of bug on the ECAM side.
// ---------------------------------------------------------------------------

/// Every variable name a condition reads.
pub fn cond_vars(c: &Cond, into: &mut Vec<String>) {
    match c {
        Cond::Always => {}
        Cond::Var { name, .. } => into.push(name.clone()),
        Cond::VarVar { a, b, .. } => {
            into.push(a.clone());
            into.push(b.clone());
        }
        Cond::And(v) | Cond::Or(v) => v.iter().for_each(|x| cond_vars(x, into)),
        Cond::Not(x) => cond_vars(x, into),
    }
}

/// FlyByWire's own prefix for the aircraft's named variables.
///
/// `Vars::get` (`lib.rs`) strips it and puts it back, so `A32NX_FOO` and
/// `FOO` are *the same variable*: an alert trigger written one way and a
/// published name written the other way do meet, and a comparison that
/// did not normalise would report a hundred perfectly live alerts as
/// dead. Same rule as `lib.rs`'s `NAME_PREFIX`.
pub const NAME_PREFIX: &str = "A32NX_";

/// A variable name as `Vars` would key it.
pub fn bare(name: &str) -> &str {
    name.strip_prefix(NAME_PREFIX).unwrap_or(name)
}

/// Every variable an alert's *trigger* reads (not its procedure lines,
/// which read cockpit controls the plugin owns rather than anything the
/// deep areas publish), normalised by [`bare`].
pub fn trigger_vars(a: &EcamAlert) -> Vec<String> {
    let mut v = Vec::new();
    cond_vars(&a.trigger, &mut v);
    let mut v: Vec<String> = v.iter().map(|n| bare(n).to_owned()).collect();
    v.sort();
    v.dedup();
    v
}

/// Three-valued truth for a trigger evaluated with only *some* of its
/// variables known.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tri {
    /// The trigger is false whatever the modelled variables do: the alert
    /// can never appear.
    Never,
    /// The trigger is true whatever they do: the alert is stuck on.
    Always,
    /// It depends on a variable something actually publishes.
    Reachable,
}

/// Can this condition ever be true?
///
/// A variable `known` accepts is left free (`Reachable`). A variable
/// nothing publishes is not free: [`Cond::eval`] reads a missing variable
/// as **0**, so its comparison has a definite answer, and that answer is
/// usually -- but not always -- false. `var(X).lt(25.0)` on an unpublished
/// X is permanently *true*, which is the same bug wearing the other face:
/// an ECAM warning that is on from the moment the aircraft loads.
pub fn reachability(c: &Cond, known: &dyn Fn(&str) -> bool) -> Tri {
    fn fixed(cmp: crate::deep::api::Cmp, x: f64, y: f64) -> Tri {
        use crate::deep::api::Cmp::*;
        let t = match cmp {
            Lt => x < y,
            Le => x <= y,
            Gt => x > y,
            Ge => x >= y,
            Eq => (x - y).abs() < 1e-9,
            Ne => (x - y).abs() >= 1e-9,
        };
        if t {
            Tri::Always
        } else {
            Tri::Never
        }
    }
    match c {
        Cond::Always => Tri::Always,
        Cond::Var { name, cmp, value } => {
            if known(bare(name)) {
                Tri::Reachable
            } else {
                fixed(*cmp, 0.0, *value)
            }
        }
        Cond::VarVar { a, cmp, b } => {
            if known(bare(a)) || known(bare(b)) {
                Tri::Reachable
            } else {
                fixed(*cmp, 0.0, 0.0)
            }
        }
        Cond::And(v) => {
            let parts: Vec<Tri> = v.iter().map(|x| reachability(x, known)).collect();
            if parts.iter().any(|&t| t == Tri::Never) {
                Tri::Never
            } else if parts.iter().all(|&t| t == Tri::Always) {
                Tri::Always
            } else {
                Tri::Reachable
            }
        }
        Cond::Or(v) => {
            let parts: Vec<Tri> = v.iter().map(|x| reachability(x, known)).collect();
            if parts.iter().any(|&t| t == Tri::Always) {
                Tri::Always
            } else if parts.iter().all(|&t| t == Tri::Never) {
                Tri::Never
            } else {
                Tri::Reachable
            }
        }
        Cond::Not(x) => match reachability(x, known) {
            Tri::Never => Tri::Always,
            Tri::Always => Tri::Never,
            Tri::Reachable => Tri::Reachable,
        },
    }
}

/// An alert whose trigger cannot be satisfied because some variable in it
/// is published by nobody.
#[derive(Clone, Debug)]
pub struct UnpublishedTrigger {
    pub key: String,
    pub title: String,
    pub ata: u16,
    pub missing: Vec<String>,
    pub present: Vec<String>,
}

/// Which alerts read a variable no area publishes.
///
/// `published` is `Deep::published_names()`. A name missing from it is not
/// automatically a dead alert -- a trigger may legitimately read a
/// variable the *plugin* owns (a cockpit switch, a FlyByWire value), and
/// `Cond::eval` reads a missing variable as 0, which some triggers
/// deliberately rely on. The caller decides; this only reports.
pub fn alerts_reading_unpublished(alerts: &[EcamAlert], published: &std::collections::BTreeSet<String>) -> Vec<UnpublishedTrigger> {
    let mut out = Vec::new();
    for a in alerts {
        let vars = trigger_vars(a);
        let (present, missing): (Vec<String>, Vec<String>) = vars.into_iter().partition(|v| published.contains(v));
        if !missing.is_empty() {
            out.push(UnpublishedTrigger { key: a.key.clone(), title: a.title.clone(), ata: a.ata, missing, present });
        }
    }
    out
}

/// Variables an ECAM trigger may legitimately read although no deep area
/// publishes them: real variables some *other* part of the plugin owns.
///
/// Established by grepping the crate for every name the triggers read that
/// `Deep::published_names` does not carry; of the whole set, these are the
/// only ones anything writes. Everything else the triggers reach for
/// exists nowhere in this repository or in FlyByWire's own systems.
pub const PLUGIN_OWNED_TRIGGER_VARS: &[&str] = &[
    // The APU START pushbutton, written by `start_state.rs` and read by
    // `deep::plugin` and the EFB's own study page.
    "OVHD_APU_START_PB_IS_ON",
];

/// Which alerts can never fire, and which are stuck on, given what the
/// areas actually publish.
pub fn trigger_verdicts(alerts: &[EcamAlert], published: &std::collections::BTreeSet<String>) -> Vec<(String, Tri, Vec<String>)> {
    let known = |n: &str| published.contains(n) || PLUGIN_OWNED_TRIGGER_VARS.contains(&n);
    alerts
        .iter()
        .map(|a| {
            let t = reachability(&a.trigger, &known);
            let missing: Vec<String> = trigger_vars(a).into_iter().filter(|v| !known(v)).collect();
            (a.key.clone(), t, missing)
        })
        .collect()
}

/// Components nothing can break, and failures naming a component nothing
/// models. The second is already a `Registry::validate` error; the first
/// is not, and is the quieter half of the same bug.
pub fn components_without_failures(r: &crate::deep::api::Registry) -> Vec<String> {
    let named: std::collections::BTreeSet<&str> = r.failures.iter().map(|f| f.component.as_str()).collect();
    r.components.iter().filter(|c| c.failures.is_empty() && !named.contains(c.id.as_str())).map(|c| c.id.clone()).collect()
}

// ---------------------------------------------------------------------------
// The legacy catalogue (`failures.rs::extra`, ids below 100 000).
// ---------------------------------------------------------------------------
//
// Everything above audits only `deep::registry()`: ids `>= 1_000_000`, armed
// through the `Faults` struct passed directly into `Deep::step`/`tick`. The
// legacy catalogue (`failures::extra::extra_failures()`) is not reachable
// that way at all -- `deep::live` areas never look at it, and in production
// `Faults` is itself *built from* `failures::active_magnitudes()`, filtered
// to the deep ids (`deep/plugin.rs`: `Faults::from_pairs(failures::
// active_magnitudes().into_iter().filter(|(id, _)| self.failure_ids.
// contains(id)))`). The legacy side of that same global state is read
// straight by the older, non-`deep` physics modules (`fuel_network.rs`,
// `physics/bays.rs`, `physics/tyre.rs`, `breakers.rs`, ... -- see the audit
// below for the full list found), which have no `Area::publish`-style
// harness this file can drive the way `sweep`/`baseline`/`diff_against` do
// for `deep`. Building an equivalent differential runtime harness across
// every legacy physics module is a materially bigger job than this file's
// scope, so this section proves the *weaker*, static half of the same
// question instead, in the same spirit `alerts_reading_unpublished`/
// `reachability` above already prove a static half for ECAM triggers: not
// "arming this changes something," but "something in this codebase has
// even claimed to read this hook back."
//
// That claim is checkable because it is already this codebase's own
// convention, not an invented heuristic: every hook family that *has* been
// wired documents itself by citing its exact `Effect::Hook { var: "..." }`
// string next to the `failures::magnitude`/`is_active` call that consumes
// it -- see `fuel_network.rs`'s comment on `pump_tank_derate`'s neighbour
// field ("`Effect::Hook { var: "FAIL_FUEL_HOOK", .. }`) and
// `physics/bays.rs`'s comment on `ENGINE_BLEED_LEAK_FAILURE_IDS`
// ("`Effect::Hook { var: "FAIL_BLEED_DUCT_LEAK_HOOK", owner: Owner::Air }`).
// So: if a hook var's exact string never appears anywhere outside
// `failures.rs` (which only *registers*/*arms* it, never reads it back),
// nothing in the tree has even claimed to consume it -- the strongest form
// of "inert" this file can prove without running the legacy physics stack.
// A var that *is* mentioned elsewhere is not proven live by this check
// alone (the mention could be stale prose); it is excluded from the dead
// list as "not provably dead," which is the conservative direction to err
// in for a report that must not fake a worse number than the evidence
// supports.
//
// `FAIL_ENGINE_COMPONENT_HOOK` is the one family where this coarse,
// whole-family check is too blunt to use alone: it is shared by 56 ids (14
// per-engine component kinds x 4 engines), and only some of those kinds are
// wired (`engine_commands.rs` reads `failures::magnitude(79_004 + i)` for
// one of the 14). [`id_literal_referenced`] does the same textual check one
// id at a time instead of one hook var at a time, so that family's count is
// not all-or-nothing.

/// Every `.rs` file under `src/`, concatenated, except `failures.rs` and
/// this file (`failure_audit.rs`) itself. `failures.rs` only
/// *registers*/*arms* a hook id (`extra::extra_failures`, `extra::drive`);
/// a mention there is not evidence anything reads it back, so it must be
/// excluded or every hook var would trivially "self-match" its own
/// registration and the check below would never find anything inert. This
/// file is excluded for the same reason from the opposite direction: the
/// tests below compare specific var names by exact quoted string (`f.var ==
/// "FAIL_..._HOOK"`) for the revert-check pins, and those literals would
/// otherwise "self-match" here even for a family nothing physical reads --
/// this was caught live the first time this audit ran (`FAIL_BUS_SHORT_HOOK`
/// showed up as "referenced" purely because this file's own `== "FAIL_BUS_
/// SHORT_HOOK"` comparison is a literal containing the exact quoted needle,
/// not because anything outside failures.rs reads it).
fn source_outside_failures_rs() -> String {
    fn walk(dir: &std::path::Path, out: &mut String) {
        let Ok(entries) = std::fs::read_dir(dir) else { return };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                walk(&path, out);
            } else if path.extension().is_some_and(|e| e == "rs")
                && path.file_name().is_some_and(|n| n != "failures.rs" && n != "failure_audit.rs")
            {
                if let Ok(s) = std::fs::read_to_string(&path) {
                    out.push_str(&s);
                    out.push('\n');
                }
            }
        }
    }
    let mut out = String::new();
    walk(std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/src")), &mut out);
    out
}

/// One legacy hook-variable family: every id `extra_failures()` files under
/// the same `Effect::Hook { var, .. }`, and whether that var's exact string
/// appears anywhere outside `failures.rs`/this file.
pub struct LegacyHookFamily {
    pub var: String,
    pub owner: &'static str,
    pub ids: Vec<u64>,
    /// `false` means: nothing outside `failures.rs`/this file mentions this
    /// hook var's own name at all. A family sharing its var across several
    /// ids (`FAIL_HYDRAULIC_HOOK`, `FAIL_ENGINE_COMPONENT_HOOK`, ...) can
    /// still be individually wired per id via `failures::magnitude`/
    /// `is_active`/`active_ids` on the bare numeric id, with no mention of
    /// the var name at all (`breakers.rs` reads `FAIL_HYDRAULIC_HOOK`'s
    /// 29_103 that way) -- see `dead_ids` for the precise, id-by-id answer;
    /// this field is the coarser, whole-family signal alone.
    pub referenced_elsewhere: bool,
    /// The subset of `ids` where *neither* the family's var name *nor* that
    /// id's own numeric literal (see [`id_literal_referenced`]) appears
    /// anywhere outside `failures.rs`/this file. This is the precise,
    /// per-id confirmed-dead list; `referenced_elsewhere` alone
    /// undercounts live ids for a shared-var family and overcounts dead
    /// ones for nothing (a var-name mention doesn't prove every sibling id
    /// is wired) -- `dead_ids` is what the total in the audit report
    /// should sum, not `ids.len()` gated on `referenced_elsewhere`.
    pub dead_ids: Vec<u64>,
}

/// Group `failures::extra::extra_failures()`'s `Effect::Hook` entries by
/// their shared var, and check each against `source` (pass
/// [`source_outside_failures_rs`]'s result; the tests below share one read
/// of the tree across several checks rather than re-walking `src/` per
/// family).
pub fn legacy_hook_families(source: &str) -> Vec<LegacyHookFamily> {
    use crate::failures::extra::{extra_failures, Effect};
    let mut by_var: BTreeMap<String, (&'static str, Vec<u64>)> = BTreeMap::new();
    for f in extra_failures() {
        if let Effect::Hook { var, owner } = f.effect {
            by_var.entry(var.to_owned()).or_insert_with(|| (owner.label(), Vec::new())).1.push(f.id);
        }
    }
    by_var
        .into_iter()
        .map(|(var, (owner, mut ids))| {
            ids.sort_unstable();
            let needle = format!("\"{var}\"");
            let referenced_elsewhere = source.contains(&needle);
            // Always check every id by its own number, whether or not the
            // family's var-name string is cited anywhere: most wired
            // families in this codebase (FAIL_HYDRAULIC_HOOK via
            // breakers.rs, FAIL_ENGINE_COMPONENT_HOOK via engine_commands.rs
            // / damage.rs) are read through `failures::magnitude`/
            // `is_active`/`active_ids` on the bare numeric id and never cite
            // the hook var's own name at all, so `referenced_elsewhere` is
            // false for them precisely because they *are* wired that way --
            // gating the per-id check on it (an earlier version of this
            // function did) silently skipped the real check for exactly the
            // families that most need it, reporting FAIL_ENGINE_COMPONENT_
            // HOOK as 100% dead when `79_004` is read on line 499 of
            // `engine_commands.rs`. And even a family whose var name *is*
            // cited (`FAIL_FUEL_HOOK`) still needs the per-id check, since a
            // var-name mention proves only that *something* in the family is
            // read, not that every sibling id is (`fuel_network.rs` reads
            // 28_000..28_011 but not the mid/inner-tank pump ids
            // 28_102..28_109 also filed under `FAIL_FUEL_HOOK`).
            let dead_ids: Vec<u64> = ids.iter().copied().filter(|&id| !id_literal_referenced(source, id)).collect();
            LegacyHookFamily { var, owner, ids, referenced_elsewhere, dead_ids }
        })
        .collect()
}

/// Whether `id` itself -- not just the hook var its family shares -- occurs
/// as a numeric literal anywhere in `source`, written either plain
/// (`79004`) or with this codebase's own thousands-grouping convention
/// (`79_004`). Boundary-checked so e.g. id `4` does not match inside `1004`.
///
/// Known blind spot, not fixed here: a consumer that reaches a sibling id
/// by arithmetic on a loop variable (`engine_commands.rs`'s
/// `failures::magnitude(79_004 + i as u64)`, `fuel_network.rs`'s
/// `failures::magnitude(28_000 + k as u64)`) only ever spells out the
/// *base* id as a literal -- `79_005`/`79_006`/`79_007` never appear in the
/// source text at all even though the loop demonstrably reads them at
/// runtime. This function has no way to evaluate that arithmetic, so it
/// under-counts "referenced" (equivalently, over-counts "dead") for exactly
/// that shape of consumer. The audit's own report is written to read
/// accordingly: "this id's own number was never found in the tree," not
/// "this id is certainly never consumed" -- a real differential run
/// (arming `failures::set_magnitude(id, m)` and diffing the owning
/// module's output, the way [`sweep`] does for `deep::registry()`) would be
/// needed to close this gap, and is out of this file's current scope (see
/// the module-level doc comment on the legacy-catalogue section above for
/// why).
pub fn id_literal_referenced(source: &str, id: u64) -> bool {
    let plain = id.to_string();
    let mut grouped = String::new();
    for (i, b) in plain.bytes().enumerate() {
        if i > 0 && (plain.len() - i) % 3 == 0 {
            grouped.push('_');
        }
        grouped.push(b as char);
    }
    [plain, grouped].iter().any(|needle| {
        source.match_indices(needle.as_str()).any(|(i, m)| {
            let before_digit = source[..i].chars().next_back().is_some_and(|c| c.is_ascii_digit());
            let after_digit = source[i + m.len()..].chars().next().is_some_and(|c| c.is_ascii_digit());
            !before_digit && !after_digit
        })
    })
}

// ---------------------------------------------------------------------------
// Reporting.
// ---------------------------------------------------------------------------

/// Group key for the dead list: failures die in families, and a family is
/// one cause.
fn family(v: &Verdict) -> String {
    let field = v.model_field.split_once('.').map_or(v.model_field.clone(), |(s, f)| format!("{s}.{f}"));
    format!("{:?}|{}|{}", v.area, v.ata, field)
}

/// A human-readable report of a sweep.
pub fn report(verdicts: &[Verdict]) -> String {
    use std::fmt::Write;
    let mut s = String::new();
    let live = verdicts.iter().filter(|v| v.is_live()).count();
    let dead = verdicts.len() - live;
    let _ = writeln!(s, "= DEEP FAILURE AUDIT =");
    let _ = writeln!(s, "registered {} | move something published {} | move nothing {}", verdicts.len(), live, dead);

    let mut per_area: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for v in verdicts {
        let e = per_area.entry(format!("{:?}", v.area)).or_default();
        e.0 += 1;
        if !v.is_live() {
            e.1 += 1;
        }
    }
    let _ = writeln!(s, "\n-- per area: total / dead --");
    for (a, (t, d)) in &per_area {
        let _ = writeln!(s, "{a:<20} {t:>6} {d:>6}");
    }

    let mut by_profile: BTreeMap<&str, usize> = BTreeMap::new();
    for v in verdicts {
        if let Some((p, _)) = v.alive_in {
            *by_profile.entry(p).or_default() += 1;
        }
    }
    let _ = writeln!(s, "\n-- live failures by the first profile that showed them --");
    for (p, n) in &by_profile {
        let _ = writeln!(s, "{p:<28} {n:>6}");
    }

    let mut fams: BTreeMap<String, Vec<&Verdict>> = BTreeMap::new();
    for v in verdicts.iter().filter(|v| !v.is_live()) {
        fams.entry(family(v)).or_default().push(v);
    }
    let mut ordered: Vec<_> = fams.into_iter().collect();
    ordered.sort_by_key(|(_, v)| std::cmp::Reverse(v.len()));
    let _ = writeln!(s, "\n-- dead families (area | ata | model field), largest first: {} families --", ordered.len());
    for (k, vs) in &ordered {
        let _ = writeln!(s, "{:>5}  {}   e.g. {} [{}] id {}", vs.len(), k, vs[0].name, vs[0].component, vs[0].id);
    }

    let _ = writeln!(s, "\n-- every dead failure --");
    for v in verdicts.iter().filter(|v| !v.is_live()) {
        let _ = writeln!(s, "{} {:?} ata{} | {} | {} | {}", v.id, v.area, v.ata, v.name, v.component, v.model_field);
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    /// Somewhere to put a sweep's full report. Not in the repo: it is an
    /// output, and it is megabytes.
    fn report_path(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(name)
    }

    #[test]
    fn a_healthy_run_is_deterministic_so_any_difference_is_the_failure() {
        // The whole audit rests on this: two cold runs of the same state
        // publish bit-identical traces, so a single differing bit in an
        // armed run is caused by the failure and nothing else.
        let t = cruise();
        let base = baseline(&t, &reference_faults(), 5);
        let again = diff_against(&base, &t, &reference_faults());
        assert!(again.is_empty(), "{} variables differ between two identical healthy runs, e.g. {:?}", again.changed.len(), again.changed.iter().take(5).map(|&i| &base.names[i]).collect::<Vec<_>>());
        assert!(!base.names.is_empty());
    }

    #[test]
    fn an_id_no_area_owns_moves_nothing_which_is_what_a_dead_failure_looks_like() {
        // The negative control. If this ever moved something, the harness
        // would be reporting noise as effect.
        let t = cruise();
        let base = baseline(&t, &reference_faults(), 5);
        let d = diff_against(&base, &t, &armed_with(999_999_998, 1.0));
        assert!(d.is_empty(), "an unregistered failure id moved {} variables, e.g. {:?}", d.changed.len(), d.changed.iter().take(6).map(|&i| &base.names[i]).collect::<Vec<_>>());

        // The other half of the same question: with no sentinel at all,
        // does the `faults.any()` branch in breakers/electrical/wiring
        // show up in what they publish? Reported, not asserted -- the
        // sweep is written not to depend on the answer either way.
        let empty = baseline(&t, &Faults::default(), 5);
        let shortcut = diff_against(&empty, &t, &Faults::from_pairs([(999_999_998, 1.0)]));
        println!("AUDIT faults.any() branch moves {} published variables on its own", shortcut.changed.len());
    }

    /// A small, always-on sample: one failure from every area, taken in
    /// registration order, run through the whole profile set.
    ///
    /// This is the regression guard. It does not assert that every sampled
    /// failure is live -- some areas' first failure may honestly be one of
    /// the dead ones -- but it does assert that the harness reaches every
    /// area and that the great majority of the sample moves something, so
    /// a change that unplugs a whole area's fault path fails here in
    /// seconds rather than in the ignored full sweep.
    #[test]
    fn a_sample_from_every_area_still_moves_something_published() {
        let r = crate::deep::registry();
        let mut seen: std::collections::BTreeSet<String> = Default::default();
        let mut sample: Vec<FailureDef> = Vec::new();
        for f in &r.failures {
            if seen.insert(format!("{:?}", f.area)) {
                sample.push(f.clone());
            }
        }
        // Two profiles, not the whole set: this runs on every `cargo
        // test`, and the point of it is that the harness still reaches
        // every area, not that it exhausts the state space. The ignored
        // full sweep is what does that.
        let quick = vec![Profile { name: "cruise", truth: cruise, frames: 8 }, Profile { name: "all_commands_exercised", truth: all_commands_exercised, frames: 16 }];
        let t0 = Instant::now();
        let verdicts = sweep_over(&quick, &sample, &mut |_, _, _| {});
        let live = verdicts.iter().filter(|v| v.is_live()).count();
        let mut areas_all_dead: BTreeMap<String, (usize, usize)> = BTreeMap::new();
        for v in &verdicts {
            let e = areas_all_dead.entry(format!("{:?}", v.area)).or_default();
            e.0 += 1;
            if v.is_live() {
                e.1 += 1;
            }
        }
        println!("AUDIT sample {} areas of {} registered failures: live {}/{} in {:.1} s", sample.len(), r.failures.len(), live, verdicts.len(), t0.elapsed().as_secs_f64());
        for (a, (t, l)) in &areas_all_dead {
            println!("AUDIT   {a:<20} live {l:>3} / {t:>3}");
        }
        for v in verdicts.iter().filter(|v| !v.is_live()) {
            println!("AUDIT   dead sample {} {:?} ata{} {} | {} | {}", v.id, v.area, v.ata, v.name, v.component, v.model_field);
        }
        assert!(live * 2 >= verdicts.len(), "over half the one-per-area sample moves nothing published: {live} of {}", verdicts.len());
    }

    /// What a handful of diagnostic variables actually read in each
    /// profile, healthy.
    ///
    /// The sweep says a failure changes nothing; this says *why the state
    /// it was run in could not have shown it*. `BREAKERS_PROTECTING_NO_
    /// MODELLED_LOAD`, for instance, is the breakers area's own count of
    /// trip units whose protected circuit draws nothing at all -- a
    /// "fails to trip" on one of those is dead for a different reason
    /// than one on a breaker that is simply not overloaded today.
    #[test]
    fn what_the_diagnostic_variables_read_in_each_profile() {
        let names = ["BREAKERS_TOTAL", "BREAKERS_OPEN_COUNT", "BREAKERS_PROTECTING_NO_MODELLED_LOAD", "BREAKERS_LOCKED_OUT_COUNT", "FUEL_TOTAL_TRUE_FOB_KG", "DEEP_APU_N", "APU_N", "FIRE_DETECTED_ENG:1"];
        for p in profiles() {
            let base = baseline(&(p.truth)(), &reference_faults(), p.frames);
            let last = base.frames.last().expect("every profile runs at least one frame");
            let mut shown = Vec::new();
            for n in names {
                if let Some(i) = base.names.iter().position(|x| bare(x) == n) {
                    shown.push(format!("{n}={}", last[i]));
                }
            }
            println!("AUDIT profile {:<24} {} published | {}", p.name, base.names.len(), shown.join(" "));
            // Which breakers are open in a *healthy* aircraft in this
            // state. A trip unit that is already open is one no
            // nuisance-trip failure can trip again (`network::Breaker::
            // step` returns early on `!self.closed`), so this is the
            // other half of why some of those read dead.
            let open: Vec<&str> = base
                .names
                .iter()
                .enumerate()
                .filter(|(i, n)| n.starts_with("BKR_") && n.ends_with("_OPEN") && last[*i] != 0.0)
                .map(|(_, n)| n.as_str())
                .collect();
            if !open.is_empty() {
                println!("AUDIT   healthy-but-open breakers {}: {}", open.len(), open.iter().take(12).cloned().collect::<Vec<_>>().join(" "));
            }
        }
    }

    /// The full sweep: every registered failure, every profile. Minutes.
    ///
    ///     cargo test --release --lib deep_failure_audit_full_sweep -- --ignored --nocapture
    #[test]
    #[ignore = "minutes: the whole catalogue against the whole profile set"]
    fn deep_failure_audit_full_sweep() {
        let r = crate::deep::registry();
        let t0 = Instant::now();
        let verdicts = sweep(&r.failures, &mut |n, total, dead| {
            println!("AUDIT {n}/{total} dead so far {dead} ({:.0} s)", t0.elapsed().as_secs_f64());
        });
        let text = report(&verdicts);
        let path = report_path("deep_failure_audit.txt");
        std::fs::write(&path, &text).expect("write the audit report");
        for l in text.lines().take_while(|l| !l.starts_with("-- every dead failure")) {
            println!("AUDIT {l}");
        }
        println!("AUDIT full report at {}", path.display());
        println!("AUDIT took {:.0} s", t0.elapsed().as_secs_f64());
    }

    /// The ECAM side of the same bug: an alert whose trigger reads a
    /// variable nobody publishes can never fire, however well the failures
    /// behind it work.
    #[test]
    fn ecam_triggers_that_read_a_variable_no_area_publishes() {
        let r = crate::deep::registry();
        let published: std::collections::BTreeSet<String> = all_areas().published_names().iter().map(|n| bare(n).to_owned()).collect();
        let verdicts = trigger_verdicts(&r.alerts, &published);
        let never: Vec<&(String, Tri, Vec<String>)> = verdicts.iter().filter(|v| v.1 == Tri::Never).collect();
        let always: Vec<&(String, Tri, Vec<String>)> = verdicts.iter().filter(|v| v.1 == Tri::Always).collect();
        let partial: Vec<&(String, Tri, Vec<String>)> = verdicts.iter().filter(|v| v.1 == Tri::Reachable && !v.2.is_empty()).collect();

        let mut every_missing: BTreeMap<String, usize> = BTreeMap::new();
        for v in &verdicts {
            for m in &v.2 {
                *every_missing.entry(m.clone()).or_default() += 1;
            }
        }
        println!("AUDIT alerts {} | can NEVER fire {} | ALWAYS on {} | partly blind (an OR arm is dead) {} | distinct unpublished names {}", r.alerts.len(), never.len(), always.len(), partial.len(), every_missing.len());
        let title = |k: &str| r.alerts.iter().find(|a| a.key == k).map_or(String::new(), |a| a.title.clone());
        for (k, _, missing) in &never {
            println!("AUDIT never-fires {k} \"{}\" needs {missing:?}", title(k));
        }
        for (k, _, missing) in &always {
            println!("AUDIT always-on  {k} \"{}\" because {missing:?} read as 0", title(k));
        }
        for (k, _, missing) in &partial {
            println!("AUDIT partly-dead {k} \"{}\" dead arm needs {missing:?}", title(k));
        }

        let mut text = String::new();
        text.push_str(&format!("alerts {} | never fires {} | always on {} | partly dead {}

", r.alerts.len(), never.len(), always.len(), partial.len()));
        text.push_str("-- variables a trigger reads that nothing publishes, and how many alerts want them --
");
        for (m, n) in &every_missing {
            text.push_str(&format!("{n:>4}  {m}
"));
        }
        for (label, set) in [("NEVER FIRES", &never), ("ALWAYS ON", &always), ("PARTLY DEAD", &partial)] {
            text.push_str(&format!("
-- {label} --
"));
            for (k, _, missing) in set.iter() {
                text.push_str(&format!("{k} \"{}\"  missing {missing:?}
", title(k)));
            }
        }
        let path = report_path("deep_ecam_unpublished.txt");
        std::fs::write(&path, text).expect("write the ECAM report");
        println!("AUDIT ECAM report at {}", path.display());
    }

    /// An alert can also be dead the other way round: its trigger reads a
    /// variable that *is* published, but no failure on its `raised_by`
    /// list can move that variable anywhere near the threshold.
    ///
    ///     cargo test --release --lib alerts_whose_named_causes -- --ignored --nocapture
    #[test]
    #[ignore = "minutes: every failure any alert names, through every profile"]
    fn alerts_whose_named_causes_cannot_move_their_trigger() {
        let r = crate::deep::registry();
        // failure id -> every trigger variable of every alert naming it.
        let mut wanted: BTreeMap<u64, std::collections::BTreeSet<String>> = BTreeMap::new();
        for a in &r.alerts {
            let vars = trigger_vars(a);
            for id in &a.failures {
                wanted.entry(*id).or_default().extend(vars.iter().cloned());
            }
        }
        let total = wanted.len();
        let work: Vec<(u64, std::collections::BTreeSet<String>)> = wanted.into_iter().collect();
        let threads = worker_threads();
        let chunk = work.len().div_ceil(threads).max(1);
        println!("AUDIT cause-check {total} failures named by an alert, {threads} workers");
        let mut inert: Vec<(u64, Vec<String>)> = std::thread::scope(|scope| {
            let handles: Vec<_> = work
                .chunks(chunk)
                .map(|slice| {
                    scope.spawn(move || {
                        let profiles = profiles();
                        let baselines: Vec<Baseline> = profiles.iter().map(|p| baseline(&(p.truth)(), &reference_faults(), p.frames)).collect();
                        let truths: Vec<Truth> = profiles.iter().map(|p| (p.truth)()).collect();
                        let mut mine: Vec<(u64, Vec<String>)> = Vec::new();
                        for (id, vars) in slice {
                            let mut reached = false;
                            'search: for p in 0..profiles.len() {
                                for m in MAGNITUDES {
                                    let d = diff_against(&baselines[p], &truths[p], &armed_with(*id, m));
                                    if d.changed.iter().any(|&i| vars.contains(bare(&baselines[p].names[i]))) {
                                        reached = true;
                                        break 'search;
                                    }
                                }
                            }
                            if !reached {
                                mine.push((*id, vars.iter().cloned().collect()));
                            }
                        }
                        mine
                    })
                })
                .collect();
            handles.into_iter().flat_map(|h| h.join().expect("a cause-check worker panicked")).collect()
        });
        inert.sort_by_key(|(id, _)| *id);
        println!("AUDIT failures named as a cause of some alert: {total} | that never move any of that alert's trigger variables: {}", inert.len());
        let mut text = String::new();
        for (id, vars) in &inert {
            let f = r.failures.iter().find(|f| f.id == *id);
            let line = match f {
                Some(f) => format!("{id} {:?} ata{} | {} | {} | wanted {:?}
", f.area, f.ata, f.name, f.model_field, vars),
                None => format!("{id} <not registered> wanted {vars:?}
"),
            };
            print!("AUDIT inert-cause {line}");
            text.push_str(&line);
        }
        let path = report_path("deep_alert_causes.txt");
        std::fs::write(&path, text).expect("write the alert-cause report");
        println!("AUDIT alert-cause report at {}", path.display());
    }

    #[test]
    fn components_nothing_can_break_and_alerts_nothing_raises() {
        let r = crate::deep::registry();
        let orphans = components_without_failures(&r);
        println!("AUDIT components {} | with no failure at all {}", r.components.len(), orphans.len());
        for c in orphans.iter().take(80) {
            println!("AUDIT   component with no failures: {c}");
        }
        let unraised: Vec<&str> = r.alerts.iter().filter(|a| a.failures.is_empty()).map(|a| a.key.as_str()).collect();
        println!("AUDIT alerts {} | with no raised_by failure {}", r.alerts.len(), unraised.len());
        for k in unraised.iter().take(80) {
            println!("AUDIT   alert with no raised_by: {k}");
        }
    }

    /// The legacy-catalogue widening: runs on every `cargo test` (pure
    /// string scanning over the tree, no sweep, seconds not minutes), so
    /// unlike `deep_failure_audit_full_sweep` this one is never `#[ignore]`d
    /// and its verdict is not something a human has to remember to go run.
    ///
    /// Reverting `legacy_hook_families`/`id_literal_referenced`/
    /// `source_outside_failures_rs` to always report "referenced" (e.g. by
    /// forgetting to exclude `failures.rs`/this file from the scan, so every
    /// hook var or id trivially "self-matches" its own registration or this
    /// test's own comparisons) makes the `known_dead_family_has_a_nonempty_
    /// dead_ids_list` assertion fail: `FAIL_ADIRU_SENSOR_HOOK` would stop
    /// showing up as dead. Reverting it to always report "not referenced"
    /// (e.g. an empty/broken source scan) makes `known_wired_ids_are_not_
    /// flagged_dead` fail instead: `FAIL_FUEL_HOOK`'s ids would show up as
    /// dead even though `fuel_network.rs` demonstrably reads them.
    #[test]
    fn legacy_catalogue_hook_families_are_audited() {
        let source = source_outside_failures_rs();
        let families = legacy_hook_families(&source);
        assert!(!families.is_empty(), "extra_failures() must carry at least one Effect::Hook family for this audit to mean anything");

        let total_ids: usize = families.iter().map(|f| f.ids.len()).sum();
        let total_dead: usize = families.iter().map(|f| f.dead_ids.len()).sum();
        println!("AUDIT legacy hook families {} | ids {total_ids} | confirmed dead by this file's static check {total_dead}", families.len());
        println!(
            "AUDIT   (compare to the 38-across-seven-families / ~40-per-engine figures already established for this catalogue by other means)"
        );
        for f in &families {
            println!(
                "AUDIT   {:<28} {:<10} {:>3} ids  dead {:>3}  referenced_elsewhere(whole var)={}",
                f.var,
                f.owner,
                f.ids.len(),
                f.dead_ids.len(),
                f.referenced_elsewhere
            );
            if !f.dead_ids.is_empty() && f.dead_ids.len() != f.ids.len() {
                // A partly-dead family: this is exactly the
                // FAIL_ENGINE_COMPONENT_HOOK/FAIL_HYDRAULIC_HOOK shape --
                // the var name (or some sibling id) is cited somewhere, but
                // not every id in the family is.
                println!("AUDIT     dead ids: {:?}", f.dead_ids);
            }
        }

        // Pins this test to actually distinguishing three cases -- a fully
        // dead family, a fully wired one, and a partly-wired shared-var
        // family -- not reporting the same answer for everything (see the
        // doc comment above for what reverting each half looks like).
        let bus_short = families.iter().find(|f| f.var == "FAIL_BUS_SHORT_HOOK").expect("FAIL_BUS_SHORT_HOOK is in the catalogue today");
        assert_eq!(
            bus_short.dead_ids.len(),
            bus_short.ids.len(),
            "known_dead_family_has_a_nonempty_dead_ids_list: nothing outside failures.rs/this file reads any of FAIL_BUS_SHORT_HOOK's 18 ids today -- if this now fails, either it got wired (update the pin) or the exclusion of failures.rs/this file broke"
        );
        assert!(!bus_short.dead_ids.is_empty(), "sanity: the family above must be nonempty for the equality check to mean anything");
        // A finer pin on the bug this test itself caught mid-session:
        // FAIL_ENGINE_COMPONENT_HOOK's var name is never cited anywhere (so
        // `referenced_elsewhere` alone would call the whole 72-id family
        // dead), yet `engine_commands.rs:499` demonstrably reads id 79_004
        // by its bare number. The per-id check must find that regardless of
        // the whole-family signal -- an earlier version of this function
        // gated the per-id check on `referenced_elsewhere` and got this
        // exact case wrong.
        let engine_component = families.iter().find(|f| f.var == "FAIL_ENGINE_COMPONENT_HOOK").expect("FAIL_ENGINE_COMPONENT_HOOK is in the catalogue today");
        assert!(
            !engine_component.dead_ids.contains(&79_004),
            "known_wired_ids_are_not_flagged_dead: FAIL_ENGINE_COMPONENT_HOOK id 79_004 (engine 1 oil leak) is read by engine_commands.rs -- if this now fails, the per-id check regressed to being gated on the whole-family signal again"
        );
        assert!(
            engine_component.dead_ids.contains(&72_002),
            "FAIL_ENGINE_COMPONENT_HOOK id 72_002 (engine 3 bearing wear) is not read anywhere yet -- if this now fails, it got wired: update this pin and the finding in the audit report"
        );
        let fuel = families.iter().find(|f| f.var == "FAIL_FUEL_HOOK").expect("FAIL_FUEL_HOOK is in the catalogue today");
        assert!(fuel.referenced_elsewhere, "known_wired_ids_are_not_flagged_dead: fuel_network.rs demonstrably reads failures::magnitude for these ids");
        // FAIL_FUEL_HOOK turns out to be a genuinely mixed family, not a
        // fully-wired one: fuel_network.rs's own `refresh_catalogue_
        // failures` reads 28_000..28_011 (feed-tank-derate loop, trim pump,
        // both cross-feeds, jettison) but the mid/inner-tank pump ids
        // 28_102..28_109 (`failures.rs`'s own fuel() list) are not read by
        // that loop or anywhere else today -- a real, causal gap this
        // audit exists to surface, not a false positive to paper over.
        // Pinning one *specific*, demonstrably-consumed id (28_008, the
        // trim pump -- `fuel_network.rs:1808`) is the correct-granularity
        // check; asserting the whole family's `dead_ids` is empty would be
        // asserting something this catalogue does not actually make true.
        assert!(
            !fuel.dead_ids.contains(&28_008),
            "known_wired_ids_are_not_flagged_dead: FAIL_FUEL_HOOK id 28_008 (trim pump) is read by fuel_network.rs -- if this now fails, the source scan is broken"
        );
        assert!(
            fuel.dead_ids.contains(&28_102),
            "FAIL_FUEL_HOOK id 28_102 (mid tank fwd pump) is not read anywhere yet -- if this now fails, it got wired: update this pin and the finding in the audit report"
        );
    }
}
