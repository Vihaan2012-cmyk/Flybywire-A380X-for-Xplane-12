//! FlyByWire's A380X automatic fuel transfer logic and the APU fuel aspect,
//! ported to drive [`FuelNetwork`] in place of MSFS's fuel system.
//!
//! * [`LegacyFuel`] is a line-by-line port of
//!   `fbw-a380x/src/systems/systems-host/CpiomF/LegacyFuel.ts`. It is an
//!   `Instrument` on the systems-host backplane (SystemsHost.ts:191,213) and
//!   runs once per systems-host frame. The comments `LegacyFuel.ts:N` give the
//!   source line.
//! * [`ApuFuelAspect`] ports the `auxiliary_power_unit` aspect
//!   (`fbw-common/src/wasm/systems/systems_wasm/src/electrical.rs:49-101`),
//!   which the A380X registers as
//!   `with_auxiliary_power_unit(Variable::named("OVHD_APU_START_PB_IS_AVAILABLE"), 8, 21)`
//!   (`a380_systems_wasm/src/lib.rs:82`).
//!
//! # How the TS sees values: once per frame, before it runs
//! The msfs-sdk `InstrumentBackplane.onUpdate()` updates publishers, then
//! instruments (msfs-avionics-mirror `src/sdk/instruments/Backplane.ts:61-64`).
//! Every `ConsumerSubject` that LegacyFuel reads (tank quantities, trigger
//! statuses, junction settings, refuel flag, CG word, total weight) therefore
//! holds the value published at the **start** of the frame. Key events sent
//! through `KeyEventManager` are handled by the sim afterwards, so reading a
//! trigger straight after toggling it in the same `onUpdate` still returns the
//! old status. The TS depends on this, for example
//! `toggleTrigger(1); if (!triggerActive(1)) ...` at LegacyFuel.ts:232-233.
//! This port reproduces it: [`LegacyFuel::update`] takes a snapshot first,
//! makes every decision from the snapshot, and sends the events to the
//! network at once. They take effect on the next `FuelNetwork::update`, which
//! matches MSFS handling them before its next fuel-system step.
//!
//! # Where values come from
//! Values that MSFS's fuel system provided are read from the network with
//! `FuelNetwork::read_simvar`: `FUELSYSTEM TANK QUANTITY:1..11` (gallons),
//! `FUELSYSTEM TRIGGER STATUS:1..46` and `FUELSYSTEM JUNCTION SETTING:1..17`.
//! Everything else goes through [`FuelVars`], under the exact names FBW uses:
//! L:vars without the `L:` prefix, simvars as `"NAME"` or `"NAME:index"`.

use crate::fuel_network::FuelNetwork;

/// Variable access for everything the fuel network does not provide.
pub trait FuelVars {
    fn read(&mut self, name: &str) -> f64;
    fn write(&mut self, name: &str, value: f64);
}

// LegacyFuel.ts:30-35
const NUMBER_OF_TRIGGERS: usize = 46;
const NUMBER_OF_JUNCTIONS: usize = 17;
const NUMBER_OF_VALVES: usize = 59;
/// "These Valves are set to true in the FLT files so we dont want to set them to false."
const VALVES_TO_SKIP: [usize; 4] = [37, 40, 50, 51];
/// LegacyFuel.ts:61 `new UpdateThrottler(250)`.
const UPDATE_INTERVAL_MS: f64 = 250.0;

/// `ValveState` (LegacyFuel.ts:16-19).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ValveState {
    Closed = 0,
    #[allow(dead_code)] // LegacyFuel.ts only ever closes valves (:215).
    Open = 1,
}

/// Port of fbw-common `UpdateThrottler` (systems/shared/src/UpdateThrottler.ts:7-42).
///
/// The TS starts from a random `refreshOffset` in [0, interval) so instruments
/// update on different frames (UpdateThrottler.ts:13). This port starts at 0
/// by default and exposes [`UpdateThrottler::with_offset`] to set one.
#[derive(Clone, Debug)]
pub struct UpdateThrottler {
    interval_ms: f64,
    current_time: f64,
    last_update_time: f64,
    refresh_offset: f64,
    refresh_number: f64,
}

impl UpdateThrottler {
    pub fn new(interval_ms: f64) -> Self {
        Self::with_offset(interval_ms, 0.0)
    }
    pub fn with_offset(interval_ms: f64, refresh_offset_ms: f64) -> Self {
        Self {
            interval_ms,
            current_time: 0.0,
            last_update_time: 0.0,
            refresh_offset: refresh_offset_ms.floor(),
            refresh_number: 0.0,
        }
    }
    /// UpdateThrottler.ts:30-42. Returns -1 when this frame should not update,
    /// otherwise the milliseconds since the last update.
    pub fn can_update(&mut self, delta_time_ms: f64, force_update: bool) -> f64 {
        self.current_time += delta_time_ms;
        let number = ((self.current_time + self.refresh_offset) / self.interval_ms).floor();
        let update = number > self.refresh_number;
        self.refresh_number = number;
        if update || force_update {
            let accumulated = self.current_time - self.last_update_time;
            self.last_update_time = self.current_time;
            accumulated
        } else {
            -1.0
        }
    }
}

/// Decoded FBW ARINC 429 word (fbw-common systems/shared/src/arinc429.ts:43-107).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Arinc429 {
    pub ssm: u8,
    pub value: f32,
}

impl Arinc429 {
    /// arinc429.ts:51-55: the value is the float32 held in the low 32 bits,
    /// the SSM is `trunc(raw / 2^32) & 0b11`.
    pub fn from_raw(raw: f64) -> Self {
        // JS `rawWord & 0xffffffff` is ToInt32, i.e. the value modulo 2^32.
        let low = raw.trunc().rem_euclid(4_294_967_296.0) as u32;
        let ssm = ((raw / 4_294_967_296.0).trunc() as i64 & 0b11) as u8;
        Self { ssm, value: f32::from_bits(low) }
    }
    /// arinc429.ts:66-69 `getRawWord`.
    pub fn to_raw(value: f32, ssm: u8) -> f64 {
        value.to_bits() as f64 + (ssm & 0b11) as f64 * 4_294_967_296.0
    }
    /// arinc429.ts:106-108: the value when NormalOperation (0b11) or
    /// FunctionalTest (0b10), otherwise `default`.
    pub fn value_or(&self, default: f64) -> f64 {
        if self.ssm == 0b11 || self.ssm == 0b10 {
            self.value as f64
        } else {
            default
        }
    }
}

/// JS `Math.round` (rounds halves towards +infinity).
fn js_round(x: f64) -> f64 {
    (x + 0.5).floor()
}

/// fbw-sdk `MathUtils.round(value, precision)` = `Math.round(value / precision) * precision`.
fn math_utils_round(value: f64, precision: f64) -> f64 {
    js_round(value / precision) * precision
}

/// The values LegacyFuel's ConsumerSubjects hold for one frame (see module docs).
#[derive(Clone, Debug)]
struct Snapshot {
    /// `fuel_tank_quantity_1..11` (LegacyFuel.ts:41-51), gallons. Index 0 unused.
    tank: [f64; 12],
    /// `fuel_trigger_status_N` (LegacyFuel.ts:91-94). Index 0 unused.
    trigger: [bool; NUMBER_OF_TRIGGERS + 1],
    /// `fuel_junction_setting_N` (LegacyFuel.ts:96-99). Index 0 unused.
    junction: [f64; NUMBER_OF_JUNCTIONS + 1],
    /// `fuel_refuel_started_by_user` = L:A32NX_REFUEL_STARTED_BY_USR (FuelSystemPublisher.ts:100-102).
    refuel_started: bool,
    /// `fqms_center_of_gravity_mac` = L:A32NX_FQMS_CENTER_OF_GRAVITY_MAC (FqmsBusPublisher.ts:81).
    cg_percent: Arinc429,
    /// `total_weight` = A:TOTAL WEIGHT, pounds (msfs-sdk WeightAndBalance.ts:50).
    aircraft_weight_lbs: f64,
}

/// Port of `LegacyFuel` (LegacyFuel.ts:29-653).
#[derive(Clone, Debug)]
pub struct LegacyFuel {
    throttler: UpdateThrottler,
    refuel_in_progress: bool,
    has_init: bool,
    init_called: bool,
    /// LegacyFuel.ts:68-73 `trimTransfersActiveForFeedTank`, index 1..4.
    trim_transfers_active_for_feed_tank: [bool; 5],
    /// LegacyFuel.ts:76-81 `innerAndMidTransfersActiveForFeedTank`, index 1..4.
    inner_and_mid_transfers_active_for_feed_tank: [bool; 5],
    snap: Snapshot,
}

impl Default for LegacyFuel {
    fn default() -> Self {
        Self::new()
    }
}

impl LegacyFuel {
    /// LegacyFuel.ts:83-100.
    pub fn new() -> Self {
        Self {
            throttler: UpdateThrottler::new(UPDATE_INTERVAL_MS),
            refuel_in_progress: false,
            has_init: false,
            init_called: false,
            trim_transfers_active_for_feed_tank: [false; 5],
            inner_and_mid_transfers_active_for_feed_tank: [false; 5],
            snap: Snapshot {
                tank: [0.0; 12],
                trigger: [false; NUMBER_OF_TRIGGERS + 1],
                junction: [1.0; NUMBER_OF_JUNCTIONS + 1],
                refuel_started: false,
                cg_percent: Arinc429 { ssm: 0, value: 0.0 },
                aircraft_weight_lbs: 0.0,
            },
        }
    }

    /// Replaces the throttler, e.g. to give it the random offset the TS uses.
    pub fn set_throttler(&mut self, throttler: UpdateThrottler) {
        self.throttler = throttler;
    }

    /// Whether the in-game initialisation (LegacyFuel.ts:105-108) has run.
    pub fn has_init(&self) -> bool {
        self.has_init
    }

    /// `init()` (LegacyFuel.ts:102-109), called once when the instrument is
    /// added. The `Wait.awaitSubscribable(GameStateProvider ... ingame)` part
    /// runs in [`LegacyFuel::update`] on the first frame with `in_game`.
    pub fn init(&mut self, vars: &mut dyn FuelVars) {
        // LegacyFuel.ts:103-104
        let fuel_weight = vars.read("A32NX_TOTAL_FUEL_QUANTITY");
        vars.write("A32NX_FUEL_DESIRED", fuel_weight);
        self.init_called = true;
    }

    fn take_snapshot(&mut self, vars: &mut dyn FuelVars, net: &FuelNetwork) {
        for i in 1..=11 {
            self.snap.tank[i] = net.read_simvar("FUELSYSTEM TANK QUANTITY", i).unwrap_or(0.0);
        }
        for i in 1..=NUMBER_OF_TRIGGERS {
            self.snap.trigger[i] = net.read_simvar("FUELSYSTEM TRIGGER STATUS", i).unwrap_or(0.0) != 0.0;
        }
        for i in 1..=NUMBER_OF_JUNCTIONS {
            self.snap.junction[i] = net.read_simvar("FUELSYSTEM JUNCTION SETTING", i).unwrap_or(1.0);
        }
        self.snap.refuel_started = vars.read("A32NX_REFUEL_STARTED_BY_USR") != 0.0;
        self.snap.cg_percent = Arinc429::from_raw(vars.read("A32NX_FQMS_CENTER_OF_GRAVITY_MAC"));
        self.snap.aircraft_weight_lbs = vars.read("TOTAL WEIGHT");
    }

    /// `onUpdate()` (LegacyFuel.ts:193-581). Call once per systems frame.
    /// `delta_time_ms` is `BaseInstrument.deltaTime`; `in_game` is
    /// `GameStateProvider` in the `ingame` state (in X-Plane: the flight is
    /// loaded and running).
    pub fn update(&mut self, delta_time_ms: f64, in_game: bool, vars: &mut dyn FuelVars, net: &mut FuelNetwork) {
        self.take_snapshot(vars, net);

        // LegacyFuel.ts:194-195
        let dt = delta_time_ms;
        let total_dt_since_last_update = self.throttler.can_update(dt, false);

        // LegacyFuel.ts:197-199
        if !self.has_init {
            // LegacyFuel.ts:105-108: the awaited game state resolves between frames.
            if self.init_called && in_game {
                self.check_empty_triggers(net);
                self.has_init = true;
            }
            return;
        }

        // LegacyFuel.ts:201
        let on_ground = vars.read("SIM ON GROUND") != 0.0;
        if !self.refuel_in_progress && self.snap.refuel_started {
            // LegacyFuel.ts:202-217
            self.refuel_in_progress = true;
            for index in 1..=NUMBER_OF_TRIGGERS {
                if self.snap.trigger[index] {
                    net.handle_key_event("FUELSYSTEM_TRIGGER_OFF", index as u32, 0);
                }
            }
            // "starts at 5 since 1-4 are the engine valves controlled by the engine masters"
            for index in 5..=NUMBER_OF_VALVES {
                if !VALVES_TO_SKIP.contains(&index) {
                    Self::set_valve(net, index, ValveState::Closed);
                }
            }
        } else if self.refuel_in_progress && !self.snap.refuel_started {
            // LegacyFuel.ts:218-221
            self.refuel_in_progress = false;
            self.check_empty_triggers(net);
        } else if !on_ground && total_dt_since_last_update > 0.0 {
            // LegacyFuel.ts:222-223
            self.check_empty_triggers(net);

            // LegacyFuel.ts:225-226
            let cg_target_start = Self::calculate_cg_target(self.snap.aircraft_weight_lbs / 1000.0);
            let cg_target_stop = cg_target_start - 1.0;

            let t = self.snap.tank;
            let feed1 = t[2];
            let feed2 = t[5];
            let feed3 = t[6];
            let feed4 = t[9];
            let left_mid = t[3];
            let right_mid = t[8];

            // LegacyFuel.ts:228-234
            if (feed1 < 6436.0 && !self.active(1) && !self.inner_and_mids_empty())
                || (feed1 >= 6437.0 && self.active(1))
            {
                self.toggle_trigger(net, 1);
                if !self.active(1) {
                    self.inner_and_mid_transfers_active_for_feed_tank[1] = true;
                }
            }
            // LegacyFuel.ts:235-244
            if (feed2 < 6857.0 && !self.active(2) && !self.inner_and_mids_empty() && !self.active(13))
                || (feed2 >= 6858.0 && self.active(2))
            {
                self.toggle_trigger(net, 2);
                if !self.active(2) {
                    self.inner_and_mid_transfers_active_for_feed_tank[2] = true;
                }
            }
            // LegacyFuel.ts:245-254
            if (feed3 < 6857.0 && !self.active(3) && !self.inner_and_mids_empty() && !self.active(13))
                || (feed3 >= 6858.0 && self.active(3))
            {
                self.toggle_trigger(net, 3);
                if !self.active(3) {
                    self.inner_and_mid_transfers_active_for_feed_tank[3] = true;
                }
            }
            // LegacyFuel.ts:255-261
            if (feed4 < 6436.0 && !self.active(4) && !self.inner_and_mids_empty())
                || (feed4 >= 6437.0 && self.active(4))
            {
                self.toggle_trigger(net, 4);
                if !self.active(4) {
                    self.inner_and_mid_transfers_active_for_feed_tank[4] = true;
                }
            }
            // LegacyFuel.ts:262-275
            if ((feed1 - feed4).abs() < 0.5
                && feed1 < 6764.0
                && feed4 < 6764.0
                && !self.active(5)
                && (self.inner_and_mid_transfers_active_for_feed_tank[1]
                    || self.inner_and_mid_transfers_active_for_feed_tank[4]))
                || ((feed1 - feed4).abs() >= 1.0 && self.active(5))
            {
                self.toggle_trigger(net, 5);
                if !self.active(5) {
                    self.inner_and_mid_transfers_active_for_feed_tank[1] = true;
                    self.inner_and_mid_transfers_active_for_feed_tank[4] = true;
                }
            }
            // LegacyFuel.ts:276-289
            if ((feed2 - feed3).abs() < 0.5
                && ((!self.active(13) && feed2 < 7185.0 && feed3 < 7185.0)
                    || (self.active(13) && feed2 < 6764.0 && feed3 < 6764.0))
                && !self.active(6)
                && (self.inner_and_mid_transfers_active_for_feed_tank[2]
                    || self.inner_and_mid_transfers_active_for_feed_tank[3]))
                || ((feed2 - feed3).abs() >= 1.0 && self.active(6))
            {
                self.toggle_trigger(net, 6);
                if !self.active(6) {
                    self.inner_and_mid_transfers_active_for_feed_tank[2] = true;
                    self.inner_and_mid_transfers_active_for_feed_tank[3] = true;
                }
            }
            // LegacyFuel.ts:290-317
            for (trigger, qty, above, below, tank) in [
                (7, feed1, 6765.0, 6764.0, 1),
                (8, feed2, 7186.0, 7186.0, 2),
                (9, feed3, 7186.0, 7186.0, 3),
                (10, feed4, 6765.0, 6764.0, 4),
            ] {
                if (qty > above && !self.active(trigger)) || (qty <= below && self.active(trigger)) {
                    self.toggle_trigger(net, trigger);
                    if !self.active(trigger) {
                        self.inner_and_mid_transfers_active_for_feed_tank[tank] = false;
                    }
                }
            }
            // LegacyFuel.ts:318-323
            if (left_mid + right_mid < 2632.0 && !self.active(13))
                || (left_mid + right_mid >= 2634.0 && self.active(13))
            {
                self.toggle_trigger(net, 13);
            }
            // LegacyFuel.ts:324-332
            if (feed2 < 6436.0 && !self.active(14) && self.active(13)) || (feed2 >= 6437.0 && self.active(14)) {
                self.toggle_trigger(net, 14);
                if !self.active(14) {
                    self.inner_and_mid_transfers_active_for_feed_tank[2] = true;
                }
            }
            // LegacyFuel.ts:333-341 (no trigger 13 check here, as in the TS)
            if (feed3 < 6436.0 && !self.active(15)) || (feed3 >= 6437.0 && self.active(15)) {
                self.toggle_trigger(net, 15);
                if !self.active(15) {
                    self.inner_and_mid_transfers_active_for_feed_tank[3] = true;
                }
            }
            // LegacyFuel.ts:342-359
            for (trigger, qty, tank) in [(16, feed2, 2), (17, feed3, 3)] {
                if (qty > 6765.0 && !self.active(trigger)) || (qty <= 6764.0 && self.active(trigger)) {
                    self.toggle_trigger(net, trigger);
                    if !self.active(trigger) {
                        self.inner_and_mid_transfers_active_for_feed_tank[tank] = false;
                    }
                }
            }
            // LegacyFuel.ts:360-419
            for (trigger, a, qa, b, qb) in [
                (18, 1, feed1, 3, feed3),
                (19, 1, feed1, 2, feed2),
                (20, 2, feed2, 4, feed4),
                (21, 3, feed3, 4, feed4),
            ] {
                if (qa < 6764.0
                    && qb < 6764.0
                    && (qa - qb).abs() < 0.5
                    && !self.active(trigger)
                    && self.active(13)
                    && (self.inner_and_mid_transfers_active_for_feed_tank[a]
                        || self.inner_and_mid_transfers_active_for_feed_tank[b]))
                    || ((qa - qb).abs() >= 1.0 && self.active(trigger))
                {
                    self.toggle_trigger(net, trigger);
                    if !self.active(trigger) {
                        self.inner_and_mid_transfers_active_for_feed_tank[a] = true;
                        self.inner_and_mid_transfers_active_for_feed_tank[b] = true;
                    }
                }
            }
            // LegacyFuel.ts:420-447
            for (trigger, qty, tank) in [(24, feed1, 1), (25, feed2, 2), (26, feed3, 3), (27, feed4, 4)] {
                if (qty < 1974.0 && !self.active(trigger) && !self.active(34)) || (qty >= 1975.0 && self.active(trigger)) {
                    self.toggle_trigger(net, trigger);
                    if !self.active(trigger) {
                        self.trim_transfers_active_for_feed_tank[tank] = true;
                    }
                }
            }
            // LegacyFuel.ts:448-519
            for (trigger, a, qa, b, qb) in [
                (28, 1, feed1, 3, feed3),
                (29, 1, feed1, 2, feed2),
                (30, 2, feed2, 4, feed4),
                (31, 3, feed3, 4, feed4),
                (32, 1, feed1, 4, feed4),
                (33, 2, feed2, 3, feed3),
            ] {
                if ((qa - qb).abs() < 0.5 && !self.active(trigger) && self.tank_lowest_and_trim_transfer_active(a, b))
                    || ((qa - qb).abs() >= 1.0 && self.active(trigger))
                {
                    self.toggle_trigger(net, trigger);
                    if !self.active(trigger) {
                        self.trim_transfers_active_for_feed_tank[a] = true;
                        self.trim_transfers_active_for_feed_tank[b] = true;
                    }
                }
            }
            // LegacyFuel.ts:520-543 (the order is 35 feed1, 36 feed2, 37 feed4, 38 feed3)
            for (trigger, qty) in [(35, feed1), (36, feed2), (37, feed4), (38, feed3)] {
                if (qty < 1316.0 && !self.active(trigger)) || (qty >= 1317.0 && self.active(trigger)) {
                    self.toggle_trigger(net, trigger);
                }
            }
            // LegacyFuel.ts:544-567
            for (trigger, qty) in [(39, feed1), (40, feed2), (41, feed3), (42, feed4)] {
                if (qty > 1481.0 && !self.active(trigger)) || (qty <= 1480.0 && self.active(trigger)) {
                    self.toggle_trigger(net, trigger);
                }
            }
            // LegacyFuel.ts:568-573
            let cg = self.snap.cg_percent;
            if (cg.value_or(0.0) > cg_target_start && !self.active(43) && !self.active(34))
                || (cg.value_or(f64::INFINITY) <= cg_target_start - 0.1 && self.active(43))
            {
                self.toggle_trigger(net, 43);
            }
            // LegacyFuel.ts:574-579
            if (cg.value_or(f64::INFINITY) < cg_target_stop && !self.active(44))
                || (cg.value_or(0.0) >= cg_target_stop + 0.1 && self.active(44))
            {
                self.toggle_trigger(net, 44);
            }
        }
    }

    /// `checkEmptyTriggers()` (LegacyFuel.ts:111-192).
    fn check_empty_triggers(&mut self, net: &mut FuelNetwork) {
        let t = self.snap.tank;
        let (left_outer, left_mid, left_inner) = (t[1], t[3], t[4]);
        let (right_inner, right_mid, right_outer, trim) = (t[7], t[8], t[10], t[11]);

        // LegacyFuel.ts:112-151
        for (trigger, qty) in [
            (11, left_inner),
            (12, right_inner),
            (22, left_mid),
            (23, right_mid),
            (45, left_outer),
            (46, right_outer),
        ] {
            if (qty < 0.1 && !self.active(trigger)) || (qty >= 1.0 && self.active(trigger)) {
                self.toggle_trigger(net, trigger);
            }
        }

        // LegacyFuel.ts:153-179
        if right_inner < 0.1 && left_inner < 0.1 && right_mid < 0.1 && left_mid < 0.1 {
            // "both mid and inner tanks are empty"
            self.set_junction_option(net, 10, 3);
            // "terminate all inner and mid transfers"
            for trigger in [7, 8, 9, 10] {
                if !self.active(trigger) {
                    self.toggle_trigger(net, trigger);
                }
            }
            for i in 1..5 {
                self.inner_and_mid_transfers_active_for_feed_tank[i] = false;
            }
        } else if right_inner >= 0.1 || left_inner >= 0.1 {
            // "inner tanks arent empty"
            self.set_junction_option(net, 10, 1);
        } else {
            // "mid tanks arent empty but inner tanks are"
            self.set_junction_option(net, 10, 2);
        }

        // LegacyFuel.ts:181-191
        if (trim < 0.1 && !self.active(34)) || (trim >= 1.0 && self.active(34)) {
            self.toggle_trigger(net, 34);
            if !self.active(34) {
                for i in 1..5 {
                    self.trim_transfers_active_for_feed_tank[i] = false;
                }
            }
        }
    }

    /// `toggleTrigger` (LegacyFuel.ts:582-586).
    fn toggle_trigger(&self, net: &mut FuelNetwork, index: usize) {
        net.handle_key_event("FUELSYSTEM_TRIGGER_TOGGLE", index as u32, 0);
    }

    /// `setJunctionOption` (LegacyFuel.ts:588-592).
    fn set_junction_option(&self, net: &mut FuelNetwork, index: usize, option: usize) {
        if self.snap.junction[index] != option as f64 {
            net.handle_key_event("FUELSYSTEM_JUNCTION_SET", index as u32, option as u32);
        }
    }

    /// `setValve` (LegacyFuel.ts:594-596).
    fn set_valve(net: &mut FuelNetwork, index: usize, state: ValveState) {
        net.handle_key_event("FUELSYSTEM_VALVE_SET", index as u32, state as u32);
    }

    /// `TankLowestAndTrimTransferActive` (LegacyFuel.ts:605-619).
    fn tank_lowest_and_trim_transfer_active(&self, tank1: usize, tank2: usize) -> bool {
        let t = self.snap.tank;
        let feed = [t[2], t[5], t[6], t[9]];
        let lowest = feed.iter().copied().fold(f64::INFINITY, f64::min);
        (feed[tank1 - 1] <= lowest + 3.0 || feed[tank2 - 1] <= lowest + 3.0)
            && (self.trim_transfers_active_for_feed_tank[tank1] || self.trim_transfers_active_for_feed_tank[tank2])
    }

    /// `calculateCGTarget` (LegacyFuel.ts:625-635). `weight` in thousands of pounds.
    pub fn calculate_cg_target(weight: f64) -> f64 {
        // "coefficients determined using regression on FCOM diagram"
        let target = 1.52792360195336e-14 * weight.powi(5) - 7.7447769532209e-11 * weight.powi(4)
            + 1.57545973208929e-7 * weight.powi(3)
            - 0.000162820304673144 * weight.powi(2)
            + 0.0884071656630996 * weight
            + 20.6522282591408;
        math_utils_round(target, 0.01)
    }

    /// `triggerActive` (LegacyFuel.ts:637-639), from the frame snapshot.
    fn active(&self, index: usize) -> bool {
        self.snap.trigger[index]
    }

    /// `innerAndMidsEmpty` (LegacyFuel.ts:650-652) = `triggerActiveAll(11, 12, 22, 23)`.
    fn inner_and_mids_empty(&self) -> bool {
        [11, 12, 22, 23].iter().all(|&i| self.active(i))
    }

    /// `trimTransfersActiveForFeedTank` (tank 1..4), for inspection.
    pub fn trim_transfer_active_for_feed_tank(&self, tank: usize) -> bool {
        self.trim_transfers_active_for_feed_tank.get(tank).copied().unwrap_or(false)
    }
    /// `innerAndMidTransfersActiveForFeedTank` (tank 1..4), for inspection.
    pub fn inner_and_mid_transfer_active_for_feed_tank(&self, tank: usize) -> bool {
        self.inner_and_mid_transfers_active_for_feed_tank.get(tank).copied().unwrap_or(false)
    }
}

// ---------------------------------------------------------------------------
// APU fuel aspect
// ---------------------------------------------------------------------------

/// The A380X's APU fuel valve and pump numbers (a380_systems_wasm/src/lib.rs:82).
/// Valve 8 is FeedTank2FwdTransferValve1_2 in flight_model.cfg. It looks like
/// a leftover from the A32NX, and the A380X's real APU valves are 50/51,
/// which the `.flt` files open. The port keeps 8 to match FBW's behaviour.
pub const A380_APU_FUEL_VALVE: usize = 8;
pub const A380_APU_FUEL_PUMP: usize = 21;

/// Port of the `auxiliary_power_unit` aspect (systems_wasm/src/electrical.rs:49-91).
///
/// The aspect observes five variables (electrical.rs:57-63):
/// `L:A32NX_OVHD_APU_START_PB_IS_AVAILABLE`, `A:APU SWITCH`, `A:BLEED AIR APU`,
/// `L:A32NX_ASU_TURNED_ON` and `L:A32NX_APU_BLEED_AIR_VALVE_OPEN`. It runs
/// its closure after the systems tick (`ExecuteOn::PostTick`) whenever any of
/// them differs from its value the previous time (aspects.rs:966-987). The
/// starting values are read when the aspect is registered (aspects.rs:241-248).
///
/// The events it sends that are not fuel events control MSFS's own APU, which
/// X-Plane does not have. They are emulated through [`FuelVars`] writes:
/// `KEY_APU_STARTER` writes `APU SWITCH` = 1, `KEY_APU_OFF_SWITCH` writes
/// `APU SWITCH` = 0, and `KEY_APU_BLEED_AIR_SOURCE_SET` writes `BLEED AIR APU`.
/// The plugin must keep those two values as its "MSFS APU" state.
#[derive(Clone, Debug)]
pub struct ApuFuelAspect {
    fuel_valve_number: usize,
    fuel_pump_number: usize,
    previous: [f64; 5],
}

const APU_OBSERVED: [&str; 5] = [
    "A32NX_OVHD_APU_START_PB_IS_AVAILABLE",
    "APU SWITCH",
    "BLEED AIR APU",
    "A32NX_ASU_TURNED_ON",
    "A32NX_APU_BLEED_AIR_VALVE_OPEN",
];

impl ApuFuelAspect {
    /// Registers the aspect with the A380X numbers (valve 8, pump 21),
    /// reading the starting values now.
    pub fn new_a380(vars: &mut dyn FuelVars) -> Self {
        Self::new(vars, A380_APU_FUEL_VALVE, A380_APU_FUEL_PUMP)
    }

    pub fn new(vars: &mut dyn FuelVars, fuel_valve_number: usize, fuel_pump_number: usize) -> Self {
        let mut previous = [0.0; 5];
        for (p, name) in previous.iter_mut().zip(APU_OBSERVED) {
            *p = vars.read(name);
        }
        Self { fuel_valve_number, fuel_pump_number, previous }
    }

    /// Call once per tick, after FBW's systems update (PostTick).
    pub fn update(&mut self, vars: &mut dyn FuelVars, net: &mut FuelNetwork) {
        let mut current = [0.0; 5];
        for (c, name) in current.iter_mut().zip(APU_OBSERVED) {
            *c = vars.read(name);
        }
        let changed = self.previous.iter().zip(current.iter()).any(|(p, c)| p != c);
        self.previous = current;
        if !changed {
            return;
        }
        // electrical.rs:64-89 (`to_bool` = value != 0)
        let is_available = current[0] != 0.0;
        let msfs_apu_is_on = current[1] != 0.0;
        let msfs_apu_bleed_on = current[2] != 0.0;
        let asu_turned_on = current[3] != 0.0;
        let apu_bleed_valve_open = current[4] != 0.0;

        if (is_available || asu_turned_on) && !msfs_apu_is_on {
            self.set_fuel_valve_and_pump(net, true);
            // start_apu(): KEY_APU_STARTER (electrical.rs:103-107)
            vars.write("APU SWITCH", 1.0);
        } else if !is_available && !asu_turned_on && msfs_apu_is_on {
            self.set_fuel_valve_and_pump(net, false);
            // stop_apu(): KEY_APU_OFF_SWITCH (electrical.rs:109-111)
            vars.write("APU SWITCH", 0.0);
        }

        if ((is_available && apu_bleed_valve_open) || asu_turned_on) && !msfs_apu_bleed_on {
            // supply_bleed(true): KEY_APU_BLEED_AIR_SOURCE_SET 1 (electrical.rs:113-115)
            vars.write("BLEED AIR APU", 1.0);
        } else if (!is_available || !apu_bleed_valve_open) && !asu_turned_on && msfs_apu_bleed_on {
            vars.write("BLEED AIR APU", 0.0);
        }
    }

    /// `set_fuel_valve_and_pump` (electrical.rs:94-101).
    fn set_fuel_valve_and_pump(&self, net: &mut FuelNetwork, on: bool) {
        if on {
            net.handle_key_event("FUELSYSTEM_VALVE_OPEN", self.fuel_valve_number as u32, 0);
            net.handle_key_event("FUELSYSTEM_PUMP_ON", self.fuel_pump_number as u32, 0);
        } else {
            net.handle_key_event("FUELSYSTEM_VALVE_CLOSE", self.fuel_valve_number as u32, 0);
            net.handle_key_event("FUELSYSTEM_PUMP_OFF", self.fuel_pump_number as u32, 0);
        }
    }
}

/// The fuel MSFS's APU would burn this tick: `APU.1 FuelBurnRate` while the
/// emulated `APU SWITCH` is on. Pass it to `FuelNetwork::update` as the APU
/// demand.
///
/// `failures.rs` `extra::apu` 49_001 ("APU fuel control fault") was
/// previously an unconsumed `Effect::Hook`. The FCU "no longer meters fuel
/// correctly" becomes a real over-fuel bias here: demand rises 35% above
/// the commanded burn rate, so the trim/feed tanks it draws from drain
/// faster and the APU runs rich -- not an isolated cosmetic flag, but the
/// same leaning error that feeds 49_000's own EGT-overtemperature damage
/// path once it runs hot for long enough.
pub fn apu_fuel_demand_gph(vars: &mut dyn FuelVars, net: &FuelNetwork) -> f64 {
    if vars.read("APU SWITCH") != 0.0 {
        let demand = net.apu_burn_rate_gph();
        if crate::failures::is_active(49_001) {
            demand * 1.35
        } else {
            demand
        }
    } else {
        0.0
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    const CFG: &str = r"D:\fbw-aircraft\fbw-a380x\src\base\flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380X\common\config\flight_model.cfg";
    const FLT_DIR: &str = r"D:\fbw-aircraft\fbw-a380x\src\base\flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380X\common\flt";

    #[derive(Default)]
    struct FakeVars {
        v: HashMap<String, f64>,
        writes: Vec<(String, f64)>,
    }
    impl FuelVars for FakeVars {
        fn read(&mut self, name: &str) -> f64 {
            *self.v.get(name).unwrap_or(&0.0)
        }
        fn write(&mut self, name: &str, value: f64) {
            self.v.insert(name.to_string(), value);
            self.writes.push((name.to_string(), value));
        }
    }

    fn network(flt: &str) -> Option<FuelNetwork> {
        let text = std::fs::read_to_string(CFG).ok()?;
        let mut net = FuelNetwork::from_cfg(&text).unwrap();
        if let Ok(f) = std::fs::read_to_string(format!(r"{FLT_DIR}\{flt}")) {
            net.apply_flt_state(&f);
        } else {
            for v in [1, 2, 3, 4, 37, 40] {
                net.open_valve(v);
            }
            for p in [1, 2, 3, 4, 5, 6, 7, 8, 22, 23, 24, 25] {
                net.pump_on(p);
            }
            net.update(5.0, [0.0; 4], 0.0);
        }
        Some(net)
    }

    const DT_MS: f64 = 1000.0 / 30.0;

    fn frames(
        fuel: &mut LegacyFuel,
        vars: &mut FakeVars,
        net: &mut FuelNetwork,
        seconds: f64,
        demand: f64,
        mut each: impl FnMut(&FuelNetwork),
    ) {
        let n = (seconds * 1000.0 / DT_MS).round() as usize;
        for _ in 0..n {
            fuel.update(DT_MS, true, vars, net);
            net.update(DT_MS / 1000.0, [demand; 4], 0.0);
            each(net);
        }
    }

    fn in_flight_vars() -> FakeVars {
        let mut vars = FakeVars::default();
        vars.v.insert("SIM ON GROUND".into(), 0.0);
        vars.v.insert("TOTAL WEIGHT".into(), 900_000.0);
        // CG word with SSM NoComputedData: no CG transfers.
        vars.v.insert("A32NX_FQMS_CENTER_OF_GRAVITY_MAC".into(), Arinc429::to_raw(40.0, 0b01));
        vars.v.insert("A32NX_TOTAL_FUEL_QUANTITY".into(), 123_456.0);
        vars
    }

    #[test]
    fn throttler_and_arinc_helpers_match_the_ts() {
        let mut t = UpdateThrottler::new(250.0);
        let updates: Vec<f64> = (0..30).map(|_| t.can_update(20.0, false)).filter(|&d| d > 0.0).collect();
        // 600 ms in 20 ms frames: updates at 260 ms and 500 ms.
        assert_eq!(updates.len(), 2);
        assert!((updates[0] - 260.0).abs() < 1e-9 && (updates[1] - 240.0).abs() < 1e-9);
        let w = Arinc429::from_raw(Arinc429::to_raw(31.25, 0b11));
        assert_eq!((w.ssm, w.value), (3, 31.25));
        assert_eq!(Arinc429::from_raw(Arinc429::to_raw(31.25, 0b01)).value_or(-1.0), -1.0);
        assert_eq!(Arinc429::from_raw(Arinc429::to_raw(-5.5, 0b10)).value_or(0.0), -5.5);
        assert!((LegacyFuel::calculate_cg_target(900.0) - 41.39).abs() < 1e-9);
        assert_eq!(js_round(2.5), 3.0);
        assert_eq!(js_round(-2.5), -2.0);
    }

    #[test]
    fn init_copies_total_fuel_and_waits_for_in_game() {
        let Some(mut net) = network("cruise.FLT") else { return };
        let mut vars = in_flight_vars();
        for t in 1..=16 {
            net.set_tank_gallons(t, 0.0);
        }
        let mut fuel = LegacyFuel::new();
        fuel.init(&mut vars);
        assert_eq!(vars.v["A32NX_FUEL_DESIRED"], 123_456.0);
        fuel.update(DT_MS, false, &mut vars, &mut net);
        assert!(!fuel.has_init());
        // With empty tanks, checkEmptyTriggers flags inner, mid, outer and trim as empty.
        fuel.update(DT_MS, true, &mut vars, &mut net);
        assert!(fuel.has_init());
        for t in [11, 12, 22, 23, 34, 45, 46] {
            assert!(net.trigger_status(t), "trigger {t}");
        }
        assert_eq!(net.junction_setting(10), 3);
        for t in [7, 8, 9, 10] {
            assert!(net.trigger_status(t));
        }
    }

    #[test]
    fn cruise_inner_tank_transfer_starts_below_and_stops_at_ts_thresholds() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let Some(mut net) = network("cruise.FLT") else { return };
        for (t, g) in [(1, 2700.0), (3, 9600.0), (4, 12000.0), (7, 12000.0), (8, 9600.0), (10, 2700.0), (11, 3000.0)] {
            net.set_tank_gallons(t, g);
        }
        for f in [2, 9] {
            net.set_tank_gallons(f, 6440.0);
        }
        for f in [5, 6] {
            net.set_tank_gallons(f, 6900.0);
        }
        for e in 12..=16 {
            net.set_tank_gallons(e, 1.0);
        }
        let mut vars = in_flight_vars();
        let mut fuel = LegacyFuel::new();
        fuel.init(&mut vars);

        // Burn about 3 t/h per engine until feed 1 passes 6436, then watch.
        let mut max_after_start = 0.0f64;
        let mut started = false;
        let mut stopped_at = None;
        let inner_before = net.tank_gallons(4) + net.tank_gallons(7);
        frames(&mut fuel, &mut vars, &mut net, 1800.0, 1000.0, |n| {
            if n.trigger_status(1) {
                started = true;
            }
            if started {
                max_after_start = max_after_start.max(n.tank_gallons(2));
            }
            if n.trigger_status(7) && stopped_at.is_none() && started {
                stopped_at = Some(n.tank_gallons(2));
            }
        });
        assert!(started, "trigger 1 (feed 1 < 6436) never started");
        let stop = stopped_at.expect("trigger 7 (feed 1 > 6765) never fired");
        assert!(stop > 6765.0 && stop < 6800.0, "stopped at {stop}");
        // After the end trigger closes valve 11, feed 1 does not keep climbing.
        assert!(max_after_start < 6800.0, "feed 1 overshot to {max_after_start}");
        // Transfers restart when it drains below 6436 again; it stays in band.
        let f1 = net.tank_gallons(2);
        assert!(f1 > 6400.0 && f1 < 6800.0, "feed 1 at {f1}");
        assert!(net.tank_gallons(4) + net.tank_gallons(7) < inner_before - 100.0);
        assert!(fuel.inner_and_mid_transfer_active_for_feed_tank(1) || !net.trigger_status(1));
        // Feed 2 and 3 use their own thresholds (6857/7186): their fuel stays in 6800..7200.
        let f2 = net.tank_gallons(5);
        assert!(f2 > 6800.0 && f2 < 7220.0, "feed 2 at {f2}");
    }

    #[test]
    fn trim_transfer_runs_until_trim_tank_is_empty() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let Some(mut net) = network("cruise.FLT") else { return };
        for (t, g) in [(1, 800.0), (10, 800.0), (11, 1500.0), (3, 0.0), (4, 0.0), (7, 0.0), (8, 0.0)] {
            net.set_tank_gallons(t, g);
        }
        for f in [2, 5, 6, 9] {
            net.set_tank_gallons(f, 1980.0);
        }
        for e in 12..=16 {
            net.set_tank_gallons(e, 1.0);
        }
        let mut vars = in_flight_vars();
        let mut fuel = LegacyFuel::new();
        fuel.init(&mut vars);
        let trim_before = net.tank_gallons(11);
        let mut trim_started = false;
        frames(&mut fuel, &mut vars, &mut net, 600.0, 1500.0, |n| {
            if (24..=27).any(|t| n.trigger_status(t)) {
                trim_started = true;
            }
        });
        assert!(trim_started, "no trim transfer started below 1974 gal");
        assert!(fuel.trim_transfer_active_for_feed_tank(1) || net.trigger_status(34));
        // Once started, it runs until the trim tank is empty; trigger 34 then
        // stops the trim pumps and closes the aft transfer valves.
        assert!(net.tank_gallons(11) < 0.1, "trim left {}", net.tank_gallons(11));
        assert!(net.trigger_status(34));
        assert_eq!(net.pump_switch(19), 0);
        assert_eq!(net.pump_switch(20), 0);
        assert!(!net.valve_switch(23));
        assert!(trim_before > 1000.0);
        // Inner and mid tanks are empty, so junction 10 is on option 3.
        assert_eq!(net.junction_setting(10), 3);
    }

    #[test]
    fn cg_above_target_starts_cg_transfer_to_inner_tanks() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let Some(mut net) = network("cruise.FLT") else { return };
        for (t, g) in [(4, 6000.0), (7, 6000.0), (3, 5000.0), (8, 5000.0), (11, 3000.0)] {
            net.set_tank_gallons(t, g);
        }
        for f in [2, 5, 6, 9] {
            net.set_tank_gallons(f, 7000.0);
        }
        let mut vars = in_flight_vars();
        let target = LegacyFuel::calculate_cg_target(900.0);
        vars.v.insert("A32NX_FQMS_CENTER_OF_GRAVITY_MAC".into(), Arinc429::to_raw((target + 0.5) as f32, 0b11));
        let mut fuel = LegacyFuel::new();
        fuel.init(&mut vars);
        let inner = net.tank_gallons(4);
        frames(&mut fuel, &mut vars, &mut net, 20.0, 0.0, |_| {});
        assert!(net.trigger_status(43));
        assert_eq!(net.pump_switch(19), 1);
        assert!(net.tank_gallons(4) > inner, "CG transfer did not reach the left inner tank");
        // CG below the stop target: trigger 44 ends it.
        vars.v.insert("A32NX_FQMS_CENTER_OF_GRAVITY_MAC".into(), Arinc429::to_raw((target - 1.5) as f32, 0b11));
        frames(&mut fuel, &mut vars, &mut net, 2.0, 0.0, |_| {});
        assert!(net.trigger_status(44));
        assert_eq!(net.pump_switch(19), 0);
    }

    #[test]
    fn nothing_transfers_on_ground_and_refuel_closes_valves() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let Some(mut net) = network("taxi.flt") else { return };
        net.set_tank_gallons(4, 5000.0);
        net.set_tank_gallons(7, 5000.0);
        for f in [2, 5, 6, 9] {
            net.set_tank_gallons(f, 3000.0);
        }
        let mut vars = in_flight_vars();
        vars.v.insert("SIM ON GROUND".into(), 1.0);
        let mut fuel = LegacyFuel::new();
        fuel.init(&mut vars);
        frames(&mut fuel, &mut vars, &mut net, 10.0, 0.0, |_| {});
        assert!(!net.trigger_status(1) && !net.trigger_status(35));
        assert_eq!(net.tank_gallons(4), 5000.0);

        net.trigger_on(35);
        net.open_valve(46);
        vars.v.insert("A32NX_REFUEL_STARTED_BY_USR".into(), 1.0);
        frames(&mut fuel, &mut vars, &mut net, 1.0, 0.0, |_| {});
        assert!(!net.trigger_status(35));
        assert!(!net.valve_switch(46) && !net.valve_switch(5));
        for v in [1, 2, 3, 4, 37, 40, 50, 51] {
            assert!(net.valve_switch(v), "valve {v} must be left alone");
        }
    }

    #[test]
    fn apu_aspect_opens_valve_and_pump_on_availability_edge() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let Some(mut net) = network("apron.FLT") else { return };
        net.set_tank_gallons(9, 3000.0);
        net.set_tank_gallons(16, 1.0);
        let mut vars = FakeVars::default();
        let mut apu = ApuFuelAspect::new_a380(&mut vars);
        apu.update(&mut vars, &mut net);
        assert!(!net.pump_active(21) && net.pump_switch(21) == 0);

        vars.v.insert("A32NX_OVHD_APU_START_PB_IS_AVAILABLE".into(), 1.0);
        apu.update(&mut vars, &mut net);
        assert!(net.valve_switch(8));
        assert_eq!(net.pump_switch(21), 1);
        assert_eq!(vars.v["APU SWITCH"], 1.0);
        assert_eq!(vars.v.get("BLEED AIR APU").copied().unwrap_or(0.0), 0.0);

        // The APU now burns through line 141 with valves 50/51 from the flt.
        for _ in 0..50 {
            apu.update(&mut vars, &mut net);
            let d = apu_fuel_demand_gph(&mut vars, &net);
            net.update(0.1, [0.0; 4], d);
        }
        assert!(net.apu_fed());
        assert!((net.line_flow_gph(141) - 33.0).abs() < 1e-6);

        vars.v.insert("A32NX_APU_BLEED_AIR_VALVE_OPEN".into(), 1.0);
        apu.update(&mut vars, &mut net);
        assert_eq!(vars.v["BLEED AIR APU"], 1.0);

        vars.v.insert("A32NX_OVHD_APU_START_PB_IS_AVAILABLE".into(), 0.0);
        apu.update(&mut vars, &mut net);
        assert!(!net.valve_switch(8));
        assert_eq!(net.pump_switch(21), 0);
        assert_eq!(vars.v["APU SWITCH"], 0.0);
        assert_eq!(vars.v["BLEED AIR APU"], 0.0);
        let writes = vars.writes.len();
        apu.update(&mut vars, &mut net);
        apu.update(&mut vars, &mut net);
        assert_eq!(vars.writes.len(), writes, "no change, no action");

        // ASU on also starts the MSFS APU and its fuel.
        vars.v.insert("A32NX_ASU_TURNED_ON".into(), 1.0);
        apu.update(&mut vars, &mut net);
        assert_eq!(net.pump_switch(21), 1);
        assert_eq!(vars.v["BLEED AIR APU"], 1.0);
    }

    /// `failures.rs` `extra::apu` 49_001 ("APU fuel control fault") was
    /// previously an unconsumed `Effect::Hook`; confirms it now biases the
    /// APU's fuel demand 35% over the commanded burn rate instead of doing
    /// nothing.
    #[test]
    fn catalogue_apu_fuel_control_fault_overfuels() {
        let _g = crate::failures::tests::serial();
        crate::failures::Failures::new();
        let Some(net) = network("taxi.flt") else { return };
        let mut vars = FakeVars::default();
        vars.v.insert("APU SWITCH".into(), 1.0);
        let normal = apu_fuel_demand_gph(&mut vars, &net);
        assert!((normal - 33.0).abs() < 1e-9, "baseline burn rate {normal}");
        crate::failures::set_active(49_001, true);
        let failed = apu_fuel_demand_gph(&mut vars, &net);
        assert!((failed - normal * 1.35).abs() < 1e-9, "over-fuel bias {failed} vs {normal}");
        crate::failures::set_active(49_001, false);
        assert!((apu_fuel_demand_gph(&mut vars, &net) - normal).abs() < 1e-9, "clearing the failure restores normal demand");
    }
}
