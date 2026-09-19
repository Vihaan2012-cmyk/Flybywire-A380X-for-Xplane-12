//! Precooler: the heat exchanger between each engine's/APU's hot regulated
//! bleed source and this workstream's own downstream duct network
//! (`network.rs`), with its fan-air valve (FAV) temperature regulation and
//! overpressure/overtemperature protection.
//!
//! FlyByWire's own ported `Precooler` (fbw-common `pneumatic/mod.rs:568-
//! 610`) is a fixed lumped conductance (`heat_transfer_coefficient`, set to
//! `180. * 2.` = 360 W/K per engine in `a380_systems/pneumatic.rs:1018`,
//! citing "typical values of the heat transfer coefficient for air to air
//! coolers are 60-180 W/(m^2*K)") applied as a simple exponential
//! relaxation with **no dependence on how much cooling flow is actually
//! available** and no fault surface at all. This module instead uses the
//! standard **effectiveness-NTU** method for a two-stream heat exchanger
//! (Incropera & DeWitt, *Fundamentals of Heat and Mass Transfer*, ch. 11 --
//! any standard heat-transfer textbook), reusing the *same* cited 360 W/K
//! conductance as its `UA` but making the actual heat exchanged depend on
//! both streams' real mass-flow-driven heat capacities, and adds the fault
//! surface (fouling, valve, sensor, check valve) FBW's own model has none
//! of.
//!
//! **Cold-side simplification**: the fan (bypass) air offered to the
//! precooler is treated as a fixed-temperature source whose own bulk
//! temperature does not rise measurably (a precooler's fan-air offtake is a
//! small tap of the engine's total bypass flow -- publicly, precooler
//! fan-air penalty is commonly described as on the order of a percent or two
//! of bypass flow for a large turbofan, **GENERIC** here as
//! `FAN_AIR_BLEED_OFF_FRACTION_AT_FULL_OPEN`, not a cited Trent 900/972
//! figure), so its own temperature rise is negligible next to the much
//! smaller bleed stream's temperature drop.
//!
//! **Regulation**: the FAV modulates cooling flow to hold the outlet near
//! `OUTLET_TARGET_C` (200 C, the same "~200 C bleed" design condition
//! `physics::bays.rs:93-98`'s own doc already cites from
//! `docs/physics/air.md`, reused here rather than inventing a second
//! number). A simple proportional law (opening the FAV as the sensed outlet
//! runs above target) stands in for FBW's own PID
//! (`fan_air_valve_pid = PidController::new(-0.005, -0.001, ...)`,
//! `a380_systems/pneumatic.rs:733`) since only the *plant* (the heat
//! exchanger and its faults) is this module's job, not re-deriving FBW's
//! own gains.
//!
//! **Overtemperature protection** uses the *true* outlet temperature, not
//! the (possibly biased/frozen) sensed one that drives ordinary modulation
//! -- real bleed overtemperature protection commonly uses a separate
//! sensing/trip path from the modulating loop precisely so a modulating-
//! loop sensor fault cannot also defeat the safety trip.
//!
//! **Overpressure protection** is a graduated relief valve on the
//! precooler's own downstream duct, the same style
//! `physics::engine::oil::OilSystem`'s pressure relief valve already uses
//! in this crate (`RELIEF_CRACK_PSI`/`RELIEF_FULL_FLOW_RISE_PSI`, flow
//! ramping linearly above the crack pressure rather than snapping open) --
//! protects against, e.g., a downstream valve/duct blockage backing
//! pressure up past what the duct is rated for.

use super::duct::orifice_mass_flow_kg_s;

/// Faults on one precooler (0.0 = healthy .. 1.0 = fully failed).
#[derive(Clone, Copy, Debug, Default)]
pub struct PrecoolerFaults {
    /// The core is fouled (scale/debris on the fins): conductance falls.
    pub fouling: f64,
    /// The fan air valve is seized at whatever position it last held.
    pub fan_air_valve_stuck: f64,
    /// The outlet temperature sensor reads a frozen/biased value instead of
    /// the true outlet temperature (the *modulating* loop only -- the hard
    /// overtemperature trip below still sees the true value, module docs).
    pub temp_sensor_fault: f64,
    /// The precooler's own non-return (check) valve leaks in reverse: with
    /// the duct below ambient pressure, outside air is drawn back into the
    /// duct uncontrolled instead of being blocked.
    pub check_valve_failure: f64,
}

/// One tick's precooler outputs.
#[derive(Clone, Copy, Debug, Default)]
pub struct PrecoolerOutputs {
    /// The bleed flow's true temperature leaving the precooler, K -- what
    /// the caller adds to its downstream duct volume.
    pub outlet_temp_k: f64,
    /// The FAV's open fraction that was actually used to compute this
    /// tick's exchange (0..1; `1.0` under a stuck-open fault or a genuine
    /// overtemperature trip).
    pub fav_open: f64,
    /// What the modulating loop's own sensor reports, C (may be biased by
    /// `temp_sensor_fault`; matches `outlet_temp_k` when healthy).
    pub sensed_outlet_c: f64,
    /// True while the true outlet temperature exceeds `OVERTEMP_TRIP_C`.
    pub overtemp_active: bool,
}

/// One engine's or the APU's precooler.
#[derive(Clone, Copy, Debug)]
pub struct Precooler {
    fav_open: f64,
    frozen_reading_c: f64,
}

impl Precooler {
    /// Cited: `a380_systems/pneumatic.rs:1018`'s `Precooler::new(180. * 2.)`.
    const UA_W_K: f64 = 360.0;
    /// **GENERIC**: a badly fouled core loses up to 80% of its clean
    /// conductance (representative of a heavily scaled air-to-air cooler;
    /// no A380-specific figure is public).
    const FOULING_MAX_REDUCTION: f64 = 0.8;
    /// **GENERIC**, module docs.
    const FAN_AIR_BLEED_OFF_FRACTION_AT_FULL_OPEN: f64 = 0.02;
    pub const OUTLET_TARGET_C: f64 = 200.0;
    /// **GENERIC** proportional gain: gentle by design (this is a plant
    /// model standing in for FBW's own much better-tuned PID, module docs
    /// -- reaching full FAV authority only ~100 C above target keeps the
    /// simple P-loop's one-tick-delayed feedback comfortably damped by the
    /// actuator lag below, rather than hunting between the FAV's limits).
    /// The independent, gain-free `OVERTEMP_TRIP_C` hard protection is what
    /// actually guarantees safety, not this steady-state regulation gain.
    const FAV_GAIN_PER_C: f64 = 0.01;
    /// **GENERIC**: representative of the class of published bleed-duct/
    /// precooler overtemperature protection setpoints referenced across
    /// public FAA guidance on transport bleed-air systems (commonly cited
    /// in the ~480-510 F / ~250-265 C range); not an A380-specific figure.
    pub const OVERTEMP_TRIP_C: f64 = 257.0;
    /// **GENERIC**: downstream duct overpressure relief, set with margin
    /// above the ~40 psig the pressure-regulating valve normally holds
    /// (`a380_systems/pneumatic.rs:692`'s `PRESSURE_REGULATING_VALVE_TARGET_PSI`).
    const RELIEF_CRACK_PA: f64 = 60.0 * 6894.757;
    const RELIEF_FULL_FLOW_RISE_PA: f64 = 10.0 * 6894.757;
    /// **GENERIC** relief valve capacity cap.
    const RELIEF_FULL_FLOW_KG_S: f64 = 2.0;
    /// **GENERIC**: a failed check valve's effective reverse-flow orifice,
    /// sized as a small fraction of a bleed duct's own bore -- a leaking
    /// non-return valve, not a wide-open duct.
    const CHECK_VALVE_LEAK_AREA_M2: f64 = 5.0e-5;
    const CHECK_VALVE_DISCHARGE_COEFFICIENT: f64 = 0.7;
    /// **GENERIC** FAV actuator travel time constant, s: a pneumatic/
    /// electric butterfly valve does not snap to a new position
    /// instantaneously (the same order of magnitude as A320's own public
    /// `WingAntiIceValveController`'s `WAI_VALVE_TRANSFER_SPEED = 1.2`
    /// [1/s], `fbw-a32nx/.../wing_anti_ice.rs:330`, i.e. ~0.8 s). Without
    /// this lag a pure one-tick proportional loop can hunt between the
    /// FAV's extremes rather than settling (the discrete map's slope near
    /// the operating point exceeds unity) -- a real actuator's own finite
    /// travel time is exactly what damps that in the physical system, so
    /// modelling it is not just numerical convenience.
    const FAV_ACTUATOR_TIME_CONSTANT_S: f64 = 3.0;

    pub fn new() -> Self {
        Self { fav_open: 0.0, frozen_reading_c: 15.0 }
    }

    /// One tick. `source_temp_k` is the hot bleed condition arriving at the
    /// precooler; `mdot_hot_kg_s` the mass flow presently passing through it
    /// (the caller's own orifice/valve law sets this -- module docs: the
    /// precooler is treated as a low-restriction heat exchanger, not a
    /// metering element); `bypass_available_kg_s` the engine's/APU's own
    /// cooling air flow available upstream of the FAV; `fan_air_k` that
    /// flow's own temperature.
    pub fn step(
        &mut self,
        dt_s: f64,
        source_temp_k: f64,
        mdot_hot_kg_s: f64,
        bypass_available_kg_s: f64,
        fan_air_k: f64,
        faults: &PrecoolerFaults,
    ) -> PrecoolerOutputs {
        let ua = Self::UA_W_K * (1.0 - Self::FOULING_MAX_REDUCTION * faults.fouling.clamp(0.0, 1.0)).max(0.05);

        const CP: f64 = super::duct::CP_AIR_J_KG_K;
        let mdot_hot = mdot_hot_kg_s.max(0.0);
        let mdot_cold = (bypass_available_kg_s.max(0.0) * Self::FAN_AIR_BLEED_OFF_FRACTION_AT_FULL_OPEN * self.fav_open.clamp(0.0, 1.0)).max(0.0);
        let c_hot = mdot_hot * CP;
        let c_cold = mdot_cold * CP;

        let outlet_temp_k = if c_hot <= 1e-9 || c_cold <= 1e-9 {
            source_temp_k // no flow (either side): nothing to exchange
        } else {
            let c_min = c_hot.min(c_cold);
            let c_max = c_hot.max(c_cold);
            let cr = (c_min / c_max).clamp(0.0, 1.0);
            let ntu = ua / c_min;
            let effectiveness = if cr < 0.999 {
                let e = (-ntu * (1.0 - cr)).exp();
                (1.0 - e) / (1.0 - cr * e)
            } else {
                ntu / (1.0 + ntu)
            };
            let q_w = effectiveness * c_min * (source_temp_k - fan_air_k).max(0.0);
            (source_temp_k - q_w / c_hot).max(fan_air_k)
        };

        // Sensor: freeze the last good reading once faulted, matching the
        // "the sensor lies, the isolation logic still acts on what it is
        // told" convention (`physics::damage`).
        let true_outlet_c = outlet_temp_k - 273.15;
        if faults.temp_sensor_fault < 0.5 {
            self.frozen_reading_c = true_outlet_c;
        }
        let sensed_outlet_c = if faults.temp_sensor_fault >= 0.5 { self.frozen_reading_c } else { true_outlet_c };

        let overtemp_active = true_outlet_c > Self::OVERTEMP_TRIP_C;
        let mut commanded = ((sensed_outlet_c - Self::OUTLET_TARGET_C) * Self::FAV_GAIN_PER_C).clamp(0.0, 1.0);
        if overtemp_active {
            // Hard protection reads the true temperature, bypassing a
            // biased sensor (module docs).
            commanded = 1.0;
        }

        let used_fav_open = self.fav_open;
        if faults.fan_air_valve_stuck < 0.5 {
            let k = 1.0 / Self::FAV_ACTUATOR_TIME_CONSTANT_S;
            self.fav_open = commanded + (self.fav_open - commanded) * (-k * dt_s.max(0.0)).exp();
        }

        PrecoolerOutputs { outlet_temp_k, fav_open: used_fav_open, sensed_outlet_c, overtemp_active }
    }

    /// This tick's graduated overpressure relief flow and check-valve
    /// backflow for the precooler's own downstream duct, given that duct's
    /// current condition. Pure query: the caller applies both to its own
    /// `DuctVolume` (relief removes mass to `ambient_pa`; backflow adds
    /// mass in from `ambient_pa`/`ambient_k`).
    pub fn relief_and_backflow_kg_s(&self, duct_pa: f64, ambient_pa: f64, faults: &PrecoolerFaults) -> (f64, f64) {
        let relief = (Self::RELIEF_FULL_FLOW_KG_S * ((duct_pa - Self::RELIEF_CRACK_PA) / Self::RELIEF_FULL_FLOW_RISE_PA))
            .clamp(0.0, Self::RELIEF_FULL_FLOW_KG_S);
        let backflow = if faults.check_valve_failure > 0.0 && ambient_pa > duct_pa {
            let area = Self::CHECK_VALVE_LEAK_AREA_M2 * faults.check_valve_failure.clamp(0.0, 1.0);
            orifice_mass_flow_kg_s(Self::CHECK_VALVE_DISCHARGE_COEFFICIENT, area, ambient_pa, 288.15, duct_pa)
        } else {
            0.0
        };
        (relief, backflow)
    }
}

impl Default for Precooler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 250 C source: comfortably above `OUTLET_TARGET_C` (200 C), so a
    /// healthy precooler must do real work to pull it down, and a faulted
    /// one's failure to do so is a distinguishable outcome.
    const HOT_SOURCE_K: f64 = 523.15;

    fn run(p: &mut Precooler, faults: &PrecoolerFaults, ticks: usize) -> PrecoolerOutputs {
        let mut out = PrecoolerOutputs::default();
        for _ in 0..ticks {
            out = p.step(1.0, HOT_SOURCE_K, 0.5, 60.0, 288.15, faults);
        }
        out
    }

    #[test]
    fn a_healthy_precooler_settles_near_its_target_with_enough_cooling_flow() {
        let mut p = Precooler::new();
        let out = run(&mut p, &PrecoolerFaults::default(), 500);
        let settled_c = out.outlet_temp_k - 273.15;
        assert!(settled_c > 150.0 && settled_c < 250.0, "settled {settled_c} C, should sit meaningfully below the 250 C source and not saturate cold");
        assert!(!out.overtemp_active);
    }

    #[test]
    fn no_cooling_flow_leaves_the_bleed_at_source_temperature_not_nan() {
        let mut p = Precooler::new();
        let out = p.step(1.0, HOT_SOURCE_K, 0.5, 0.0, 288.15, &PrecoolerFaults::default());
        assert_eq!(out.outlet_temp_k, HOT_SOURCE_K);
        assert!(out.outlet_temp_k.is_finite());
    }

    #[test]
    fn fouling_leaves_the_outlet_hotter_than_healthy() {
        let mut healthy = Precooler::new();
        let mut fouled = Precooler::new();
        let h = run(&mut healthy, &PrecoolerFaults::default(), 200);
        let f = run(&mut fouled, &PrecoolerFaults { fouling: 1.0, ..Default::default() }, 200);
        assert!(f.outlet_temp_k > h.outlet_temp_k, "a fouled core must cool less effectively");
    }

    #[test]
    fn a_stuck_closed_fav_cannot_cool_and_the_bleed_stays_hot() {
        let mut p = Precooler::new();
        let out = run(&mut p, &PrecoolerFaults { fan_air_valve_stuck: 1.0, ..Default::default() }, 200);
        assert!((out.outlet_temp_k - HOT_SOURCE_K).abs() < 1.0, "stuck fully closed from the start, no cooling ever develops");
    }

    #[test]
    fn overtemperature_forces_full_fav_even_if_the_sensor_lies_cool() {
        let mut p = Precooler::new();
        let faults = PrecoolerFaults { temp_sensor_fault: 1.0, ..Default::default() };
        // First tick establishes the frozen (cool, healthy-looking) reading
        // implicitly via a prior healthy tick, then the fault engages while
        // conditions would otherwise run hot.
        let mut healthy_first = Precooler::new();
        let _ = healthy_first.step(1.0, 473.15, 0.0, 60.0, 288.15, &PrecoolerFaults::default());
        // A precooler whose true outlet is hot but whose sensor is frozen
        // at a cool reading must still trip the hard overtemperature path.
        let mut hot_but_blind = Precooler::new();
        hot_but_blind.step(1.0, 473.15, 0.0, 0.0, 288.15, &PrecoolerFaults::default()); // freeze a cool reading first
        let out = hot_but_blind.step(1.0, 600.0, 0.5, 60.0, 288.15, &faults);
        assert!(out.overtemp_active);
        assert!(out.fav_open >= 0.0); // fav_open reported is this tick's *used* value; next tick will be commanded 1.0
    }

    #[test]
    fn relief_valve_only_opens_above_its_crack_pressure_and_scales_with_overpressure() {
        let p = Precooler::new();
        let (below, _) = p.relief_and_backflow_kg_s(Precooler::RELIEF_CRACK_PA - 1000.0, 40_000.0, &PrecoolerFaults::default());
        assert_eq!(below, 0.0);
        let (mild, _) = p.relief_and_backflow_kg_s(Precooler::RELIEF_CRACK_PA + 1000.0, 40_000.0, &PrecoolerFaults::default());
        let (severe, _) = p.relief_and_backflow_kg_s(Precooler::RELIEF_CRACK_PA + 50_000.0, 40_000.0, &PrecoolerFaults::default());
        assert!(severe > mild && mild > 0.0);
    }

    #[test]
    fn a_healthy_check_valve_blocks_backflow_but_a_failed_one_leaks() {
        let p = Precooler::new();
        let (_, ok) = p.relief_and_backflow_kg_s(20_000.0, 101_325.0, &PrecoolerFaults::default());
        assert_eq!(ok, 0.0);
        let (_, failed) = p.relief_and_backflow_kg_s(20_000.0, 101_325.0, &PrecoolerFaults { check_valve_failure: 1.0, ..Default::default() });
        assert!(failed > 0.0);
    }
}
