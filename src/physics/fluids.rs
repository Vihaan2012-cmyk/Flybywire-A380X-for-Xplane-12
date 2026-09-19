//! Fluid physics shared by `fuel.rs` and the hydraulics glue
//! (`physics::hydraulics`): orifice/valve flow, heat transfer, Jet A
//! properties and pump pressure-flow curves. Pure functions, unit-tested
//! without an X-Plane or FBW context, per docs/briefs/hyperrealism.md's
//! "physics must be real" rule (orifice/valve flow equations, heat
//! exchangers with effectiveness).
//!
//! Every constant below is cited or, where no public figure exists, flagged
//! as a derived placeholder. Sources are repeated in `docs/physics/fluids.md`.

/// Standard sharp-edged/nozzle orifice equation: `Q = Cd * A * sqrt(2 * dP / rho)`,
/// the textbook relation FBW's own systems crate uses for restrictors
/// elsewhere (e.g. `hydraulic/mod.rs`'s leak valves size flow the same way).
/// `delta_pressure_pa` is clamped to non-negative: flow does not reverse
/// through a nozzle from a downstream-side reading alone here (callers that
/// need reverse flow negate the result themselves).
pub fn orifice_flow_m3_s(discharge_coefficient: f64, area_m2: f64, delta_pressure_pa: f64, density_kg_m3: f64) -> f64 {
    if area_m2 <= 0. || density_kg_m3 <= 0. || delta_pressure_pa <= 0. {
        return 0.;
    }
    discharge_coefficient * area_m2 * (2. * delta_pressure_pa / density_kg_m3).sqrt()
}

/// Back-solves the orifice equation for the effective `Cd * A` (m^2) that
/// reproduces a known reference flow at a known reference pressure drop.
/// Used to calibrate the jettison nozzles' effective throat area from the
/// one sourced reference flow rate (`flight_model.cfg`'s
/// `GravityBasedFuelFlow`) at an assumed reference head, rather than
/// inventing a nozzle diameter outright.
pub fn effective_cda_m2(reference_flow_m3_s: f64, reference_delta_pressure_pa: f64, density_kg_m3: f64) -> f64 {
    if reference_delta_pressure_pa <= 0. || density_kg_m3 <= 0. {
        return 0.;
    }
    reference_flow_m3_s / (2. * reference_delta_pressure_pa / density_kg_m3).sqrt()
}

/// Recovery temperature at speed: `T_recovery = T_static * (1 + r * (gamma-1)/2 * M^2)`,
/// the standard compressible-flow adiabatic-wall relation (used elsewhere in
/// this plugin for TAT-adjacent quantities; `engine_commands.rs` reuses
/// X-Plane's own equivalent dataref rather than recomputing it, but no such
/// dataref exists at the fuel tank skin, so it is computed here). `r` is the
/// recovery factor: about 0.9 for a turbulent boundary layer (standard
/// aerodynamic heating reference value), ~1.0 for a fully stagnated point.
/// Static temperature in Kelvin in, Kelvin out.
pub fn recovery_temperature_k(static_temp_k: f64, mach: f64, recovery_factor: f64) -> f64 {
    const GAMMA: f64 = 1.4; // Air's ratio of specific heats.
    static_temp_k * (1. + recovery_factor * (GAMMA - 1.) / 2. * mach * mach)
}

/// Simple conduction/convection heat rate through a wetted area:
/// `Q = h * A * dT` (Newton's law of cooling), watts. `h` is an overall
/// (film) heat transfer coefficient, `area_m2` the wetted surface, `dT` the
/// temperature difference driving heat *into* the fluid (positive warms it).
pub fn convective_heat_w(h_w_m2k: f64, area_m2: f64, delta_t_k: f64) -> f64 {
    h_w_m2k * area_m2 * delta_t_k
}

/// Wetted area of a tank of `volume_m3`, approximated as a cuboid of aspect
/// ratio `aspect` (wingspan-direction length / depth), i.e. a thin, wide
/// wing-tank-shaped box rather than a sphere: `area ~ k * volume^(2/3)`.
/// A380 wing tanks are long and shallow, not cubic, so a plain sphere/cube
/// assumption would understate wetted area for the same volume; this uses a
/// box of `aspect:1:aspect` (long, unit-width, long) whose surface-to-volume
/// ratio is derived algebraically rather than sourced (no AMM tank
/// dimensions are public), flagged as a defensible geometric derivation, not
/// a cited figure.
pub fn tank_wetted_area_m2(volume_m3: f64, aspect: f64) -> f64 {
    if volume_m3 <= 0. {
        return 0.;
    }
    let a = aspect.max(1.);
    let w = tank_box_height_m(volume_m3, aspect);
    // Box of dimensions (a*w, w, a*w) has volume a^2*w^3 and surface
    // 2*(a*w*w + a*w*w + a^2*w^2) = 2*w^2*(2*a + a^2).
    2. * w * w * (2. * a + a * a)
}

/// The short ("depth"/height) dimension `w` of the same aspect-ratio box
/// `tank_wetted_area_m2` uses: a box of dimensions `(a*w, w, a*w)` has volume
/// `a^2*w^3`, so `w = (volume / a^2)^(1/3)`. Used as a stand-in for the fuel
/// column's own depth, e.g. to derive a gravity/suction feed head pressure
/// when a tank's boost pump has failed (see `fuel.rs`'s engine feed pressure
/// publish).
pub fn tank_box_height_m(volume_m3: f64, aspect: f64) -> f64 {
    if volume_m3 <= 0. {
        return 0.;
    }
    let a = aspect.max(1.);
    (volume_m3 / (a * a)).cbrt()
}

// ---- Jet A properties -----------------------------------------------------
//
// CRC Report No. 663, "Handbook of Aviation Fuel Properties" (Coordinating
// Research Council, 2nd ed. 2014/2016) is the standard reference FBW's own
// analysis doc points to for jet fuel property correlations (no numeric
// table from it was accessible here; the two reference points below are the
// commonly published ones repeated across ASTM D1655/CRC-derived sources,
// e.g. https://wiki.anton-paar.com/us-en/aviation-fuels/ and
// https://www.engineeringtoolbox.com/jet-fuel-temperature-density-petroleum-volume-correction-ASTM-D1250-gravity-d_1944.html).

/// Jet A density at 15 C, matching the `JET_A_LBS_PER_GAL` (6.699 lb/US gal)
/// MSFS/FBW constant already used in `fuel.rs`, converted to SI.
pub const JET_A_DENSITY_KG_M3_AT_15C: f64 = 802.5; // 6.699 lb/gal * 0.45359237 kg/lb / 0.00378541 m3/gal

/// Thermal expansion coefficient for kerosene-type jet fuel, a widely
/// published typical figure for petroleum distillates in this density range
/// (API/ASTM D1250 volume correction tables imply approximately this slope
/// around 15-20 C; not a single-sourced Jet A constant, a common
/// petroleum-fuels order of magnitude).
pub const JET_A_THERMAL_EXPANSION_PER_K: f64 = 9.0e-4;

/// Jet A density at `temp_c`, from the 15 C reference density and a linear
/// thermal expansion (`rho(T) = rho_15 / (1 + beta * (T - 15))`, the small
/// -angle form of `V(T) = V_15 * (1 + beta * dT)`).
pub fn jet_a_density_kg_m3(temp_c: f64) -> f64 {
    JET_A_DENSITY_KG_M3_AT_15C / (1. + JET_A_THERMAL_EXPANSION_PER_K * (temp_c - 15.))
}

/// Two commonly published Jet A/Jet A-1 kinematic viscosity reference
/// points: about 1.5 mm^2/s at 20 C typical of dispatch-quality Jet A, and
/// the ASTM D1655/DEF STAN 91-091 specification's own low-temperature
/// viscosity limit, 8 mm^2/s at -20 C (widely repeated aviation-fuel
/// specification figure). The true correlation is the two-slope
/// ASTM D341/MacCoull log-log form; this uses a single log-log (Walther-type)
/// fit through these two points as a defensible one-segment approximation,
/// not the full two-segment ASTM D341 curve (whose exact coefficients were
/// not accessible here). Flagged as a derived approximation.
const VISC_REF1_TEMP_K: f64 = 293.15; // 20 C
const VISC_REF1_CST: f64 = 1.5;
const VISC_REF2_TEMP_K: f64 = 253.15; // -20 C
const VISC_REF2_CST: f64 = 8.0;

/// Jet A kinematic viscosity (centistokes) at `temp_c`, from the Walther-type
/// two-point log-log fit above: `log10(log10(v + 0.7)) = A - B*log10(T)`.
pub fn jet_a_viscosity_cst(temp_c: f64) -> f64 {
    let temp_k = temp_c + 273.15;
    let z = |v: f64| (v + 0.7).log10().log10();
    let (z1, z2) = (z(VISC_REF1_CST), z(VISC_REF2_CST));
    let (t1, t2) = (VISC_REF1_TEMP_K.log10(), VISC_REF2_TEMP_K.log10());
    let b = (z1 - z2) / (t2 - t1);
    let a = z1 + b * t1;
    let zt = a - b * temp_k.log10();
    (10f64.powf(10f64.powf(zt))) - 0.7
}

/// Jet A's specification maximum freeze point (ASTM D1655), matching
/// `fuel.rs`'s existing `FUEL_FREEZE_POINT_C`.
pub const FUEL_FREEZE_POINT_C: f64 = -40.;

/// Viscosity multiplier applied to a pump's rated flow as fuel approaches
/// its freeze point: pumps are rated against a reference viscosity
/// (`REFERENCE_VISCOSITY_CST`, a typical dispatch-temperature Jet A
/// viscosity) and lose flow capacity roughly in inverse proportion to
/// viscosity once it rises well above that reference, a standard
/// centrifugal-pump derating behaviour (viscosity correction charts, e.g.
/// Hydraulic Institute ANSI/HI 9.6.7). Clamped so a merely cool tank does not
/// derate the pump, only one approaching wax/slush formation.
const REFERENCE_VISCOSITY_CST: f64 = 1.5;
pub fn viscosity_flow_derate(viscosity_cst: f64) -> f64 {
    (REFERENCE_VISCOSITY_CST / viscosity_cst.max(REFERENCE_VISCOSITY_CST)).clamp(0.05, 1.0)
}

/// Hydrostatic (gravity/suction feed) pressure from a fuel column of
/// `height_m` at `density_kg_m3`: `P = rho * g * h`. Used as the pressure
/// floor an engine feed line still sees from tank head alone when its boost
/// pump has failed or is unpowered (suction feed), instead of reporting zero.
pub fn hydrostatic_pressure_pa(height_m: f64, density_kg_m3: f64) -> f64 {
    const G: f64 = 9.80665;
    height_m.max(0.) * density_kg_m3 * G
}

/// Boost/feed pump inlet unporting from pitch/bank tilt and low fuel level.
///
/// A real submerged boost pump inlet sits near the tank's low point, a small
/// fixed margin above the tank floor (a few percent of tank depth -- the
/// physical origin of "unusable fuel", already modelled elsewhere as a hard
/// cliff by `FuelSystemDef`'s own `unusablecapacity`). This adds the
/// *dynamic* effect on top of that static cliff: pitch and bank tilt the
/// fuel's free surface relative to the tank, displacing it toward one
/// end/side and potentially uncovering the inlet even with usable fuel
/// remaining elsewhere in the tank -- a real cause of transient pump
/// pressure/flow loss in sustained low-fuel unusual attitudes, distinct from
/// simply running the tank dry.
///
/// The inlet's exact position within the tank is not public (no AMM
/// drawing); this assumes the conservative case of an inlet at the tank's
/// low corner, an upper bound on unporting risk rather than a late one. Bank
/// is applied over the tank's long (span-direction) box dimension
/// (`aspect * box_height_m`, matching `tank_wetted_area_m2`'s box geometry --
/// A380 wing tanks are long span-wise), while pitch is applied over the
/// short depth dimension alone: no public chord-wise tank extent exists to
/// derive a second long dimension from, so pitch's contribution is
/// deliberately kept small/conservative rather than inventing an unsupported
/// chordwise length.
///
/// Returns a smooth 0-1 pressure/flow derate: 1.0 while the inlet stays
/// comfortably submerged, ramping to 0 over one submersion-margin band
/// (`margin_fraction * box_height_m`) as the tilted surface reaches or
/// passes the inlet -- a progressive derate, not a binary cliff, since a
/// partially uncovered inlet ingests air progressively rather than cutting
/// flow instantly.
pub fn unporting_factor(
    fill_fraction: f64,
    pitch_deg: f64,
    bank_deg: f64,
    box_height_m: f64,
    aspect: f64,
    margin_fraction: f64,
) -> f64 {
    if box_height_m <= 0. {
        return 1.;
    }
    let fill_fraction = fill_fraction.clamp(0., 1.);
    let box_length_m = aspect.max(1.) * box_height_m;
    let tilt_m =
        (box_length_m / 2.) * bank_deg.to_radians().tan().abs() + (box_height_m / 2.) * pitch_deg.to_radians().tan().abs();
    let depth_at_inlet_m = fill_fraction * box_height_m - tilt_m;
    let margin_m = margin_fraction.max(1e-6) * box_height_m;
    (depth_at_inlet_m / margin_m).clamp(0., 1.)
}

/// hyperrealism.md physics workstream (fuel second pass): the free surface's
/// tilt, as a fraction of the tank's own box height, from pitch/bank -- the
/// same tilt geometry `unporting_factor` already uses internally (bank over
/// the long span dimension, pitch over the short depth dimension alone, same
/// aspect-ratio tank box), extracted as its own function so the FQMS probe
/// model below can share the identical physical tilt rather than
/// recomputing a second, possibly-divergent version of the same geometry.
pub fn tank_tilt_fraction(pitch_deg: f64, bank_deg: f64, box_height_m: f64, aspect: f64) -> f64 {
    if box_height_m <= 0. {
        return 0.;
    }
    let box_length_m = aspect.max(1.) * box_height_m;
    let tilt_m =
        (box_length_m / 2.) * bank_deg.to_radians().tan().abs() + (box_height_m / 2.) * pitch_deg.to_radians().tan().abs();
    tilt_m / box_height_m
}

/// hyperrealism.md physics workstream (fuel second pass): FQMS quantity
/// indication (brief item 1, "independent of true quantity"). A real
/// capacitance-probe FQI reconstructs the tank's fill fraction by averaging
/// `probe_count` probes evenly spaced along the tank's long span dimension,
/// each reading the *local* fuel depth at its own position (clipped to the
/// tank's physical envelope, 0=dry to 1=full -- a probe cannot read past its
/// own ends). At zero tilt, or any tilt small enough that every probe stays
/// within the envelope, the average of a linear ramp equals its centre value,
/// so this recovers the true fill fraction *exactly* -- matching that real
/// multi-probe FQI systems are accurate in normal flight. Indication error
/// only appears once `tilt_fraction` is large enough (steep bank/pitch at low
/// or high fill) to clip one end of the probe array before the true surface
/// itself would run off the tank -- the real, physical source of FQI
/// indication error under attitude that a single always-exact totaliser
/// cannot reproduce, and exactly why real aircraft have `probe_count` discrete
/// probes rather than one continuous integrator.
pub fn probe_indicated_fill_fraction(fill_fraction: f64, tilt_fraction: f64, probe_count: u32) -> f64 {
    let n = probe_count.max(1);
    let fill_fraction = fill_fraction.clamp(0., 1.);
    let mut sum = 0.0;
    for i in 0..n {
        // Probe i's relative span position, evenly spaced and centred:
        // s in (-0.5, 0.5).
        let s = (i as f64 + 0.5) / n as f64 - 0.5;
        sum += (fill_fraction + tilt_fraction * s).clamp(0., 1.);
    }
    sum / n as f64
}

// ---- Pumps -----------------------------------------------------------------

/// A linear pressure-flow ("droop") pump curve: maximum (shutoff) pressure
/// at zero flow, falling to zero pressure at the rated free-flow rate, the
/// standard first-order approximation of a centrifugal boost-pump curve used
/// where only the two rated points (shutoff head, rated free flow) are
/// published, e.g. FlyByWire's own `flight_model.cfg` pump entries
/// (`fuel_network.rs`'s `Pump` parsing already reads `Pressure`/`Type` from
/// there for the pressure *setpoint*; this adds the flow-vs-back-pressure
/// slope `flight_model.cfg` does not itself encode).
pub struct PumpCurve {
    pub shutoff_pressure_pa: f64,
    pub rated_flow_m3_s: f64,
}
impl PumpCurve {
    /// Flow delivered against `back_pressure_pa` (the downstream pressure
    /// the pump must push against), zero at or above shutoff pressure,
    /// linear in between.
    pub fn flow_m3_s(&self, back_pressure_pa: f64) -> f64 {
        if self.shutoff_pressure_pa <= 0. {
            return 0.;
        }
        let ratio = 1. - (back_pressure_pa / self.shutoff_pressure_pa).clamp(0., 1.);
        self.rated_flow_m3_s * ratio
    }
}

/// Electric pump hydraulic power (pressure x flow), watts.
pub fn hydraulic_power_w(pressure_pa: f64, flow_m3_s: f64) -> f64 {
    pressure_pa * flow_m3_s
}

/// Electric motor current draw for a given hydraulic output and motor
/// efficiency: `I = P_hydraulic / (efficiency * V)`, three-phase or DC alike
/// at this level of fidelity (matches the magnitude convention FBW's own
/// `ElectricalPumpPhysics`/`VariableSpeedPump` use for their EHA/electric
/// pumps: hydraulic power divided by an efficiency factor gives the
/// electrical power drawn).
pub fn pump_current_a(hydraulic_power_w: f64, voltage_v: f64, motor_efficiency: f64) -> f64 {
    if voltage_v <= 0. || motor_efficiency <= 0. {
        return 0.;
    }
    hydraulic_power_w / (motor_efficiency * voltage_v)
}

// ---- Heat exchanger effectiveness ------------------------------------------

/// Effectiveness-NTU heat exchanger: heat actually transferred as a fraction
/// of the maximum thermodynamically possible
/// (`Q = effectiveness * C_min * (T_hot_in - T_cold_in)`), the standard
/// method the brief calls for ("heat exchangers with effectiveness").
/// `effectiveness` (0-1) stands in for the exchanger's real NTU/flow-ratio
/// curve; the A380's fuel/hydraulic heat exchangers (HHX) are described only
/// qualitatively in public sources (Power & Motion Technology, "Hydraulics
/// onboard the A380": two fuel/hydraulic heat exchangers per circuit, one
/// per pylon, on the outer-engine feed circuits), with no published
/// effectiveness figure, so a mid-range plate/shell-and-tube figure of 0.6 is
/// used, flagged as a derived placeholder, not a sourced HHX spec.
pub const HHX_EFFECTIVENESS: f64 = 0.6;

pub fn heat_exchanger_transfer_w(effectiveness: f64, c_min_w_per_k: f64, hot_in_k: f64, cold_in_k: f64) -> f64 {
    effectiveness * c_min_w_per_k * (hot_in_k - cold_in_k)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orifice_flow_scales_with_sqrt_of_pressure() {
        let f1 = orifice_flow_m3_s(0.7, 0.001, 100_000., 800.);
        let f4 = orifice_flow_m3_s(0.7, 0.001, 400_000., 800.);
        // Quadrupling delta-P should double flow (sqrt relationship).
        assert!((f4 / f1 - 2.0).abs() < 1e-9);
    }

    #[test]
    fn orifice_flow_is_zero_with_no_pressure_or_closed_area() {
        assert_eq!(orifice_flow_m3_s(0.7, 0., 100_000., 800.), 0.);
        assert_eq!(orifice_flow_m3_s(0.7, 0.001, 0., 800.), 0.);
        assert_eq!(orifice_flow_m3_s(0.7, 0.001, -1., 800.), 0.);
    }

    #[test]
    fn effective_cda_reproduces_the_reference_flow() {
        let cda = effective_cda_m2(0.01, 50_000., 800.);
        let flow = orifice_flow_m3_s(1.0, cda, 50_000., 800.);
        assert!((flow - 0.01).abs() < 1e-9);
    }

    #[test]
    fn recovery_temperature_exceeds_static_in_flight() {
        let static_k = 273.15 - 50.; // -50 C, typical cruise TAT input
        let recovered = recovery_temperature_k(static_k, 0.85, 0.9);
        assert!(recovered > static_k);
        // Sanity: a few tens of kelvin of ram rise at M0.85, not hundreds.
        assert!(recovered - static_k < 40.);
    }

    #[test]
    fn recovery_temperature_at_zero_mach_is_static() {
        assert_eq!(recovery_temperature_k(280., 0., 0.9), 280.);
    }

    #[test]
    fn wetted_area_grows_with_volume() {
        let small = tank_wetted_area_m2(1.0, 4.0);
        let big = tank_wetted_area_m2(8.0, 4.0);
        assert!(big > small);
        // Doubling every linear dimension (8x volume) should give 4x area.
        assert!((big / small - 4.0).abs() < 1e-6);
    }

    #[test]
    fn jet_a_density_falls_with_temperature() {
        let cold = jet_a_density_kg_m3(-20.);
        let warm = jet_a_density_kg_m3(40.);
        assert!(cold > warm, "fuel should be denser when cold");
        // At 15 C it should match the reference constant exactly.
        assert!((jet_a_density_kg_m3(15.) - JET_A_DENSITY_KG_M3_AT_15C).abs() < 1e-9);
    }

    #[test]
    fn jet_a_viscosity_matches_its_two_reference_points() {
        assert!((jet_a_viscosity_cst(20.) - VISC_REF1_CST).abs() < 1e-6);
        assert!((jet_a_viscosity_cst(-20.) - VISC_REF2_CST).abs() < 1e-6);
    }

    #[test]
    fn jet_a_viscosity_rises_as_fuel_cools() {
        assert!(jet_a_viscosity_cst(-40.) > jet_a_viscosity_cst(-20.));
        assert!(jet_a_viscosity_cst(-20.) > jet_a_viscosity_cst(20.));
    }

    #[test]
    fn viscosity_derate_is_full_at_reference_and_falls_when_thick() {
        assert_eq!(viscosity_flow_derate(REFERENCE_VISCOSITY_CST), 1.0);
        assert_eq!(viscosity_flow_derate(0.1), 1.0); // thinner than reference: no bonus above 1.
        assert!(viscosity_flow_derate(30.) < 1.0);
        assert!(viscosity_flow_derate(1000.) >= 0.05); // clamped, never zero (avoids div-by-zero downstream)
    }

    #[test]
    fn pump_curve_is_linear_between_shutoff_and_rated_flow() {
        let curve = PumpCurve { shutoff_pressure_pa: 400_000., rated_flow_m3_s: 0.002 };
        assert_eq!(curve.flow_m3_s(0.), 0.002);
        assert_eq!(curve.flow_m3_s(400_000.), 0.);
        assert!((curve.flow_m3_s(200_000.) - 0.001).abs() < 1e-9);
        assert_eq!(curve.flow_m3_s(1_000_000.), 0.); // clamped past shutoff
    }

    #[test]
    fn pump_current_scales_inversely_with_voltage_and_efficiency() {
        let i = pump_current_a(1000., 115., 0.9);
        assert!((i - 1000. / (0.9 * 115.)).abs() < 1e-9);
        assert_eq!(pump_current_a(1000., 0., 0.9), 0.);
    }

    #[test]
    fn tank_box_height_grows_with_volume() {
        let h1 = tank_box_height_m(1.0, 6.0);
        let h8 = tank_box_height_m(8.0, 6.0);
        assert!((h8 / h1 - 2.0).abs() < 1e-6); // 8x volume -> 2x linear dimension
    }

    #[test]
    fn hydrostatic_pressure_scales_with_height_and_density() {
        assert_eq!(hydrostatic_pressure_pa(0., 800.), 0.);
        let p1 = hydrostatic_pressure_pa(1., 800.);
        let p2 = hydrostatic_pressure_pa(2., 800.);
        assert!((p2 / p1 - 2.0).abs() < 1e-9);
        assert!(p1 > 0.);
    }

    #[test]
    fn heat_exchanger_transfer_is_zero_at_equal_temperatures() {
        assert_eq!(heat_exchanger_transfer_w(0.6, 500., 300., 300.), 0.);
        assert!(heat_exchanger_transfer_w(0.6, 500., 350., 300.) > 0.);
    }

    #[test]
    fn unporting_full_tank_level_attitude_is_full_pressure() {
        assert_eq!(unporting_factor(1.0, 0., 0., 1.0, 6.0, 0.15), 1.0);
    }

    #[test]
    fn unporting_empty_tank_is_zero_regardless_of_attitude() {
        assert_eq!(unporting_factor(0.0, 0., 0., 1.0, 6.0, 0.15), 0.0);
        assert_eq!(unporting_factor(0.0, 20., 30., 1.0, 6.0, 0.15), 0.0);
    }

    #[test]
    fn unporting_zero_box_height_is_a_safe_default() {
        // No tank geometry known: never divide by zero, never derate.
        assert_eq!(unporting_factor(0.5, 10., 10., 0.0, 6.0, 0.15), 1.0);
    }

    #[test]
    fn unporting_increases_with_bank_at_low_fill() {
        let level = unporting_factor(0.2, 0., 0., 1.0, 6.0, 0.15);
        let banked = unporting_factor(0.2, 0., 20., 1.0, 6.0, 0.15);
        assert!(banked < level, "banked {banked} should derate more than level {level}");
    }

    #[test]
    fn unporting_bank_derates_more_than_equal_pitch_for_a_span_elongated_tank() {
        // Same tilt magnitude, but bank acts over the long (aspect*height)
        // span dimension while pitch acts over the short depth dimension
        // alone -- bank must dominate for an aspect > 1 (span-elongated)
        // wing tank.
        let pitched = unporting_factor(0.3, 20., 0., 1.0, 6.0, 0.15);
        let banked = unporting_factor(0.3, 0., 20., 1.0, 6.0, 0.15);
        assert!(banked <= pitched);
    }

    #[test]
    fn unporting_never_negative_or_above_one() {
        for fill in [0.0, 0.05, 0.3, 0.7, 1.0] {
            for angle in [-45., -20., 0., 20., 45.] {
                let f = unporting_factor(fill, angle, angle, 1.2, 6.0, 0.15);
                assert!((0.0..=1.0).contains(&f), "fill={fill} angle={angle} -> {f}");
            }
        }
    }

    #[test]
    fn probe_indication_is_exact_at_level_attitude() {
        for fill in [0.0, 0.1, 0.5, 0.9, 1.0] {
            for n in [1, 4, 8, 14] {
                let ind = probe_indicated_fill_fraction(fill, 0.0, n);
                assert!((ind - fill).abs() < 1e-9, "fill={fill} n={n} -> {ind}");
            }
        }
    }

    #[test]
    fn probe_indication_diverges_from_true_only_once_tilt_clips_the_array() {
        // A mild tilt with plenty of fuel: every probe stays wet, no error.
        let mild = probe_indicated_fill_fraction(0.6, 0.2, 8);
        assert!((mild - 0.6).abs() < 1e-9);
        // A steep tilt at low fill: some probes clip dry, biasing the
        // average above the true (low) fill fraction.
        let steep = probe_indicated_fill_fraction(0.1, 1.0, 8);
        assert!(steep > 0.1, "steep tilt at low fill should read higher than true: {steep}");
    }

    #[test]
    fn tilt_fraction_is_zero_when_level() {
        assert_eq!(tank_tilt_fraction(0., 0., 1.0, 6.0), 0.);
        assert!(tank_tilt_fraction(20., 20., 1.0, 6.0) > 0.);
    }

    #[test]
    fn unporting_is_monotonic_in_fill_fraction_at_fixed_attitude() {
        let low = unporting_factor(0.1, 5., 10., 1.0, 6.0, 0.15);
        let high = unporting_factor(0.4, 5., 10., 1.0, 6.0, 0.15);
        assert!(high >= low);
    }
}
