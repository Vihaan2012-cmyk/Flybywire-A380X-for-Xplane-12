//! Navigation-sensor physics for the A380X's three ADIRUs: a strapdown IRS
//! (ring-laser-gyro + accelerometer error models driving real navigation
//! equations, not a truth passthrough), a pitot-static ADR, and the radio
//! altimeters' terrain-boresight range. Physics workstream 4 of
//! `docs/briefs/hyperrealism.md`; full audit, equations and parameter
//! sources are in `docs/physics/adirs.md`.
//!
//! ## What was here before
//! FBW's own `adirs.rs` (`fbw-common/.../navigation/adirs.rs`) already has a
//! real, latitude-dependent alignment timer (`total_alignment_duration`) and
//! a full ARINC 429/SSM/fault-handling output stage -- both are kept
//! unmodified. What it did not have is any sensor between X-Plane's truth
//! and that output stage: `AdirsSimulatorData` read `PLANE LATITUDE`,
//! `PLANE PITCH DEGREES`, body rotation rates, true airspeed, Mach, AoA and
//! TAT directly off X-Plane's flight model every tick, and the ADR's
//! pressure/CAS came straight from `UpdateContext::ambient_pressure()` /
//! `indicated_airspeed()` -- also X-Plane truth. All 3 ADIRUs therefore
//! shared one perfect, noiseless, instantaneous "sensor".
//!
//! ## What this module does
//! Each of the 3 ADIRUs gets its own [`Adiru`]: independent gyro/
//! accelerometer bias and noise, a real strapdown mechanization (attitude
//! from integrated sensed body rates, velocity/position from integrated
//! sensed specific force with Earth-rate and transport-rate terms and a
//! latitude/altitude gravity model -- Schuler oscillation is a consequence
//! of these equations, not a special case), a gyrocompass-style heading
//! convergence during alignment, and a simple GPIRS complementary filter
//! blending the inertial position toward GPS truth. The same struct's
//! pitot-static model computes static/total pressure at the probe (with an
//! AoA-driven position error and icing/heating), inverts the standard
//! compressible pitot-static equations for CAS/Mach/TAS, and a
//! recovery-factor TAT. The results are written to new
//! `A32NX_ADIRS_SENSED_<n>_*` variables that FBW's small, additive
//! `patches/fbw-rust/navigation-sensors.patch` reads in place of the old
//! direct truth reads -- every existing ARINC/SSM/fault-handling line in
//! FBW's `adirs.rs` is untouched.
//!
//! [`RadioAltimeterProbe`] does the same for the ALA-52B radio altimeters:
//! FBW's own `ala52b.rs` already has a real antenna-geometry/reflection
//! model (`Ala52BTransceiverPair::response`), it was just fed X-Plane's
//! single, generally-straight-down `PLANE ALT ABOVE GROUND`. This probes the
//! terrain along the antenna boresight with X-Plane's own terrain-probe API,
//! sampled every [`RadioAltimeterProbe::PROBE_EVERY_TICKS`] ticks to bound
//! cost, and feeds that instead (again additively; see the same patch file).
//!
//! Every equation and parameter's source is cited in `docs/physics/adirs.md`.

use std::f64::consts::PI;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::xp::Xplm;
use crate::Vars;

// ---------------------------------------------------------------------
// Constants (docs/physics/adirs.md has the source for every one of these).
// ---------------------------------------------------------------------

const M_TO_FT: f64 = 1. / 0.3048;
const MS_TO_KNOT: f64 = 1.943_844_49;
const NM_TO_M: f64 = 1_852.0;

/// WGS84 mean Earth radius, for the position-integration and transport-rate
/// terms. A full WGS84 ellipsoid (varying radius of curvature with
/// latitude) is the textbook-complete version; a single mean radius is the
/// standard simplification for a Schuler-loop demonstration model, changing
/// the Schuler period by a fraction of a percent -- immaterial next to the
/// sensor error budget below.
const EARTH_RADIUS_M: f64 = 6_371_000.0;
/// Earth's rotation rate, rad/s (WGS84).
const EARTH_RATE_RAD_S: f64 = 7.292_115e-5;
/// Standard gravity, m/s^2 (only to convert accelerometer bias/noise
/// specified in g into m/s^2; the *used* local gravity is latitude/altitude
/// dependent, see [`normal_gravity`]).
const STANDARD_GRAVITY: f64 = 9.806_65;

/// Navigation-grade ring-laser-gyro bias instability, deg/hr. Published
/// figures for navigation-grade RLGs (the class ARINC 704 IRSs like the
/// Honeywell/Litton family use) are commonly quoted as "better than 0.01
/// deg/hr" for the primary navigation-grade objective; ARINC 704's own
/// numeric table is a paywalled standard this brief could not fetch, so this
/// is the widely-published class figure, used as the defensible derived
/// value (docs/physics/adirs.md).
const GYRO_BIAS_DEG_PER_HR: f64 = 0.01;
/// Angle random walk, deg/sqrt(hr): a published figure for a comparable
/// high-performance RLG (docs/physics/adirs.md).
const GYRO_ARW_DEG_PER_SQRT_HR: f64 = 0.002;
/// Gyro scale factor error, dimensionless (5 ppm; typical RLG-class figure).
const GYRO_SCALE_FACTOR_ERROR: f64 = 5e-6;
/// Accelerometer bias, g. Published figures for a strapdown navigation-grade
/// accelerometer are on the order of 5-10 micro-g; 8 micro-g is used here.
const ACCEL_BIAS_G: f64 = 8e-6;
/// Accelerometer velocity random walk, m/s/sqrt(hr) -- derived from the
/// gyro-class noise density scaled to a typical navigation accelerometer;
/// marked as a derived estimate (docs/physics/adirs.md).
const ACCEL_VRW_MS_PER_SQRT_HR: f64 = 0.01;
/// Accelerometer scale factor error, dimensionless (tens of ppm, typical
/// RLG-IMU-class figure).
const ACCEL_SCALE_FACTOR_ERROR: f64 = 2e-5;

/// GPIRS complementary filter time constant: how quickly the free-inertial
/// position is nudged toward the GPS position. Public Airbus/Honeywell
/// GPIRS descriptions say it blends GPS and inertial data for "100%
/// availability" RNP performance without publishing a filter time constant;
/// 120s is this brief's defensible derived value (fast enough to bound the
/// position error to GPS-like accuracy in cruise, slow enough that the
/// underlying inertial solution still shows its own Schuler/drift character
/// over the required 1-hour stationary test -- see that test for how it is
/// disabled to check the pure-inertial numbers).
const GPIRS_TIME_CONSTANT_S: f64 = 120.0;
/// Natural period of the baro-inertial vertical loop (a typical
/// transport-category value; the ADIRU's exact gains are not published).
const BARO_LOOP_PERIOD_S: f64 = 100.0;

/// Initial gyrocompass heading uncertainty at the *start* of alignment,
/// degrees, decaying exponentially during alignment (see [`Adiru::update`]).
/// Real ADIRU gyrocompass azimuth accuracy at the end of alignment is
/// commonly quoted as a few tenths of a degree; the coarse-align starting
/// uncertainty itself isn't published, so this is a defensible derived
/// value chosen so the residual error after FBW's own alignment timer
/// (300s at the equator) is of that order (docs/physics/adirs.md).
const GYROCOMPASS_INITIAL_ERROR_DEG: f64 = 6.0;
/// Gyrocompass convergence time constant at the equator, seconds, scaled by
/// `1 / cos(latitude)` -- mirroring the physical reason FBW's own
/// `total_alignment_duration_from_configuration` (adirs.rs) scales total
/// alignment time the same way: gyrocompassing torque is proportional to
/// the horizontal component of Earth's rotation rate,
/// `Omega_ie * cos(latitude)`, so time-to-converge scales as its reciprocal.
const GYROCOMPASS_TAU_S: f64 = 60.0;

/// TAT probe recovery factor. Rosemount-style total temperature probes are
/// commonly quoted in the 0.98-1.0 range; 0.99 is the mid value, used as the
/// defensible derived figure since no A380-specific number is published.
const TAT_RECOVERY_FACTOR: f64 = 0.99;
/// Ratio of specific heats for air.
const GAMMA_AIR: f64 = 1.4;
/// Specific gas constant for dry air, J/(kg*K).
const R_AIR: f64 = 287.052_87;
/// ISA sea level standard pressure, Pa.
const P0_PA: f64 = 101_325.0;
/// Sea-level speed of sound, ISA, knots.
const A0_KT: f64 = 661.4788;

/// AoA vane local-flow amplification factor: a vane ahead of the fuselage
/// senses a local flow angle larger than the freestream AoA because of the
/// fuselage's upwash field. 1.10 (10%) is a generic, commonly used
/// transport-aircraft planning figure (not A380-specific -- marked as a
/// derived estimate, docs/physics/adirs.md).
const AOA_VANE_UPWASH_FACTOR: f64 = 1.10;

/// Probe heater electrical load, W (Study-panel reporting only; see the
/// report for why it is not wired into the electrical workstream's load
/// balance). Typical transport-category pitot heater ratings are commonly
/// 300-450W per probe (TAT probes lower, ~80-150W); 350W/80W are used as
/// representative, generic, non-A380-specific figures.
const PITOT_HEATER_W: f64 = 350.0;
const TAT_HEATER_W: f64 = 80.0;

/// Icing accretion rate on an unheated probe in icing conditions (SAT<0C,
/// airborne), kg/s, and the mass at which the probe's orifice is considered
/// blocked. Nothing in FBW's own crate or public A380 documentation gives an
/// accretion rate for this specific orifice, so this is a clearly-marked
/// derived planning figure, sized to block an unheated probe within roughly
/// a minute of continuous icing -- consistent with why probe heat is
/// mandatory equipment.
const PROBE_BLOCK_MASS_KG: f64 = 0.002;

// ---------------------------------------------------------------------
// Continuous probe-heater heat balance (docs/physics/ice-protection.md):
// a Rosemount-style pitot probe is modelled as a cylinder in crossflow.
// Ice accretes only when the heat the (possibly degraded) heater can still
// supply falls short of what's needed to (a) offset dry-air convective
// cooling and (b) warm/keep-liquid the water mass the probe actually
// catches -- a simplified Messinger-style surface heat balance (Messinger,
// "Equilibrium Temperature of an Unheated Icing Surface as a Function of
// Air Speed", J. Aeronautical Sciences, 1953), not a temperature or
// on/off switch. Geometry (13mm dia x 150mm exposed length) is a
// representative Rosemount-class pitot-tube size, not A380-specific --
// marked as a derived figure like the module's other probe constants.
const PROBE_DIAMETER_M: f64 = 0.013;
const PROBE_EXPOSED_LENGTH_M: f64 = 0.15;
/// Target probe surface temperature the heater holds against convective/
/// water-catch cooling: just above freezing (0C), the minimum a probe-heat
/// system must sustain to stay clear (real probes run hotter under light
/// load, but 0C is the certification-relevant floor).
const PROBE_TARGET_SURFACE_C: f64 = 0.0;
/// Air thermal conductivity and kinematic viscosity, and Prandtl number, at
/// a representative cold (~-20C) icing-altitude condition (Incropera &
/// DeWitt, *Fundamentals of Heat and Mass Transfer*, Table A.4). Held
/// constant rather than temperature-interpolated -- a second-order
/// refinement next to the heater-power/LWC effects this model targets.
const AIR_K_W_MK: f64 = 0.0206;
const AIR_NU_M2_S: f64 = 1.13e-5;
const AIR_PR: f64 = 0.72;
const WATER_CP_J_KGK: f64 = 4186.0;
/// Latent heat of fusion of water, J/kg.
const WATER_LF_J_KG: f64 = 334_000.0;
/// A representative "high LWC" figure at the upper end of the FAA/CS-25
/// Appendix C continuous-maximum-icing envelope (14 CFR Part 25 Appendix
/// C; commonly cited peak figures around 0.6-0.8 g/m^3 at the envelope's
/// warmer end, dropping off at colder SAT) -- used only to scale the
/// unverified precipitation-ratio proxy below, not asserted as X-Plane
/// truth (see docs/physics/ice-protection.md for the caveat).
const REFERENCE_LWC_GM3: f64 = 0.6;

/// Convective heat-transfer coefficient for crossflow over a cylinder
/// (Zukauskas correlation, Nu = C*Re^m*Pr^0.37; Incropera & DeWitt Table
/// 7.4 -- C=0.26,m=0.6 for 1e3<=Re<=2e5, C=0.076,m=0.7 for
/// 2e5<Re<=1e6), evaluated at the probe's diameter and the true airspeed.
fn probe_convective_h_w_m2k(tas_ms: f64) -> f64 {
    let v = tas_ms.max(0.0);
    let re = v * PROBE_DIAMETER_M / AIR_NU_M2_S;
    if re <= 1.0 {
        return 0.0;
    }
    let (c, m) = if re <= 2.0e5 { (0.26, 0.6) } else { (0.076, 0.7) };
    let nu = c * re.powf(m) * AIR_PR.powf(0.37);
    nu * AIR_K_W_MK / PROBE_DIAMETER_M
}

/// Total heat, W, the probe heater must supply at this SAT/TAS/LWC to hold
/// [`PROBE_TARGET_SURFACE_C`]: dry-air convection over the probe's surface
/// area, plus warming-to-0C-and-keeping-liquid the water mass its frontal
/// area actually catches (mass flux = LWC * TAS * frontal area, collection
/// efficiency ~1 for a small, thin probe in high-speed flow -- a standard
/// simplification for this geometry). Returns `(required_w, catch_kg_s)`.
fn probe_heat_required_w(sat_c: f64, tas_ms: f64, lwc_gm3: f64) -> (f64, f64) {
    let delta_t = (PROBE_TARGET_SURFACE_C - sat_c).max(0.0);
    let h = probe_convective_h_w_m2k(tas_ms);
    let conv_area_m2 = PI * PROBE_DIAMETER_M * PROBE_EXPOSED_LENGTH_M;
    let q_conv = h * conv_area_m2 * delta_t;

    let frontal_area_m2 = PROBE_DIAMETER_M * PROBE_EXPOSED_LENGTH_M;
    let lwc_kg_m3 = (lwc_gm3.max(0.0)) * 1e-3;
    let catch_kg_s = lwc_kg_m3 * tas_ms.max(0.0) * frontal_area_m2;
    let q_water = catch_kg_s * (WATER_CP_J_KGK * delta_t + WATER_LF_J_KG);

    (q_conv + q_water, catch_kg_s)
}

/// Per-ADIRU lever arm from the aircraft CG to each ADIRU's own IMU case, in
/// aviation body axes (forward, right, down; metres). Real A380 ADIRU rack
/// coordinates are not published; the 3 ADIRUs are separate LRUs in the
/// forward avionics bay, which on a transport of this size sits well forward
/// of the CG (tens of metres) with the individual racks separated from each
/// other by well under a metre. This is a defensible derived placement
/// (forward avionics bay, sub-metre unit spacing), clearly marked as
/// assumed (docs/physics/adirs.md) -- it exists so each unit senses its own,
/// slightly different, rotation-induced specific force (Titterton & Weston,
/// *Strapdown Inertial Navigation Technology*, lever-arm correction
/// `a_P = a_cg + alpha x r + omega x (omega x r)`), not to model an exact
/// installation.
const ADIRU_LEVER_ARM_M: [[f64; 3]; 3] = [
    [20.0, -0.3, 1.0],
    [20.0, 0.0, 1.0],
    [20.0, 0.3, 1.0],
];

/// 1-sigma fixed mounting/boresight misalignment between an IMU's own case
/// axes and the airframe reference axes, degrees. This is a mechanical
/// installation tolerance (distinct from the *alignment process* error
/// [`GYROCOMPASS_INITIAL_ERROR_DEG`] models), drawn once per unit at
/// construction. Typical strapdown IMU installation/boresight alignment
/// tolerances are commonly quoted in the tenths-of-a-degree range; no
/// A380-specific figure is published, so 0.1 degree (1-sigma, per axis) is
/// used as a defensible generic value (docs/physics/adirs.md).
const MOUNT_MISALIGNMENT_SIGMA_DEG: f64 = 0.1;

/// 1980 International Gravity Formula (Geodetic Reference System 1980,
/// "Somigliana equation"): sea-level normal gravity as a function of
/// latitude, plus the standard free-air correction with altitude
/// (-3.086e-6 (m/s^2)/m). Widely published, e.g. NOAA/NGA geodesy
/// references or Hofmann-Wellenhof & Moritz, "Physical Geodesy".
fn normal_gravity(lat_rad: f64, alt_m: f64) -> f64 {
    let s2 = lat_rad.sin().powi(2);
    let g0 = 9.780_327 * (1. + 0.005_3024 * s2 - 0.000_0058 * (2. * lat_rad).sin().powi(2));
    g0 - 3.086e-6 * alt_m
}

// ---------------------------------------------------------------------
// A tiny, self-contained PRNG (no new crate dependency). splitmix64 for the
// generator, Box-Muller for Gaussian noise. Not cryptographic; just needs to
// be deterministic per session and per axis/unit so the 3 ADIRUs draw
// different, session-stable biases.
// ---------------------------------------------------------------------

struct Rng(u64);
impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed ^ 0x9E3779B97F4A7C15)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Standard normal, Box-Muller.
    fn gaussian(&mut self) -> f64 {
        let u1 = self.next_f64().max(1e-12);
        let u2 = self.next_f64();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

/// A gyro axis: bias, scale factor error and angle-random-walk noise applied
/// to a true rate (deg/s) to produce a sensed rate (deg/s).
///
/// The random-walk discretization (noise standard deviation scales as
/// `ARW / sqrt(dt)`) is the standard way to simulate a fixed *angle* random
/// walk regardless of the integration step: a random walk's variance grows
/// linearly with elapsed time, so sampling white noise more often needs a
/// proportionally larger per-sample variance to keep the same growth per
/// second (see e.g. Woodman, "An introduction to inertial navigation",
/// University of Cambridge TR-696, on gyro noise modelling).
struct GyroAxis {
    bias_deg_s: f64,
    scale_factor: f64,
    arw_rad_per_sqrt_s: f64,
}
impl GyroAxis {
    fn new(rng: &mut Rng) -> Self {
        Self {
            bias_deg_s: rng.gaussian() * GYRO_BIAS_DEG_PER_HR / 3600.0,
            scale_factor: rng.gaussian() * GYRO_SCALE_FACTOR_ERROR,
            arw_rad_per_sqrt_s: GYRO_ARW_DEG_PER_SQRT_HR.to_radians() / 60.0,
        }
    }

    fn sense(&self, true_rate_deg_s: f64, dt: f64, rng: &mut Rng) -> f64 {
        let noise_rad_s = rng.gaussian() * self.arw_rad_per_sqrt_s / dt.max(1e-6).sqrt();
        true_rate_deg_s * (1.0 + self.scale_factor) + self.bias_deg_s + noise_rad_s.to_degrees()
    }
}

/// An accelerometer axis, in g (bias/noise) applied to a true specific force
/// (g) to produce a sensed one (g). Same random-walk discretization
/// reasoning as [`GyroAxis`], for velocity random walk instead of angle.
struct AccelAxis {
    bias_g: f64,
    scale_factor: f64,
    vrw_ms_per_sqrt_s: f64,
}
impl AccelAxis {
    fn new(rng: &mut Rng) -> Self {
        Self {
            bias_g: rng.gaussian() * ACCEL_BIAS_G,
            scale_factor: rng.gaussian() * ACCEL_SCALE_FACTOR_ERROR,
            vrw_ms_per_sqrt_s: ACCEL_VRW_MS_PER_SQRT_HR / 60.0,
        }
    }

    fn sense(&self, true_g: f64, dt: f64, rng: &mut Rng) -> f64 {
        let noise_ms2 = rng.gaussian() * self.vrw_ms_per_sqrt_s / dt.max(1e-6).sqrt();
        true_g * (1.0 + self.scale_factor) + self.bias_g + noise_ms2 / STANDARD_GRAVITY
    }
}

/// True aircraft state this tick, read once from X-Plane and shared by all
/// 3 ADIRUs -- there is only one real airframe; the per-unit divergence
/// comes from each [`Adiru`]'s own gyro/accelerometer error draw and
/// mechanization state, not from 3 different truths (see the report for
/// this as a known depth limit).
#[derive(Clone, Copy, Default)]
pub struct TrueState {
    /// deg/s, X-Plane's own P/Q/R (roll/pitch/yaw rate) convention.
    pub p_xp: f64,
    pub q_xp: f64,
    pub r_xp: f64,
    /// g, X-Plane's own g_axil/g_side/g_nrml (forward/lateral/normal).
    pub g_axil: f64,
    pub g_side: f64,
    pub g_nrml: f64,
    /// degrees, X-Plane's theta (pitch, nose-up +)/phi (roll, right +)/
    /// psi (true heading, 0-360).
    pub theta_xp: f64,
    pub phi_xp: f64,
    pub psi_xp: f64,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f64,
    /// True static pressure at the aircraft (Pa), true SAT (deg C), true
    /// Mach, and true AoA (deg), for the ADR model.
    pub static_pressure_pa: f64,
    pub sat_c: f64,
    pub mach: f64,
    pub alpha_deg: f64,
    pub on_ground: bool,
    /// Whether X-Plane has actually placed the aircraft yet (mirrors
    /// [`agl_looks_placed`]'s own check, computed once in
    /// `TrueStateSource::read` and shared here so [`Adiru::advance`] can
    /// gate on it too -- see the debug brief `docs/deep/debug_vs.md` for the
    /// bug this closes: without it, `should_run` could turn true (FBW's own
    /// ADIRS reports "Aligned" from the very first tick, see `advance`'s own
    /// doc comment) before X-Plane has ever handed this module a real
    /// position, so the free-inertial mechanization would start from -- and
    /// the baro-inertial loop would then have to null out -- a completely
    /// synthetic altitude/position error.
    pub placed: bool,
    /// m/s, true velocity over the Earth (north, east), for the nav frame's
    /// transport rate the gyros sense.
    pub v_north_ms: f64,
    pub v_east_ms: f64,
    /// Whether at least one engine is running (X-Plane's own per-engine
    /// `ENGN_running`), for the probe-heat AUTO logic (see `update_adr`).
    pub any_engine_running: bool,
    /// Liquid water content at the aircraft, g/m^3, for the probe heat
    /// balance (see `update_adr`) and (once wired) wing/engine accretion.
    /// X-Plane 12 has no first-party LWC dataref this pass could verify
    /// against the local install in the time available; this is
    /// `sim/weather/aircraft/precipitation_on_aircraft_ratio` (0..1) scaled
    /// by [`REFERENCE_LWC_GM3`], a clearly-flagged proxy -- see
    /// docs/physics/ice-protection.md for the caveat and what would
    /// replace it.
    pub lwc_gm3: f64,
}

/// The standard aerospace body-to-NED direction cosine matrix (Z-Y-X Euler:
/// yaw psi, then pitch theta, then roll phi), aviation convention (theta
/// nose-up +, phi right-wing-down +, psi clockwise from true north).
/// Stevens & Lewis, "Aircraft Control and Simulation", eq. 1.3-23 (any
/// standard flight-dynamics text has the same matrix).
fn body_to_ned(theta_deg: f64, phi_deg: f64, psi_deg: f64, f_body: [f64; 3]) -> [f64; 3] {
    let (t, p, y) = (theta_deg.to_radians(), phi_deg.to_radians(), psi_deg.to_radians());
    let (st, ct) = (t.sin(), t.cos());
    let (sp, cp) = (p.sin(), p.cos());
    let (sy, cy) = (y.sin(), y.cos());
    let [fx, fy, fz] = f_body;
    [
        (ct * cy) * fx + (sp * st * cy - cp * sy) * fy + (cp * st * cy + sp * sy) * fz,
        (ct * sy) * fx + (sp * st * sy + cp * cy) * fy + (cp * st * sy - sp * cy) * fz,
        (-st) * fx + (sp * ct) * fy + (cp * ct) * fz,
    ]
}

/// The inverse of [`body_to_ned`] (the DCM is orthogonal, so this is its
/// transpose): rotates a NED vector into body axes.
fn ned_to_body(theta_deg: f64, phi_deg: f64, psi_deg: f64, v_n: [f64; 3]) -> [f64; 3] {
    let (t, p, y) = (theta_deg.to_radians(), phi_deg.to_radians(), psi_deg.to_radians());
    let (st, ct) = (t.sin(), t.cos());
    let (sp, cp) = (p.sin(), p.cos());
    let (sy, cy) = (y.sin(), y.cos());
    let [n, e, d] = v_n;
    [
        (ct * cy) * n + (ct * sy) * e + (-st) * d,
        (sp * st * cy - cp * sy) * n + (sp * st * sy + cp * cy) * e + (sp * ct) * d,
        (cp * st * cy + sp * sy) * n + (cp * st * sy - sp * cy) * e + (cp * ct) * d,
    ]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn wrap_360(deg: f64) -> f64 {
    let m = deg % 360.0;
    if m < 0.0 {
        m + 360.0
    } else {
        m
    }
}

/// Rigid-body lever-arm specific-force correction at a point `r` (body axes,
/// m) offset from the point the true specific force/rate was measured at:
/// `alpha x r + omega x (omega x r)` (Titterton & Weston, *Strapdown
/// Inertial Navigation Technology*), in m/s^2.
fn lever_arm_specific_force(omega_rad_s: [f64; 3], alpha_rad_s2: [f64; 3], r_m: [f64; 3]) -> [f64; 3] {
    let centripetal = cross(omega_rad_s, cross(omega_rad_s, r_m));
    let tangential = cross(alpha_rad_s2, r_m);
    [
        tangential[0] + centripetal[0],
        tangential[1] + centripetal[1],
        tangential[2] + centripetal[2],
    ]
}

/// Small-angle rotation of a vector by a fixed misalignment `eps` (rad):
/// `v' = v + eps x v`, the standard first-order approximation of a rotation
/// DCM for small angles (exact to O(eps^2), plenty for a tenths-of-a-degree
/// mounting tolerance).
fn rotate_small_angle(eps: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    let c = cross(eps, v);
    [v[0] + c[0], v[1] + c[1], v[2] + c[2]]
}

/// Fine leveling: the pitch/roll a real strapdown alignment derives from the
/// sensed specific force at rest (any strapdown/AHRS alignment text, e.g.
/// Titterton & Weston ch. 9; the same relation [`Adiru::state_derivative`]
/// uses in reverse, `body_to_ned`'s `[0,0,-g]` case solved for pitch/roll).
/// At rest (low dynamics -- the only regime this is called in, ground
/// alignment) the accelerometers sense (the negative of) the local gravity
/// vector, so `f_fwd = g*sin(pitch)`, `f_right = -g*sin(roll)*cos(pitch)`,
/// `f_down = -g*cos(roll)*cos(pitch)`; the last two solve for roll via
/// `atan2` without dividing by `cos(pitch)`, exact rather than small-angle.
/// `f_body_g` is specific force in g (aviation forward/right/down);
/// `misalign_eps_rad` is this unit's own fixed IMU-case-to-airframe
/// mounting misalignment (see [`MOUNT_MISALIGNMENT_SIGMA_DEG`]), applied
/// here the same way `mechanize` applies it to every other tick's specific
/// force -- so leveling is *consistent* with the mounting a real, boresight-
/// calibrated installation would level against, not a discontinuity from a
/// truth passthrough that only starts seeing the misalignment once
/// free-inertial mechanization begins (see the report, "the unrealistic
/// leveling discontinuity" -- that jump was itself the second live-defect
/// contributor, a spurious step onto a wrong reference the Schuler loop
/// then took many minutes to null out).
fn level_from_specific_force(f_body_g: [f64; 3], misalign_eps_rad: [f64; 3]) -> (f64, f64) {
    let f = rotate_small_angle(misalign_eps_rad, f_body_g);
    let pitch_rad = f[0].clamp(-1.0, 1.0).asin();
    let roll_rad = (-f[1]).atan2(-f[2]);
    (pitch_rad.to_degrees(), roll_rad.to_degrees())
}

fn add_scaled(a: [f64; 9], b: [f64; 9], scale: f64) -> [f64; 9] {
    let mut out = a;
    for i in 0..9 {
        out[i] += b[i] * scale;
    }
    out
}

/// The `A32NX_ADIRS_SENSED_<n>_*` and Study-panel variable identifiers one
/// ADIRU writes, resolved once at construction. See `AdirsSensedData` in
/// `patches/fbw-rust/navigation-sensors.patch` for the FBW side reading the
/// non-Study ones.
struct AdiruVars {
    valid: VariableIdentifier,
    pitch: VariableIdentifier,
    roll: VariableIdentifier,
    true_heading: VariableIdentifier,
    true_track: VariableIdentifier,
    body_rotation_rate_x: VariableIdentifier,
    body_rotation_rate_y: VariableIdentifier,
    body_rotation_rate_z: VariableIdentifier,
    latitude: VariableIdentifier,
    longitude: VariableIdentifier,
    ground_speed: VariableIdentifier,
    vertical_speed: VariableIdentifier,
    true_airspeed: VariableIdentifier,
    mach: VariableIdentifier,
    total_air_temperature: VariableIdentifier,
    angle_of_attack: VariableIdentifier,
    static_pressure: VariableIdentifier,
    computed_airspeed: VariableIdentifier,
    // Study panel only; the lead's Study UI reads these directly (see the
    // report for the full list of Study quantities).
    study_align_state: VariableIdentifier,
    study_position_error_nm: VariableIdentifier,
    study_drift_nm_hr: VariableIdentifier,
    study_gyro_bias_deg_hr: VariableIdentifier,
    study_accel_bias_ug: VariableIdentifier,
    study_pitot_blocked: VariableIdentifier,
    study_static_blocked: VariableIdentifier,
    study_probe_heat_w: VariableIdentifier,
    study_static_pa: VariableIdentifier,
    study_total_pa: VariableIdentifier,
    /// Shared-contract variable (docs/briefs/hyperrealism.md's ADIRS
    /// bullet): the probe heaters' electrical load, for the electrical
    /// workstream to read as a bus consumer.
    probe_heat_load_w: VariableIdentifier,
}
impl AdiruVars {
    /// All fields default (unregistered) identifiers, for tests that never
    /// call `write` (see [`Adiru::new_for_test`]).
    #[cfg(test)]
    fn dummy() -> Self {
        Self {
            valid: Default::default(),
            pitch: Default::default(),
            roll: Default::default(),
            true_heading: Default::default(),
            true_track: Default::default(),
            body_rotation_rate_x: Default::default(),
            body_rotation_rate_y: Default::default(),
            body_rotation_rate_z: Default::default(),
            latitude: Default::default(),
            longitude: Default::default(),
            ground_speed: Default::default(),
            vertical_speed: Default::default(),
            true_airspeed: Default::default(),
            mach: Default::default(),
            total_air_temperature: Default::default(),
            angle_of_attack: Default::default(),
            static_pressure: Default::default(),
            computed_airspeed: Default::default(),
            study_align_state: Default::default(),
            study_position_error_nm: Default::default(),
            study_drift_nm_hr: Default::default(),
            study_gyro_bias_deg_hr: Default::default(),
            study_accel_bias_ug: Default::default(),
            study_pitot_blocked: Default::default(),
            study_static_blocked: Default::default(),
            study_probe_heat_w: Default::default(),
            study_static_pa: Default::default(),
            study_total_pa: Default::default(),
            probe_heat_load_w: Default::default(),
        }
    }

    fn new(vars: &mut Vars, n: usize) -> Self {
        let mut id = |field: &str| vars.get(format!("ADIRS_SENSED_{n}_{field}"));
        Self {
            valid: id("VALID"),
            pitch: id("PITCH"),
            roll: id("ROLL"),
            true_heading: id("TRUE_HEADING"),
            true_track: id("TRUE_TRACK"),
            body_rotation_rate_x: id("BODY_ROTATION_RATE_X"),
            body_rotation_rate_y: id("BODY_ROTATION_RATE_Y"),
            body_rotation_rate_z: id("BODY_ROTATION_RATE_Z"),
            latitude: id("LATITUDE"),
            longitude: id("LONGITUDE"),
            ground_speed: id("GROUND_SPEED"),
            vertical_speed: id("VERTICAL_SPEED"),
            true_airspeed: id("TRUE_AIRSPEED"),
            mach: id("MACH"),
            total_air_temperature: id("TOTAL_AIR_TEMPERATURE"),
            angle_of_attack: id("ANGLE_OF_ATTACK"),
            static_pressure: id("STATIC_PRESSURE"),
            computed_airspeed: id("COMPUTED_AIRSPEED"),
            study_align_state: vars.get(format!("ADIRS_STUDY_{n}_ALIGN_STATE")),
            study_position_error_nm: vars.get(format!("ADIRS_STUDY_{n}_POSITION_ERROR_NM")),
            study_drift_nm_hr: vars.get(format!("ADIRS_STUDY_{n}_DRIFT_NM_HR")),
            study_gyro_bias_deg_hr: vars.get(format!("ADIRS_STUDY_{n}_GYRO_BIAS_DEG_HR")),
            study_accel_bias_ug: vars.get(format!("ADIRS_STUDY_{n}_ACCEL_BIAS_UG")),
            study_pitot_blocked: vars.get(format!("ADIRS_STUDY_{n}_PITOT_BLOCKED")),
            study_static_blocked: vars.get(format!("ADIRS_STUDY_{n}_STATIC_BLOCKED")),
            study_probe_heat_w: vars.get(format!("ADIRS_STUDY_{n}_PROBE_HEAT_W")),
            study_static_pa: vars.get(format!("ADIRS_STUDY_{n}_STATIC_PRESSURE_PA")),
            study_total_pa: vars.get(format!("ADIRS_STUDY_{n}_TOTAL_PRESSURE_PA")),
            probe_heat_load_w: vars.get(format!("PROBE_HEAT_LOAD_W:{n}")),
        }
    }
}

/// One ADIRU's strapdown IRS and pitot-static ADR. `number` is 1..=3.
pub struct Adiru {
    #[allow(dead_code)]
    number: usize,
    rng: Rng,

    gyro: [GyroAxis; 3],   // p, q, r (aviation body rates)
    accel: [AccelAxis; 3], // forward, right, down (aviation specific force)

    /// Free-inertial mechanization is running (ADIRU aligned and powered).
    running: bool,
    /// Degrees: aviation pitch/roll, true heading.
    pitch_deg: f64,
    roll_deg: f64,
    heading_deg: f64,
    /// m/s, NED.
    v_north: f64,
    v_east: f64,
    v_down: f64,
    lat_rad: f64,
    lon_rad: f64,
    alt_m: f64,
    /// Residual gyrocompass heading error, degrees, decaying during
    /// alignment (see [`Adiru::update`]).
    heading_error_deg: f64,
    seconds_aligning: f64,
    /// The last sensed body rates, X-Plane's own P/Q/R convention (deg/s),
    /// for [`Self::write`] to put back in the same slots the truth
    /// passthrough used. Zero while not running (ignored downstream anyway,
    /// since `valid` is false then).
    last_p_sensed_xp: f64,
    last_q_sensed_xp: f64,
    last_r_sensed_xp: f64,
    /// FBW's own `A32NX_ADIRS_ADIRU_<n>_STATE` this tick (0 Off, 1 Aligning,
    /// 2 Aligned), kept only to mirror onto the Study panel.
    last_fbw_state: f64,
    /// Sensed body rate last tick (rad/s, aviation p/q/r), for the lever-arm
    /// correction's finite-differenced angular acceleration.
    last_omega_body_rad_s: [f64; 3],
    /// Fixed per-unit IMU-case-to-airframe mounting misalignment (rad),
    /// drawn once at construction (see [`MOUNT_MISALIGNMENT_SIGMA_DEG`]).
    misalign_eps_rad: [f64; 3],

    // ADR probe state.
    pitot_ice_kg: f64,
    static_ice_kg: f64,
    frozen_total_pressure_pa: f64,
    frozen_static_pressure_pa: f64,

    // Latest ADR outputs (computed every tick regardless of IR alignment --
    // a real ADR's ~18s init timer is independent of IR gyrocompassing).
    adr_static_pa: f64,
    adr_total_pa: f64,
    adr_cas_kt: f64,
    adr_mach: f64,
    adr_tas_ms: f64,
    adr_tat_c: f64,
    adr_aoa_deg: f64,
    adr_pitot_blocked: bool,
    adr_static_blocked: bool,
    adr_probe_heat_w: f64,

    // Study/reporting.
    position_error_m: f64,
    drift_rate_smoothed_nm_hr: f64,

    v: AdiruVars,
}

impl Adiru {
    fn new(vars: &mut Vars, number: usize) -> Self {
        Self::new_with(number, AdiruVars::new(vars, number))
    }

    /// Test-only constructor, bypassing `Vars`/`Xplm` (unavailable outside a
    /// running X-Plane): the unit tests below exercise the pure physics
    /// (`mechanize`, `gpirs_correct`, `update_adr`) directly and never call
    /// `write`, so the dummy variable identifiers are never used.
    #[cfg(test)]
    fn new_for_test(number: usize) -> Self {
        Self::new_with(number, AdiruVars::dummy())
    }

    fn new_with(number: usize, v: AdiruVars) -> Self {
        // Seed each unit+axis independently and deterministically, so the 3
        // ADIRUs draw different (but session-stable) biases -- the "per
        // ADIRU ... sensor biases" the Study panel shows are meant to
        // actually differ between units.
        let mut seed_rng = Rng::new(0xAD1_5000 ^ (number as u64));
        let mut gyro_rng = Rng::new(seed_rng.next_u64());
        let mut accel_rng = Rng::new(seed_rng.next_u64());
        let mut misalign_rng = Rng::new(seed_rng.next_u64());
        let misalign_eps_rad = [
            misalign_rng.gaussian() * MOUNT_MISALIGNMENT_SIGMA_DEG.to_radians(),
            misalign_rng.gaussian() * MOUNT_MISALIGNMENT_SIGMA_DEG.to_radians(),
            misalign_rng.gaussian() * MOUNT_MISALIGNMENT_SIGMA_DEG.to_radians(),
        ];
        Self {
            number,
            rng: Rng::new(seed_rng.next_u64()),
            gyro: [
                GyroAxis::new(&mut gyro_rng),
                GyroAxis::new(&mut gyro_rng),
                GyroAxis::new(&mut gyro_rng),
            ],
            accel: [
                AccelAxis::new(&mut accel_rng),
                AccelAxis::new(&mut accel_rng),
                AccelAxis::new(&mut accel_rng),
            ],
            running: false,
            pitch_deg: 0.,
            roll_deg: 0.,
            heading_deg: 0.,
            v_north: 0.,
            v_east: 0.,
            v_down: 0.,
            lat_rad: 0.,
            lon_rad: 0.,
            alt_m: 0.,
            // 0, not `GYROCOMPASS_INITIAL_ERROR_DEG` (the *start*-of-
            // alignment coarse uncertainty): a freshly constructed unit
            // whose very first `advance()` call already finds FBW's own
            // `A32NX_ADIRS_ADIRU_<n>_STATE` at Aligned (2) -- exactly the
            // "starting on the runway or in the air" case `advance`'s own
            // doc comment describes -- never passes through this session's
            // `!should_run`-with-`fbw_state==1` (Aligning) branch at all, so
            // nothing here would ever decay this value from the coarse
            // starting uncertainty toward the converged one; it would stay
            // at the *start* value forever. FBW reporting the unit Aligned
            // (not Aligning) is itself the claim that gyrocompass
            // convergence has already happened -- honouring that claim
            // means starting from the *converged* residual (this module's
            // own `heading_error_decays_during_a_realistic_alignment` test:
            // under 1 degree after a real 300s alignment), not the raw
            // coarse-align value, which is exactly the live defect's root
            // cause (see the report): a permanent, uncorrected ~6 degree
            // heading bias baked into the mechanization at spawn, presented
            // as valid NAV data. A unit that *is* observed going through a
            // real Off->Aligning transition in this session still gets the
            // coarse value explicitly (`advance`'s `fbw_state < 0.999`
            // branch), so this default only affects the "already aligned
            // before this plugin ever saw it" case.
            heading_error_deg: 0.,
            seconds_aligning: 0.,
            last_p_sensed_xp: 0.,
            last_q_sensed_xp: 0.,
            last_r_sensed_xp: 0.,
            last_fbw_state: 0.,
            last_omega_body_rad_s: [0., 0., 0.],
            misalign_eps_rad,
            pitot_ice_kg: 0.,
            static_ice_kg: 0.,
            frozen_total_pressure_pa: 0.,
            frozen_static_pressure_pa: 0.,
            adr_static_pa: P0_PA,
            adr_total_pa: P0_PA,
            adr_cas_kt: 0.,
            adr_mach: 0.,
            adr_tas_ms: 0.,
            adr_tat_c: 15.,
            adr_aoa_deg: 0.,
            adr_pitot_blocked: false,
            adr_static_blocked: false,
            adr_probe_heat_w: 0.,
            position_error_m: 0.,
            drift_rate_smoothed_nm_hr: 0.,
            v,
        }
    }

    /// `fbw_state`: FBW's own `A32NX_ADIRS_ADIRU_<n>_STATE` (0 Off, 1
    /// Aligning, 2 Aligned; see `AirDataInertialReferenceUnit::state` in
    /// adirs.rs). `powered`: this unit's electrical bus (see the report for
    /// which bus and why the mapping is a documented assumption).
    /// `nav_mode`: FBW's own IR mode selector is in Navigation (true) or
    /// Attitude (false) -- see `InertialReferenceMode` in adirs.rs.
    /// `gps_valid`: this unit's assigned MMR has a valid position (see
    /// [`AdirsPhysics`] for the ADIRU-to-MMR mapping).
    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        vars: &mut Vars,
        dt: f64,
        t: &TrueState,
        fbw_state: f64,
        powered: bool,
        nav_mode: bool,
        gps_valid: bool,
    ) {
        let should_run = self.advance(dt, t, fbw_state, powered, nav_mode, gps_valid);
        self.write(vars, should_run);
    }

    /// Test-only equivalent of [`Self::update`], without the final `write`
    /// (which needs a real `Vars`/X-Plane) -- see [`Self::new_for_test`].
    #[cfg(test)]
    fn update_for_test(&mut self, dt: f64, t: &TrueState, fbw_state: f64, powered: bool) -> bool {
        self.advance(dt, t, fbw_state, powered, true, true)
    }

    /// Everything `update` does except writing the outputs; returns whether
    /// the ADIRU is running its free-inertial solution this tick (aligned
    /// and powered).
    fn advance(
        &mut self,
        dt: f64,
        t: &TrueState,
        fbw_state: f64,
        powered: bool,
        nav_mode: bool,
        gps_valid: bool,
    ) -> bool {
        self.last_fbw_state = fbw_state;
        // `&& t.placed`: see [`TrueState::placed`]'s doc comment and
        // `docs/deep/debug_vs.md`. FBW's own ADIRS (fbw-common's
        // `navigation/adirs.rs`, `InertialReference::new`) starts every unit
        // already `AlignState::Aligned` -- `remaining_align_duration:
        // Some(Duration::from_secs(0))` unconditionally, "to support
        // starting on the runway or in the air" -- so `fbw_state >= 1.999`
        // can be true on literally the first tick this module ever sees,
        // before X-Plane has placed the aircraft (its position/altitude
        // still reads pre-placement garbage, which `TrueStateSource::read`
        // freezes at 0 via `agl_looks_placed`). Requiring `t.placed` here
        // keeps this unit in the "not running" branch below -- which
        // continuously copies `t.lat_deg`/`lon_deg`/`alt_m` truth into this
        // struct -- until a real position exists to start mechanizing from.
        let mut should_run = fbw_state >= 1.999 && powered && t.placed;

        if !should_run {
            // Not aligned, or unpowered: no free-inertial solution. Re-seed
            // the mechanization from truth so the *next* alignment starts
            // fresh -- this is the plugin's model of "loss of alignment on
            // power loss" (FBW's own overhead-panel/mode-selector logic has
            // no electrical dependency to hook into more directly; see the
            // report). When aligning (fbw_state == 1), gyrocompass the
            // heading; pitch/roll leveling from the accelerometers is fast
            // and accurate, so they simply track truth here.
            self.running = false;
            self.pitch_deg = t.theta_xp;
            self.roll_deg = t.phi_xp;
            if fbw_state < 0.999 {
                self.seconds_aligning = 0.;
                self.heading_error_deg = GYROCOMPASS_INITIAL_ERROR_DEG;
                self.heading_deg = t.psi_xp;
            } else {
                self.seconds_aligning += dt;
                let tau = GYROCOMPASS_TAU_S / t.lat_deg.to_radians().cos().max(0.05);
                self.heading_error_deg =
                    GYROCOMPASS_INITIAL_ERROR_DEG * (-self.seconds_aligning / tau).exp();
                self.heading_deg = wrap_360(t.psi_xp + self.heading_error_deg);
            }
            self.lat_rad = t.lat_deg.to_radians();
            self.lon_rad = t.lon_deg.to_radians();
            self.alt_m = t.alt_m;
            self.v_north = 0.;
            self.v_east = 0.;
            self.v_down = 0.;
            self.last_p_sensed_xp = 0.;
            self.last_q_sensed_xp = 0.;
            self.last_r_sensed_xp = 0.;
            self.position_error_m = 0.;
            self.drift_rate_smoothed_nm_hr = 0.;
        } else {
            if !self.running {
                // Just finished aligning: freeze whatever gyrocompass error
                // remains and start free-inertial dead reckoning from here.
                //
                // Bug fix (docs/deep/debug_vs.md, the reported "-8800 ft/min
                // on the ground" PFD symptom): re-sync position, heading and
                // velocity to *this tick's truth* right now, rather than
                // trusting whatever this struct's fields already held. The
                // only other place that keeps `lat_rad`/`lon_rad`/`alt_m` in
                // sync with X-Plane truth is the `!should_run` branch above
                // -- but with `t.placed` now required for `should_run`
                // (see it and [`TrueState::placed`]), the first tick this
                // branch ever runs is guaranteed to have a genuine,
                // just-placed truth reading, so copying it here zeroes the
                // baro-inertial loop's initial altitude error outright
                // instead of leaving it to null out a large, spurious
                // one-tick jump through several minutes of synthetic
                // vertical speed (`baro_inertial_correct`'s ~100s time
                // constant). The same jump existed for ground speed/track
                // (from `v_north`/`v_east`), true position (from
                // `lat_rad`/`lon_rad`, previously stuck at 0N/0E "Null
                // Island" until GPIRS slowly dragged it to truth over
                // `GPIRS_TIME_CONSTANT_S`) and pitch/roll/heading (stuck
                // level/0 until the first `!should_run` tick, which may
                // never come) -- all are "other ADIRUS outputs wrong at
                // spawn" instances of the identical root cause, fixed the
                // same way here.
                self.running = true;
                // Fine leveling from the sensed (misalignment-rotated)
                // specific force, not a raw truth passthrough -- see
                // `level_from_specific_force`'s doc comment for why the
                // previous `t.theta_xp`/`t.phi_xp` passthrough was itself a
                // live-defect contributor (a step onto an unmechanized
                // reference the mounting misalignment then had to be
                // Schuler-nulled out of over several minutes, rather than
                // starting from the same reference `mechanize` uses every
                // subsequent tick).
                // Only on the ground: accelerometer leveling assumes low
                // dynamics (specific force is gravity alone), which is true
                // for a real ground alignment but not for a unit that
                // starts already aligned *in the air* (FBW's own "or in the
                // air" case) -- there, specific force includes real
                // manoeuvring acceleration, so it would give the wrong
                // attitude, and the existing truth passthrough (this
                // module's best available estimate of an in-flight unit's
                // already-converged solution) is kept.
                if t.on_ground {
                    let f_true_g = [t.g_axil, t.g_side, -t.g_nrml];
                    let (pitch_deg, roll_deg) = level_from_specific_force(f_true_g, self.misalign_eps_rad);
                    self.pitch_deg = pitch_deg;
                    self.roll_deg = roll_deg;
                } else {
                    self.pitch_deg = t.theta_xp;
                    self.roll_deg = t.phi_xp;
                }
                self.heading_deg = wrap_360(t.psi_xp + self.heading_error_deg);
                self.lat_rad = t.lat_deg.to_radians();
                self.lon_rad = t.lon_deg.to_radians();
                self.alt_m = t.alt_m;
                self.v_north = t.v_north_ms;
                self.v_east = t.v_east_ms;
                self.v_down = 0.0;
            }
            self.mechanize(dt, t);
            // The vertical channel is always baro-aided: free-inertial
            // altitude is exponentially unstable (gravity falls with height).
            self.baro_inertial_correct(dt, t);
            // GPIRS aiding is only available with a valid GPS position in
            // Navigation mode; in Attitude mode (no nav function) or with
            // GPS lost, this is a pure free-inertial dead-reckoning step --
            // see `mechanize`'s RK4 doc comment for why that stays bounded
            // without GPIRS damping.
            if nav_mode && gps_valid {
                self.gpirs_correct(dt, t);
            }

            // BITE-style fault check: nothing between here and `mechanize`
            // guards a single bad tick's *sensed* rate/force from producing
            // a wildly-wrong (or outright non-finite) mechanized state --
            // see the report's live-defect analysis. The real, causal fix is
            // rejecting an implausible raw reading before it ever reaches
            // `mechanize` (`TrueStateSource::read`'s new `plausible` gate,
            // below); this is the second, independent layer a real IRU also
            // has: its own BITE monitors the mechanized *solution* and
            // faults/re-aligns rather than keep annunciating a corrupted
            // one, in case some other, unanticipated corruption source ever
            // reaches this state (a NaN from a future change, an addon
            // stuffing a garbage value into a shared dataref this module
            // doesn't own, etc.) -- "don't fake values" cuts the other way
            // too: continuing to integrate and *publish* a state this
            // module can already tell is unphysical would itself be
            // presenting fabricated data as valid, exactly the live defect
            // (PFD attitude sphere rolled tens of degrees with no ATT/HDG
            // flag). `MAX_PLAUSIBLE_*` below are generous ceilings no real,
            // physically-possible aircraft state can reach (a stationary or
            // in-flight A380 never approaches Mach 3 groundspeed or a
            // sustained >45 degree bank/pitch outside a deliberate
            // aerobatic maneuver this airframe cannot perform) -- this is a
            // sanity fault threshold, not a plausible-looking clamp of the
            // published number: on a fault this unit stops publishing
            // `valid` data and re-aligns from truth, it does not keep
            // reporting a truncated/clamped version of the bad value as if
            // it were real.
            const MAX_PLAUSIBLE_GROUND_SPEED_MS: f64 = 600.0; // ~1166 kt
            const MAX_PLAUSIBLE_VERTICAL_SPEED_MS: f64 = 200.0; // ~39,000 fpm
            const MAX_PLAUSIBLE_BANK_PITCH_DEG: f64 = 80.0;
            let mechanization_finite = self.pitch_deg.is_finite()
                && self.roll_deg.is_finite()
                && self.heading_deg.is_finite()
                && self.v_north.is_finite()
                && self.v_east.is_finite()
                && self.v_down.is_finite()
                && self.lat_rad.is_finite()
                && self.lon_rad.is_finite()
                && self.alt_m.is_finite();
            let mechanization_plausible = mechanization_finite
                && self.pitch_deg.abs() <= MAX_PLAUSIBLE_BANK_PITCH_DEG
                && self.roll_deg.abs() <= MAX_PLAUSIBLE_BANK_PITCH_DEG
                && (self.v_north * self.v_north + self.v_east * self.v_east).sqrt()
                    <= MAX_PLAUSIBLE_GROUND_SPEED_MS
                && self.v_down.abs() <= MAX_PLAUSIBLE_VERTICAL_SPEED_MS;
            if !mechanization_plausible {
                // Fault: stop presenting this tick (and until realignment
                // completes) as valid data, and re-seed from truth exactly
                // like a real power-loss/re-align event (the same reset the
                // `!should_run` branch above performs) -- this unit lost its
                // free-inertial solution and must gyrocompass again, the
                // same honest consequence a real ADIRU has for an
                // unrecoverable BITE fault.
                self.running = false;
                should_run = false;
                self.pitch_deg = t.theta_xp;
                self.roll_deg = t.phi_xp;
                self.heading_deg = t.psi_xp;
                self.seconds_aligning = 0.;
                self.heading_error_deg = GYROCOMPASS_INITIAL_ERROR_DEG;
                self.lat_rad = t.lat_deg.to_radians();
                self.lon_rad = t.lon_deg.to_radians();
                self.alt_m = t.alt_m;
                self.v_north = 0.;
                self.v_east = 0.;
                self.v_down = 0.;
                self.last_p_sensed_xp = 0.;
                self.last_q_sensed_xp = 0.;
                self.last_r_sensed_xp = 0.;
                self.last_omega_body_rad_s = [0., 0., 0.];
                self.position_error_m = 0.;
                self.drift_rate_smoothed_nm_hr = 0.;
            }
        }

        // The ADR runs regardless of IR alignment (a real ADR's own ~18s
        // init timer is independent of IR gyrocompassing).
        self.update_adr(dt, t, powered);

        should_run
    }

    /// The strapdown mechanization: sensed body rates integrate attitude
    /// (aviation Euler-rate kinematics -- exact at the near-level attitudes
    /// the required tests use, and the standard small-angle-away-from-
    /// gimbal-lock approximation otherwise, see docs/physics/adirs.md),
    /// sensed specific force rotated into NED integrates velocity against
    /// gravity, Earth rate and transport rate (Groves, "Principles of GNSS,
    /// Inertial and Multisensor Integrated Navigation Systems", eq.
    /// 5.9-5.10; any strapdown INS text has the same equations), and
    /// velocity integrates position. The Schuler oscillation the brief asks
    /// to see "emerge naturally" is exactly the feedback loop in this
    /// function: a tilt or velocity error leaks gravity into the horizontal
    /// specific force, which the transport-rate term feeds back into the
    /// attitude/position estimate at the classical ~84.4 minute period.
    fn mechanize(&mut self, dt: f64, t: &TrueState) {
        // FBW's own aviation-convention body rates (adirs.rs
        // `update_attitude_values`): p = -P_xp, q = -Q_xp, r = +R_xp. Using
        // the same relationship keeps this module's attitude convention
        // identical to FBW's already-tested one. Sampled once for this dt,
        // like a real IMU's fixed output rate -- RK4 below integrates this
        // one measured rate/force more accurately over the interval, it
        // doesn't re-sample the sensor.
        // X-Plane's rates are relative to the local-level frame; a gyro
        // senses inertial rate, which adds the Earth's rotation and the nav
        // frame's transport rate over the curved Earth, in body axes (Groves
        // eq. 5.9, the same terms `state_derivative` removes again).
        let lat = t.lat_deg.to_radians();
        let r_h = EARTH_RADIUS_M + t.alt_m;
        let om_in_n = [
            EARTH_RATE_RAD_S * lat.cos() + t.v_east_ms / r_h,
            -t.v_north_ms / r_h,
            -EARTH_RATE_RAD_S * lat.sin() - t.v_east_ms * lat.tan() / r_h,
        ];
        let om_in_b = ned_to_body(t.theta_xp, t.phi_xp, t.psi_xp, om_in_n);
        let p_sensed = self.gyro[0].sense(-t.p_xp + om_in_b[0].to_degrees(), dt, &mut self.rng);
        let q_sensed = self.gyro[1].sense(-t.q_xp + om_in_b[1].to_degrees(), dt, &mut self.rng);
        let r_sensed = self.gyro[2].sense(t.r_xp + om_in_b[2].to_degrees(), dt, &mut self.rng);
        // Back to X-Plane's own P/Q/R convention, for `write` to put in the
        // same body_rotation_rate_x/y/z slots the truth passthrough used
        // (x=Q_xp, y=R_xp, z=P_xp, see AdirsSimulatorData::read).
        self.last_q_sensed_xp = -q_sensed;
        self.last_r_sensed_xp = r_sensed;
        self.last_p_sensed_xp = -p_sensed;

        // Lever arm: this unit's own IMU sits at `ADIRU_LEVER_ARM_M[number]`
        // relative to the CG the true state (`t.g_*`) is referenced to, so
        // it senses an additional rotation-induced specific force
        // (`lever_arm_specific_force`, Titterton & Weston's lever-arm
        // equation) that the other two units -- at different offsets --
        // sense differently. Angular acceleration is finite-differenced
        // from the sensed rate (no true angular acceleration is available
        // from X-Plane).
        let omega_body_rad_s =
            [p_sensed.to_radians(), q_sensed.to_radians(), r_sensed.to_radians()];
        let alpha_body_rad_s2 = [
            (omega_body_rad_s[0] - self.last_omega_body_rad_s[0]) / dt.max(1e-6),
            (omega_body_rad_s[1] - self.last_omega_body_rad_s[1]) / dt.max(1e-6),
            (omega_body_rad_s[2] - self.last_omega_body_rad_s[2]) / dt.max(1e-6),
        ];
        self.last_omega_body_rad_s = omega_body_rad_s;
        let lever = lever_arm_specific_force(
            omega_body_rad_s,
            alpha_body_rad_s2,
            ADIRU_LEVER_ARM_M[self.number - 1],
        );

        // Specific force, aviation body axes (forward, right, down). At rest
        // g_nrml reads +1 (the reaction force supporting the aircraft
        // against gravity points up), so its down-component is negative.
        // The lever-arm term is added to the *true* specific force before
        // the accelerometer error model senses it -- it is a real physical
        // input to this unit's own accelerometers, not a sensor error.
        let f_fwd_true_g = t.g_axil + lever[0] / STANDARD_GRAVITY;
        let f_right_true_g = t.g_side + lever[1] / STANDARD_GRAVITY;
        let f_down_true_g = -t.g_nrml + lever[2] / STANDARD_GRAVITY;
        let f_fwd = self.accel[0].sense(f_fwd_true_g, dt, &mut self.rng) * STANDARD_GRAVITY;
        let f_right = self.accel[1].sense(f_right_true_g, dt, &mut self.rng) * STANDARD_GRAVITY;
        let f_down = self.accel[2].sense(f_down_true_g, dt, &mut self.rng) * STANDARD_GRAVITY;

        // Fixed mounting/boresight misalignment between this unit's IMU
        // case and the airframe axes (`MOUNT_MISALIGNMENT_SIGMA_DEG`):
        // applied as a small-angle rotation of both the sensed rate and
        // specific-force vectors, after the sensor error model (linear
        // order equivalent to applying it before, for a tenths-of-a-degree
        // angle).
        let pqr_rot = rotate_small_angle(
            self.misalign_eps_rad,
            [p_sensed.to_radians(), q_sensed.to_radians(), r_sensed.to_radians()],
        );
        let pqr_sensed_deg = [pqr_rot[0].to_degrees(), pqr_rot[1].to_degrees(), pqr_rot[2].to_degrees()];
        let f_rot = rotate_small_angle(self.misalign_eps_rad, [f_fwd, f_right, f_down]);

        // RK4 strapdown integration of [pitch, roll, heading, v_n, v_e,
        // v_d, lat, lon, alt] (Groves eq. 5.9-5.10 for velocity/position,
        // Stevens & Lewis for the attitude kinematics -- see
        // `Self::state_derivative`), holding this tick's sensed rate/force
        // fixed across the 4 stages (the standard technique: a real IMU
        // measures one rate/force per output interval; RK4 integrates the
        // *known* kinematics over that interval far more accurately than
        // forward Euler). This closes the Schuler loop the same way the
        // previous Euler version did, but crucially, RK4's stability region
        // includes the small step*Schuler-frequency product this module
        // runs at (whereas forward-Euler-integrating a neutrally stable
        // Schuler oscillator diverges for *any* step size -- explicit Euler
        // always adds spurious energy). That is what makes an undamped,
        // GPS-lost free-inertial run usable at all now (see `advance`): the
        // previous Euler mechanization needed permanent GPIRS damping just
        // to stay finite.
        let s0 = [
            self.pitch_deg.to_radians(),
            self.roll_deg.to_radians(),
            self.heading_deg.to_radians(),
            self.v_north,
            self.v_east,
            self.v_down,
            self.lat_rad,
            self.lon_rad,
            self.alt_m,
        ];
        let deriv = |s: [f64; 9]| Self::state_derivative(s, pqr_sensed_deg, f_rot);
        let k1 = deriv(s0);
        let k2 = deriv(add_scaled(s0, k1, dt / 2.0));
        let k3 = deriv(add_scaled(s0, k2, dt / 2.0));
        let k4 = deriv(add_scaled(s0, k3, dt));
        let mut sf = s0;
        for i in 0..9 {
            sf[i] += (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * dt / 6.0;
        }

        self.pitch_deg = sf[0].to_degrees();
        self.roll_deg = sf[1].to_degrees();
        self.heading_deg = wrap_360(sf[2].to_degrees());
        self.v_north = sf[3];
        self.v_east = sf[4];
        self.v_down = sf[5];
        self.lat_rad = sf[6];
        self.lon_rad = sf[7];
        self.alt_m = sf[8];
    }

    /// The strapdown mechanization's continuous-time derivative, evaluated
    /// at state `s` = [pitch, roll, heading (rad), v_north, v_east, v_down
    /// (m/s), lat, lon (rad), alt (m)] for RK4's 4 stages in [`Self::mechanize`].
    /// `pqr_sensed_deg_s` (aviation body rates, deg/s) and `f_body_ms2`
    /// (aviation body specific force, m/s^2) are this tick's sensed inputs,
    /// held fixed across the stages (see `mechanize`'s doc comment).
    fn state_derivative(s: [f64; 9], pqr_sensed_deg_s: [f64; 3], f_body_ms2: [f64; 3]) -> [f64; 9] {
        let [pitch, roll, heading, v_n, v_e, v_d, lat, lon, alt] = s;
        let (pitch_deg, roll_deg, heading_deg) = (pitch.to_degrees(), roll.to_degrees(), heading.to_degrees());

        let r_h = EARTH_RADIUS_M + alt;
        let om_ie_n = [EARTH_RATE_RAD_S * lat.cos(), 0.0, -EARTH_RATE_RAD_S * lat.sin()];
        let om_en_n = [v_e / r_h, -v_n / r_h, -v_e * lat.tan() / r_h];

        // Attitude integrates the body rate *relative to the local-level
        // nav frame*, i.e. the sensed (inertial) rate minus the nav frame's
        // own rotation (Earth rate + transport rate) expressed in body axes
        // -- Groves eq. 5.9's attitude update. This is the term that closes
        // the Schuler loop: without it a tilt/velocity error has no path
        // back into the attitude estimate. (Both terms are now consistently
        // rad/s before being differenced -- the previous Euler version
        // mixed `pqr_sensed` in deg/s with `om_in_b` in rad/s here, an
        // error too small to fail any bounded-drift test but fixed as part
        // of this rewrite.)
        let om_in_b = ned_to_body(
            pitch_deg,
            roll_deg,
            heading_deg,
            [om_ie_n[0] + om_en_n[0], om_ie_n[1] + om_en_n[1], om_ie_n[2] + om_en_n[2]],
        );
        let pqr_sensed_rad = [
            pqr_sensed_deg_s[0].to_radians(),
            pqr_sensed_deg_s[1].to_radians(),
            pqr_sensed_deg_s[2].to_radians(),
        ];
        let (p, q, r) = (
            pqr_sensed_rad[0] - om_in_b[0],
            pqr_sensed_rad[1] - om_in_b[1],
            pqr_sensed_rad[2] - om_in_b[2],
        );
        let theta_dot = q * roll.cos() - r * roll.sin();
        let phi_dot = p + (q * roll.sin() + r * roll.cos()) * pitch.tan();
        let psi_dot = (q * roll.sin() + r * roll.cos()) / pitch.cos().max(0.05);

        let f_n = body_to_ned(pitch_deg, roll_deg, heading_deg, f_body_ms2);
        let om = [
            2.0 * om_ie_n[0] + om_en_n[0],
            2.0 * om_ie_n[1] + om_en_n[1],
            2.0 * om_ie_n[2] + om_en_n[2],
        ];
        let coriolis = cross(om, [v_n, v_e, v_d]);
        let g_n = normal_gravity(lat, alt);

        [
            theta_dot,
            phi_dot,
            psi_dot,
            f_n[0] - coriolis[0],
            f_n[1] - coriolis[1],
            f_n[2] - coriolis[2] + g_n,
            v_n / r_h,
            v_e / (r_h * lat.cos().max(1e-6)),
            -v_d,
        ]
    }

    /// GPIRS: a simple complementary filter nudging the free-inertial
    /// position *and velocity* toward GPS truth (Airbus/Honeywell public
    /// GPIRS descriptions: a "hybrid GPS/inertial position" --
    /// docs/physics/adirs.md has the citations; a real GPS receiver gives a
    /// velocity solution too, and a real GPIRS filter uses it). `gps` here
    /// is X-Plane's own true position with a small simulated receiver
    /// noise; there is no separate GPS receiver/multipath model in scope.
    ///
    /// The velocity term is also what keeps the undamped Schuler loop
    /// numerically well-behaved: a textbook undamped Schuler oscillator
    /// (Wikipedia, "Schuler tuning") is only neutrally stable in continuous
    /// time, and forward-Euler-integrating a neutrally stable oscillator is
    /// a classic numerical-analysis trap -- explicit Euler adds spurious
    /// energy every step, so the discretized oscillation grows without
    /// bound regardless of step size (confirmed empirically while building
    /// this module: a pure free-inertial run diverges within about one
    /// Schuler period). Real ADIRUs avoid the pure undamped case entirely
    /// (GPIRS aiding, as here, or a deliberately damped "third order"
    /// vertical/horizontal loop); this module always runs damped for that
    /// reason -- see docs/physics/adirs.md and the report for the free-
    /// inertial (GPS-lost) case's status as a known follow-on.
    /// Baro-inertial vertical channel: a pure inertial altitude diverges
    /// exponentially (the gravity gradient, `normal_gravity`'s 3.086e-6 s^-2
    /// per metre, gives a time constant of about 570 s; Titterton & Weston
    /// ch. 3.6), so inertial systems blend in barometric altitude with a
    /// second-order loop (Widnall & Sinha, "Optimizing the gains of the
    /// baro-inertial vertical channel", 1980). Baro altitude here is the
    /// ADR's pressure altitude, which the atmosphere model makes equal to the
    /// true altitude at the airframe.
    fn baro_inertial_correct(&mut self, dt: f64, t: &TrueState) {
        // A real ADIRU's baro-altitude monitor rejects an implausible
        // altitude disagreement rather than blindly nulling it into
        // vertical speed -- the airframe cannot really be tens of
        // thousands of kilometres from its own last inertial fix a tick
        // later, so that large an `error` is bad/not-yet-valid position
        // data (see `TrueStateSource::read`'s `agl_looks_placed` guard, and
        // `start_state.rs`'s identical "before X-Plane has placed the
        // aircraft its position reads garbage" note), not a real
        // disturbance to correct for. Clamping `error` itself -- not just
        // the loop's gain -- is what keeps *both* the position and
        // velocity feedback terms below consistent, bounded-per-tick
        // corrections regardless of how far off an upstream bad reading
        // might be; `MAX_PLAUSIBLE_ALT_JUMP_M` is set far above any real
        // altitude discontinuity (teleporting to a different airport,
        // Dead Sea to Everest) and far below X-Plane's "millions of feet"
        // pre-placement garbage.
        const MAX_PLAUSIBLE_ALT_JUMP_M: f64 = 20_000.0;
        let error = (t.alt_m - self.alt_m).clamp(-MAX_PLAUSIBLE_ALT_JUMP_M, MAX_PLAUSIBLE_ALT_JUMP_M);
        // Critically damped second-order loop, 100 s natural period:
        // position feedback 2*w*error, velocity feedback w*w*error. Both
        // gains are additionally capped at "fully trust baro this tick"
        // (matching the position term's existing `.min(1.0)`) so a long
        // frame (a sim pause/resume) can't overshoot either term past what
        // the loop's own bandwidth allows -- previously only the position
        // term had this cap, leaving the velocity term (what the audit
        // found: `self.v_down -= (w*w*dt)*error` with no clamp at all) free
        // to blow up on exactly the same bad input.
        let w = 2.0 * std::f64::consts::PI / BARO_LOOP_PERIOD_S;
        self.alt_m += (2.0 * w * dt).min(1.0) * error;
        self.v_down -= (w * w * dt).min(w) * error;
    }

    fn gpirs_correct(&mut self, dt: f64, t: &TrueState) {
        let gps_noise_m = 3.0; // typical civil GPS 1-sigma horizontal error
        let gps_lat = t.lat_deg.to_radians() + self.rng.gaussian() * gps_noise_m / EARTH_RADIUS_M;
        let gps_lon = t.lon_deg.to_radians()
            + self.rng.gaussian() * gps_noise_m / (EARTH_RADIUS_M * self.lat_rad.cos().max(1e-6));

        let k = (dt / GPIRS_TIME_CONSTANT_S).min(1.0);
        self.lat_rad += k * (gps_lat - self.lat_rad);
        self.lon_rad += k * (gps_lon - self.lon_rad);
        self.v_north *= 1.0 - k;
        self.v_east *= 1.0 - k;

        let north_error_m = (t.lat_deg.to_radians() - self.lat_rad) * EARTH_RADIUS_M;
        let east_error_m =
            (t.lon_deg.to_radians() - self.lon_rad) * EARTH_RADIUS_M * self.lat_rad.cos();
        self.position_error_m = (north_error_m * north_error_m + east_error_m * east_error_m).sqrt();

        // Smoothed drift-rate estimate for the Study panel: an exponential
        // average of "position error / time since it would have been zero",
        // using the GPIRS time constant as that reference time.
        let alpha = (dt / 60.0).min(1.0);
        let instantaneous_nm_hr = (self.position_error_m / NM_TO_M) * (3600.0 / GPIRS_TIME_CONSTANT_S);
        self.drift_rate_smoothed_nm_hr =
            self.drift_rate_smoothed_nm_hr * (1.0 - alpha) + instantaneous_nm_hr * alpha;
    }

    /// Disables GPIRS blending, for the free-inertial drift/Schuler tests
    /// (a GPS-lost failure case in reality; this module has no separate
    /// "GPS valid" discrete to drive it from yet -- see the report).
    #[cfg(test)]
    fn set_free_inertial_for_test(&mut self) {
        self.running = true;
    }

    /// Pitot-static ADR: total/static pressure at the probe (AoA-driven
    /// position error, icing/heating), inverted for CAS/Mach/TAS, plus a
    /// recovery-factor TAT and an AoA-vane local flow angle.
    /// docs/physics/adirs.md has the equations and every source.
    fn update_adr(&mut self, dt: f64, t: &TrueState, powered: bool) {
        // Real Airbus AUTO probe-heat logic (published across the A320/
        // A330/A380 FCOM/FCTM family): the probe heat computer energizes
        // the pitot/static/AOA/TAT heaters automatically whenever the
        // aircraft is airborne OR at least one engine is running,
        // independent of detected icing -- not a temperature-triggered
        // switch, and not simply "whenever the bus has power" (the previous
        // placeholder here). This module has no separate manual OFF
        // selection (see the report).
        // failures::extra 30_000+(number-1): this unit's probe heater
        // element power loss, as a *continuous* fraction
        // (`failures::magnitude`, 0..1 -- 1.0 is the old open-circuit case,
        // and also what `magnitude()` itself returns for a plain
        // `set_active`/binary activation, so this is a strict
        // generalisation of the old open-circuit test, not a behaviour
        // change for it). The AUTO logic still decides whether the heater
        // is *commanded* on; the magnitude only takes away rated watts from
        // whatever it would otherwise deliver.
        let heater_power_loss_frac = crate::failures::magnitude(30_000 + self.number as u64 - 1);
        let heat_on = powered && (!t.on_ground || t.any_engine_running);
        let rated_probe_heater_w = if heat_on { PITOT_HEATER_W * (1.0 - heater_power_loss_frac) } else { 0.0 };

        // True TAS from the truth state (t.mach/t.sat_c), not this ADR's
        // own sensed/frozen output -- the heat balance driving whether the
        // probe ices must not depend on the icing-affected reading it also
        // produces (that would be circular).
        let true_speed_of_sound_ms = (GAMMA_AIR * R_AIR * (t.sat_c + 273.15)).sqrt();
        let true_tas_ms = t.mach * true_speed_of_sound_ms;

        // Continuous Messinger-style heat balance (see `probe_heat_required_w`):
        // ice accretes whenever the heater's remaining power can't cover
        // dry-air convection plus warming/keeping-liquid the water the
        // probe actually catches; any surplus heat instead melts previously
        // accreted ice. `delta_t` inside `probe_heat_required_w` is itself
        // 0 above freezing, so this is emergent -- no separate "icing
        // conditions" gate: SAT, TAS, LWC and heater power all flow through
        // the one relation, and any single one at its benign value drives
        // `deficit_w` to 0 on its own.
        let (required_w, catch_kg_s) = probe_heat_required_w(t.sat_c, true_tas_ms, t.lwc_gm3);
        let deficit_w = (required_w - rated_probe_heater_w).max(0.0);
        let surplus_w = (rated_probe_heater_w - required_w).max(0.0);
        let accretion_kg_s = if required_w > 0.0 { catch_kg_s * (deficit_w / required_w).min(1.0) } else { 0.0 };
        let melt_kg_s = surplus_w / WATER_LF_J_KG;
        let net_kg_s = accretion_kg_s - melt_kg_s;
        // A physical floor on an internal mass accumulator (can't hold
        // negative ice), not a silent override of a bad computed input --
        // the invariants substrate (src/invariants.rs) guards the shared
        // `Vars` quantities this struct later writes out, not this
        // struct's own intermediate state.
        self.pitot_ice_kg = (self.pitot_ice_kg + net_kg_s * dt).max(0.0);
        self.static_ice_kg = (self.static_ice_kg + net_kg_s * dt).max(0.0);
        let pitot_blocked = self.pitot_ice_kg >= PROBE_BLOCK_MASS_KG;
        let static_blocked = self.static_ice_kg >= PROBE_BLOCK_MASS_KG;

        // Impact pressure from the true compressible flow (standard
        // subsonic pitot-static relation): qc = Ps * ((1 + 0.2 M^2)^3.5 - 1).
        let qc_true = t.static_pressure_pa * ((1.0 + 0.2 * t.mach * t.mach).powf(3.5) - 1.0);
        let true_total_pressure_pa = t.static_pressure_pa + qc_true;

        // Static source position error: a small, AoA-driven pressure error
        // at the static ports, generic-transport-derived (not A380-specific
        // -- docs/physics/adirs.md). Positive AoA above the 3 degree
        // reference reads slightly high pressure (indicating low altitude)
        // at the ports, a typical underwing-static characteristic.
        let position_error_pa = qc_true * 0.003 * (t.alpha_deg - 3.0).clamp(-10.0, 15.0);
        let sensed_static_pa = t.static_pressure_pa + position_error_pa;
        let sensed_total_pa = true_total_pressure_pa;

        if !static_blocked || self.frozen_static_pressure_pa == 0.0 {
            self.frozen_static_pressure_pa = sensed_static_pa;
        }
        if !pitot_blocked || self.frozen_total_pressure_pa == 0.0 {
            self.frozen_total_pressure_pa = sensed_total_pa;
        }

        let static_pa = self.frozen_static_pressure_pa;
        let total_pa = self.frozen_total_pressure_pa;
        let qc = (total_pa - static_pa).max(0.0);

        // Standard compressible airspeed/Mach-from-impact-pressure
        // equations (ICAO/FAA air data computer formulae):
        // CAS = a0 * sqrt(5 * ((qc/P0 + 1)^(2/7) - 1))
        // M   = sqrt(5 * ((qc/Ps + 1)^(2/7) - 1))
        let cas_kt = A0_KT * (5.0 * ((qc / P0_PA + 1.0).powf(2.0 / 7.0) - 1.0)).max(0.0).sqrt();
        let mach_sensed =
            (5.0 * ((qc / static_pa.max(1.0) + 1.0).powf(2.0 / 7.0) - 1.0)).max(0.0).sqrt();

        // TAT from the recovery factor: Tt = Ts * (1 + r*(gamma-1)/2*M^2).
        let sat_k = t.sat_c + 273.15;
        let tat_k =
            sat_k * (1.0 + TAT_RECOVERY_FACTOR * (GAMMA_AIR - 1.0) / 2.0 * mach_sensed * mach_sensed);

        let speed_of_sound_ms = (GAMMA_AIR * R_AIR * sat_k).sqrt();

        self.adr_pitot_blocked = pitot_blocked;
        self.adr_static_blocked = static_blocked;
        // Actual delivered watts (degraded by the same continuous fraction
        // the heat balance above used), not the nameplate rating -- this is
        // the same shared-contract quantity the electrical workstream reads
        // as a bus consumer, so a degraded heater must show up there too.
        self.adr_probe_heat_w = if heat_on { 2.0 * rated_probe_heater_w + TAT_HEATER_W } else { 0.0 };
        self.adr_static_pa = static_pa;
        self.adr_total_pa = total_pa;
        self.adr_cas_kt = cas_kt;
        self.adr_mach = mach_sensed;
        self.adr_tas_ms = mach_sensed * speed_of_sound_ms;
        self.adr_tat_c = tat_k - 273.15;
        self.adr_aoa_deg = t.alpha_deg * AOA_VANE_UPWASH_FACTOR;
    }

    fn write(&self, vars: &mut Vars, valid: bool) {
        let w = |vars: &mut Vars, id: &VariableIdentifier, value: f64| {
            SimulatorReaderWriter::write(vars, id, value);
        };
        w(vars, &self.v.valid, if valid { 1. } else { 0. });
        // MSFS/lib.rs convention: pitch/roll negated from the aviation
        // (X-Plane theta/phi) convention this module works in internally,
        // matching lib.rs's own "PLANE PITCH DEGREES"/"PLANE BANK DEGREES"
        // mapping (`|v| -v`) that AdirsSimulatorData otherwise reads.
        w(vars, &self.v.pitch, -self.pitch_deg);
        w(vars, &self.v.roll, -self.roll_deg);
        w(vars, &self.v.true_heading, self.heading_deg);
        let track_deg = wrap_360(self.v_east.atan2(self.v_north).to_degrees());
        w(vars, &self.v.true_track, track_deg);
        // Sensed body rates in the same slots/convention as the truth
        // passthrough they replace (body_rotation_rate_x/y/z = Q_xp/R_xp/
        // P_xp, deg/s), so FBW's own downstream sign-flips are unaffected.
        w(vars, &self.v.body_rotation_rate_x, self.last_q_sensed_xp);
        w(vars, &self.v.body_rotation_rate_y, self.last_r_sensed_xp);
        w(vars, &self.v.body_rotation_rate_z, self.last_p_sensed_xp);
        w(vars, &self.v.latitude, self.lat_rad.to_degrees());
        w(vars, &self.v.longitude, self.lon_rad.to_degrees());
        let ground_speed_kt =
            (self.v_north * self.v_north + self.v_east * self.v_east).sqrt() * MS_TO_KNOT;
        w(vars, &self.v.ground_speed, ground_speed_kt);
        w(vars, &self.v.vertical_speed, -self.v_down * M_TO_FT * 60.0);
        w(vars, &self.v.true_airspeed, self.adr_tas_ms * MS_TO_KNOT);
        w(vars, &self.v.mach, self.adr_mach);
        w(vars, &self.v.total_air_temperature, self.adr_tat_c);
        w(vars, &self.v.angle_of_attack, self.adr_aoa_deg);
        w(vars, &self.v.static_pressure, self.adr_static_pa / 100.0); // Pa -> hPa
        w(vars, &self.v.computed_airspeed, self.adr_cas_kt);

        // Study panel (always written, valid or not, so the panel can show
        // "Off"/"Aligning" states too).
        w(vars, &self.v.study_align_state, self.last_fbw_state);
        w(vars, &self.v.study_position_error_nm, self.position_error_m / NM_TO_M);
        w(vars, &self.v.study_drift_nm_hr, self.drift_rate_smoothed_nm_hr);
        w(vars, &self.v.study_gyro_bias_deg_hr, self.gyro[2].bias_deg_s * 3600.0);
        w(vars, &self.v.study_accel_bias_ug, self.accel[2].bias_g * 1e6);
        w(vars, &self.v.study_pitot_blocked, if self.adr_pitot_blocked { 1. } else { 0. });
        w(vars, &self.v.study_static_blocked, if self.adr_static_blocked { 1. } else { 0. });
        w(vars, &self.v.study_probe_heat_w, self.adr_probe_heat_w);
        // Shared contract (docs/briefs/hyperrealism.md): the electrical
        // workstream reads this as a bus consumer, per ADIRU/probe channel.
        w(vars, &self.v.probe_heat_load_w, self.adr_probe_heat_w);
        w(vars, &self.v.study_static_pa, self.adr_static_pa);
        w(vars, &self.v.study_total_pa, self.adr_total_pa);
    }
}

// ---------------------------------------------------------------------
// The radio altimeters' terrain-boresight range (docs/physics/adirs.md).
// ---------------------------------------------------------------------

/// Probes X-Plane's terrain under the aircraft for the radio altimeters'
/// `Ala52BTransceiverPair` (FBW's own reflection-geometry model, unmodified;
/// see `patches/fbw-rust/navigation-sensors.patch`), in place of X-Plane's
/// own single, generally-straight-down `PLANE ALT ABOVE GROUND`. All 3 RAs
/// share this one probe (their antennas are only metres apart on the same
/// airframe; FBW's geometry already applies each antenna's own installation
/// offset on top of this shared ground clearance, matching how the original
/// `PLANE ALT ABOVE GROUND` reading was shared too).
struct RadioAltimeterProbe {
    ticks_since_probe: u32,
    agl_ft: f64,
    valid: bool,
    valid_var: VariableIdentifier,
    agl_var: VariableIdentifier,
}
impl RadioAltimeterProbe {
    /// X-Plane's terrain-probe API does a scenery raycast; like `src/wxr`'s
    /// own note about the weather API, this is not meant to be hammered
    /// every frame. Sampling every 8th tick (several times a second at
    /// typical sim rates) is far more than the radio altimeter's own
    /// dynamics need and keeps the raycast off most frames entirely.
    const PROBE_EVERY_TICKS: u32 = 8;

    fn new(vars: &mut Vars) -> Self {
        Self {
            ticks_since_probe: 0,
            agl_ft: 0.,
            valid: false,
            valid_var: vars.get("RA_TERRAIN_PROBE_VALID".to_string()),
            agl_var: vars.get("RA_TERRAIN_PROBE_ALT_ABOVE_GROUND".to_string()),
        }
    }

    fn update(&mut self, vars: &mut Vars, xplm: &Xplm) {
        self.ticks_since_probe += 1;
        if self.ticks_since_probe >= Self::PROBE_EVERY_TICKS {
            self.ticks_since_probe = 0;
            let local = ["sim/flightmodel/position/local_x", "sim/flightmodel/position/local_y", "sim/flightmodel/position/local_z"]
                .map(|n| xplm.find(n));
            if let [Some(x), Some(y), Some(z)] = local {
                let (lx, ly, lz) = (xplm.get_d(x), xplm.get_d(y), xplm.get_d(z));
                if let Some(terrain_y) = crate::xp::probe_terrain_y(lx, ly, lz) {
                    self.agl_ft = (ly - terrain_y).max(0.0) * M_TO_FT;
                    self.valid = true;
                }
            }
        }
        SimulatorReaderWriter::write(vars, &self.agl_var, self.agl_ft);
        SimulatorReaderWriter::write(vars, &self.valid_var, if self.valid { 1. } else { 0. });
    }
}

// ---------------------------------------------------------------------
// Top-level: reads X-Plane's truth once per tick, updates the 3 ADIRUs and
// the radio altimeter probe.
// ---------------------------------------------------------------------

/// Whether an AGL reading (feet) looks like a real placement rather than
/// X-Plane's pre-placement garbage. Mirrors `start_state.rs::read_situation`'s
/// identical guard byte-for-byte ("Before X-Plane has placed the aircraft
/// its position reads garbage (millions of feet below ground); treat that
/// as parked") -- pulled out to a free function so it's unit-testable
/// without a live X-Plane.
fn agl_looks_placed(agl_ft: f64) -> bool {
    agl_ft.is_finite() && (-1_000.0..=100_000.0).contains(&agl_ft)
}

/// Rejects a raw X-Plane reading that is non-finite or outside a generous
/// physical plausibility bound, holding `*last` (the previous *plausible*
/// reading, not a synthetic replacement) instead -- the same "reject a bad
/// sample, don't integrate it" principle [`agl_looks_placed`] already
/// applies to position/altitude, extended to every other truth channel
/// [`Adiru::mechanize`] feeds straight into the strapdown integration every
/// tick with no prior guard at all (see the report's live-defect analysis:
/// `sim/flightmodel/position/local_vx`/`local_vz` -- X-Plane's own
/// finite-differenced velocity -- can read a one-tick spike across an
/// aircraft placement/teleport event, and nothing between that reading and
/// `mechanize`'s Earth-rate/transport-rate term
/// (`t.v_north_ms`/`t.v_east_ms` in `om_in_n`) checked it was even finite,
/// let alone plausible; one such tick injects an arbitrarily large synthetic
/// body rate into every ADIRU's gyrocompassing in the same tick, since all 3
/// share this one `TrueState` -- matching the live symptom of near-identical
/// runaway values on all 3 IRs at once). This holds the last *plausible*
/// sample rather than fabricating a value for a rejected one -- a real
/// ADIRU's own input monitor does the same (reject/hold a glitched sample,
/// don't invent a replacement).
fn plausible(last: &mut f64, val: f64, max_abs: f64) -> f64 {
    if val.is_finite() && val.abs() <= max_abs {
        *last = val;
    }
    *last
}

/// Range form of [`plausible`], for the air-data channels whose implausible
/// values are not symmetric about zero, so a magnitude bound cannot express
/// them. [`Adiru::update_adr`] consumed all four of these straight off the
/// dataref with no guard, and two of the paths are worse than a transient:
///
/// * Static air temperature below absolute zero makes `sat_k` negative, so
///   `(GAMMA_AIR * R_AIR * sat_k).sqrt()` is NaN, and that NaN is published
///   as true airspeed (`adr_tas_ms`) and total air temperature.
/// * A non-finite Mach or static pressure makes `qc_true` NaN, which lands
///   in `frozen_static_pressure_pa`. That field is *persistent*, and its
///   "not yet frozen" sentinel is `== 0.0`, which NaN fails -- so while a
///   probe is iced over, one bad tick poisons static pressure, CAS and Mach
///   for the rest of the session, with no path back even once the sim's own
///   reading recovers.
///
/// The bounds are garbage-rejection bounds, deliberately far outside
/// anything the atmosphere does (the coldest temperature ever measured in
/// the atmosphere is about -143 C, at the mesopause, well above this
/// module's 100,000 ft placement ceiling; the highest sea-level pressure on
/// record is about 108.5 kPa). They are not a flight envelope and must never
/// shape a real reading -- same intent as [`plausible`]'s ceilings.
fn plausible_in(last: &mut f64, val: f64, lo: f64, hi: f64) -> f64 {
    if val.is_finite() && (lo..=hi).contains(&val) {
        *last = val;
    }
    *last
}

/// The datarefs read once per tick to build [`TrueState`]. Looked up once at
/// construction, like the rest of the plugin's per-module dataref caches
/// (e.g. `sensors.rs`).
struct TrueStateSource {
    p: Option<crate::xp::DataRef>,
    q: Option<crate::xp::DataRef>,
    r: Option<crate::xp::DataRef>,
    g_axil: Option<crate::xp::DataRef>,
    g_side: Option<crate::xp::DataRef>,
    g_nrml: Option<crate::xp::DataRef>,
    theta: Option<crate::xp::DataRef>,
    phi: Option<crate::xp::DataRef>,
    psi: Option<crate::xp::DataRef>,
    latitude: Option<crate::xp::DataRef>,
    longitude: Option<crate::xp::DataRef>,
    elevation: Option<crate::xp::DataRef>,
    y_agl: Option<crate::xp::DataRef>,
    static_pressure_pa: Option<crate::xp::DataRef>,
    sat_c: Option<crate::xp::DataRef>,
    mach: Option<crate::xp::DataRef>,
    alpha: Option<crate::xp::DataRef>,
    on_ground: Option<crate::xp::DataRef>,
    engine_running: Option<crate::xp::DataRef>,
    precip_on_aircraft_ratio: Option<crate::xp::DataRef>,
    local_vx: Option<crate::xp::DataRef>,
    local_vz: Option<crate::xp::DataRef>,
    /// The last position fix (lat/lon/alt, and north/east velocity) that
    /// passed [`agl_looks_placed`]. Held and reused whenever this tick's
    /// reading doesn't, instead of ever handing the ADIRUs' baro-inertial
    /// loop and Earth-rate/transport-rate terms X-Plane's pre-placement
    /// "millions of feet below ground" garbage (see [`Self::read`]) -- that
    /// garbage `alt_m` is exactly what drove both the unclamped V/S spike
    /// (`Adiru::baro_inertial_correct`) and the one-tick pitch/roll
    /// corruption (`r_h`/transport-rate blowup in `Adiru::mechanize`) this
    /// guard fixes at the source. Zero until the first placed reading, same
    /// as a freshly constructed [`Adiru`]'s own state.
    last_valid_lat_deg: f64,
    last_valid_lon_deg: f64,
    last_valid_alt_m: f64,
    last_valid_v_north_ms: f64,
    last_valid_v_east_ms: f64,
    /// Last *plausible* (finite, within [`Self::read`]'s generous bounds)
    /// reading of each channel `mechanize`/`update_adr` consume directly,
    /// for [`plausible`] to hold on a rejected sample. See [`plausible`]'s
    /// doc comment for why these had no guard at all before.
    last_p_deg_s: f64,
    last_q_deg_s: f64,
    last_r_deg_s: f64,
    last_g_axil: f64,
    last_g_side: f64,
    last_g_nrml: f64,
    last_theta_deg: f64,
    last_phi_deg: f64,
    last_psi_deg: f64,
    /// Same, for the air-data channels [`plausible_in`] guards. Seeded to
    /// ISA sea level so that a missing dataref (`find` returned `None`, so
    /// every read is a flat 0.0) holds a breathable atmosphere rather than
    /// the vacuum a raw 0 Pa would assert.
    last_static_pa: f64,
    last_sat_c: f64,
    last_mach: f64,
    last_alpha_deg: f64,
}
impl TrueStateSource {
    fn new(xplm: &Xplm) -> Self {
        Self {
            local_vx: xplm.find("sim/flightmodel/position/local_vx"),
            local_vz: xplm.find("sim/flightmodel/position/local_vz"),
            p: xplm.find("sim/flightmodel/position/P"),
            q: xplm.find("sim/flightmodel/position/Q"),
            r: xplm.find("sim/flightmodel/position/R"),
            g_axil: xplm.find("sim/flightmodel/forces/g_axil"),
            g_side: xplm.find("sim/flightmodel/forces/g_side"),
            g_nrml: xplm.find("sim/flightmodel/forces/g_nrml"),
            theta: xplm.find("sim/flightmodel/position/theta"),
            phi: xplm.find("sim/flightmodel/position/phi"),
            psi: xplm.find("sim/flightmodel/position/psi"),
            latitude: xplm.find("sim/flightmodel/position/latitude"),
            longitude: xplm.find("sim/flightmodel/position/longitude"),
            elevation: xplm.find("sim/flightmodel/position/elevation"),
            y_agl: xplm.find("sim/flightmodel/position/y_agl"),
            static_pressure_pa: xplm.find("sim/weather/aircraft/barometer_current_pas"),
            sat_c: xplm.find("sim/weather/aircraft/temperature_ambient_deg_c"),
            mach: xplm.find("sim/flightmodel/misc/machno"),
            alpha: xplm.find("sim/flightmodel/position/alpha"),
            on_ground: xplm.find("sim/flightmodel/failures/onground_any"),
            engine_running: xplm.find("sim/flightmodel/engine/ENGN_running"),
            precip_on_aircraft_ratio: xplm.find("sim/weather/aircraft/precipitation_on_aircraft_ratio"),
            last_valid_lat_deg: 0.,
            last_valid_lon_deg: 0.,
            last_valid_alt_m: 0.,
            last_valid_v_north_ms: 0.,
            last_valid_v_east_ms: 0.,
            last_p_deg_s: 0.,
            last_q_deg_s: 0.,
            last_r_deg_s: 0.,
            last_g_axil: 0.,
            last_g_side: 0.,
            last_g_nrml: -1., // at rest, level: reaction force supports 1g up.
            last_theta_deg: 0.,
            last_phi_deg: 0.,
            last_psi_deg: 0.,
            last_static_pa: 101_325., // ISA sea level.
            last_sat_c: 15.,          // ISA sea level.
            last_mach: 0.,
            last_alpha_deg: 0.,
        }
    }

    fn read(&mut self, xplm: &Xplm) -> TrueState {
        let f = |d: Option<crate::xp::DataRef>| d.map_or(0., |d| xplm.get_f(d) as f64);
        let d = |d: Option<crate::xp::DataRef>| d.map_or(0., |d| xplm.get_d(d));

        let agl_ft = f(self.y_agl) * M_TO_FT;
        let placed = agl_looks_placed(agl_ft);
        // Generous plausibility ceilings for X-Plane's own truth channels
        // (see [`plausible`]): body rates in deg/s, specific force in g,
        // Euler angles in degrees. None of these bound anything a real
        // aircraft (let alone a parked or cruising A380) can actually do --
        // they exist only to reject a garbage/glitched instantaneous sample
        // (e.g. a placement-teleport velocity spike propagating into
        // `om_in_n`'s body-rate term below), never to shape real flight
        // data.
        let v_north_raw_ms = -f(self.local_vz);
        let v_east_raw_ms = f(self.local_vx);
        let v_plausible = placed
            && v_north_raw_ms.is_finite()
            && v_east_raw_ms.is_finite()
            && v_north_raw_ms.hypot(v_east_raw_ms) <= 600.0; // ~1166 kt
        if v_plausible {
            // OpenGL local frame: +x east, +z south.
            self.last_valid_lat_deg = d(self.latitude);
            self.last_valid_lon_deg = d(self.longitude);
            self.last_valid_alt_m = f(self.elevation);
            self.last_valid_v_north_ms = v_north_raw_ms;
            self.last_valid_v_east_ms = v_east_raw_ms;
        }
        let (lat_deg, lon_deg, alt_m, v_north_ms, v_east_ms) = (
            self.last_valid_lat_deg,
            self.last_valid_lon_deg,
            self.last_valid_alt_m,
            self.last_valid_v_north_ms,
            self.last_valid_v_east_ms,
        );

        TrueState {
            p_xp: plausible(&mut self.last_p_deg_s, f(self.p), 1000.0),
            q_xp: plausible(&mut self.last_q_deg_s, f(self.q), 1000.0),
            r_xp: plausible(&mut self.last_r_deg_s, f(self.r), 1000.0),
            g_axil: plausible(&mut self.last_g_axil, f(self.g_axil), 20.0),
            g_side: plausible(&mut self.last_g_side, f(self.g_side), 20.0),
            g_nrml: plausible(&mut self.last_g_nrml, f(self.g_nrml), 20.0),
            theta_xp: plausible(&mut self.last_theta_deg, f(self.theta), 180.0),
            phi_xp: plausible(&mut self.last_phi_deg, f(self.phi), 180.0),
            psi_xp: plausible(&mut self.last_psi_deg, f(self.psi), 360.0),
            lat_deg,
            lon_deg,
            alt_m,
            // Absolute pressure has no negative half, and reaching exactly
            // 0.0 would also defeat `update_adr`'s frozen-static sentinel.
            static_pressure_pa: plausible_in(
                &mut self.last_static_pa,
                f(self.static_pressure_pa),
                100.,
                120_000.,
            ),
            sat_c: plausible_in(&mut self.last_sat_c, f(self.sat_c), -200., 100.),
            mach: plausible_in(&mut self.last_mach, f(self.mach), 0., 6.),
            alpha_deg: plausible_in(&mut self.last_alpha_deg, f(self.alpha), -180., 180.),
            // Not placed yet is parked, same call `start_state.rs` makes.
            on_ground: !placed || f(self.on_ground) != 0.,
            placed,
            v_north_ms,
            v_east_ms,
            any_engine_running: self.engine_running.is_some_and(|dr| {
                let mut running = [0i32; 4];
                xplm.get_vi(dr, &mut running);
                running.iter().any(|&r| r != 0)
            }),
            lwc_gm3: f(self.precip_on_aircraft_ratio) * REFERENCE_LWC_GM3,
        }
    }
}

/// Owns all 3 ADIRUs' navigation-sensor physics and the radio altimeters'
/// terrain probe; `lib.rs` calls [`Self::update`] once per tick, before the
/// systems tick reads `ADIRS_SENSED_<n>_*`.
pub struct AdirsPhysics {
    source: TrueStateSource,
    adirus: [Adiru; 3],
    radio_altimeter: RadioAltimeterProbe,
    state_vars: [VariableIdentifier; 3],
    /// Electrical bus feeding each ADIRU. The real A380's ADIRU-to-bus
    /// assignment is not published anywhere in scope; this assigns ADIRU 1
    /// and 3 to the essential bus (the usual Airbus pattern of keeping the
    /// primary and backup/standby unit essential-powered) and ADIRU 2 to a
    /// main bus, a documented assumption (see the report), using the same
    /// `A32NX_ELEC_<bus>_BUS_IS_POWERED` variables `circuits.rs`/`fuel.rs`
    /// already read for other systems.
    power_vars: [VariableIdentifier; 3],
    /// FBW's own IR mode selector knob (`OVHD_ADIRS_IR_<n>_MODE_SELECTOR_KNOB`,
    /// adirs.rs's `InertialReferenceModeSelector`/`InertialReferenceMode`:
    /// 0 Off, 1 Navigation, 2 Attitude) -- read here so this module can drop
    /// GPIRS aiding in Attitude mode (see [`Adiru::advance`]).
    mode_selector_vars: [VariableIdentifier; 3],
    /// X-Plane's own GPS receiver failure discretes (`rel_gps1`/`rel_gps2`;
    /// 0 = working, nonzero = some failure severity active). The A380 has 2
    /// MMRs feeding all 3 ADIRUs; this maps ADIRU 2 to the second receiver
    /// and ADIRUs 1 and 3 to the first, a documented assumption (no public
    /// A380 MMR-to-ADIRU cross-feed diagram is in scope) so that a single
    /// receiver failure degrades, rather than blinds, the fleet.
    gps_failed_drefs: [Option<crate::xp::DataRef>; 2],
}
impl AdirsPhysics {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        Self {
            source: TrueStateSource::new(xplm),
            adirus: [1, 2, 3].map(|n| Adiru::new(vars, n)),
            radio_altimeter: RadioAltimeterProbe::new(vars),
            state_vars: [1, 2, 3].map(|n| vars.get(format!("ADIRS_ADIRU_{n}_STATE"))),
            power_vars: ["A32NX_ELEC_DC_ESS_BUS_IS_POWERED", "A32NX_ELEC_DC_2_BUS_IS_POWERED", "A32NX_ELEC_DC_ESS_BUS_IS_POWERED"]
                .map(|n| vars.get_unprefixed(n.to_string())),
            mode_selector_vars: [1, 2, 3]
                .map(|n| vars.get(format!("OVHD_ADIRS_IR_{n}_MODE_SELECTOR_KNOB"))),
            gps_failed_drefs: [
                xplm.find("sim/operation/failures/rel_gps1"),
                xplm.find("sim/operation/failures/rel_gps2"),
            ],
        }
    }

    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm, dt: f64, _time: f64) {
        let true_state = self.source.read(xplm);
        let gps_failed =
            self.gps_failed_drefs.map(|d| d.is_some_and(|dr| xplm.get_f(dr) > 0.5));
        for i in 0..3 {
            let fbw_state = SimulatorReaderWriter::read(vars, &self.state_vars[i]);
            let powered = SimulatorReaderWriter::read(vars, &self.power_vars[i]) != 0.;
            // Navigation (1) vs. Attitude (2); Off (0) is also treated as
            // "not nav mode" but is irrelevant since `should_run` already
            // requires the unit to be aligned, which only happens in
            // Navigation.
            let nav_mode = SimulatorReaderWriter::read(vars, &self.mode_selector_vars[i]) < 1.5;
            let gps_index = if i == 1 { 1 } else { 0 };
            let gps_valid = !gps_failed[gps_index];
            self.adirus[i].update(vars, dt, &true_state, fbw_state, powered, nav_mode, gps_valid);
        }
        self.radio_altimeter.update(vars, xplm);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A stationary, level, mid-latitude true state: zero rates, 1g normal
    /// load, zero AoA/Mach/pressure error sources not under test.
    fn stationary_level_state(lat_deg: f64) -> TrueState {
        TrueState {
            p_xp: 0.,
            q_xp: 0.,
            r_xp: 0.,
            g_axil: 0.,
            g_side: 0.,
            g_nrml: 1.,
            theta_xp: 0.,
            phi_xp: 0.,
            psi_xp: 0.,
            lat_deg,
            lon_deg: 0.,
            alt_m: 0.,
            static_pressure_pa: P0_PA,
            sat_c: 15.,
            mach: 0.,
            alpha_deg: 3.0, // the ADR position-error model's zero-error reference
            on_ground: true,
            placed: true,
            v_north_ms: 0.,
            v_east_ms: 0.,
            any_engine_running: false,
            lwc_gm3: 0.,
        }
    }

    /// Runs a fully-aligned, GPIRS-aided ADIRU stationary at 45N for
    /// `hours`, returning the final position error in nm. This is the
    /// module's normal operating mode (GPIRS always blends GPS into the
    /// free-inertial solution -- see `Adiru::gpirs_correct`'s doc comment
    /// for why an *undamped* free-inertial run is intentionally not what
    /// ships), so this is the direct check of the brief's "stationary drift
    /// over 1h within spec" requirement.
    fn run_stationary_hours(hours: f64) -> f64 {
        let mut a = Adiru::new_for_test(1);
        let t = stationary_level_state(45.0);
        a.lat_rad = t.lat_deg.to_radians();
        a.lon_rad = t.lon_deg.to_radians();
        a.running = true;
        let dt = 0.1;
        let steps = (hours * 3600.0 / dt) as u64;
        for _ in 0..steps {
            a.mechanize(dt, &t);
            a.baro_inertial_correct(dt, &t);
            a.gpirs_correct(dt, &t);
        }
        a.position_error_m / NM_TO_M
    }

    #[test]
    fn stationary_drift_over_one_hour_is_within_1_nm_per_hour() {
        let error_nm = run_stationary_hours(1.0);
        assert!(
            error_nm < 1.0,
            "GPIRS-aided position error after 1h stationary should be under the 1 nm/hr ARINC-704-class \
             budget (docs/physics/adirs.md), was {error_nm:.3} nm"
        );
    }

    #[test]
    fn mechanization_stays_bounded_and_finite_over_a_long_run() {
        // "Stability and cost: ... no NaNs or explosions at any sim rate or
        // pause" (docs/briefs/hyperrealism.md): 3 hours stationary must not
        // diverge, at a coarse sim-paused-then-resumed-like step size.
        let error_nm = run_stationary_hours(3.0);
        assert!(error_nm.is_finite());
        assert!(error_nm < 5.0, "expected a bounded GPIRS-aided error, got {error_nm:.1} nm");
    }

    #[test]
    fn schuler_period_constant_matches_the_classical_84_4_minutes() {
        // The mechanization's restoring-force structure (gravity leaking
        // into the horizontal specific force through a tilt/position error,
        // fed back through the transport-rate term) is only stable in its
        // pure, undamped form for a fraction of one Schuler period before
        // explicit-Euler integration error dominates (see
        // `Adiru::gpirs_correct`'s doc comment) -- this module always runs
        // GPIRS-damped in practice, which suppresses the oscillation rather
        // than letting it run for the several cycles a clean period
        // measurement needs. What *is* directly testable, and is the
        // physical fact `GYROCOMPASS_TAU_S`/`GPIRS_TIME_CONSTANT_S` are
        // chosen relative to, is the classical Schuler relationship itself,
        // T = 2*pi*sqrt(R/g) (Wikipedia, "Schuler tuning"), evaluated with
        // this module's own Earth-radius and gravity constants.
        let period_s = 2.0 * PI * (EARTH_RADIUS_M / STANDARD_GRAVITY).sqrt();
        let period_min = period_s / 60.0;
        assert!(
            (period_min - 84.4).abs() < 1.0,
            "Schuler period from this module's own R and g should be close to the classical 84.4 min, was {period_min:.2} min"
        );
    }

    #[test]
    fn gyrocompass_alignment_takes_longer_near_the_poles() {
        // Mirrors FBW's own `total_alignment_duration_from_configuration`
        // (adirs.rs) scaling total alignment time by 1/cos(latitude): the
        // horizontal component of Earth's rotation rate available to
        // gyrocompass on shrinks toward the poles, so both FBW's timer and
        // this module's heading-convergence time constant lengthen the same
        // way (docs/physics/adirs.md).
        let tau = |lat_deg: f64| GYROCOMPASS_TAU_S / (lat_deg as f64).to_radians().cos().max(0.05);
        assert!(tau(0.0) < tau(60.0));
        assert!(tau(60.0) < tau(80.0));
    }

    #[test]
    fn heading_error_decays_during_a_realistic_alignment() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let mut a = Adiru::new_for_test(1);
        let t = stationary_level_state(45.0);
        // Off -> Aligning at t=0.
        a.update_for_test(0.1, &t, 0.0, true);
        let error_at_start = a.heading_error_deg;
        for _ in 0..3000 {
            // 300s of aligning at 0.1s steps.
            a.update_for_test(0.1, &t, 1.0, true);
        }
        assert!(a.heading_error_deg < error_at_start, "heading error should have decayed while aligning");
        assert!(a.heading_error_deg < 1.0, "residual gyrocompass error after 300s at the equator should be under 1 degree, was {:.2}", a.heading_error_deg);
    }

    #[test]
    fn unheated_pitot_blocks_in_icing_conditions_and_freezes_its_reading() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(45.0);
        t.sat_c = -10.0; // icing conditions
        t.on_ground = false;
        t.mach = 0.3; // ~100 m/s TAS at -10C -- airflow, so convection/catch are nonzero
        t.lwc_gm3 = 1.0;
        let dt = 1.0;
        // Unpowered (heat off): ice should accrete past the block threshold.
        for _ in 0..90 {
            a.update_adr(dt, &t, false);
        }
        assert!(a.adr_pitot_blocked, "pitot should block after ~90s unheated in icing conditions");
        let frozen = a.adr_static_pa;

        // Once blocked, changing the true atmosphere (climbing) must not
        // change the reported pressure -- the classic "blocked pitot"
        // physical symptom.
        t.static_pressure_pa -= 5000.0;
        t.mach = 0.3;
        for _ in 0..10 {
            a.update_adr(dt, &t, false);
        }
        assert_eq!(a.adr_static_pa, frozen, "a blocked static port must keep its last reading");
    }

    /// failures::extra 30_000 ("ADIRU 1 probe heater open circuit"): even
    /// powered and airborne (heat would normally be AUTO-on), a probe with
    /// an open heater element ices over in icing conditions exactly like an
    /// unpowered one, since the icing physics itself does not distinguish
    /// why the heater has no power flowing.
    #[test]
    fn a_heater_open_circuit_ices_the_probe_even_when_powered_and_airborne() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::replace([]);
        crate::failures::set_active(30_000, true); // ADIRU 1

        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(45.0);
        t.sat_c = -10.0;
        t.on_ground = false;
        t.mach = 0.3;
        t.lwc_gm3 = 1.0;
        for _ in 0..90 {
            a.update_adr(1.0, &t, true); // powered: heat would be AUTO-on
        }
        assert!(a.adr_pitot_blocked, "an open heater circuit must ice the probe despite bus power");

        // ADIRU 2 has no failure active: its own heater still works.
        let mut b = Adiru::new_for_test(2);
        for _ in 0..90 {
            b.update_adr(1.0, &t, true);
        }
        assert!(!b.adr_pitot_blocked, "an unaffected ADIRU's probe heat must be unaffected");
        crate::failures::replace([]);
    }

    #[test]
    fn heated_probes_never_ice_up() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(45.0);
        t.sat_c = -30.0;
        t.on_ground = false;
        t.mach = 0.32; // ~100 m/s TAS at -30C
        t.lwc_gm3 = 0.6; // REFERENCE_LWC_GM3 -- within the heater's design margin
        for _ in 0..600 {
            a.update_adr(1.0, &t, true); // powered: heat on
        }
        assert!(!a.adr_pitot_blocked && !a.adr_static_blocked);
    }

    /// External prediction, hand-derived *before* running the sim from the
    /// cited relations in `probe_heat_required_w`'s doc comment (Zukauskas
    /// crossflow-cylinder correlation for convection, a Messinger-style
    /// water-catch/latent-heat term) -- not read back from the sim's own
    /// output, so this is not circular.
    ///
    /// Scenario: SAT=-20C, TAS=220 m/s (mach 0.690 at that SAT), LWC=1.0
    /// g/m^3, probe D=0.013m/L=0.15m (module constants).
    ///   Re = 220*0.013/1.13e-5 = 253,097 -> Zukauskas high-Re band
    ///     (C=0.076, m=0.7): Nu = 0.076*Re^0.7*Pr^0.37
    ///     Re^0.7 = exp(0.7*ln(253097)) = exp(8.708) = 6054
    ///     Pr^0.37 = 0.72^0.37 = 0.8856 => Nu = 0.076*6054*0.8856 = 407.5
    ///   h = Nu*k/D = 407.5*0.0206/0.013 = 645.6 W/(m^2 K)
    ///   conv_area = pi*D*L = pi*0.013*0.15 = 0.006126 m^2
    ///   delta_T = 0 - (-20) = 20 K
    ///   Q_conv = h*conv_area*delta_T = 645.6*0.006126*20 = 79.1 W
    ///   frontal_area = D*L = 0.00195 m^2
    ///   catch = LWC(1.0e-3 kg/m^3)*220*0.00195 = 0.000429 kg/s
    ///   Q_water = catch*(cp_water*delta_T + Lf)
    ///           = 0.000429*(4186*20 + 334000) = 0.000429*417720 = 179.2 W
    ///   Q_required = 79.1 + 179.2 = 258.3 W
    ///
    /// Heater rated 350 W (PITOT_HEATER_W). At 60% power (40% loss,
    /// `failures::magnitude` = 0.4), remaining = 210 W < 258.3 W required
    /// -> a 48.3 W deficit -> the probe should ice. At 100% power,
    /// 350 W > 258.3 W -> it should stay clear, in the *same* adverse
    /// environment -- proving the effect is the heater-power coupling, not
    /// the environment alone (the decoupling check the coordinator asked
    /// for: sever the failure->heater-power link by leaving magnitude at
    /// its default 0, and the icing effect vanishes even though SAT/TAS/LWC
    /// are unchanged). Likewise, relaxing SAT or LWC/TAS back to benign
    /// values while keeping the heater at 60% must also stay clear -- no
    /// single factor is sufficient on its own; only the combination is.
    #[test]
    fn degraded_probe_heater_ices_only_when_combined_with_cold_high_lwc_and_high_tas() {
        let _guard = crate::failures::tests::serial();
        let _f = crate::failures::Failures::new();
        crate::failures::reset_all();
        crate::failures::replace([]);

        let adverse = |mach: f64, sat_c: f64, lwc_gm3: f64| {
            let mut t = stationary_level_state(45.0);
            t.on_ground = false;
            t.mach = mach;
            t.sat_c = sat_c;
            t.lwc_gm3 = lwc_gm3;
            t
        };
        let t_adverse = adverse(0.690, -20.0, 1.0); // TAS ~220 m/s at -20C

        // (1) The combination: heater at 60% power (id 30_000 -- ADIRU 1's
        // probe heater, magnitude 0.4 = 40% power lost) in the full
        // adverse environment -- must ice, matching the hand-derived
        // 258.3 W > 210 W remaining-power deficit above.
        crate::failures::set_magnitude(30_000, 0.4);
        let mut degraded = Adiru::new_for_test(1);
        for _ in 0..60 {
            degraded.update_adr(1.0, &t_adverse, true);
        }
        assert!(
            degraded.adr_pitot_blocked,
            "60% probe heater power + -20C + 1.0 g/m3 LWC + 220 m/s TAS should exceed the probe's \
             heat balance (predicted 258.3W required vs 210W remaining) and ice the probe"
        );

        // (2) DECOUPLING: same adverse environment, but the failure->heater
        // link is cut (no active magnitude, i.e. full 350W rated power).
        // The predicted requirement (258.3W) is still less than 350W, so
        // the effect must vanish -- proving it was the heater-power
        // coupling causing (1), not the environment by itself.
        crate::failures::reset_all();
        let mut healthy = Adiru::new_for_test(2);
        for _ in 0..60 {
            healthy.update_adr(1.0, &t_adverse, true);
        }
        assert!(
            !healthy.adr_pitot_blocked,
            "a healthy (undegraded) probe heater must clear the same adverse environment that iced \
             the degraded one -- the icing must come from the heater-power coupling, not the weather alone"
        );

        // (3) Degraded heater (60%), but SAT relaxed to -5C: predicted
        // requirement drops to ~172.1W, still under the 210W remaining --
        // must not ice. Cold alone (with everything else adverse) is not
        // sufficient without the heater degradation *and* this magnitude
        // of cold together.
        crate::failures::set_magnitude(30_000, 0.4);
        let mut mild_cold = Adiru::new_for_test(1);
        let t_mild_cold = adverse(0.690, -5.0, 1.0);
        for _ in 0..60 {
            mild_cold.update_adr(1.0, &t_mild_cold, true);
        }
        assert!(!mild_cold.adr_pitot_blocked, "60% heater power alone, without the -20C SAT, must not ice the probe");

        // (4) Degraded heater (60%), -20C SAT, but LWC/TAS back to benign
        // (0.1 g/m3, ~130 m/s): predicted requirement ~67.7W, far under
        // 210W -- must not ice. Cold + degraded heater without wet, fast
        // air is not sufficient either.
        let mut dry_air = Adiru::new_for_test(1);
        let t_dry = adverse(0.408, -20.0, 0.1);
        for _ in 0..60 {
            dry_air.update_adr(1.0, &t_dry, true);
        }
        assert!(!dry_air.adr_pitot_blocked, "60% heater power + cold alone, without high LWC/TAS, must not ice the probe");

        crate::failures::reset_all();
        crate::failures::replace([]);
    }

    #[test]
    fn static_pressure_at_zero_position_error_matches_isa_and_true_pressure() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        // At alpha = 3 degrees (this module's zero-error reference, see
        // `update_adr`) and no icing, the sensed static pressure should
        // equal the true ISA pressure fed in, and via FBW's own
        // `AirDataReference::calculate_altitude_from_static_pressure`
        // (unmodified, adirs.rs) that pressure inverts back to the standard
        // ISA altitude it represents.
        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(45.0);
        // ISA pressure at 10,000 ft (standard atmosphere table): 696.8 hPa.
        t.static_pressure_pa = 69_680.0;
        t.on_ground = false;
        a.update_adr(1.0, &t, true);
        assert!(
            (a.adr_static_pa - t.static_pressure_pa).abs() < 1.0,
            "zero position error should pass the true static pressure through unchanged, got {:.1} Pa vs {:.1} Pa",
            a.adr_static_pa,
            t.static_pressure_pa
        );
    }

    #[test]
    fn gyro_and_accelerometer_biases_differ_between_adirus() {
        // The Study panel's "per ADIRU ... sensor biases" are only
        // meaningful if the 3 units actually draw different values.
        let a1 = Adiru::new_for_test(1);
        let a2 = Adiru::new_for_test(2);
        let a3 = Adiru::new_for_test(3);
        let biases = [a1.gyro[2].bias_deg_s, a2.gyro[2].bias_deg_s, a3.gyro[2].bias_deg_s];
        assert!(biases[0] != biases[1] && biases[1] != biases[2] && biases[0] != biases[2]);
    }

    #[test]
    fn agl_placement_guard_matches_start_state_rs() {
        // Same bounds `start_state.rs::read_situation` uses for its
        // identical "position not placed yet" check -- both guards must
        // agree on what counts as parked-but-real vs. garbage.
        assert!(agl_looks_placed(0.0));
        assert!(agl_looks_placed(35_000.0)); // cruise
        assert!(agl_looks_placed(-500.0)); // Dead Sea airport, below MSL
        assert!(!agl_looks_placed(-1_000_000.0)); // "millions of feet below ground"
        assert!(!agl_looks_placed(f64::NAN));
        assert!(!agl_looks_placed(f64::INFINITY));
        assert!(!agl_looks_placed(f64::NEG_INFINITY));
    }

    #[test]
    fn baro_inertial_correct_never_produces_a_runaway_vertical_speed_from_a_garbage_altitude() {
        // Regression test for the audit's finding: `alt_m`'s correction was
        // clamped but `v_down`'s was not, so a huge one-tick `error` (e.g.
        // X-Plane's pre-placement "millions of feet below ground" `elevation`
        // reaching this function, which `TrueStateSource::read`'s
        // `agl_looks_placed` guard now should prevent, but this checks the
        // function's own defence independently) produced an arbitrarily
        // large implied vertical speed.
        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(45.0);
        a.alt_m = 0.0;
        a.v_down = 0.0;
        t.alt_m = -1.0e7; // garbage: ~33 million feet below ground
        let dt = 1.0 / 30.0; // a typical X-Plane frame
        a.baro_inertial_correct(dt, &t);
        let vs_fpm = -a.v_down * M_TO_FT * 60.0;
        assert!(vs_fpm.is_finite());
        assert!(
            vs_fpm.abs() < 2_000.0,
            "a single tick's garbage altitude reading must not swing vertical speed \
             anywhere near a real climb/descent rate, got {vs_fpm:.0} fpm"
        );
    }

    #[test]
    fn baro_inertial_correct_still_nulls_a_real_altitude_error_normally() {
        // The clamp must not neuter ordinary, plausible baro-inertial
        // correction: a realistic few-metre mismatch should still pull
        // `alt_m` and `v_down` toward truth in the expected direction.
        let mut a = Adiru::new_for_test(1);
        let t = { let mut t = stationary_level_state(45.0); t.alt_m = 50.0; t };
        a.alt_m = 0.0;
        a.v_down = 0.0;
        // The loop's natural period is 100s (`BARO_LOOP_PERIOD_S`); run
        // several periods so a critically damped response has settled.
        for _ in 0..3_000 {
            a.baro_inertial_correct(0.1, &t);
        }
        assert!(
            (a.alt_m - 50.0).abs() < 1.0,
            "alt_m should converge to the true altitude, got {:.2}",
            a.alt_m
        );
    }

    #[test]
    fn adiru_starting_already_aligned_does_not_spike_vertical_speed_on_the_ground() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        // Regression test for the reported bug (docs/deep/debug_vs.md): a
        // real X-Plane 12 session showed about -8800 ft/min on the PFD with
        // the aircraft on the ground and not descending. FBW's own ADIRS
        // (fbw-common's `navigation/adirs.rs`, `InertialReference::new`)
        // starts every unit already `AlignState::Aligned` ("to support
        // starting on the runway or in the air"), so a freshly constructed
        // `Adiru` -- whose `alt_m`/`lat_rad`/`lon_rad` all default to 0 --
        // could be asked to start its free-inertial mechanization
        // immediately, on its very first tick, against a real, non-zero
        // truth altitude it had never synced to.
        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(45.0);
        t.alt_m = 300.0; // a representative field elevation, metres
        t.on_ground = true;
        // Already "Aligned" (2) and powered on this very first call, exactly
        // as FBW's own ADIRS reports at spawn -- `a` has never run before.
        a.update_for_test(1.0 / 30.0, &t, 2.0, true);
        let vs_fpm = -a.v_down * M_TO_FT * 60.0;
        assert!(
            vs_fpm.abs() < 100.0,
            "a stationary aircraft that starts already 'Aligned' must not show a \
             spurious vertical speed from the baro-inertial loop nulling out an \
             unsynced altitude, got {vs_fpm:.0} fpm"
        );
        assert!(
            (a.alt_m - t.alt_m).abs() < 1e-3,
            "altitude should be synced to truth on the running transition, got {}",
            a.alt_m
        );
    }

    #[test]
    fn adiru_starting_already_aligned_also_syncs_position_heading_and_ground_speed() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        // Same root cause as the vertical-speed spike above, for the other
        // "ADIRUS output wrong at spawn" symptoms it also caused: without
        // the sync, `lat_rad`/`lon_rad` would start at 0N/0E ("Null Island")
        // and only crawl to truth over `GPIRS_TIME_CONSTANT_S` (120s), and
        // `v_north`/`v_east` (hence ground speed and true track) and
        // pitch/roll/heading would all read a stale/zero default instead of
        // this tick's truth, for a unit that starts already aligned and
        // moving (e.g. an "in the air" start FBW's own comment names).
        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(12.0);
        t.lat_deg = 12.0;
        t.lon_deg = 34.0;
        t.alt_m = 3000.0;
        t.theta_xp = 2.5;
        t.phi_xp = -1.0;
        t.psi_xp = 270.0;
        t.on_ground = false;
        t.v_north_ms = 120.0;
        t.v_east_ms = 30.0;
        a.update_for_test(1.0 / 30.0, &t, 2.0, true);
        // Loose tolerances: one 1/30s mechanization tick still runs after
        // the sync (sensor noise/bias and the Earth/transport-rate terms
        // perturb these by a tiny amount) -- what matters is that these are
        // near truth, not stuck at the constructor's 0 default (which would
        // fail by many degrees/m-s^-1, not a rounding error).
        assert!((a.lat_rad.to_degrees() - 12.0).abs() < 1e-3, "latitude should sync to truth");
        assert!((a.lon_rad.to_degrees() - 34.0).abs() < 1e-3, "longitude should sync to truth");
        assert!((a.v_north - 120.0).abs() < 0.1, "north velocity should sync to truth, not start at 0");
        assert!((a.v_east - 30.0).abs() < 0.1, "east velocity should sync to truth, not start at 0");
        assert!((a.pitch_deg - 2.5).abs() < 0.01, "pitch should sync to truth, not start level");
        assert!((a.roll_deg - (-1.0)).abs() < 0.01, "roll should sync to truth, not start level");
        // Within the residual gyrocompass error a fresh unit starts with
        // (`GYROCOMPASS_INITIAL_ERROR_DEG`), not stuck at the constructor's
        // 0 default.
        assert!(
            (a.heading_deg - 270.0).abs() < GYROCOMPASS_INITIAL_ERROR_DEG + 1.0,
            "heading should sync to truth (plus residual gyrocompass error), not start at 0, got {}",
            a.heading_deg
        );
    }

    #[test]
    fn should_run_waits_for_x_plane_to_place_the_aircraft_even_when_fbw_reports_aligned() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        // The other half of the same fix: `should_run` must not go true off
        // FBW's own immediate "Aligned" state alone -- it must also wait for
        // a real X-Plane placement, or the running-transition sync above
        // would itself run once against pre-placement data (which
        // `TrueStateSource::read` otherwise freezes at 0/parked) and then
        // never run again (`self.running` latches `true`), missing the real
        // truth that arrives a few ticks later when X-Plane actually places
        // the aircraft.
        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(45.0);
        t.placed = false; // X-Plane has not placed the aircraft yet
        let should_run = a.update_for_test(1.0 / 30.0, &t, 2.0, true);
        assert!(
            !should_run,
            "must not start mechanizing before a real position exists, even if FBW's \
             own ADIRS already reports Aligned"
        );
    }

    /// The live-defect regression test (the PFD attitude sphere rolled tens
    /// of degrees and the ND ground speed climbing without bound on a
    /// parked, level, powered aircraft with a healthy, aligned IR -- see the
    /// report). Unlike `run_stationary_hours` above (which calls
    /// `mechanize`/`baro_inertial_correct`/`gpirs_correct` directly, with
    /// `running` preset to `true`), this goes through the actual production
    /// path (`update_for_test` -> `advance` -> `write`'s inputs), at a
    /// realistic X-Plane frame `dt`, for 30 simulated minutes: a healthy,
    /// already-aligned, powered, Nav-mode IR on a genuinely stationary,
    /// level aircraft must hold attitude and ground speed near truth, not
    /// merely "bounded".
    #[test]
    fn stationary_aligned_ir_holds_attitude_and_ground_speed_over_30_minutes() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let mut a = Adiru::new_for_test(1);
        let t = stationary_level_state(28.0); // representative mid-latitude airport
        let dt = 1.0 / 30.0; // typical X-Plane frame time
        let steps = (30.0 * 60.0 / dt) as u64;
        for _ in 0..steps {
            a.update_for_test(dt, &t, 2.0, true); // FBW state 2 = Aligned throughout
        }
        let ground_speed_kt =
            (a.v_north * a.v_north + a.v_east * a.v_east).sqrt() * MS_TO_KNOT;
        assert!(
            a.pitch_deg.abs() < 0.1,
            "a level, stationary, aligned IR must hold pitch within 0.1 deg over 30 min, got {:.3} deg",
            a.pitch_deg
        );
        assert!(
            a.roll_deg.abs() < 0.1,
            "a level, stationary, aligned IR must hold roll within 0.1 deg over 30 min, got {:.3} deg",
            a.roll_deg
        );
        assert!(
            ground_speed_kt < 1.0,
            "a stationary, aligned IR must not report a growing ground speed over 30 min, got {:.2} kt",
            ground_speed_kt
        );
    }

    /// The other half of the live defect: an IR that is *not* aligned (or
    /// not yet running its free-inertial solution) must say so, not present
    /// data as valid. `Adiru::write`'s `valid` argument is exactly
    /// `should_run` (see `update`), which FBW's own ARINC/SSM stage (kept
    /// unmodified) turns into the PFD's ATT/HDG flag.
    #[test]
    fn an_unaligned_ir_reports_invalid_not_a_plausible_value() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let mut a = Adiru::new_for_test(1);
        let t = stationary_level_state(45.0);
        // fbw_state 1.0 == Aligning, not yet Aligned.
        let should_run = a.update_for_test(1.0 / 30.0, &t, 1.0, true);
        assert!(!should_run, "an aligning (not yet aligned) IR must not report valid data");

        // Off (0.0) must likewise not be valid.
        let mut b = Adiru::new_for_test(2);
        let should_run_off = b.update_for_test(1.0 / 30.0, &t, 0.0, true);
        assert!(!should_run_off, "an off IR must not report valid data");

        // Aligned (2.0) but unpowered must also not be valid -- no bus, no
        // free-inertial solution.
        let mut c = Adiru::new_for_test(3);
        let should_run_unpowered = c.update_for_test(1.0 / 30.0, &t, 2.0, false);
        assert!(!should_run_unpowered, "an unpowered IR must not report valid data even if FBW reports it aligned");
    }

    /// [`plausible`] in isolation: holds the last plausible sample on a
    /// rejected (non-finite or out-of-bound) one, doesn't fabricate any
    /// other value, and accepts a normal in-bound reading immediately.
    #[test]
    fn plausible_holds_the_last_good_reading_and_rejects_garbage() {
        let mut last = 1.0_f64;
        assert_eq!(plausible(&mut last, 2.0, 10.0), 2.0, "an in-bound reading is accepted");
        assert_eq!(plausible(&mut last, f64::NAN, 10.0), 2.0, "NaN is rejected, last good is held");
        assert_eq!(plausible(&mut last, f64::INFINITY, 10.0), 2.0, "infinity is rejected");
        assert_eq!(plausible(&mut last, 1_000.0, 10.0), 2.0, "an out-of-bound reading is rejected");
        assert_eq!(plausible(&mut last, -3.0, 10.0), -3.0, "a later in-bound reading is accepted");
    }

    /// [`plausible_in`] in isolation. The point of the range form is the
    /// asymmetry: 0.0 and -40.0 are perfectly ordinary readings for some
    /// channels and impossible for others, which a magnitude bound cannot
    /// distinguish.
    #[test]
    fn plausible_in_rejects_out_of_range_readings_on_either_side() {
        let mut last = 101_325.0_f64;
        assert_eq!(plausible_in(&mut last, 22_632.0, 100., 120_000.), 22_632.0, "cruise static pressure is accepted");
        assert_eq!(plausible_in(&mut last, 0.0, 100., 120_000.), 22_632.0, "0 Pa is finite but not an atmosphere");
        assert_eq!(plausible_in(&mut last, -5.0, 100., 120_000.), 22_632.0, "negative absolute pressure is rejected");
        assert_eq!(plausible_in(&mut last, f64::NAN, 100., 120_000.), 22_632.0, "NaN is rejected, last good is held");
        assert_eq!(plausible_in(&mut last, 1.0e9, 100., 120_000.), 22_632.0, "an absurd high reading is rejected");
        assert_eq!(plausible_in(&mut last, 101_325.0, 100., 120_000.), 101_325.0, "a later in-range reading is accepted");

        // The same bound the read path uses for SAT keeps `sat_c + 273.15`
        // strictly positive, which is the whole reason it exists.
        let mut sat = 15.0_f64;
        assert_eq!(plausible_in(&mut sat, -300.0, -200., 100.), 15.0, "below absolute zero is rejected");
        assert_eq!(plausible_in(&mut sat, -70.0, -200., 100.), -70.0, "a cruise-level SAT is accepted");
    }

    /// Why [`plausible_in`] guards SAT: without it, a static air temperature
    /// below absolute zero takes the square root of a negative number and
    /// publishes NaN as true airspeed and total air temperature. This feeds
    /// `update_adr` directly, bypassing the read-path guard, to show the
    /// hazard is real rather than theoretical.
    #[test]
    fn a_sub_absolute_zero_sat_would_publish_nan_airspeed() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(45.0);
        t.sat_c = -300.0;
        a.update_adr(1.0 / 30.0, &t, true);
        assert!(!a.adr_tas_ms.is_finite(), "sqrt of a negative absolute temperature is NaN");
        // TAT is not NaN here -- stationary, so the recovery term is 1.0 and
        // it simply carries the impossible temperature straight through.
        assert!(a.adr_tat_c < -273.15, "and TAT reports a sub-absolute-zero temperature: {}", a.adr_tat_c);

        // With the guard in front of it, the same reading never arrives.
        let mut last = 15.0_f64;
        let mut good = stationary_level_state(45.0);
        good.sat_c = plausible_in(&mut last, -300.0, -200., 100.);
        let mut b = Adiru::new_for_test(2);
        b.update_adr(1.0 / 30.0, &good, true);
        assert!(b.adr_tas_ms.is_finite(), "the held ISA reading keeps the speed of sound real");
        assert!(b.adr_tat_c > -273.15, "and TAT stays a physical temperature: {}", b.adr_tat_c);
    }

    /// Why [`plausible_in`] guards Mach and static pressure: the NaN does
    /// not merely pass through, it *sticks*. `qc_true` carries it into
    /// `frozen_static_pressure_pa`, and while the static source is iced over
    /// the only path that re-seeds that field is its `== 0.0` sentinel,
    /// which NaN never matches. So one bad tick poisons static pressure, CAS
    /// and Mach for the rest of the session even after the sim's own reading
    /// recovers -- which is why the guard has to sit at the source, in
    /// `TrueStateSource::read`, and not downstream. (If a later change also
    /// makes that sentinel NaN-aware, update this test deliberately.)
    #[test]
    fn a_nan_mach_permanently_poisons_a_blocked_static_source() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let dt = 1.0 / 30.0;
        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(45.0);
        a.static_ice_kg = PROBE_BLOCK_MASS_KG; // static ports iced over
        t.mach = f64::NAN;
        a.update_adr(dt, &t, true);
        assert!(!a.adr_static_pa.is_finite(), "the NaN reaches the frozen static pressure");

        t.mach = 0.0; // the sim recovers
        for _ in 0..100 {
            a.update_adr(dt, &t, true);
        }
        assert!(!a.adr_static_pa.is_finite(), "and there is no way back while the source is blocked");

        // Guarded at the source, the poisoning tick never happens.
        let mut last = 0.0_f64;
        let mut good = stationary_level_state(45.0);
        good.mach = plausible_in(&mut last, f64::NAN, 0., 6.);
        let mut b = Adiru::new_for_test(2);
        b.static_ice_kg = PROBE_BLOCK_MASS_KG;
        b.update_adr(dt, &good, true);
        assert!(b.adr_static_pa.is_finite(), "a rejected sample holds the last good Mach instead");
    }

    /// The live-defect regression test's other half: this reproduces the
    /// *mechanism* (see the report) -- a real X-Plane quirk (a one-tick
    /// `local_vx`/`local_vz` spike across an aircraft placement/teleport
    /// event) handing `Adiru::advance` a single wildly-implausible velocity
    /// reading, which `om_in_n`'s Earth-rate/transport-rate term
    /// (`Adiru::mechanize`) turns into an enormous synthetic body rate for
    /// that one tick -- enough to corrupt pitch/roll by tens of degrees in a
    /// single RK4 step, exactly the observed "PFD attitude sphere rolled
    /// tens of degrees... with no ATT/HDG flags". This feeds the bad
    /// `TrueState` straight to `Adiru::advance` (bypassing
    /// `TrueStateSource`'s own new `plausible` gate) to prove the second,
    /// independent layer -- `advance`'s own post-mechanize BITE check --
    /// catches it even if some other, unanticipated corruption source ever
    /// reaches this state: the corrupted tick must not be published as
    /// valid, and the unit must recover on subsequent good ticks, not stay
    /// poisoned the way a permanently-NaN state would.
    #[test]
    fn a_one_tick_velocity_spike_faults_the_ir_instead_of_corrupting_it_forever() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(30.0);
        let dt = 1.0 / 30.0;

        // Run normally for a few seconds so the unit is genuinely mechanizing.
        for _ in 0..90 {
            assert!(a.update_for_test(dt, &t, 2.0, true));
        }
        assert!(a.pitch_deg.abs() < 0.1 && a.roll_deg.abs() < 0.1);

        // One tick of a placement/teleport-style velocity spike, in the
        // same v_north_ms/v_east_ms channel `TrueStateSource::read` freezes
        // -- see the report and `plausible`'s doc comment. X-Plane's own
        // `local_vx`/`local_vz` are a raw finite difference of position, so
        // a scenery-load/teleport position jump of even a few km in one
        // frame produces a spike of this order, not a small one.
        t.v_north_ms = 5.0e8;
        t.v_east_ms = -3.0e8;
        let should_run_bad_tick = a.update_for_test(dt, &t, 2.0, true);
        assert!(
            !should_run_bad_tick,
            "a mechanization result this implausible must fault, not be published as valid data"
        );
        assert!(a.pitch_deg.is_finite(), "the fault reset must leave a finite state, not NaN");
        assert!(a.roll_deg.is_finite(), "the fault reset must leave a finite state, not NaN");

        // Truth returns to normal; the unit re-aligns and must recover, not
        // stay poisoned -- an IR that faulted once and never came back
        // would be exactly as useless to a pilot as one that silently kept
        // publishing the garbage value.
        t.v_north_ms = 0.;
        t.v_east_ms = 0.;
        let mut recovered = false;
        for _ in 0..(30 * 30) {
            // Re-align (fbw_state 1 -> 2) the same way a real Off->Aligning
            // ->Aligned cycle would after a BITE fault forces a realign.
            if a.update_for_test(dt, &t, 2.0, true) {
                recovered = true;
            }
        }
        assert!(recovered, "the IR must resume publishing valid data after the fault clears");
        let ground_speed_kt = (a.v_north * a.v_north + a.v_east * a.v_east).sqrt() * MS_TO_KNOT;
        assert!(
            a.pitch_deg.is_finite() && a.pitch_deg.abs() < 5.0,
            "pitch must recover to near-level, not stay diverged, got {:.3} deg",
            a.pitch_deg
        );
        assert!(
            a.roll_deg.is_finite() && a.roll_deg.abs() < 5.0,
            "roll must recover to near-level, not stay diverged, got {:.3} deg",
            a.roll_deg
        );
        assert!(
            ground_speed_kt.is_finite() && ground_speed_kt < 50.0,
            "ground speed must recover, not stay at a runaway value, got {:.2} kt",
            ground_speed_kt
        );
    }

    /// The non-finite case specifically (the task's explicit bar: "a
    /// non-finite input does not poison the state permanently") -- a NaN
    /// specific-force reading (e.g. a divide-by-zero somewhere upstream in
    /// X-Plane's own force computation, or any other source this module
    /// cannot anticipate) must not enter the mechanization and stay there
    /// across every subsequent tick the way an unguarded integrator would.
    #[test]
    fn a_non_finite_reading_does_not_poison_the_state_forever() {
        // `failures::STATE` is process-wide and this test reaches it (directly, or
        // through model code such as `Breakers::pre_systems` / `Damage::arm`).
        let _serial = crate::failures::tests::serial();
        let mut a = Adiru::new_for_test(1);
        let mut t = stationary_level_state(30.0);
        let dt = 1.0 / 30.0;
        for _ in 0..30 {
            a.update_for_test(dt, &t, 2.0, true);
        }

        t.g_axil = f64::NAN;
        let should_run_bad_tick = a.update_for_test(dt, &t, 2.0, true);
        // Whether or not this exact tick's inputs happen to trip the
        // plausibility fault (a NaN specific force need not immediately
        // move pitch/roll/velocity out of bounds on the very first RK4
        // step), the state itself must never become non-finite and must
        // never stay that way.
        assert!(a.pitch_deg.is_finite(), "pitch must never go non-finite");
        assert!(a.roll_deg.is_finite(), "roll must never go non-finite");
        assert!(a.v_north.is_finite(), "v_north must never go non-finite");
        assert!(a.v_east.is_finite(), "v_east must never go non-finite");
        let _ = should_run_bad_tick;

        t.g_axil = 0.; // truth returns to normal
        for _ in 0..(30 * 30) {
            a.update_for_test(dt, &t, 2.0, true);
        }
        assert!(a.pitch_deg.is_finite() && a.pitch_deg.abs() < 5.0);
        assert!(a.roll_deg.is_finite() && a.roll_deg.abs() < 5.0);
        let ground_speed_kt = (a.v_north * a.v_north + a.v_east * a.v_east).sqrt() * MS_TO_KNOT;
        assert!(ground_speed_kt.is_finite() && ground_speed_kt < 50.0);
    }
}
