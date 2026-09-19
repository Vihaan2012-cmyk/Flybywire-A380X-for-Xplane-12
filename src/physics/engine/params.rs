//! Cited parameters for the engine FlyByWire's A380X package models: a
//! Rolls-Royce Trent 972B-84 (`engines.cfg`'s own header comment, "TazX -
//! First attempt at the Trent 972B-84", and `[TURBINEENGINEDATA]
//! static_thrust = 80213` lbf = 356.8 kN, matching the Trent 972-84's
//! published rated thrust exactly). The A380 also flies with the Engine
//! Alliance GP7200; FlyByWire's package does not model it, so this file
//! does not either.
//!
//! Every constant below says where it comes from. Three tiers, in
//! decreasing order of certainty:
//! - **Package data**: read directly from
//!   `fbw-a380x/.../common/config/engines.cfg`, the data FlyByWire tuned for
//!   this airframe.
//! - **Public reference**: a published, cited figure for the Trent 900
//!   family (EASA/CAA type-certificate data sheets are not readable in this
//!   environment; the figures below come from Rolls-Royce's own published
//!   specifications and the aircraft-commerce.com Trent family spec sheet,
//!   cross-checked against Wikipedia's "Rolls-Royce Trent 900" summary).
//! - **Derived/generic**: no public source gives this number for this
//!   engine. It is derived from cited geometry/mass or taken from generic,
//!   textbook turbomachinery values, clearly marked, per the brief's
//!   allowance ("If no source exists, say so and choose the most defensible
//!   derived value"). These are the values `docs/physics/engine.md`
//!   discusses in the most depth, and the ones most worth revisiting if a
//!   real Trent 900 component map ever surfaces.

/// Standard-day reference conditions (ICAO standard atmosphere), used for
/// "corrected" spool speeds and mass flows throughout.
pub const T_REF_K: f64 = 288.15;
pub const P_REF_PA: f64 = 101_325.0;

// ---- Package data (engines.cfg) ---------------------------------------

/// `engines.cfg` header comment: "LP - Real N1 - Sim N1 - 2,900RPM". The
/// fan/LP spool's 100%-corrected physical speed.
pub const N1_DESIGN_RPM: f64 = 2900.0;
/// `engines.cfg` header comment: "IP - Real N2 - Sim XX - 8,300RPM".
pub const N2_DESIGN_RPM: f64 = 8300.0;
/// `engines.cfg` header comment: "HP - Real N3 - Sim N2 - 12,200RPM".
pub const N3_DESIGN_RPM: f64 = 12200.0;

/// `[TURBINEENGINEDATA] static_thrust = 80213` lbf, one engine, sea level
/// static, converted to newtons (× 4.4482216153). Matches the Trent 972-84's
/// published 356.8 kN rating.
pub const STATIC_THRUST_N: f64 = 80213.0 * 4.448_221_615_3;

/// `[TURBINEENGINEDATA] low_idle_n1`/`low_idle_n2`: ground idle spool
/// speeds, percent. engines.cfg's "n2" is MSFS's generic two-spool engine's
/// second spool, which the package (and this plugin, matching it) reads as
/// the A380's N3/HP spool.
pub const IDLE_N1_PCT: f64 = 15.0;
pub const IDLE_N3_PCT: f64 = 60.0;

/// `min_n1_for_combustion`/`min_n2_for_combustion`: the corrected speeds
/// below which the package considers the engine unable to sustain
/// combustion (light-off floor). Used the same way here: fuel is cut below
/// these, not as a scripted state but because the combustor energy balance
/// below them cannot be sustained at a stable temperature (see
/// `governor.rs`).
pub const MIN_N1_FOR_COMBUSTION_PCT: f64 = 10.0;
pub const MIN_N3_FOR_COMBUSTION_PCT: f64 = 20.0;

/// `max_n1_protection`/`max_n2_protection`: FADEC overspeed protection
/// setpoints, percent corrected speed.
pub const MAX_N1_PROTECTION_PCT: f64 = 101.0;
pub const MAX_N3_PROTECTION_PCT: f64 = 116.5;

/// `starter_N1_max_pct`: the percent of max RPM the pneumatic starter alone
/// can reach (used as `starter.rs`'s self-sustaining ceiling check).
pub const STARTER_ALONE_MAX_N1_PCT: f64 = 12.0;

/// A backstop on the combustor's overall fuel-to-air ratio (fuel mass flow
/// ÷ the core air mass flow actually reaching the combustor this frame),
/// well above this model's own design-point FAR (`docs/physics/engine.md`:
/// ≈3.4 kg/s design fuel flow ÷ ≈136.6 kg/s design core flow ≈ 0.025 at
/// 100% N1/SL/ISA, and `mdot_corrected ∝ N_corrected` in `compressor.rs`
/// means this model's own idle-to-TOGA spool-up, already calibrated to the
/// CS-E 745/14 CFR 33.73 5-second requirement checked in `mod.rs`'s tests,
/// genuinely needs to run several times design FAR relative to the still-
/// building core flow early in that transient — this is not meant to bite
/// there). It exists only so a governor bug (or a future change) can never
/// schedule literally unbounded fuel for whatever little air is actually
/// flowing (the combustor's energy balance, `combustor.rs`, has no ceiling
/// of its own on T4): a real annular combustor liner does not survive
/// arbitrarily rich combustion, so *some* finite ratio must cap
/// `governor.rs`'s output even before overspeed/other protections engage.
/// **Derived/generic**: chosen well clear of both this model's design FAR
/// and its certificated-transient FAR so it is a backstop, not the thing
/// actually keeping a cold start's fuel flow realistic (`governor.rs`'s own
/// proportional gain does that, see its module docs).
pub const MAX_COMBUSTOR_FUEL_AIR_RATIO: f64 = 0.08;

// ---- Public reference (Trent 900 family) -------------------------------

/// Bypass ratio. Public Trent 900 family figure (Rolls-Royce/aircraft-
/// commerce.com specification sheet; commonly quoted 8-8.7 across the
/// 970/972/977/980 thrust ratings).
pub const BYPASS_RATIO: f64 = 8.5;

/// Overall pressure ratio at max climb. Public Trent 900 family figure
/// (~39-42:1 quoted across sources).
pub const OPR_DESIGN: f64 = 42.0;

/// Fan tip diameter, metres. Public Trent 900 specification.
pub const FAN_DIAMETER_M: f64 = 2.95;

/// Dry weight, kg. Public Trent 900 specification (used only to derive
/// spool inertia below).
pub const DRY_WEIGHT_KG: f64 = 6246.0;

// ---- Derived/generic ----------------------------------------------------

/// Design-point total (fan × IPC × HPC) pressure ratio split across the
/// three spools. No public per-spool split exists for this engine; this is
/// a plausible architecture-typical split for a 3-spool, fan+8-stage-IPC+
/// 6-stage-HPC layout (Trent architecture), chosen so the product
/// reproduces `OPR_DESIGN` (1.6 × 4.0 × 6.6 = 42.24 ≈ 42). Derived, not
/// measured.
pub const PR_FAN_DESIGN: f64 = 1.6;
pub const PR_IPC_DESIGN: f64 = 4.0;
pub const PR_HPC_DESIGN: f64 = 6.6;

/// Total (core + bypass) design mass flow, kg/s, one engine, sea level
/// static. Derived from a typical high-bypass turbofan specific thrust of
/// ~275 N per kg/s of airflow (a generic, commonly cited figure for this
/// class of engine) applied to `STATIC_THRUST_N`: 356,834 N / 275 ≈ 1298
/// kg/s. No public certificated airflow figure was available to check this
/// against; it is in the range commonly quoted for GE90/Trent-900-class
/// large turbofans (~1200-1300 kg/s).
pub const MDOT_TOTAL_DESIGN_KG_S: f64 = 1298.0;

/// Isentropic component efficiencies at the design point. Generic values
/// typical of a modern large high-bypass civil turbofan (see e.g.
/// Saravanamuttoo, Rogers & Cohen, *Gas Turbine Theory*, for typical
/// component efficiency ranges); no Trent-specific map is public.
pub const ETA_FAN_DESIGN: f64 = 0.90;
pub const ETA_IPC_DESIGN: f64 = 0.89;
pub const ETA_HPC_DESIGN: f64 = 0.86;
pub const ETA_HPT_DESIGN: f64 = 0.90;
pub const ETA_IPT_DESIGN: f64 = 0.91;
pub const ETA_LPT_DESIGN: f64 = 0.925;

/// Combustor efficiency and total pressure loss fraction. Typical modern
/// annular combustor values (Mattingly, *Elements of Gas Turbine
/// Propulsion*, gives 0.98-0.995 combustion efficiency and 3-6% pressure
/// loss as typical ranges).
pub const COMBUSTOR_EFFICIENCY: f64 = 0.999;
pub const COMBUSTOR_PRESSURE_LOSS_FRAC: f64 = 0.05;

/// Inlet ram recovery at subsonic Mach numbers. Typical modern podded
/// nacelle value (Mattingly gives ~0.98-0.995 for a well-designed subsonic
/// inlet up to about Mach 1).
pub const RAM_RECOVERY: f64 = 0.99;

/// Mechanical (shaft/bearing) transmission efficiency, each spool. Typical
/// value for a rigid shaft with rolling-element bearings.
pub const MECH_EFFICIENCY: f64 = 0.99;

/// Fuel-duct/bypass-duct pressure loss fractions. Small, typical values.
pub const BYPASS_DUCT_LOSS_FRAC: f64 = 0.02;

/// Jet A-1 net (lower) heating value, J/kg. Standard published figure
/// (ASTM D1655 / DEF STAN 91-091 typical spec value, ~43.1 MJ/kg).
pub const LHV_JET_A1_J_KG: f64 = 43.1e6;

/// Spool polar moment of inertia, kg·m², derived from `DRY_WEIGHT_KG` and
/// `FAN_DIAMETER_M` since no public figure exists for this engine.
///
/// Method: each spool's rotating assembly is treated as a thin ring of a
/// literature-typical mass fraction of the engine's dry weight (fan+booster
/// module ~18% for a large high-bypass fan, IP compressor module ~8%, HP
/// compressor+turbine module ~10% — generic turbofan module-mass fractions,
/// not Trent-specific), at an effective radius of 75% of the fan tip radius
/// for the LP spool (accounting for the fan disc's mass being spread from
/// hub to tip) and geometrically scaled-down effective radii for the
/// smaller-diameter IP/HP spools (35% and 28% of the fan radius, typical of
/// a compressor's diameter relative to the fan it feeds in this engine
/// class). `I = m * r_eff^2`.
///
/// These are order-of-magnitude derived values, not measured ones. They are
/// the primary free parameter tuned (within the derived range) so the
/// modelled idle-to-TOGA spool-up time meets the certification requirement
/// checked in `mod.rs`'s tests — see `docs/physics/engine.md`.
pub mod inertia {
    use super::{DRY_WEIGHT_KG, FAN_DIAMETER_M};

    const FAN_RADIUS_M: f64 = FAN_DIAMETER_M / 2.0;

    pub const LP_MASS_FRACTION: f64 = 0.18;
    pub const IP_MASS_FRACTION: f64 = 0.08;
    pub const HP_MASS_FRACTION: f64 = 0.10;

    pub const LP_RADIUS_FRAC: f64 = 0.75;
    pub const IP_RADIUS_FRAC: f64 = 0.35;
    pub const HP_RADIUS_FRAC: f64 = 0.28;

    pub fn i_lp() -> f64 {
        (DRY_WEIGHT_KG * LP_MASS_FRACTION) * (FAN_RADIUS_M * LP_RADIUS_FRAC).powi(2)
    }
    pub fn i_ip() -> f64 {
        (DRY_WEIGHT_KG * IP_MASS_FRACTION) * (FAN_RADIUS_M * IP_RADIUS_FRAC).powi(2)
    }
    pub fn i_hp() -> f64 {
        (DRY_WEIGHT_KG * HP_MASS_FRACTION) * (FAN_RADIUS_M * HP_RADIUS_FRAC).powi(2)
    }
}
