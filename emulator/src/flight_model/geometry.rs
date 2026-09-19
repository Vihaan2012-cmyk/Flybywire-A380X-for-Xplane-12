//! Airframe geometry and the empty-aircraft mass baseline, sourced from
//! FlyByWire's own public MSFS `flight_model.cfg`:
//! `D:\fbw-aircraft\fbw-a380x\src\base\flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380X\common\config\flight_model.cfg`
//! (line numbers cited per constant below; this is the same file
//! `src/weight_balance.rs` parses for payload/fuel balance, and its unit
//! tests already pin the empty-CG and gear-adjacent numbers used here, so
//! no `*.acf` was needed once that file's numbers were confirmed present.
//! The X-Plane port's own converted `.acf` was not found under the
//! provided download folder at the time of writing (only textures/objects,
//! no aircraft file) -- see this module's tests for cross-checks against
//! `weight_balance.rs`'s already-verified numbers instead).
//!
//! ## Axis convention
//!
//! MSFS's cfg expresses every position as `(z, x, y)` feet from the
//! reference datum: `z` **forward-positive** (the nose gear at
//! `point.0` sits at `z=99.15`, well forward; the tailstrike point at
//! `point.17` sits at `z=-72.4`, aft), `x` right-positive, `y` up-positive
//! (gear and engines, below the datum, have negative `y`).
//!
//! This model uses the standard SAE aerospace body frame instead --
//! `x` forward, `y` right, `z` **down** -- because that is what the
//! rigid-body equations of motion in `rigid_body.rs` and the textbooks they
//! come from (Stevens & Lewis, "Aircraft Control and Simulation") are
//! written in. [`msfs_point`] converts once, at the boundary:
//! `x_body = z_msfs`, `y_body = x_msfs`, `z_body = -y_msfs`, all in metres.

use super::math::Vec3;

pub const FT_TO_M: f64 = 0.3048;
pub const LB_TO_KG: f64 = 0.453_592_37;
/// 1 slug*ft^2 = 1 lbf*s^2*ft (a slug is 1 lbf*s^2/ft) converted to kg*m^2:
/// `slug_to_kg * ft_to_m^2` with `slug_to_kg = lb_to_kg / g0_ft` is the long
/// way; the standard tabulated factor (e.g. NIST SP811) is used directly.
pub const SLUG_FT2_TO_KG_M2: f64 = 1.355_817_95;
pub const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;
pub const KT_TO_M_S: f64 = 0.514_444_44;

/// Converts one of the cfg's `(z, x, y)` feet positions into the body frame
/// (metres), per the module doc's axis convention.
pub fn msfs_point(z_ft: f64, x_ft: f64, y_ft: f64) -> Vec3 {
    Vec3::new(z_ft * FT_TO_M, x_ft * FT_TO_M, -y_ft * FT_TO_M)
}

// ---------------------------------------------------------------------
// Mass baseline (flight_model.cfg `[WEIGHT_AND_BALANCE]`).
// ---------------------------------------------------------------------

/// `max_gross_weight`, line 17.
pub const MTOW_KG: f64 = 1_124_355.0 * LB_TO_KG;
/// `empty_weight`; confirmed by `weight_balance.rs`'s
/// `the_cfg_balance_is_read_whole` test (`661_403.` lb).
pub const EMPTY_MASS_KG: f64 = 661_403.0 * LB_TO_KG;
/// `empty_weight_cg_position` = `16, 0, 2.8` (z, x, y ft), same test.
pub fn empty_cg_m() -> Vec3 {
    msfs_point(16.0, 0.0, 2.8)
}
/// `empty_weight_roll_MOI` = 53,559,900 (line 24) -- about the body x
/// (roll) axis despite the cfg's own inline comment mislabelling it `Jzz`;
/// MSFS's key names (`roll`/`pitch`/`yaw`), not its comments, are
/// authoritative here.
pub const EMPTY_IXX_KG_M2: f64 = 53_559_900.0 * SLUG_FT2_TO_KG_M2;
/// `empty_weight_pitch_MOI` = 60,518,042 (line 23), about body y.
pub const EMPTY_IYY_KG_M2: f64 = 60_518_042.0 * SLUG_FT2_TO_KG_M2;
/// `empty_weight_yaw_MOI` = 88,229,309 (line 25), about body z.
pub const EMPTY_IZZ_KG_M2: f64 = 88_229_309.0 * SLUG_FT2_TO_KG_M2;

/// `static_pitch` (line 62) and `static_cg_height` (line 63): attitude and
/// CG height at rest, gear compressed to its static position.
pub const STATIC_PITCH_DEG: f64 = -0.13;
pub const STATIC_CG_HEIGHT_M: f64 = 14.30 * FT_TO_M;

// ---------------------------------------------------------------------
// Wing (`[airplane_geometry]`).
// ---------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct Wing {
    pub area_m2: f64,
    pub span_m: f64,
    pub root_chord_m: f64,
    pub mean_chord_m: f64,
    pub aspect_ratio: f64,
    pub dihedral_rad: f64,
    pub sweep_rad: f64,
    pub incidence_rad: f64,
    pub twist_rad: f64,
    /// Oswald span efficiency, `oswald_efficiency_factor` line 590 -- used
    /// directly in the induced-drag build-up (`aerodynamics.rs`).
    pub oswald_e: f64,
}

/// `wing_area` = 9096 sqft (582), `wing_span` = 261.65 ft (583),
/// `wing_root_chord` = 58.86 ft (584), `wing_dihedral` = 5.6 deg (587),
/// `wing_sweep` = 33.5 deg (592), `wing_incidence` = 2 deg (588),
/// `wing_twist` = -5.5 deg (589), `oswald_efficiency_factor` = 0.70 (590).
/// Mean aerodynamic chord is approximated as `area / span` (a rectangular
/// -equivalent chord; the cfg does not publish the true tapered MAC) --
/// GENERIC, close enough for the lift-curve-slope and induced-drag
/// build-up this feeds, not used for CG/stability-margin purposes (that
/// uses `weight_balance.rs`'s real station arms elsewhere in the plugin).
pub fn wing() -> Wing {
    let area_m2 = 9096.0 * FT_TO_M * FT_TO_M;
    let span_m = 261.65 * FT_TO_M;
    Wing {
        area_m2,
        span_m,
        root_chord_m: 58.86 * FT_TO_M,
        mean_chord_m: area_m2 / span_m,
        aspect_ratio: span_m * span_m / area_m2,
        dihedral_rad: 5.6 * DEG_TO_RAD,
        sweep_rad: 33.5 * DEG_TO_RAD,
        incidence_rad: 2.0 * DEG_TO_RAD,
        twist_rad: -5.5 * DEG_TO_RAD,
        oswald_e: 0.70,
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Tail {
    pub area_m2: f64,
    pub span_m: f64,
    pub aspect_ratio: f64,
    pub sweep_rad: f64,
    /// Position of the tail's own aerodynamic centre, body frame, metres.
    pub position_m: Vec3,
}

/// `htail_area` = 1603 sqft (594), `htail_span` = 99.6 ft (595),
/// `htail_pos_lon` = -105 ft (596), `htail_pos_vert` = 17.5 ft (597),
/// `htail_sweep` = 30 deg (599).
pub fn htail() -> Tail {
    let area_m2 = 1603.0 * FT_TO_M * FT_TO_M;
    let span_m = 99.6 * FT_TO_M;
    Tail {
        area_m2,
        span_m,
        aspect_ratio: span_m * span_m / area_m2,
        sweep_rad: 30.0 * DEG_TO_RAD,
        position_m: msfs_point(-105.0, 0.0, 17.5),
    }
}

/// `vtail_area` = 2800.4 sqft (601), `vtail_span` = 47.87 ft (602),
/// `vtail_pos_lon` = -90.5 ft (604), `vtail_pos_vert` = 37 ft (605),
/// `vtail_sweep` = 45 deg (603). The span used here is one fin's
/// (root-to-tip), so `aspect_ratio` uses `2*span` as a mirror-image
/// convention would (a single vertical fin's effective AR for the
/// lift-curve slope, e.g. USAF DATCOM's half-span treatment).
pub fn vtail() -> Tail {
    let area_m2 = 2800.4 * FT_TO_M * FT_TO_M;
    let span_m = 47.87 * FT_TO_M;
    Tail {
        area_m2,
        span_m,
        aspect_ratio: (2.0 * span_m) * (2.0 * span_m) / area_m2,
        sweep_rad: 45.0 * DEG_TO_RAD,
        position_m: msfs_point(-90.5, 0.0, 37.0),
    }
}

/// The aerodynamic centre of one wing half, body frame, metres --
/// `aerodynamics.rs`'s strip-theory model needs left/right halves
/// separately (aileron/spoiler differential, roll damping from the
/// span-wise arm times roll rate). **GENERIC** construction (the cfg has
/// no per-half aerodynamic-centre data): spanwise station at
/// `4/(3*pi)` of the semispan, the classic Prandtl elliptical-lift-
/// distribution centroid; longitudinal position is the wing root leading
/// edge (`point.20`, z=51.91 x=16.86 ft) swept aft by that station's
/// `tan(sweep)` and moved to the quarter chord; vertical position is the
/// root's height (`point.20` y=0.96 ft) raised by that station's
/// `tan(dihedral)`.
pub fn wing_half_center_m(right_side: bool) -> Vec3 {
    let w = wing();
    let semispan_ft = w.span_m / FT_TO_M / 2.0;
    let station_ft = semispan_ft * 4.0 / (3.0 * std::f64::consts::PI);
    let root_le_z_ft = 51.91;
    let root_le_x_ft = 16.86;
    let root_y_ft = 0.96;
    let quarter_chord_ft = 0.25 * w.root_chord_m / FT_TO_M;
    let z_ft = root_le_z_ft - quarter_chord_ft - station_ft * w.sweep_rad.tan();
    let y_ft = root_y_ft + station_ft * w.dihedral_rad.tan();
    let x_ft = if right_side { station_ft } else { -station_ft };
    msfs_point(z_ft, x_ft, y_ft)
}

/// One horizontal-tail half's aerodynamic centre, body frame, metres --
/// **GENERIC**: the cfg only publishes the whole tail's centreline
/// position (`htail_pos_lon`/`htail_pos_vert`); each half is offset
/// laterally by the same elliptical-centroid station used for the wing
/// (`4/(3*pi)` of the htail semispan), longitudinal/vertical position
/// unchanged (no sweep/dihedral breakdown published for the tail).
pub fn htail_half_center_m(right_side: bool) -> Vec3 {
    let t = htail();
    let semispan_ft = t.span_m / FT_TO_M / 2.0;
    let station_ft = semispan_ft * 4.0 / (3.0 * std::f64::consts::PI);
    let x_ft = if right_side { station_ft } else { -station_ft };
    msfs_point(-105.0, x_ft, 17.5)
}

/// Control surface areas and limits (`[flight_tuning]`/aerodynamic
/// deflection section): `elevator_area` 500 sqft (610), `aileron_area`
/// 258 sqft (611), `rudder_area` 416.6 sqft (612); limits all +/-30 deg
/// (613-617); `elevator_effectiveness` 0.70 (752), `aileron_effectiveness`
/// 1.8 (754), `rudder_effectiveness` 0.2 (755) -- cfg's own scalars on how
/// much of the flap-effectiveness theory value each surface achieves.
#[derive(Clone, Copy, Debug)]
pub struct ControlSurfaceGeometry {
    pub area_m2: f64,
    pub limit_rad: f64,
    pub effectiveness: f64,
}
pub fn elevator_geometry() -> ControlSurfaceGeometry {
    ControlSurfaceGeometry { area_m2: 500.0 * FT_TO_M * FT_TO_M, limit_rad: 30.0 * DEG_TO_RAD, effectiveness: 0.70 }
}
pub fn aileron_geometry() -> ControlSurfaceGeometry {
    ControlSurfaceGeometry { area_m2: 258.0 * FT_TO_M * FT_TO_M, limit_rad: 30.0 * DEG_TO_RAD, effectiveness: 1.8 }
}
pub fn rudder_geometry() -> ControlSurfaceGeometry {
    ControlSurfaceGeometry { area_m2: 416.6 * FT_TO_M * FT_TO_M, limit_rad: 30.0 * DEG_TO_RAD, effectiveness: 0.2 }
}

/// `full_flaps_stall_speed` = 115 KTAS (785), `flaps_up_stall_speed` = 171
/// KTAS (786): used by `aerodynamics.rs` to size `CL_MAX` at 1g, MTOW,
/// sea level, from `v_stall = sqrt(2*W / (rho*S*CL_max))`.
pub const FLAPS_UP_STALL_KT: f64 = 171.0;
pub const FULL_FLAPS_STALL_KT: f64 = 115.0;
/// `max_mach` (MMO), line 790.
pub const MMO: f64 = 0.97;

// ---------------------------------------------------------------------
// Landing gear (`[CONTACT_POINTS]`).
// ---------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GearLeg {
    Nose,
    WingLeft,
    WingRight,
    BodyLeft,
    BodyRight,
}
pub const GEAR_LEGS: [GearLeg; 5] = [GearLeg::Nose, GearLeg::WingLeft, GearLeg::WingRight, GearLeg::BodyLeft, GearLeg::BodyRight];

/// `point.0`..`point.4`: nose (z=99.15,x=0,y=-15.08), wing LH/RH
/// (z=-5.7,x=+/-11.5,y=-15.75), body LH/RH (z=5.7,x=+/-23.0,y=-15.63) --
/// the A380's five-leg gear (one nose, two wing, two body), each with its
/// own steerable/braked/bogie behaviour in `landing_gear.rs`.
pub fn gear_position_m(leg: GearLeg) -> Vec3 {
    match leg {
        GearLeg::Nose => msfs_point(99.15, 0.0, -15.08),
        GearLeg::WingLeft => msfs_point(-5.7, -11.5, -15.75),
        GearLeg::WingRight => msfs_point(-5.7, 11.5, -15.75),
        GearLeg::BodyLeft => msfs_point(5.7, -23.0, -15.63),
        GearLeg::BodyRight => msfs_point(5.7, 23.0, -15.63),
    }
}

/// `point.17`, "Body tailstrike location": z=-72.402222, x=0, y=0.002656 --
/// essentially on the fuselage centreline, aft, used by `landing_gear.rs`
/// to detect and react a tail strike on rotation/derotation.
pub fn tailstrike_point_m() -> Vec3 {
    msfs_point(-72.402222, 0.0, 0.002656)
}

// ---------------------------------------------------------------------
// Engines.
// ---------------------------------------------------------------------

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Engine {
    /// Engine 1, outboard left.
    One,
    /// Engine 2, inboard left.
    Two,
    /// Engine 3, inboard right.
    Three,
    /// Engine 4, outboard right.
    Four,
}
pub const ENGINES: [Engine; 4] = [Engine::One, Engine::Two, Engine::Three, Engine::Four];

/// Engine mount positions, body frame, metres. The cfg has no dedicated
/// "engine position" key (MSFS engine thrust lines come from the 3-D
/// model's attach node, not `flight_model.cfg`), so this is built from the
/// two `[CONTACT_POINTS]` that *are* engine-labelled:
/// - `point.9`/`point.10`, "#2 Engine Pod"/"#2 Engine cowl lip":
///   z=33.315/41.816, x=-48.682/-48.665, y=-10.257/-4.113 (inner-left).
/// - `point.11`/`point.12`, "#3 Engine Pod"/"#3 Engine cowl lip":
///   z=34.690/43.821, x=48.536/49.040, y=-10.586/-3.868 (inner-right,
///   mirrored onto engine 2's numbers below rather than kept asymmetric --
///   the two differ only by modelling noise in the source file, and the
///   airframe is symmetric).
/// - Engines 1/4 (outboard) have no dedicated contact point; `point.5`/
///   `point.6` ("Engine N max upflex position", x=-84/84, z=4, y=-3) mark
///   a *wing-flex* reference, not the engine mount, so instead this
///   **GENERIC**-labelled position is a straight-line extrapolation, in
///   (x, z) and (x, y), from the wing root leading edge (`point.20`,
///   x=16.86, z=51.91, y=0.96) through the inner-engine position above,
///   continued out to the outboard engines' known lateral station
///   (x=+/-84 ft, from `point.5`/`point.6`, which *is* trustworthy for `x`
///   since it is on the correct rib). This keeps the sweep-driven aft
///   shift and the pylon's below-wing drop consistent with the inner
///   engine instead of reusing an unrelated point's z/y.
pub fn engine_position_m(engine: Engine) -> Vec3 {
    // Average of the pod and cowl-lip contact points for the inner engine,
    // in MSFS feet, symmetrized left/right.
    let inner_z_ft = ((33.315_075 + 41.815_704) + (34.689_693 + 43.820_728)) / 4.0;
    let inner_x_ft = 48.7;
    let inner_y_ft = ((-10.257_178 - 4.112_969) + (-10.585_604 - 3.867_688)) / 4.0;
    let root_z_ft = 51.91;
    let root_x_ft = 16.86;
    let root_y_ft = 0.96;
    let outer_x_ft = 84.0;
    let slope_z = (inner_z_ft - root_z_ft) / (inner_x_ft - root_x_ft);
    let slope_y = (inner_y_ft - root_y_ft) / (inner_x_ft - root_x_ft);
    let outer_z_ft = root_z_ft + slope_z * (outer_x_ft - root_x_ft);
    let outer_y_ft = root_y_ft + slope_y * (outer_x_ft - root_x_ft);
    match engine {
        Engine::One => msfs_point(outer_z_ft, -outer_x_ft, outer_y_ft),
        Engine::Two => msfs_point(inner_z_ft, -inner_x_ft, inner_y_ft),
        Engine::Three => msfs_point(inner_z_ft, inner_x_ft, inner_y_ft),
        Engine::Four => msfs_point(outer_z_ft, outer_x_ft, outer_y_ft),
    }
}

/// `+1` for the two right-side engines (3, 4), `-1` for the two left-side
/// engines (1, 2) -- the sign an engine-out yaw test checks.
pub fn engine_side_sign(engine: Engine) -> f64 {
    match engine {
        Engine::One | Engine::Two => -1.0,
        Engine::Three | Engine::Four => 1.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_mass_and_cg_match_weight_balance_rs_own_verified_numbers() {
        // src/weight_balance.rs `the_cfg_balance_is_read_whole`: 661_403 lb
        // at (16, 0, 2.8) ft.
        assert!((EMPTY_MASS_KG - 661_403.0 * LB_TO_KG).abs() < 1e-6);
        let cg = empty_cg_m();
        assert!((cg.x - 16.0 * FT_TO_M).abs() < 1e-9);
        assert!((cg.z - (-2.8 * FT_TO_M)).abs() < 1e-9);
    }

    #[test]
    fn msfs_point_matches_the_documented_axis_flip() {
        // Nose gear: far forward (+z msfs) and below the datum (-y msfs).
        let nose = gear_position_m(GearLeg::Nose);
        assert!(nose.x > 0.0, "forward in body +x");
        assert!(nose.z > 0.0, "below the datum is body +z (down)");
        // Tailstrike point is aft: body -x.
        assert!(tailstrike_point_m().x < 0.0);
    }

    #[test]
    fn wing_tip_span_matches_the_cfg_wingspan() {
        // point.7/point.8 ("wing tip max up flex") sit at x=+/-130 ft;
        // half of `wing_span` (261.65 ft) is 130.825 ft -- a sanity cross
        // check that the cfg's own numbers are mutually consistent.
        let half_span_ft = wing().span_m / FT_TO_M / 2.0;
        assert!((half_span_ft - 130.825).abs() < 0.2);
    }

    #[test]
    fn engines_are_left_right_symmetric_and_correctly_signed() {
        let e1 = engine_position_m(Engine::One);
        let e4 = engine_position_m(Engine::Four);
        assert!((e1.x - e4.x).abs() < 1e-9, "same longitudinal station");
        assert!((e1.y + e4.y).abs() < 1e-9, "mirrored laterally");
        assert!((e1.z - e4.z).abs() < 1e-9, "same drop below the wing");
        assert_eq!(engine_side_sign(Engine::One), -1.0);
        assert_eq!(engine_side_sign(Engine::Four), 1.0);
    }

    #[test]
    fn wing_and_tail_halves_are_mirrored_left_right() {
        let (l, r) = (wing_half_center_m(false), wing_half_center_m(true));
        assert!((l.x - r.x).abs() < 1e-9 && (l.z - r.z).abs() < 1e-9);
        assert!((l.y + r.y).abs() < 1e-9 && l.y < 0.0, "left half should be at negative (left) y");
        let (tl, tr) = (htail_half_center_m(false), htail_half_center_m(true));
        assert!((tl.y + tr.y).abs() < 1e-9);
    }

    #[test]
    fn no_nan_anywhere_in_the_static_tables() {
        let all = [
            empty_cg_m(),
            htail().position_m,
            vtail().position_m,
            tailstrike_point_m(),
            engine_position_m(Engine::One),
            engine_position_m(Engine::Two),
        ];
        for v in all {
            assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite());
        }
    }
}
