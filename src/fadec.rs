//! FlyByWire's A380X engine control (FADEC), ported from their C++.
//!
//! Source: `fbw-a380x/src/wasm/fadec_a380x/src/Fadec/` (EngineControl_A380X,
//! Polynomials_A380X, Table1502_A380X, ThrustLimits_A380X) and
//! `fbw-common/src/wasm/fadec_common/src/` (EngineRatios, Fadec). The maths
//! is translated line for line; only where the numbers come from and go to
//! has changed.
//!
//! Unlike the original, FlyByWire's engine control is the authority on
//! thrust here, not a reader of whatever the simulator's own engine decided.
//! In MSFS, EngineControl_A380X read N1/N2/thrust back from the sim's
//! generic turbine model and only dressed it up as A380 numbers. X-Plane's
//! own generic turbofan curve is just as disconnected from the Trent
//! 972B-84, so it is not trustworthy as a thrust source either. Instead:
//! - `engine_commands.rs`'s FADEC/EEC step (ported Simulink, the actual
//!   `A380FadecComputer`) computes the commanded corrected N1 every tick
//!   (`o.N1_c_percent`); `physics::engine::Engine` (a component-level
//!   thermodynamic gas-turbine model of the package's own Trent 972B-84,
//!   `docs/physics/engine.md`) is the physical engine that responds to it,
//!   and `engine_commands.rs` closes a bounded loop on
//!   `sim/flightmodel/engine/POINT_thrust` by trimming the throttle X-Plane
//!   sees, so X-Plane's own per-engine physics (thrust vector geometry,
//!   moments, ground effect, gyroscopics) keeps running but converges on
//!   the physics model's thrust instead of its own generic curve.
//! - The alternative, `sim/operation/override/override_engines`/
//!   `override_engine_forces`, was rejected: DataRefs.txt describes both as
//!   replacing the aircraft's whole propulsive force and moment
//!   (`fside/fnrml/faxil_prop`, `L/M/N_prop`), not each engine's, so taking
//!   either would mean this plugin computing every engine's own force and
//!   moment geometry (including asymmetric-thrust yaw) itself from
//!   `POINT_XYZ`, with its sign conventions unverifiable without X-Plane
//!   running — see the engine workstream's report. The throttle-trim loop
//!   reaches the same physical result by steering X-Plane's own, already
//!   correct, per-engine model instead.
//! - N1, N3, EGT, fuel flow and oil are computed twice each tick in this
//!   plugin: once here, by FlyByWire's own `EngineControl_A380X` polynomials
//!   (ported verbatim below, reading whatever X-Plane's own generic engine
//!   is doing as their input), and then again, physically, by
//!   `physics::engine::Engine` inside `engine_commands.rs`, which runs
//!   later in the tick and overwrites the same simulator variables (see
//!   that module's docs). The polynomials below are kept because
//!   `next_state`'s discrete state machine, `update_thrust_limits`'s
//!   EASA-cited N1 limit schedule, `generate_idle_parameters`'s Table1502
//!   idle schedule and fuel-used integration are all still authoritative
//!   from here; only the physical *quantities* (spool speed, EGT, fuel
//!   flow, oil) are superseded.
//!
//! What differs from MSFS, and why:
//! - The MSFS engine has two spools, so FlyByWire reads its N2 as the A380's
//!   N3. X-Plane's engine also has two, and is read the same way.
//! - The engine master switches and start selector are MSFS simulator
//!   variables (GENERAL ENG STARTER:n, TURB ENG IGNITION SWITCH EX1:n) that
//!   the cockpit sets; here they are the same variables, published as
//!   datarefs the converted cockpit writes. X-Plane's engine is then made to
//!   answer them the way MSFS's does: the master opens and cuts the fuel,
//!   and the starter turns while FlyByWire runs its start on IGN START.
//! - Corrected N1 and N2 are X-Plane's N1 and N2 corrected by total
//!   temperature, as the MSFS simvars are.
//! - The thrust lever angles come from FlyByWire's throttle axis mapping
//!   (see `throttle.rs`).
//! - Quick mode skips the artificial delay before the starter engages
//!   (`engine_start`), instead of forcing X-Plane's engine straight to idle
//!   the way the original forces MSFS's: unlike MSFS, X-Plane's engine here
//!   is owned by `engine_commands.rs`'s physical engine, which would just
//!   overwrite a forced value with its own genuine spool state next tick, so
//!   an outright jump can only ever be cosmetic and wrong.

use std::ffi::c_int;

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::xp::{DataRef, Xplm};
use crate::Vars;

const LBS_TO_KGS: f64 = 0.4535934;
pub(crate) const N_TO_LBF: f64 = 1. / 4.4482216153;
pub(crate) const INHG_TO_HPA: f64 = 33.8639;
const MAX_OIL: u64 = 200;
const MIN_OIL: u64 = 170;
const MAX_OIL_TEMP: f64 = 85.;
const TRANSITION_WAIT_TIME: f64 = 10.;

/// Temperature and pressure ratios (EngineRatios.hpp).
pub mod ratios {
    pub fn theta(ambient_temp: f64) -> f64 {
        (273.15 + ambient_temp) / 288.15
    }

    pub fn delta(ambient_pressure: f64) -> f64 {
        ambient_pressure / 1013.0
    }

    pub fn theta2(mach: f64, ambient_temp: f64) -> f64 {
        theta(ambient_temp) * (1. + 0.2 * mach.powi(2))
    }

    pub fn delta2(mach: f64, ambient_pressure: f64) -> f64 {
        delta(ambient_pressure) * (1. + 0.2 * mach.powi(2)).powf(3.5)
    }
}

/// Linear interpolation, clamped at both ends (Fadec::interpolate).
pub fn interpolate(x: f64, x0: f64, x1: f64, y0: f64, y1: f64) -> f64 {
    if x0 == x1 {
        return y0;
    }
    if x < x0 {
        return y0;
    }
    if x > x1 {
        return y1;
    }
    ((y0 * (x1 - x)) + (y1 * (x - x0))) / (x1 - x0)
}

/// Calibrated airspeed in knots to Mach at this pressure (Fadec::cas2mach).
pub fn cas2mach(cas: f64, ambient_pressure: f64) -> f64 {
    let k = 2188648.141;
    let delta = ambient_pressure / 1013.;
    ((5. * ((((cas.powi(2) / k) + 1.).powf(3.5) * (1. / delta)) - (1. / delta) + 1.).powf(0.285714286)) - 5.).sqrt()
}

/// The regression polynomials (Polynomials_A380X.hpp).
pub mod polynomial {
    fn series(coefficients: &[f64], x: f64) -> f64 {
        coefficients.iter().enumerate().map(|(i, c)| c * x.powi(i as i32)).sum()
    }

    pub fn start_n3(current_sim_n3: f64, previous_n3: f64, idle_n3: f64) -> f64 {
        let normalized = current_sim_n3 * 60.0 / idle_n3;
        const C: [f64; 16] = [
            4.03649879e+00,
            -9.41981960e-01,
            1.98426614e-01,
            -2.11907840e-02,
            1.00777507e-03,
            -1.57319166e-06,
            -2.15034888e-06,
            1.08288379e-07,
            -2.48504632e-09,
            2.52307089e-11,
            -2.06869243e-14,
            8.99045761e-16,
            -9.94853959e-17,
            1.85366499e-18,
            -1.44869928e-20,
            4.31033031e-23,
        ];
        let mut out = series(&C, normalized) * current_sim_n3;
        if out < previous_n3 {
            out = previous_n3 + 0.002;
        }
        if out >= idle_n3 + 0.1 {
            out = idle_n3 + 0.05;
        }
        out
    }

    pub fn start_n1(fbw_n3: f64, idle_n3: f64, idle_n1: f64) -> f64 {
        let normalized = fbw_n3 / idle_n3;
        const C: [f64; 9] = [
            -2.2812156e-12,
            -5.9830374e+01,
            7.0629094e+02,
            -3.4580361e+03,
            9.1428923e+03,
            -1.4097740e+04,
            1.2704110e+04,
            -6.2099935e+03,
            1.2733071e+03,
        ];
        let pre = (-2.4698087 * normalized.powi(3)) + (0.9662026 * normalized.powi(2)) + (0.0701367 * normalized);
        let post = series(&C, normalized);
        if post >= pre {
            post * idle_n1
        } else {
            pre * idle_n1
        }
    }

    pub fn start_ff(fbw_n3: f64, idle_n3: f64, idle_ff: f64) -> f64 {
        let normalized = fbw_n3 / idle_n3;
        const C: [f64; 9] = [
            3.1110282e-12,
            1.0804331e+02,
            -1.3972629e+03,
            7.4874131e+03,
            -2.1511983e+04,
            3.5957757e+04,
            -3.5093994e+04,
            1.8573033e+04,
            -4.1220062e+03,
        ];
        let mut ff = if normalized > 0.37 { series(&C, normalized) } else { 0. };
        if ff < 0. {
            ff = 0.;
        }
        ff * idle_ff
    }

    pub fn start_egt(fbw_n3: f64, idle_n3: f64, ambient_temp: f64, idle_egt: f64) -> f64 {
        let normalized = fbw_n3 / idle_n3;
        let egt = if normalized < 0.17 {
            0.
        } else if normalized <= 0.4 {
            (0.04783 * normalized) - 0.00813
        } else {
            const C: [f64; 9] = [
                -6.8725167e+02,
                7.7548864e+03,
                -3.7507098e+04,
                1.0147016e+05,
                -1.6779273e+05,
                1.7357157e+05,
                -1.0960924e+05,
                3.8591956e+04,
                -5.7912600e+03,
            ];
            series(&C, normalized)
        };
        (egt * (idle_egt - ambient_temp)) + ambient_temp
    }

    pub fn start_oil_temp(fbw_n3: f64, idle_n3: f64, ambient_temp: f64) -> f64 {
        if fbw_n3 < 0.79 * idle_n3 {
            return ambient_temp;
        }
        if fbw_n3 < 0.98 * idle_n3 {
            return ambient_temp + 5.;
        }
        ambient_temp + 10.
    }

    pub fn shutdown_n3(previous_n3: f64, delta_time: f64) -> f64 {
        let decay = if previous_n3 < 30. { -0.0515 } else { -0.08183 };
        previous_n3 * (decay * delta_time).exp()
    }

    pub fn shutdown_n1(previous_n1: f64, delta_time: f64) -> f64 {
        let decay = if previous_n1 < 4. { -0.08 } else { -0.164 };
        previous_n1 * (decay * delta_time).exp()
    }

    pub fn shutdown_egt(previous_egt: f64, ambient_temp: f64, delta_time: f64) -> f64 {
        let threshold = ambient_temp + 140.;
        let (decay, steady) = if previous_egt > threshold {
            (0.0257743, 135. + ambient_temp)
        } else {
            (0.00072756, 30. + ambient_temp)
        };
        steady + (previous_egt - steady) * (-decay * delta_time).exp()
    }

    pub fn corrected_egt(cn1: f64, cff: f64, mach: f64, alt: f64) -> f64 {
        // FlyByWire divides by three to allow for the A380's doubled fuel flow.
        let cff = cff / 3.;
        const C: [f64; 16] = [
            3.2636e+02,
            0.0000e+00,
            9.2893e-01,
            3.9505e-02,
            3.9070e+02,
            -4.7911e-04,
            7.7679e-03,
            5.8361e-05,
            -2.5566e+00,
            5.1227e-06,
            1.0178e-07,
            -7.4602e-03,
            1.2106e-07,
            -5.1639e+01,
            -2.7356e-03,
            1.9312e-08,
        ];
        C[0] + C[1]
            + (C[2] * cn1)
            + (C[3] * cff)
            + (C[4] * mach)
            + (C[5] * alt)
            + (C[6] * cn1.powi(2))
            + (C[7] * cn1 * cff)
            + (C[8] * cn1 * mach)
            + (C[9] * cn1 * alt)
            + (C[10] * cff.powi(2))
            + (C[11] * mach * cff)
            + (C[12] * cff * alt)
            + (C[13] * mach.powi(2))
            + (C[14] * mach * alt)
            + (C[15] * alt.powi(2))
    }

    pub fn corrected_fuel_flow(cn1: f64, mach: f64, alt: f64) -> f64 {
        const C: [f64; 21] = [
            -1.7630e+02,
            -2.1542e-01,
            4.7119e+01,
            6.1519e+02,
            1.8047e-03,
            -4.4554e-01,
            -4.3940e+01,
            4.0459e-05,
            -3.2912e+01,
            -6.2894e-03,
            -1.2544e-07,
            1.0938e-02,
            4.0936e-01,
            -5.5841e-06,
            -2.3829e+01,
            9.3269e-04,
            2.0273e-11,
            -2.4100e+02,
            1.4171e-02,
            -9.5581e-07,
            1.2728e-11,
        ];
        let out = C[0] + C[1]
            + (C[2] * cn1)
            + (C[3] * mach)
            + (C[4] * alt)
            + (C[5] * cn1.powi(2))
            + (C[6] * cn1 * mach)
            + (C[7] * cn1 * alt)
            + (C[8] * mach.powi(2))
            + (C[9] * mach * alt)
            + (C[10] * alt.powi(2))
            + (C[11] * cn1.powi(3))
            + (C[12] * cn1.powi(2) * mach)
            + (C[13] * cn1.powi(2) * alt)
            + (C[14] * cn1 * mach.powi(2))
            + (C[15] * cn1 * mach * alt)
            + (C[16] * cn1 * alt.powi(2))
            + (C[17] * mach.powi(3))
            + (C[18] * mach.powi(2) * alt)
            + (C[19] * mach * alt.powi(2))
            + (C[20] * alt.powi(3));
        // FlyByWire's allowance for the A380's doubled fuel flow.
        2.8 * out
    }

    pub fn oil_temperature(thermal_energy: f64, previous_oil_temp: f64, max_oil_temp: f64, delta_time: f64) -> f64 {
        let k = 0.0001;
        let dt = thermal_energy * delta_time * 0.02;
        let steady = ((max_oil_temp * k * delta_time) + previous_oil_temp) / (1. + (k * delta_time));
        if steady + dt >= max_oil_temp {
            max_oil_temp
        } else if steady + dt >= max_oil_temp - 10. {
            (steady + dt) * 0.999997
        } else {
            steady + dt
        }
    }

    pub fn oil_gulp_pct(thrust_newtons: f64) -> f64 {
        let c = [20.1968848, -1.2270302e-6, 1.78442e-10];
        (c[0] + (c[1] * thrust_newtons) + (c[2] * thrust_newtons.powi(2))) / 100.
    }
}

/// Idle corrected spool speeds (Table1502_A380X.hpp).
pub mod table1502 {
    use super::interpolate;

    const TABLE: [[f64; 4]; 13] = [
        [16.012, 0.000, 0.000, 17.000],
        [19.355, 1.6253, 1.6253, 17.345],
        [22.874, 2.1385, 2.1385, 18.127],
        [50.147, 10.949, 10.949, 26.627],
        [60.000, 16.299, 16.299, 33.728],
        [67.742, 22.240, 22.240, 40.082],
        [73.021, 26.877, 26.877, 43.854],
        [78.299, 35.047, 35.047, 48.899],
        [81.642, 43.625, 43.625, 53.557],
        [85.337, 63.107, 63.107, 63.107],
        [87.977, 74.757, 74.757, 74.757],
        [97.800, 97.200, 97.200, 97.200],
        [118.000, 115.347, 115.347, 115.347],
    ];

    pub fn icn3(pressure_altitude: f64, mach: f64) -> f64 {
        63. / (((288.15 - (1.98 * pressure_altitude / 1000.)) / 288.15).sqrt() * (1. + (0.2 * mach.powi(2))).sqrt())
    }

    pub fn icn1(pressure_altitude: f64, mach: f64, _ambient_temp: f64) -> f64 {
        let cn3 = icn3(pressure_altitude, mach);
        let mut i = 0;
        while i < 13 && TABLE[i][0] <= cn3 {
            i += 1;
        }
        // The original reads past either end of the table when the idle
        // speed falls outside it; keep inside the table instead.
        let i = i.clamp(1, 12);
        let (lo, hi) = (TABLE[i - 1], TABLE[i]);
        let cn1_lo = interpolate(cn3, lo[0], hi[0], lo[1], hi[1]);
        let cn1_hi = interpolate(cn3, lo[0], hi[0], lo[3], hi[3]);
        interpolate(mach, 0.2, 0.9, cn1_lo, cn1_hi)
    }
}

/// FlyByWire's actual (uncorrected) idle N1 and N3 at this altitude, Mach
/// and ambient temperature (deg C): Table1502's referred/ISA-corrected idle
/// speeds scaled to real spool speed by `ratios::theta`/`theta2`, exactly as
/// `Fadec::generate_idle_parameters` computes the `ENGINE_IDLE_N1`/`_N3`
/// Vars that `next_state`'s Starting/Restarting -> On gate compares real N3
/// against below. Shared with `engine_commands.rs`'s quick-mode snap so a
/// quick-started engine lands on the same idle the state machine is
/// actually waiting for, not the ISA-referred table value one
/// `theta.sqrt()` short of it on a non-ISA day.
pub fn idle_n1_n3(pressure_altitude: f64, mach: f64, ambient_temp: f64) -> (f64, f64) {
    let idle_cn1 = table1502::icn1(pressure_altitude, mach, ambient_temp);
    let idle_n1 = idle_cn1 * ratios::theta2(0., ambient_temp).sqrt();
    let idle_n3 = table1502::icn3(pressure_altitude, mach) * ratios::theta(ambient_temp).sqrt();
    (idle_n1, idle_n3)
}

/// N1 limits for take-off, go-around, climb and continuous thrust
/// (ThrustLimits_A380X.hpp).
pub mod thrust_limits {
    use super::{cas2mach, interpolate, ratios};

    /// Altitude, corner point, limit point, CN1 flat, CN1 last, CN1 flex.
    const LIMITS: [[f64; 6]; 72] = [
        // TO
        [-2000., 48.000, 55.000, 81.351, 79.370, 61.535],
        [-1000., 46.000, 55.000, 82.605, 80.120, 62.105],
        [0., 44.000, 55.000, 83.832, 80.776, 62.655],
        [500., 42.000, 52.000, 84.210, 81.618, 62.655],
        [1000., 42.000, 52.000, 84.579, 81.712, 62.655],
        [2000., 40.000, 50.000, 85.594, 82.720, 62.655],
        [3000., 36.000, 48.000, 86.657, 83.167, 61.960],
        [4000., 32.000, 46.000, 87.452, 83.332, 61.206],
        [5000., 29.000, 44.000, 88.833, 84.166, 61.206],
        [6000., 25.000, 42.000, 90.232, 84.815, 61.206],
        [7000., 21.000, 40.000, 91.711, 85.565, 61.258],
        [8000., 17.000, 38.000, 93.247, 86.225, 61.777],
        [9000., 15.000, 36.000, 94.031, 86.889, 60.968],
        [10000., 13.000, 34.000, 94.957, 88.044, 60.935],
        [11000., 12.000, 32.000, 95.295, 88.526, 59.955],
        [12000., 11.000, 30.000, 95.568, 88.818, 58.677],
        [13000., 10.000, 28.000, 95.355, 88.819, 59.323],
        [14000., 10.000, 26.000, 95.372, 89.311, 59.965],
        [15000., 8.000, 24.000, 95.686, 89.907, 58.723],
        [16000., 5.000, 22.000, 96.160, 89.816, 57.189],
        [16600., 5.000, 22.000, 96.560, 89.816, 57.189],
        // GA
        [-2000., 47.751, 54.681, 84.117, 81.901, 63.498],
        [-1000., 45.771, 54.681, 85.255, 82.461, 63.920],
        [0., 43.791, 54.681, 86.411, 83.021, 64.397],
        [500., 42.801, 52.701, 86.978, 83.740, 64.401],
        [1000., 41.811, 52.701, 87.568, 83.928, 64.525],
        [2000., 38.841, 50.721, 88.753, 84.935, 64.489],
        [3000., 36.861, 48.741, 89.930, 85.290, 63.364],
        [4000., 32.901, 46.761, 91.004, 85.836, 62.875],
        [5000., 28.941, 44.781, 92.198, 86.293, 62.614],
        [6000., 24.981, 42.801, 93.253, 86.563, 62.290],
        [7000., 21.022, 40.821, 94.273, 86.835, 61.952],
        [8000., 17.062, 38.841, 94.919, 87.301, 62.714],
        [9000., 15.082, 36.861, 95.365, 87.676, 61.692],
        [10000., 13.102, 34.881, 95.914, 88.150, 60.906],
        [11000., 12.112, 32.901, 96.392, 88.627, 59.770],
        [12000., 11.122, 30.921, 96.640, 89.206, 58.933],
        [13000., 10.132, 28.941, 96.516, 89.789, 60.503],
        [14000., 9.142, 26.961, 96.516, 90.475, 62.072],
        [15000., 9.142, 24.981, 96.623, 90.677, 59.333],
        [16000., 7.162, 23.001, 96.845, 90.783, 58.045],
        [16600., 5.182, 21.022, 97.366, 91.384, 58.642],
        // CLB
        [-2000., 30.800, 56.870, 80.280, 72.000, 0.000],
        [2000., 20.990, 48.157, 82.580, 74.159, 0.000],
        [5000., 16.139, 43.216, 84.642, 75.737, 0.000],
        [8000., 7.342, 38.170, 86.835, 77.338, 0.000],
        [10000., 4.051, 34.518, 88.183, 77.999, 0.000],
        [10000.1, 4.051, 34.518, 87.453, 77.353, 0.000],
        [12000., 0.760, 30.865, 88.303, 78.660, 0.000],
        [15000., -4.859, 25.039, 89.748, 79.816, 0.000],
        [17000., -9.934, 19.813, 90.668, 80.895, 0.000],
        [20000., -15.822, 13.676, 92.106, 81.894, 0.000],
        [24000., -22.750, 6.371, 94.588, 83.543, 0.000],
        [27000., -29.105, -0.304, 96.203, 85.358, 0.000],
        [29314., -32.049, -3.377, 96.820, 85.906, 0.000],
        [31000., -34.980, -6.452, 98.568, 86.909, 0.000],
        [35000., -45.679, -17.150, 100.977, 89.570, 0.000],
        [39000., -45.679, -17.150, 103.085, 90.377, 0.000],
        [41500., -45.679, -17.150, 104.509, 91.476, 0.000],
        // MCT
        [-1000., 26.995, 54.356, 82.465, 74.086, 0.000],
        [3000., 18.170, 45.437, 86.271, 77.802, 0.000],
        [7000., 9.230, 40.266, 89.128, 79.604, 0.000],
        [11000., 4.019, 31.046, 92.194, 82.712, 0.000],
        [15000., -5.226, 21.649, 95.954, 85.622, 0.000],
        [17000., -9.913, 20.702, 97.520, 85.816, 0.000],
        [20000., -15.129, 15.321, 99.263, 86.770, 0.000],
        [22000., -19.947, 10.382, 98.977, 86.661, 0.000],
        [25000., -25.397, 4.731, 99.424, 86.623, 0.000],
        [27000., -30.369, -0.391, 99.730, 87.711, 0.000],
        [31000., -36.806, -7.165, 101.958, 89.534, 0.000],
        [35000., -43.628, -14.384, 103.375, 90.095, 0.000],
        [39000., -47.286, -18.508, 104.234, 91.663, 0.000],
    ];

    fn finder(altitude: f64, mut index: usize) -> usize {
        while index < LIMITS.len() - 1 && altitude >= LIMITS[index][0] {
            index += 1;
        }
        index
    }

    #[allow(clippy::too_many_arguments)]
    pub fn bleed_total(kind: usize, altitude: f64, oat: f64, cp: f64, lp: f64, flex_temp: f64, packs: f64, nacelle: f64, wing: f64) -> f64 {
        if flex_temp > lp && kind <= 1 {
            return packs * -0.6 + nacelle * -0.7 + wing * -0.7;
        }
        // The original looks these up in a map; a combination it does not
        // list reads as zero, as std::map's operator[] gives.
        let (n1_packs, n1_nai, n1_wai) = match (kind, altitude < 8000., oat < cp) {
            (0, true, true) => (-0.4, -0.6, -0.7),
            (0, true, false) => (-0.5, -0.6, -0.7),
            (0, false, true) => (-0.6, -0.8, -0.8),
            (0, false, false) => (-0.7, -0.8, -0.8),
            (1, true, _) => (-0.4, -0.6, -0.6),
            (1, false, _) => (-0.6, -0.7, -0.8),
            (2, true, false) => (-0.2, -0.8, -0.4),
            (2, false, false) => (-0.3, -0.8, -0.4),
            (3, _, false) => (-0.6, -0.9, -1.2),
            _ => (0., 0., 0.),
        };
        packs * n1_packs + nacelle * n1_nai + wing * n1_wai
    }

    /// The N1 limit: kind 0 take-off, 1 go-around, 2 climb, 3 continuous.
    #[allow(clippy::too_many_arguments)]
    pub fn limit_n1(
        kind: usize,
        altitude: f64,
        ambient_temp: f64,
        ambient_pressure: f64,
        flex_temp: f64,
        packs: f64,
        nacelle: f64,
        wing: f64,
    ) -> f64 {
        let (row_min, row_max, mach) = match kind {
            0 => (0, 20, 0.),
            1 => (21, 41, 0.225),
            2 => {
                let mach = if altitude <= 10000. {
                    cas2mach(250., ambient_pressure)
                } else {
                    cas2mach(300., ambient_pressure).min(0.78)
                };
                (42, 58, mach)
            }
            _ => (59, 71, cas2mach(230., ambient_pressure)),
        };
        let (lo, hi) = if altitude <= LIMITS[row_min][0] {
            (row_min, row_min)
        } else if altitude >= LIMITS[row_max][0] {
            (row_max, row_max)
        } else {
            let hi = finder(altitude, row_min);
            (hi - 1, hi)
        };
        let at = |col: usize| interpolate(altitude, LIMITS[lo][0], LIMITS[hi][0], LIMITS[lo][col], LIMITS[hi][col]);
        let (cp, lp, cn1_flat, cn1_last, cn1_flex) = (at(1), at(2), at(3), at(4), at(5));

        let cn1 = if flex_temp > 0. && kind <= 1 {
            if flex_temp <= cp {
                cn1_flat
            } else if flex_temp > lp {
                let m = (cn1_flex - cn1_last) / (100. - lp);
                let b = cn1_flex - m * 100.;
                (m * flex_temp) + b
            } else {
                let m = (cn1_last - cn1_flat) / (lp - cp);
                let b = cn1_last - m * lp;
                (m * flex_temp) + b
            }
        } else if ambient_temp <= cp {
            cn1_flat
        } else {
            let m = (cn1_last - cn1_flat) / (lp - cp);
            let b = cn1_last - m * lp;
            (m * ambient_temp) + b
        };

        let bleed = bleed_total(kind, altitude, ambient_temp, cp, lp, flex_temp, packs, nacelle, wing);
        (cn1 * ratios::theta2(mach, ambient_temp).sqrt()) + bleed
    }
}

// The `thrust_table` module that used to stand in here (a transcription of
// engines.cfg's `n1_and_mach_on_thrust_table`, thrust as a fraction of
// static rating scaled by ambient pressure ratio) has been replaced by the
// hyperrealism engine workstream's `physics::engine::Engine`: a real
// thermodynamic gas-turbine model (inlet, compressor/turbine maps,
// combustor energy balance, spool dynamics, nozzle thrust), stepped from
// `engine_commands.rs`. See `docs/physics/engine.md`. `engine_commands.rs`
// still closes a bounded throttle-trim loop on X-Plane's own
// `POINT_thrust`, now against that model's thrust instead of this table.

/// The engine states FlyByWire's state machine moves between.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum EngineState {
    Off = 0,
    On = 1,
    Starting = 2,
    Restarting = 3,
    Shutting = 4,
}

impl EngineState {
    // pub(crate): engine_commands.rs's physics coupling also reads the
    // state machine's current state, to know when the pneumatic starter is
    // engaged.
    pub(crate) fn from(value: f64) -> Self {
        match value as i64 {
            1 => Self::On,
            2 => Self::Starting,
            3 => Self::Restarting,
            4 => Self::Shutting,
            _ => Self::Off,
        }
    }
}

/// One step of the state machine (engineStateMachine). The ignition selector
/// is 0 crank, 1 norm, 2 ign start; `starter` is the engine master. Returns
/// the new state and whether the engine timer restarts.
pub fn next_state(state: EngineState, igniter: i32, starter: bool, sim_n3: f64, idle_n3: f64, egt: f64, ambient_temp: f64) -> (EngineState, bool) {
    use EngineState::*;
    match state {
        Off => {
            if igniter == 1 && starter && sim_n3 > 20. {
                (On, false)
            } else if igniter == 2 && starter {
                (Starting, false)
            } else {
                (Off, false)
            }
        }
        On => {
            if starter {
                (On, false)
            } else {
                (Shutting, false)
            }
        }
        Starting | Restarting => {
            if starter && sim_n3 >= idle_n3 - 0.1 {
                (On, true)
            } else if !starter {
                (Shutting, true)
            } else {
                (state, false)
            }
        }
        Shutting => {
            if igniter == 2 && starter {
                (Restarting, true)
            } else if !starter && sim_n3 < 0.05 && egt <= ambient_temp {
                (Off, true)
            } else if starter && sim_n3 > 50. {
                (Restarting, true)
            } else {
                (Shutting, false)
            }
        }
    }
}

/// A pack is drawing bleed air -- and so costs the FADEC's packs-bleed
/// thrust-limit correction (`thrust_limits::limit_n1`'s `packs` flag) --
/// whenever either of its two flow valves is open. Combines both packs into
/// the single flag `update_thrust_limits` wants, exactly like the old
/// `COND_PACK_n_IS_OPERATING != 0.` OR did, just fed from variables that are
/// actually written (see the `packs` field comment in `Fadec::new`).
pub fn pack_bleed_active(pack1_valve1: bool, pack1_valve2: bool, pack2_valve1: bool, pack2_valve2: bool) -> bool {
    pack1_valve1 || pack1_valve2 || pack2_valve1 || pack2_valve2
}

/// The variables one engine reads and writes.
struct EngineVars {
    // FlyByWire's engine readings.
    state: VariableIdentifier,
    n1: VariableIdentifier,
    n2: VariableIdentifier,
    n3: VariableIdentifier,
    egt: VariableIdentifier,
    ff: VariableIdentifier,
    fuel_used: VariableIdentifier,
    oil_qty: VariableIdentifier,
    oil_total: VariableIdentifier,
    pre_ff: VariableIdentifier,
    timer: VariableIdentifier,
    pump_state: VariableIdentifier,
    tla: VariableIdentifier,
    // The engine controls, as the cockpit sets them: the master switch
    // (MSFS's starter, toggled by TOGGLE_STARTERn) and the start selector
    // (TURBINE_IGNITION_SWITCH_SET: 0 crank, 1 norm, 2 ign start).
    master: VariableIdentifier,
    throttle_input: VariableIdentifier,
    lever_3d: VariableIdentifier,
    // MSFS's own lever-position simvar (Percent, 0-100): every EWD/SD/ND/
    // MFD/FCU/OIT/PFD bundle reads this directly, separately from
    // FlyByWire's own A32NX_3D_THROTTLE_LEVER_POSITION_n above. Nothing
    // fed it before, so it read 0 forever.
    general_eng_throttle_lever_position: VariableIdentifier,
    // The simulator's engine, as FlyByWire's systems read it.
    corrected_n1: VariableIdentifier,
    corrected_n2: VariableIdentifier,
    jet_thrust: VariableIdentifier,
    starter_active: VariableIdentifier,
    ignition: VariableIdentifier,
    anti_ice: VariableIdentifier,
    oil_temp: VariableIdentifier,
}

impl EngineVars {
    fn new(vars: &mut Vars, n: usize) -> Self {
        let mut get = |name: String| vars.get(name);
        Self {
            state: get(format!("ENGINE_STATE:{n}")),
            n1: get(format!("ENGINE_N1:{n}")),
            n2: get(format!("ENGINE_N2:{n}")),
            n3: get(format!("ENGINE_N3:{n}")),
            egt: get(format!("ENGINE_EGT:{n}")),
            ff: get(format!("ENGINE_FF:{n}")),
            fuel_used: get(format!("FUEL_USED:{n}")),
            oil_qty: get(format!("ENGINE_OIL_QTY:{n}")),
            oil_total: get(format!("ENGINE_OIL_TOTAL:{n}")),
            pre_ff: get(format!("ENGINE_PRE_FF:{n}")),
            timer: get(format!("ENGINE_TIMER:{n}")),
            pump_state: get(format!("PUMP_STATE:{n}")),
            tla: get(format!("AUTOTHRUST_TLA:{n}")),
            master: get(format!("GENERAL ENG STARTER:{n}")),
            throttle_input: get(format!("THROTTLE_MAPPING_INPUT:{n}")),
            lever_3d: get(format!("3D_THROTTLE_LEVER_POSITION_{n}")),
            general_eng_throttle_lever_position: get(format!("GENERAL ENG THROTTLE LEVER POSITION:{n}")),
            corrected_n1: get(format!("TURB ENG CORRECTED N1:{n}")),
            corrected_n2: get(format!("TURB ENG CORRECTED N2:{n}")),
            jet_thrust: get(format!("TURB ENG JET THRUST:{n}")),
            starter_active: get(format!("GENERAL ENG STARTER ACTIVE:{n}")),
            ignition: get(format!("TURB ENG IGNITION SWITCH EX1:{n}")),
            anti_ice: get(format!("ENG ANTI ICE:{n}")),
            oil_temp: get(format!("GENERAL ENG OIL TEMPERATURE:{n}")),
        }
    }
}

/// X-Plane's engine, read once per tick for all four engines.
struct XPlaneEngine {
    n1: Option<DataRef>,
    n2: Option<DataRef>,
    thrust: Option<DataRef>,
    mixture: Option<DataRef>,
    starter_running: Option<DataRef>,
    anti_ice: Option<DataRef>,
    fuel: Option<DataRef>,
    running: Option<DataRef>,
    ignition_key: Option<DataRef>,
    igniter_on: Option<DataRef>,
}

/// This tick's readings of X-Plane's engines.
#[derive(Default)]
struct Reading {
    n1: [f32; 4],
    n2: [f32; 4],
    thrust_n: [f32; 4],
    mixture: [f32; 4],
    starter: [c_int; 4],
    anti_ice: [c_int; 4],
    running: [c_int; 4],
    fuel_kg: f64,
}

/// FlyByWire's A380X engine control.
pub struct Fadec {
    engines: [EngineVars; 4],
    xp: XPlaneEngine,
    /// The start state the plugin chose (start_state.rs).
    start_state: VariableIdentifier,

    // Air data, as the systems read it.
    mach: VariableIdentifier,
    pressure_altitude: VariableIdentifier,
    ambient_temperature: VariableIdentifier,
    ambient_pressure_inhg: VariableIdentifier,
    on_ground: VariableIdentifier,

    idle_n1: VariableIdentifier,
    idle_n3: VariableIdentifier,
    idle_ff: VariableIdentifier,
    idle_egt: VariableIdentifier,

    limit_type: VariableIdentifier,
    limit_idle: VariableIdentifier,
    limit_clb: VariableIdentifier,
    limit_flx: VariableIdentifier,
    limit_mct: VariableIdentifier,
    limit_toga: VariableIdentifier,
    // Order: pack 1 flow valve 1, pack 1 flow valve 2, pack 2 flow valve 1,
    // pack 2 flow valve 2 -- see the constructor comment.
    packs: [VariableIdentifier; 4],
    wing_anti_ice: VariableIdentifier,
    flex_temp: VariableIdentifier,
    quick_mode: VariableIdentifier,

    initialized: bool,
    prev_sim_n3: [f64; 4],
    thermal_energy: [f64; 4],

    is_transition_active: bool,
    latched_flex_temperature: f64,
    prev_thrust_limit_type: f64,
    was_flex_active: bool,
    transition_start_time: f64,
}

impl Fadec {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        let engines = [
            EngineVars::new(vars, 1),
            EngineVars::new(vars, 2),
            EngineVars::new(vars, 3),
            EngineVars::new(vars, 4),
        ];
        Self {
            engines,
            xp: XPlaneEngine {
                n1: xplm.find("sim/flightmodel/engine/ENGN_N1_"),
                n2: xplm.find("sim/flightmodel/engine/ENGN_N2_"),
                thrust: xplm.find("sim/flightmodel/engine/POINT_thrust"),
                mixture: xplm.find("sim/cockpit2/engine/actuators/mixture_ratio"),
                starter_running: xplm.find("sim/flightmodel2/engines/starter_is_running"),
                anti_ice: xplm.find("sim/cockpit2/ice/ice_inlet_heat_on_per_engine"),
                fuel: xplm.find("sim/flightmodel/weight/m_fuel"),
                running: xplm.find("sim/flightmodel/engine/ENGN_running"),
                ignition_key: xplm.find("sim/cockpit2/engine/actuators/ignition_key"),
                igniter_on: xplm.find("sim/cockpit2/engine/actuators/igniter_on"),
            },
            start_state: vars.get("START_STATE".into()),
            mach: vars.get("AIRSPEED MACH".into()),
            pressure_altitude: vars.get("PRESSURE ALTITUDE".into()),
            ambient_temperature: vars.get("AMBIENT TEMPERATURE".into()),
            ambient_pressure_inhg: vars.get("AMBIENT PRESSURE".into()),
            on_ground: vars.get("SIM ON GROUND".into()),
            idle_n1: vars.get("ENGINE_IDLE_N1".into()),
            idle_n3: vars.get("ENGINE_IDLE_N3".into()),
            idle_ff: vars.get("ENGINE_IDLE_FF".into()),
            idle_egt: vars.get("ENGINE_IDLE_EGT".into()),
            limit_type: vars.get("AUTOTHRUST_THRUST_LIMIT_TYPE".into()),
            limit_idle: vars.get("AUTOTHRUST_THRUST_LIMIT_IDLE".into()),
            limit_clb: vars.get("AUTOTHRUST_THRUST_LIMIT_CLB".into()),
            limit_flx: vars.get("AUTOTHRUST_THRUST_LIMIT_FLX".into()),
            limit_mct: vars.get("AUTOTHRUST_THRUST_LIMIT_MCT".into()),
            limit_toga: vars.get("AUTOTHRUST_THRUST_LIMIT_TOGA".into()),
            // COND_PACK_n_IS_OPERATING (both FBW's MSFS C++ FADEC and this
            // plugin's own linked a380_systems FADEC use read it) is never
            // written by anything -- see FadecSimData_A380X.hpp:437-438 for
            // the matching blind read on the MSFS side, and
            // AirGenerationSystemApplication::pack_is_operating (cpiom_b.rs)
            // for why: it only feeds a private ARINC discrete bit, never a
            // plain L:var. What IS written every tick is each pack's flow
            // valve open state (PackComplex::write, fbw-a380x pneumatic.rs);
            // a pack is bleeding air, and so costing thrust, whenever either
            // of its two flow valves is open.
            packs: [
                vars.get("COND_PACK_1_FLOW_VALVE_1_IS_OPEN".into()),
                vars.get("COND_PACK_1_FLOW_VALVE_2_IS_OPEN".into()),
                vars.get("COND_PACK_2_FLOW_VALVE_1_IS_OPEN".into()),
                vars.get("COND_PACK_2_FLOW_VALVE_2_IS_OPEN".into()),
            ],
            wing_anti_ice: vars.get("PNEU_WING_ANTI_ICE_SYSTEM_ON".into()),
            flex_temp: vars.get("AIRLINER_TO_FLEX_TEMP".into()),
            quick_mode: vars.get("AIRCRAFT_PRESET_QUICK_MODE".into()),
            initialized: false,
            prev_sim_n3: [0.; 4],
            thermal_energy: [0.; 4],
            is_transition_active: false,
            latched_flex_temperature: 0.,
            prev_thrust_limit_type: 0.,
            was_flex_active: false,
            transition_start_time: 0.,
        }
    }

    fn read_xplane(&self, xplm: &Xplm) -> Reading {
        let mut r = Reading::default();
        if let Some(d) = self.xp.n1 {
            xplm.get_vf(d, &mut r.n1);
        }
        if let Some(d) = self.xp.n2 {
            xplm.get_vf(d, &mut r.n2);
        }
        if let Some(d) = self.xp.thrust {
            xplm.get_vf(d, &mut r.thrust_n);
        }
        if let Some(d) = self.xp.mixture {
            xplm.get_vf(d, &mut r.mixture);
        }
        if let Some(d) = self.xp.starter_running {
            xplm.get_vi(d, &mut r.starter);
        }
        if let Some(d) = self.xp.anti_ice {
            xplm.get_vi(d, &mut r.anti_ice);
        }
        if let Some(d) = self.xp.running {
            xplm.get_vi(d, &mut r.running);
        }
        if let Some(d) = self.xp.fuel {
            let mut tanks = [0f32; 9];
            let n = xplm.get_vf(d, &mut tanks);
            r.fuel_kg = tanks[..n].iter().map(|&t| t as f64).sum();
        }
        r
    }

    /// Run one tick: feed the simulator engine variables from X-Plane, then
    /// FlyByWire's engine control on top of them. Call before the systems
    /// tick so they see this tick's engines.
    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm, levers: &crate::throttle::Levers, delta: f64, sim_time: f64) {
        let xp = self.read_xplane(xplm);

        let mach = vars.read(&self.mach);
        let pressure_altitude = vars.read(&self.pressure_altitude);
        let ambient_temperature = vars.read(&self.ambient_temperature);
        let ambient_pressure = vars.read(&self.ambient_pressure_inhg) * INHG_TO_HPA;
        let on_ground = vars.read(&self.on_ground) != 0.;

        // The simulator engine, as MSFS would give it to FlyByWire.
        if !self.initialized {
            // A cold start on the ground (1 hangar, 2 apron) is decided before
            // X-Plane has placed the aircraft, and X-Plane may then load the
            // flight with its engines running: that must not turn the masters
            // on over cold engines. X-Plane's engines are shut down instead.
            let state = vars.read(&self.start_state);
            if state == 1. || state == 2. {
                for i in 0..4 {
                    if let Some(d) = self.xp.running {
                        xplm.set_vi_at(d, i, 0);
                    }
                    if let Some(d) = self.xp.mixture {
                        xplm.set_vf_at(d, i, 0.);
                    }
                }
            }
            let cold = state == 1. || state == 2.;
            self.initialize(vars, &xp, ambient_temperature, cold);
            self.initialized = true;
        }

        let correction = ratios::theta2(mach, ambient_temperature).sqrt();
        let mut starter = [false; 4];
        let mut igniter = [1i32; 4];
        for (i, e) in self.engines.iter().enumerate() {
            // The cockpit's master switch and start selector, as FlyByWire reads them.
            starter[i] = vars.read(&e.master) != 0.;
            igniter[i] = vars.read(&e.ignition).round() as i32;
            let (id_cn1, id_cn2, id_thrust, id_start, id_ice) =
                (e.corrected_n1, e.corrected_n2, e.jet_thrust, e.starter_active, e.anti_ice);
            vars.write_from_xplane(&id_cn1, xp.n1[i] as f64 / correction);
            vars.write_from_xplane(&id_cn2, xp.n2[i] as f64 / correction);
            vars.write_from_xplane(&id_thrust, xp.thrust_n[i] as f64 * N_TO_LBF);
            vars.write_from_xplane(&id_start, (xp.starter[i] != 0) as i32 as f64);
            vars.write_from_xplane(&id_ice, (xp.anti_ice[i] != 0) as i32 as f64);

            // The thrust levers, through FlyByWire's axis mapping.
            let (id_tla, id_input, id_lever) = (e.tla, e.throttle_input, e.lever_3d);
            vars.write(&id_tla, levers.angle[i]);
            vars.write(&id_input, levers.axis[i]);
            vars.write(&id_lever, levers.lever_3d[i]);
            // Same 0-100 scale as the 3D lever above: MSFS's own simvar the
            // cockpit bundles poll directly.
            vars.write(&e.general_eng_throttle_lever_position, levers.lever_3d[i]);
        }

        let delta_time = delta.max(0.002);
        let idle_n3 = vars.read(&self.idle_n3);
        self.generate_idle_parameters(vars, pressure_altitude, mach, ambient_temperature, ambient_pressure);

        for i in 0..4 {
            let state_now = EngineState::from(vars.read(&self.engines[i].state));
            let egt = vars.read(&self.engines[i].egt);
            let (state, reset_timer) =
                next_state(state_now, igniter[i], starter[i], self.prev_sim_n3[i], idle_n3, egt, ambient_temperature);
            let (id_state, id_timer) = (self.engines[i].state, self.engines[i].timer);
            vars.write(&id_state, state as i32 as f64);
            if reset_timer {
                vars.write(&id_timer, 0.);
            }

            let engine_timer = vars.read(&id_timer);
            let sim_cn1 = xp.n1[i] as f64 / correction;
            let sim_n1 = xp.n1[i] as f64;
            let sim_n3 = xp.n2[i] as f64;
            let delta_n3 = sim_n3 - self.prev_sim_n3[i];
            self.prev_sim_n3[i] = sim_n3;

            match state {
                EngineState::Starting | EngineState::Restarting => {
                    self.engine_start(vars, i, state, delta_time, engine_timer, sim_n3, ambient_temperature, on_ground)
                }
                EngineState::Shutting => {
                    self.engine_shutdown(vars, i, delta_time, engine_timer, sim_n1, ambient_temperature);
                    self.update_ff(vars, i, sim_cn1, mach, pressure_altitude, ambient_temperature, ambient_pressure);
                }
                _ => {
                    self.update_primary(vars, i, sim_n1, sim_n3);
                    let cff = self.update_ff(vars, i, sim_cn1, mach, pressure_altitude, ambient_temperature, ambient_pressure);
                    self.update_egt(vars, i, state, delta_time, sim_cn1, cff, mach, pressure_altitude, ambient_temperature, on_ground);
                    self.update_oil(vars, i, state, delta_time, on_ground, ambient_temperature, delta_n3, xp.thrust_n[i] as f64);
                }
            }
            self.drive_engine(vars, xplm, i, sim_n3);
        }

        self.update_fuel(vars, delta_time, xp.fuel_kg);

        let packs = pack_bleed_active(
            vars.read(&self.packs[0]) != 0.,
            vars.read(&self.packs[1]) != 0.,
            vars.read(&self.packs[2]) != 0.,
            vars.read(&self.packs[3]) != 0.,
        ) as i32 as f64;
        let nai = xp.anti_ice.iter().any(|&a| a != 0) as i32 as f64;
        let wai = vars.read(&self.wing_anti_ice).trunc();
        self.update_thrust_limits(vars, sim_time, pressure_altitude, ambient_temperature, ambient_pressure, mach, packs, nai, wai);
    }

    /// Make X-Plane's engine do what MSFS's does with the same controls: the
    /// master switch opens and cuts the engine's fuel, the starter turns
    /// while FlyByWire is starting the engine on IGN START once its start
    /// delay has run, and `ENGN_running` mirrors FlyByWire's own (now
    /// honest, see `engine_start`) On/Off state.
    ///
    /// This used to also force `ENGN_N1_`/`ENGN_N2_` to idle here under
    /// quick mode: dead code, since `engine_commands.rs`'s physical engine
    /// (`physics::engine::Engine`, module docs above) runs later the same
    /// tick and overwrites those same datarefs from its own genuine spool
    /// simulation regardless of what was written here, and it never checked
    /// for real fuel flow either. Removed; `engine_start`'s expedited
    /// starter timing is what actually gets the real engine to idle now.
    fn drive_engine(&self, vars: &mut Vars, xplm: &Xplm, i: usize, sim_n3: f64) {
        let e = &self.engines[i];
        let master = vars.read(&e.master) != 0.;
        let igniter = vars.read(&e.ignition).round() as i32;
        let state = EngineState::from(vars.read(&e.state));
        let timer = vars.read(&e.timer);
        let idle_n3 = vars.read(&self.idle_n3);

        if let Some(d) = self.xp.mixture {
            xplm.set_vf_at(d, i, if master { 1. } else { 0. });
        }

        // Nothing in `engine_commands.rs` sets `ENGN_running` (it only
        // reads it back); this is the only place that does. `state == On`
        // is only reached once the real N3 (`engine_commands.rs`'s physics
        // engine output, read back next tick as `sim_n3`) is genuinely at
        // idle -- see `next_state` and `engine_start` -- so reporting it
        // here is truthful in both quick and normal mode, not a forced
        // value. `state == Off` is likewise only reached once the real N3
        // has decayed near zero.
        if let Some(d) = self.xp.running {
            if master && state == EngineState::On {
                xplm.set_vi_at(d, i, 1);
            } else if state == EngineState::Off {
                xplm.set_vi_at(d, i, 0);
            }
        }

        let cranking = master
            && igniter == 2
            && matches!(state, EngineState::Starting | EngineState::Restarting)
            && timer >= 1.7
            && sim_n3 < idle_n3 - 0.1;
        if cranking {
            // X-Plane's key is spring loaded: it has to be held at start.
            if let Some(d) = self.xp.ignition_key {
                xplm.set_vi_at(d, i, 4);
            }
            if let Some(d) = self.xp.igniter_on {
                xplm.set_vi_at(d, i, 1);
            }
        }
    }

    fn initialize(&mut self, vars: &mut Vars, xp: &Reading, ambient_temperature: f64, cold: bool) {
        // An aircraft joined with its engines running has its masters on and
        // its selector at NORM; a cold one has them off.
        for i in 0..4 {
            let running = !cold && (xp.running[i] != 0 || (xp.mixture[i] > 0.5 && xp.n2[i] > 20.));
            let (master, ignition) = (self.engines[i].master, self.engines[i].ignition);
            vars.write(&master, running as i32 as f64);
            vars.write(&ignition, 1.);
        }
        // The original seeds C's rand with the time; any varying seed does.
        let mut seed = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(1, |d| d.as_nanos() as u64);
        let mut random_oil = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            ((seed >> 33) % (MAX_OIL - MIN_OIL + 1) + MIN_OIL) as f64 / 10.0
        };
        for i in 0..4 {
            self.prev_sim_n3[i] = xp.n2[i] as f64;
            self.thermal_energy[i] = 0.;
            let e = &self.engines[i];
            let ids = [e.egt, e.ff, e.fuel_used, e.n1, e.n2, e.oil_qty, e.pre_ff, e.timer, e.pump_state];
            let (oil_total, oil_temp, state) = (e.oil_total, e.oil_temp, e.state);
            for id in ids {
                vars.write(&id, 0.);
            }
            vars.write(&oil_total, random_oil());
            vars.write(&oil_temp, ambient_temperature);
            vars.write(&state, EngineState::Off as i32 as f64);
        }
        for id in [
            self.idle_egt,
            self.idle_ff,
            self.idle_n1,
            self.idle_n3,
            self.limit_idle,
            self.limit_clb,
            self.limit_flx,
            self.limit_mct,
            self.limit_toga,
        ] {
            vars.write(&id, 0.);
        }
    }

    fn generate_idle_parameters(&self, vars: &mut Vars, pressure_altitude: f64, mach: f64, ambient_temperature: f64, ambient_pressure: f64) {
        let idle_cn1 = table1502::icn1(pressure_altitude, mach, ambient_temperature);
        let (idle_n1, idle_n3) = idle_n1_n3(pressure_altitude, mach, ambient_temperature);
        let idle_cff = polynomial::corrected_fuel_flow(idle_cn1, 0., pressure_altitude);
        let idle_ff = idle_cff * LBS_TO_KGS * ratios::delta2(0., ambient_pressure) * ratios::theta2(0., ambient_temperature).sqrt();
        let idle_egt = polynomial::corrected_egt(idle_cn1, idle_cff, 0., pressure_altitude) * ratios::theta2(0., ambient_temperature);
        vars.write(&self.idle_n1, idle_n1);
        vars.write(&self.idle_n3, idle_n3);
        vars.write(&self.idle_ff, idle_ff);
        vars.write(&self.idle_egt, idle_egt);
    }

    #[allow(clippy::too_many_arguments)]
    fn engine_start(
        &self,
        vars: &mut Vars,
        i: usize,
        state: EngineState,
        delta_time: f64,
        engine_timer: f64,
        sim_n3: f64,
        ambient_temperature: f64,
        on_ground: bool,
    ) {
        let e = &self.engines[i];
        let idle_n3 = vars.read(&self.idle_n3);
        let idle_n1 = vars.read(&self.idle_n1);
        let idle_ff = vars.read(&self.idle_ff);
        let idle_egt = vars.read(&self.idle_egt);

        // Quick mode used to jump straight to idle N3/N1/FF/EGT and mark
        // the state On here, both purely on FlyByWire's own side. That
        // raced `engine_commands.rs`'s physical engine
        // (`physics::engine::Engine`, fadec.rs's own module docs), which
        // runs later the same tick and overwrites these same Vars from its
        // own genuine spool simulation: the jump was overwritten back down
        // before the next frame, so ENGINE_STATE read On while X-Plane's
        // real engine was still at rest with no fuel flow -- the bug this
        // ports.
        //
        // What quick mode should actually skip is the artificial delay
        // between the master switch and the starter engaging: jump the
        // timer variable straight past the 1.7s threshold
        // `engine_commands.rs`'s `starter_engaged` reads from this same
        // Var, so the real starter (and thus the real physics engine) is
        // engaged from the very next tick instead of after 1.7 real
        // seconds. `next_state` below still only moves Starting/Restarting
        // to On once X-Plane's real N3 genuinely reaches idle, so an engine
        // with no real fuel flow simply never gets there, quick mode or not.
        let quick = vars.read(&self.quick_mode) != 0.;
        if quick && engine_timer < 1.7 {
            vars.write(&e.timer, 1.7);
        } else if engine_timer < 1.7 {
            // The delay between the master switch and the engine starting.
            if on_ground {
                vars.write(&e.fuel_used, 0.);
            }
            vars.write(&e.timer, engine_timer + delta_time);
            return;
        }

        let pre_n3 = vars.read(&e.n3);
        let pre_egt = vars.read(&e.egt);
        let new_n3 = polynomial::start_n3(sim_n3, pre_n3, idle_n3);
        let start_n1 = polynomial::start_n1(new_n3, idle_n3, idle_n1);
        let start_ff = polynomial::start_ff(new_n3, idle_n3, idle_ff);
        let start_egt = polynomial::start_egt(new_n3, idle_n3, ambient_temperature, idle_egt);
        let shutdown_egt = polynomial::shutdown_egt(pre_egt, ambient_temperature, delta_time);

        vars.write(&e.n3, new_n3);
        vars.write(&e.n2, if new_n3 == 0. { 0. } else { new_n3 + 0.7 });
        vars.write(&e.n1, start_n1);
        vars.write(&e.ff, start_ff);

        if state == EngineState::Restarting {
            if (start_egt - pre_egt).abs() <= 1.5 {
                vars.write(&e.egt, start_egt);
                vars.write(&e.state, EngineState::Starting as i32 as f64);
            } else if start_egt > pre_egt {
                vars.write(&e.egt, pre_egt + (0.75 * delta_time * (idle_n3 - new_n3)));
            } else {
                vars.write(&e.egt, shutdown_egt);
            }
        } else {
            vars.write(&e.egt, start_egt);
        }
        vars.write(&e.oil_temp, polynomial::start_oil_temp(new_n3, idle_n3, ambient_temperature));
    }

    fn engine_shutdown(&self, vars: &mut Vars, i: usize, delta_time: f64, engine_timer: f64, sim_n1: f64, ambient_temperature: f64) {
        let e = &self.engines[i];
        if vars.read(&self.quick_mode) != 0. && vars.read(&e.n3) > 0. {
            for id in [e.n1, e.n2, e.n3, e.ff] {
                vars.write(&id, 0.);
            }
            vars.write(&e.egt, ambient_temperature);
            vars.write(&e.timer, 2.0);
            return;
        }
        if engine_timer < 1.8 {
            vars.write(&e.timer, engine_timer + delta_time);
            return;
        }
        let pre_n1 = vars.read(&e.n1);
        let pre_n3 = vars.read(&e.n3);
        let pre_egt = vars.read(&e.egt);
        let mut new_n1 = polynomial::shutdown_n1(pre_n1, delta_time);
        // Windmilling.
        if sim_n1 < 5. && sim_n1 > new_n1 {
            new_n1 = sim_n1;
        }
        let new_n3 = polynomial::shutdown_n3(pre_n3, delta_time);
        vars.write(&e.n1, new_n1);
        vars.write(&e.n2, if new_n3 == 0. { 0. } else { new_n3 + 0.7 });
        vars.write(&e.n3, new_n3);
        vars.write(&e.egt, polynomial::shutdown_egt(pre_egt, ambient_temperature, delta_time));
    }

    /// Fuel flow in kg/h. Returns the corrected fuel flow truncated to a
    /// whole number, as the original's int return type does.
    #[allow(clippy::too_many_arguments)]
    fn update_ff(&self, vars: &mut Vars, i: usize, sim_cn1: f64, mach: f64, pressure_altitude: f64, ambient_temperature: f64, ambient_pressure: f64) -> f64 {
        let cff = polynomial::corrected_fuel_flow(sim_cn1, mach, pressure_altitude);
        let out = if cff >= 1. {
            (cff * LBS_TO_KGS * ratios::delta2(mach, ambient_pressure) * ratios::theta2(mach, ambient_temperature).sqrt()).max(0.)
        } else {
            0.
        };
        vars.write(&self.engines[i].ff, out);
        cff.trunc()
    }

    fn update_primary(&self, vars: &mut Vars, i: usize, sim_n1: f64, sim_n3: f64) {
        let e = &self.engines[i];
        vars.write(&e.n1, sim_n1);
        vars.write(&e.n2, if sim_n3 > 0. { sim_n3 + 0.7 } else { sim_n3 });
        vars.write(&e.n3, sim_n3);
    }

    #[allow(clippy::too_many_arguments)]
    fn update_egt(
        &self,
        vars: &mut Vars,
        i: usize,
        state: EngineState,
        delta_time: f64,
        sim_cn1: f64,
        corrected_fuel_flow: f64,
        mach: f64,
        pressure_altitude: f64,
        ambient_temperature: f64,
        on_ground: bool,
    ) {
        let id = self.engines[i].egt;
        if on_ground && state == EngineState::Off {
            vars.write(&id, ambient_temperature);
        } else {
            let corrected = polynomial::corrected_egt(sim_cn1, corrected_fuel_flow, mach, pressure_altitude);
            let previous = vars.read(&id);
            let actual = corrected * ratios::theta2(mach, ambient_temperature);
            vars.write(&id, actual + (previous - actual) * (-0.1 * delta_time).exp());
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn update_oil(
        &mut self,
        vars: &mut Vars,
        i: usize,
        state: EngineState,
        delta_time: f64,
        on_ground: bool,
        ambient_temperature: f64,
        delta_n3: f64,
        thrust_newtons: f64,
    ) {
        let e = &self.engines[i];
        let (id_temp, id_qty, id_total) = (e.oil_temp, e.oil_qty, e.oil_total);
        let pre_temp = vars.read(&id_temp);
        let oil_temperature = if on_ground && state == EngineState::Off && ambient_temperature > pre_temp - 10. {
            ambient_temperature
        } else {
            self.thermal_energy[i] = (0.995 * self.thermal_energy[i]) + (delta_n3 / delta_time);
            polynomial::oil_temperature(self.thermal_energy[i], pre_temp, MAX_OIL_TEMP, delta_time)
        };

        let oil_total = vars.read(&id_total);
        let objective = oil_total * (1. - polynomial::oil_gulp_pct(thrust_newtons));
        let burn = 0.00011111 * delta_time;

        vars.write(&id_temp, oil_temperature);
        vars.write(&id_qty, objective - burn);
        vars.write(&id_total, oil_total - burn);
        // Oil pressure is the physical engine's (`engine_commands.rs`, from
        // its oil pump and N3), written after this: FlyByWire's curve fit
        // (FlyByWire's `oilPressure` polynomial, negative below ~3.7% N3) would only be
        // a second, conflicting writer.
    }

    /// Fuel used per engine, integrated from fuel flow the way the original
    /// integrates it, while there is fuel to burn.
    fn update_fuel(&self, vars: &mut Vars, delta_time: f64, fuel_kg: f64) {
        let hours = delta_time / 3600.;
        for e in &self.engines {
            let ff = vars.read(&e.ff);
            let pre_ff = vars.read(&e.pre_ff);
            if fuel_kg > 0. {
                let change = (ff - pre_ff) / hours;
                let burn = ((change * hours.powi(2) / 2.) + (pre_ff * hours)).min(fuel_kg);
                let used = vars.read(&e.fuel_used);
                vars.write(&e.fuel_used, used + burn);
            }
            vars.write(&e.pre_ff, ff);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn update_thrust_limits(
        &mut self,
        vars: &mut Vars,
        simulation_time: f64,
        pressure_altitude: f64,
        ambient_temperature: f64,
        ambient_pressure: f64,
        mach: f64,
        packs: f64,
        nai: f64,
        wai: f64,
    ) {
        use thrust_limits::limit_n1;
        let flex_temp = vars.read(&self.flex_temp);
        let limit_type = vars.read(&self.limit_type);
        // No `Vec` here: this ran every tick purely to check four levers,
        // heap-allocating a throwaway `Vec<VariableIdentifier>` each time
        // (see docs/deep/debug_start_fps.md) for what a plain iterator
        // check does with no allocation at all.
        let all_at_flex = self.engines.iter().all(|e| vars.read(&e.tla) == 35.0);

        // Only latch the flex temperature when flex is not both selected and
        // set on the levers.
        if !self.is_transition_active && (limit_type != 3. || !all_at_flex) {
            self.latched_flex_temperature = flex_temp;
        }

        let altitude = pressure_altitude.min(16600.);
        let args = (ambient_temperature, ambient_pressure);
        let to = limit_n1(0, altitude, args.0, args.1, 0., packs, nai, wai);
        let ga = limit_n1(1, altitude, args.0, args.1, 0., packs, nai, wai);
        let (mut flex_to, mut flex_ga) = (0., 0.);
        if self.latched_flex_temperature > 0. {
            flex_to = limit_n1(0, altitude, args.0, args.1, self.latched_flex_temperature, packs, nai, wai);
            flex_ga = limit_n1(1, altitude, args.0, args.1, self.latched_flex_temperature, packs, nai, wai);
        }
        let mut clb = limit_n1(2, pressure_altitude, args.0, args.1, 0., packs, nai, wai);
        let mut mct = limit_n1(3, pressure_altitude, args.0, args.1, 0., packs, nai, wai);

        let mach_factor_low = ((mach - 0.04) / 0.04).clamp(0., 1.);
        let flex = flex_to + (flex_ga - flex_to) * mach_factor_low;
        let mut toga = to + (ga - to) * mach_factor_low;

        if self.prev_thrust_limit_type != 3. && limit_type == 3. {
            self.was_flex_active = true;
        } else if limit_type == 4. {
            self.was_flex_active = false;
        }
        if self.was_flex_active && !self.is_transition_active && limit_type == 1. {
            self.is_transition_active = true;
            self.transition_start_time = simulation_time;
        } else if !self.was_flex_active {
            self.is_transition_active = false;
            self.transition_start_time = 0.;
        }
        if self.is_transition_active {
            let waited = ((simulation_time - self.transition_start_time) - TRANSITION_WAIT_TIME).max(0.);
            if waited > 0. && clb > flex {
                self.was_flex_active = false;
            }
        }
        if self.was_flex_active {
            clb = clb.min(flex);
        }
        self.prev_thrust_limit_type = limit_type;

        let mach_factor = ((mach - 0.37) / 0.05).clamp(0., 1.);
        let altitude_factor_low = ((pressure_altitude - 16600.) / 500.).clamp(0., 1.);
        let altitude_factor_high = ((pressure_altitude - 25000.) / 500.).clamp(0., 1.);
        if pressure_altitude >= 25000. {
            mct = clb.max(mct + (clb - mct) * altitude_factor_high);
            toga = mct;
        } else if mct > toga {
            mct = toga + (mct - toga) * (altitude_factor_low + mach_factor).min(1.);
            toga = mct;
        } else {
            toga += (mct - toga) * (altitude_factor_low + mach_factor).min(1.);
        }

        let idle = vars.read(&self.idle_n1);
        vars.write(&self.limit_idle, idle);
        vars.write(&self.limit_toga, toga);
        vars.write(&self.limit_flx, flex);
        vars.write(&self.limit_clb, clb);
        vars.write(&self.limit_mct, mct);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, tolerance: f64) -> bool {
        (a - b).abs() <= tolerance
    }

    #[test]
    fn interpolation_clamps_like_the_original() {
        assert_eq!(interpolate(5., 0., 10., 0., 100.), 50.);
        assert_eq!(interpolate(-1., 0., 10., 0., 100.), 0.);
        assert_eq!(interpolate(11., 0., 10., 0., 100.), 100.);
        assert_eq!(interpolate(3., 2., 2., 7., 9.), 7.);
    }

    #[test]
    fn a_pack_is_bleeding_if_either_of_its_own_flow_valves_is_open() {
        assert!(!pack_bleed_active(false, false, false, false), "no valve open, no bleed");
        assert!(pack_bleed_active(true, false, false, false), "pack 1 valve 1 alone");
        assert!(pack_bleed_active(false, true, false, false), "pack 1 valve 2 alone");
        assert!(pack_bleed_active(false, false, true, false), "pack 2 valve 1 alone");
        assert!(pack_bleed_active(false, false, false, true), "pack 2 valve 2 alone");
        assert!(pack_bleed_active(true, true, true, true), "both packs fully open");
    }

    #[test]
    fn standard_day_ratios_are_one() {
        assert!(close(ratios::theta(15.), 1., 1e-12));
        assert!(close(ratios::delta(1013.), 1., 1e-12));
        assert!(close(ratios::theta2(0., 15.), 1., 1e-12));
    }

    #[test]
    fn cas_equals_mach_speed_at_sea_level() {
        // 250 kt calibrated at sea level is about Mach 0.378.
        assert!(close(cas2mach(250., 1013.), 0.378, 0.002));
    }

    #[test]
    fn idle_spool_speeds_are_what_the_table_gives_at_sea_level() {
        let n3 = table1502::icn3(0., 0.);
        assert!(close(n3, 63., 1e-9));
        // CN3 63 sits between 60 and 67.742 in the table: CN1 about 18.4 at Mach 0.2.
        let cn1 = table1502::icn1(0., 0., 15.);
        assert!(cn1 > 16.3 && cn1 < 22.3, "{cn1}");
    }

    #[test]
    fn an_idle_speed_off_the_table_stays_inside_it() {
        // At 105,000 ft the idle CN3 passes the table's last row, 118.
        assert!(table1502::icn3(105_000., 0.) > 118.);
        assert!(table1502::icn1(105_000., 0., 15.).is_finite());
    }

    #[test]
    fn take_off_limit_is_the_flat_rating_on_a_cool_sea_level_day() {
        let n1 = thrust_limits::limit_n1(0, 0., 15., 1013., 0., 0., 0., 0.);
        assert!(close(n1, 83.832, 0.01), "{n1}");
        // Packs on take the documented 0.4 off.
        let with_packs = thrust_limits::limit_n1(0, 0., 15., 1013., 0., 1., 0., 0.);
        assert!(close(n1 - with_packs, 0.4, 1e-9));
    }

    #[test]
    fn climb_limit_rises_with_altitude() {
        let low = thrust_limits::limit_n1(2, 5000., -5., 843., 0., 0., 0., 0.);
        let high = thrust_limits::limit_n1(2, 35000., -54., 238., 0., 0., 0., 0.);
        assert!(high > low, "{low} {high}");
    }

    #[test]
    fn the_engine_starts_through_the_states_the_original_uses() {
        use EngineState::*;
        let idle = 63.;
        assert_eq!(next_state(Off, 2, true, 0., idle, 15., 15.), (Starting, false));
        assert_eq!(next_state(Starting, 2, true, 30., idle, 300., 15.), (Starting, false));
        assert_eq!(next_state(Starting, 2, true, 62.95, idle, 400., 15.), (On, true));
        assert_eq!(next_state(On, 1, false, 63., idle, 400., 15.), (Shutting, false));
        assert_eq!(next_state(Shutting, 1, false, 0.01, idle, 15., 15.), (Off, true));
    }

    #[test]
    fn a_running_engine_at_spawn_is_on_without_a_start() {
        assert_eq!(next_state(EngineState::Off, 1, true, 63., 63., 15., 15.), (EngineState::On, false));
    }

    #[test]
    fn a_starved_engine_never_reaches_on_no_matter_how_long_the_starter_cranks() {
        // Fuel-starved: the pneumatic starter alone (starter.rs) can spin
        // the HP spool partway, but with no real combustion the real N3
        // plateaus well short of idle. However long the starter cranks
        // (master and igniter held on IGN START), `next_state` must never
        // promote Starting to On on a starved engine -- quick mode or not,
        // since quick mode no longer bypasses this check (see
        // `engine_start`).
        use EngineState::*;
        let idle = 63.;
        let mut state = Starting;
        for _ in 0..10_000 {
            (state, _) = next_state(state, 2, true, 25., idle, 300., 15.);
        }
        assert_eq!(state, Starting, "a starved engine must not fake its way to On");
    }

    #[test]
    fn quick_mode_engages_the_real_starter_immediately_instead_of_faking_idle() {
        // engine_start used to jump ENGINE_STATE and ENGINE_N3 straight to
        // On/idle here under quick mode, purely on FlyByWire's own side.
        // `engine_commands.rs`'s physical engine (`physics::engine::Engine`)
        // runs later the same tick and overwrites those same Vars from its
        // own genuine spool simulation (module docs), so that jump was
        // immediately undone -- ENGINE_STATE said On while X-Plane's real
        // engine stayed at rest with no fuel flow. The fix: quick mode only
        // jumps the start-delay timer straight past the 1.7s threshold
        // `engine_commands.rs`'s real `starter_engaged` reads from this same
        // Var, so the real starter engages immediately; it must not touch
        // ENGINE_STATE or ENGINE_N3 at all.
        let xplm: &'static crate::xp::Xplm = Box::leak(Box::new(crate::xp::Xplm::dummy()));
        let mut vars = crate::Vars::new(xplm);
        let fadec = Fadec::new(&mut vars, xplm);
        vars.write(&fadec.idle_n3, 63.);
        vars.write(&fadec.quick_mode, 1.);

        fadec.engine_start(&mut vars, 0, EngineState::Starting, 0.016, 0., 0., 15., true);

        assert_eq!(
            vars.read(&fadec.engines[0].timer),
            1.7,
            "quick mode should jump the starter-engaged timer immediately, not wait 1.7 real seconds"
        );
        assert_ne!(
            vars.read(&fadec.engines[0].state),
            EngineState::On as i32 as f64,
            "engine_start must not fake ENGINE_STATE On before X-Plane's real N3 is at idle"
        );
        assert_ne!(vars.read(&fadec.engines[0].n3), 63., "engine_start must not fake ENGINE_N3 to idle either");
    }

    #[test]
    fn general_eng_throttle_lever_position_mirrors_the_3d_lever() {
        // MSFS's own simvar the cockpit bundles poll directly (Percent,
        // 0-100): nothing fed it before this fix, so every EWD/SD/ND/MFD/
        // FCU/OIT/PFD lever readout stuck at 0.
        let xplm: &'static crate::xp::Xplm = Box::leak(Box::new(crate::xp::Xplm::dummy()));
        let mut vars = crate::Vars::new(xplm);
        let mut fadec = Fadec::new(&mut vars, xplm);
        let mut levers = crate::throttle::Levers::default();
        levers.lever_3d[0] = 55.;
        levers.angle[0] = 25.;
        fadec.update(&mut vars, xplm, &levers, 0.016, 0.);
        let id = vars.get("GENERAL ENG THROTTLE LEVER POSITION:1".to_owned());
        assert_eq!(vars.read(&id), 55.);
    }

    #[test]
    fn shutdown_decays_toward_ambient() {
        let n3 = polynomial::shutdown_n3(60., 1.);
        assert!(n3 < 60. && n3 > 50.);
        let egt = polynomial::shutdown_egt(600., 15., 10.);
        assert!(egt < 600. && egt > 150.);
    }

    #[test]
    fn start_n3_never_goes_backwards_or_past_idle() {
        assert!(polynomial::start_n3(10., 25., 63.) > 25.);
        assert!(polynomial::start_n3(70., 60., 63.) <= 63.05 + 1e-9);
    }

    #[test]
    fn fuel_flow_is_higher_at_higher_fan_speed() {
        let idle = polynomial::corrected_fuel_flow(20., 0., 0.);
        let climb = polynomial::corrected_fuel_flow(85., 0., 0.);
        assert!(climb > idle);
    }

    #[test]
    fn fuel_flow_at_toga_n1_is_far_above_idle() {
        // Idle CN1 is around 20 (table1502); TOGA CN1 is around 84 (thrust
        // limits' flat rating at sea level). engines.cfg's own idle_fuel_flow
        // is 1366 lb/h per engine; the ported polynomial should land in the
        // same order of magnitude at idle and go well past it at TOGA.
        let idle = polynomial::corrected_fuel_flow(20., 0., 0.) * LBS_TO_KGS;
        let toga = polynomial::corrected_fuel_flow(84., 0., 0.) * LBS_TO_KGS;
        assert!(idle > 0. && idle < 2000., "{idle}");
        assert!(toga > idle * 5., "{idle} {toga}");
    }

    // `static_thrust_climbs_with_n1_like_engines_cfg_says` and
    // `thrust_scales_with_ambient_pressure` used to test the pressure-ratio-
    // scaled thrust table that stood here; that approximation is gone (see
    // `physics::engine`'s own tests, e.g.
    // `static_takeoff_thrust_matches_the_certificated_rating`).

    #[test]
    fn thrust_trim_integrates_toward_a_positive_error_and_stays_bounded() {
        // The closed loop on POINT_thrust (engine_commands.rs) should nudge
        // the throttle up while X-Plane's engine is short of FlyByWire's
        // commanded thrust, ramp toward a steady value as its integrator
        // saturates, and never exceed its bound (spool dynamics stay
        // X-Plane's own; this only trims the steady state).
        let mut integral = 0.;
        let mut last_trim = f64::MIN;
        for _ in 0..50 {
            let (new_integral, trim) = crate::engine_commands::thrust_trim(0.05, integral, 0.1);
            assert!(trim >= last_trim - 1e-9, "{trim} < {last_trim}");
            assert!(trim.abs() <= crate::engine_commands::TRIM_LIMIT + 1e-9, "{trim}");
            integral = new_integral;
            last_trim = trim;
        }
        // KP*0.05 + KI*(integral capped at TRIM_LIMIT) = 0.03 + 0.15*0.12.
        assert!(close(last_trim, 0.048, 1e-6), "{last_trim}");
    }
}
