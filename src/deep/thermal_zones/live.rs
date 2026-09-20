//! The live thermal-zone system: one owned [`topology_a380::A380Thermal`]
//! stepped every frame from [`Truth`], with every failure this area's
//! [`super::registry`] registers driving the exact model field that
//! registry entry names, and every variable its ECAM triggers read
//! published back out.
//!
//! Until this file existed the airframe thermal network was a type with
//! tests and no instance: `topology_a380::build()` was never called
//! anywhere in the running plugin, so no zone had a temperature, no
//! `THERMAL_ZONE_*` variable existed, and every ECAM alert
//! `registry.rs` hangs off those variables was unreachable. This is the
//! instance.
//!
//! ## What drives the network from `Truth`
//! - **Outside air** ([`OutsideAir`]): static air temperature, Mach and
//!   true airspeed straight from `truth.environment` (X-Plane's own real
//!   weather, see `integration::weather_truth`). Ram/recovery heating and
//!   the forced-convection coefficient both come out of those three, so
//!   altitude and speed reach every zone's exterior through the one input
//!   the network already has for them.
//! - **AC power**: the avionics- and cargo-bay extract fans are electric.
//!   An induction fan motor either runs at essentially its synchronous
//!   speed or it does not run at all, so the fans are modelled as
//!   delivering full nameplate flow while at least one main AC bus is at
//!   or above [`MIN_FAN_BUS_VOLTS`], and none below it -- which makes a
//!   total AC loss heat the avionics bay for the same physical reason a
//!   fan failure does, with no separate code path. The per-fan bus
//!   assignment is not public for the A380, so "any main AC bus" is used
//!   rather than inventing one (see the report/`PROGRESS.md`).
//!
//! Ram-air paths (nacelle, pylon, APU compartment, belly fairing, tail
//! cone) need no power and are driven only by their own failures.
//!
//! ## What now drives what used to be zero/resting-state
//! - **Solar flux**: [`Truth::sun_elevation_deg`] is real (X-Plane's own
//!   `sim/graphics/scenery/sun_pitch_degrees`), but it is an elevation, not
//!   an irradiance -- turning one into the other needs the atmosphere's own
//!   optical depth, which that dataref does not carry (`docs/deep/
//!   truth-requests.md`), so [`solar_flux_w_m2`] derives a **GENERIC**
//!   clear-sky flux (`SOLAR_CONSTANT_W_M2 * CLEAR_SKY_TRANSMITTANCE *
//!   sin(elevation)`, zero once the sun is below the horizon) rather than
//!   leaving every zone's real `sun_exposure_fraction` permanently
//!   multiplied by zero. Cloud attenuation is not modelled (no optical
//!   depth to model it from); this is a clear-sky figure only.
//! - **Gear bay door position**: `registry`'s ATA 32 door-jam failures
//!   freeze a door's ventilation link away from its commanded position.
//!   The jam itself was always implemented (the link's health latches at
//!   the value it held when the fault engaged); what was missing was a
//!   real commanded position for it to diverge *from* --
//!   `truth.controls.gear_door_commanded_open` (`[nose, left, right]`,
//!   FlyByWire's own undamaged door-actuator output) now drives every
//!   healthy door every tick, with a jam blending toward its own stuck
//!   value exactly as before.
//!
//! ## What is still not driven, and why
//! - **Zone heat from other areas** (engine heat into a nacelle, a
//!   pneumatic duct leak's enthalpy, brake heat into a gear bay): those
//!   are other areas' models. `ThermalNetwork::inject_heat_w` is the
//!   interface they will use; nothing is injected on their behalf here.

use super::network::OutsideAir;
use super::topology_a380::{self, A380Thermal};
use crate::deep::api::{failure_id, Area as RegArea};
use crate::deep::live::{Faults, Truth};

/// Minimum main-AC-bus voltage at which a ventilation fan motor is
/// treated as running. Aircraft 115 V 400 Hz AC: RTCA DO-160/MIL-STD-704F
/// put normal steady-state operation at 108-118 V rms and the abnormal
/// low-voltage limit at 100 V, so 100 V is the documented floor below
/// which equipment is not required to operate.
pub const MIN_FAN_BUS_VOLTS: f64 = 100.0;

/// Solar constant at the top of the atmosphere, W/m^2 (WMO/NASA-cited
/// public figure, ~1361 W/m^2 at 1 AU average Earth-Sun distance).
const SOLAR_CONSTANT_W_M2: f64 = 1361.0;

/// **GENERIC** broadband clear-sky atmospheric transmittance at typical
/// operating altitudes: commonly cited clear-sky broadband transmittance
/// figures (solar-engineering/ASHRAE-style clear-sky models) sit roughly
/// 0.7-0.8 at low altitude, higher at cruise altitude above most of the
/// attenuating atmosphere; a single mid-range value is used uniformly
/// rather than modelling altitude-dependent optical depth, which `Truth`
/// has no input for (module doc).
const CLEAR_SKY_TRANSMITTANCE: f64 = 0.75;

/// Incident solar flux handed to `ThermalNetwork::step`, from
/// [`Truth::sun_elevation_deg`] (module doc): a clear-sky flux, zero once
/// the sun is below the horizon. Not a claim about real cloud cover --
/// `Truth` carries no optical-depth/cloud-attenuation input to model that
/// with, only elevation.
fn solar_flux_w_m2(truth: &Truth) -> f64 {
    let elevation_rad = truth.sun_elevation_deg.to_radians();
    if elevation_rad <= 0.0 {
        0.0
    } else {
        SOLAR_CONSTANT_W_M2 * CLEAR_SKY_TRANSMITTANCE * elevation_rad.sin()
    }
}

// Reference full-severity magnitudes. Each one is the exact figure the
// matching `FailureDef::model_field`/`magnitude` text in `registry.rs`
// cites, repeated here so the two cannot drift silently (the registry
// builds its documentation strings from its own copies).
const CARGO_FIRE_MAX_HEAT_W: f64 = 200_000.0;
const CARGO_FIRE_MAX_SMOKE_KG_S: f64 = 0.01;
const NACELLE_FIRE_MAX_HEAT_W: f64 = 500_000.0;
const NACELLE_FIRE_MAX_SMOKE_KG_S: f64 = 0.005;
const APU_FIRE_MAX_HEAT_W: f64 = 300_000.0;
const APU_FIRE_MAX_SMOKE_KG_S: f64 = 0.008;
const WING_DUCT_LEAK_MAX_HEAT_W: f64 = 30_000.0;
const NACELLE_DUCT_LEAK_MAX_HEAT_W: f64 = 20_000.0;
const APU_DUCT_LEAK_MAX_HEAT_W: f64 = 25_000.0;

// ---------------------------------------------------------------------------
// ATA 36: the pylon bleed duct leak, derived rather than assumed.
//
// This failure used to be one more entry in the block above,
// `PYLON_BLEED_LEAK_MAX_HEAT_W = 40_000.0`, injected as `magnitude * 40 kW`
// with no reference to the duct it leaks from. That was wrong twice over,
// and the two errors hid each other:
//
// 1. **Wrong value.** `registry.rs`'s own note derived it as "a fraction of
//    the ~200 C/44 psi bleed source's enthalpy flow through a small crack"
//    but never did the arithmetic. Done properly (below), a full-severity
//    crack at *those* conditions carries 14.7 kW, not 40 kW -- the constant
//    overstated its own stated basis by 2.7x.
// 2. **Wrong form.** A fixed wattage is not bounded by the source it comes
//    from, so it goes on heating a bay that is already hotter than the air
//    leaking into it. The sibling constants above still have that defect
//    and it is not hypothetical: a full-severity `WING_DUCT_LEAK_MAX_HEAT_W`
//    settles `WingLeLeft` at 853 C, from a duct whose air is at 200 C
//    (measured; see `PROGRESS.md`). A leak can only ever carry its own
//    enthalpy *above the bay*, so the heat must fall to zero as the bay
//    approaches the duct -- which only a state-dependent term can do.
//
// Both are fixed here by computing the leak the way the physics does:
// choked-orifice mass flow through the crack at the duct's own real
// conditions, times its sensible enthalpy above the bay it escapes into.
// `Truth` already carries those conditions per engine
// (`engine_bleed_pressure_pa`/`_temp_k`, documented in `deep::live` as
// "bleed air available *at the pylon*, from this crate's own engine model's
// IP8/HP6 port outputs") -- exactly the duct run the `PylonEngine<n>` zone
// contains -- so nothing new is needed from it.
// ---------------------------------------------------------------------------

/// Ratio of specific heats for dry air. Standard value; reproduced here
/// rather than imported per this area's self-contained-module rule
/// (`docs/deep/BRIEF.md` hard rule 2), like `network.rs`'s own copy.
const GAMMA_AIR_LEAK: f64 = 1.4;

/// Specific gas constant for dry air, J/(kg*K). Standard value -- the same
/// figure `physics::bays::R_AIR` and FlyByWire's own
/// `PneumaticContainer::GAS_CONSTANT_DRY_AIR` use.
const R_AIR_J_PER_KG_K: f64 = 287.057_005;

/// Critical (choked) pressure ratio for `gamma = 1.4`:
/// `(2/(gamma+1))^(gamma/(gamma-1))`.
const CRITICAL_PRESSURE_RATIO: f64 = 0.528_281_787_717_685_7;

/// Nominal bore of the engine bleed duct run through a pylon, m: 4 in
/// (0.1016 m), the bleed pipework diameter FlyByWire's own ported
/// `a380_systems/pneumatic.rs` sizes this duct with, and the same figure
/// `deep::pneumatic_ducts::network`'s `ENGINE_DUCT_DIAMETER_M` cites from
/// it. Reproduced, not imported (hard rule 2). Bore area is
/// `pi/4 * 0.1016^2 = 8.107e-3 m^2`.
const PYLON_BLEED_DUCT_BORE_M: f64 = 0.1016;

/// **GENERIC**: the opening a *full-severity leak* presents, as a fraction
/// of the duct's own bore area. 2% of a 4 in bore is `1.621e-4 m^2`, a
/// 14 mm equivalent-diameter hole -- a cracked weld or a partly-let-go
/// V-band coupling, which is how a bleed duct actually fails, and a full
/// order of magnitude below a severance. A severance is a different fault
/// with a different consequence (immediate isolation, not a bay slowly
/// heating); `deep::pneumatic_ducts` models it separately with its own
/// `rupture` magnitude up to `1.0 x` bore, and this area registers only the
/// leak. That neighbouring area's `leak::LEAK_AREA_FRACTION_OF_BORE`
/// derives the same 2% independently for the same class of opening -- the
/// two agreeing is deliberate, so that one physical fault is the same size
/// whichever side of the coupling names it.
const PYLON_BLEED_LEAK_AREA_FRACTION_OF_BORE: f64 = 0.02;

/// Discharge coefficient for a crack in a duct wall. Matches
/// `physics::bays::LEAK_DISCHARGE_COEFFICIENT` and
/// `deep::pneumatic_ducts::leak::LEAK_DISCHARGE_COEFFICIENT`
/// (`docs/physics/air.md`'s cited duct/valve figure), reused rather than
/// inventing a second number for the same class of opening.
const BLEED_LEAK_DISCHARGE_COEFFICIENT: f64 = 0.65;

/// Compressible mass flow through a sharp-edged orifice, kg/s: choked at or
/// below the critical pressure ratio, subsonic above it. The standard
/// isentropic relation (Anderson, *Modern Compressible Flow*) -- the same
/// one `deep::pneumatic_ducts::duct::orifice_mass_flow_kg_s` and
/// FlyByWire's own `compressible_orifice_mass_flow_rate` implement,
/// reproduced independently here per hard rule 2.
///
/// Zero for a non-positive area or upstream state, and zero when the bay is
/// at or above duct pressure: this models gas escaping a pressurised duct,
/// never backflow into one.
fn orifice_mass_flow_kg_s(discharge_coefficient: f64, area_m2: f64, upstream_pa: f64, upstream_k: f64, downstream_pa: f64) -> f64 {
    if area_m2 <= 0.0 || upstream_pa <= 0.0 || upstream_k <= 0.0 || downstream_pa >= upstream_pa {
        return 0.0;
    }
    let pressure_ratio = (downstream_pa.max(0.0) / upstream_pa).clamp(0.0, 1.0);
    let flow_function = if pressure_ratio <= CRITICAL_PRESSURE_RATIO {
        (GAMMA_AIR_LEAK * (2.0 / (GAMMA_AIR_LEAK + 1.0)).powf((GAMMA_AIR_LEAK + 1.0) / (GAMMA_AIR_LEAK - 1.0))).sqrt()
    } else {
        (2.0 * GAMMA_AIR_LEAK / (GAMMA_AIR_LEAK - 1.0) * (pressure_ratio.powf(2.0 / GAMMA_AIR_LEAK) - pressure_ratio.powf((GAMMA_AIR_LEAK + 1.0) / GAMMA_AIR_LEAK)))
            .max(0.0)
            .sqrt()
    };
    discharge_coefficient.max(0.0) * area_m2 * upstream_pa / (R_AIR_J_PER_KG_K * upstream_k).sqrt() * flow_function
}

/// Heat a pylon bleed duct leak of `severity` (0 healthy .. 1 full)
/// delivers to its own bay this tick, W.
///
/// # Derivation
///
/// 1. **Crack area.** `severity * 0.02 * pi/4 * 0.1016^2`; at full severity
///    `1.621e-4 m^2`, and with [`BLEED_LEAK_DISCHARGE_COEFFICIENT`] an
///    effective `1.054e-4 m^2`.
/// 2. **Duct condition.** The duct the `PylonEngine<n>` zone contains is
///    the engine bleed run upstream of the precooler, so it sits at the
///    engine's own bleed port condition -- `duct_pa`/`duct_k`, which the
///    caller takes straight from `Truth::engine_bleed_pressure_pa`/`_temp_k`.
///    For a Trent 972 at take-off that IP8 tap is roughly 9-10 bar and
///    580-600 K: a fan-hub pressure ratio of about 1.75 through an IP
///    compressor ratio of about 5.5 is `~9.6x` ambient, and the same ratio
///    through a `~0.90` polytropic efficiency gives
///    `288 K * 9.6^(0.2857/0.90) = 590 K`. At idle and in the descent it is
///    a small fraction of that -- which is the entire reason this is read
///    rather than fixed: a leak in a shut-down engine's pylon must deliver
///    nothing, and a fixed wattage cannot express that.
/// 3. **Mass flow.** At take-off `p_bay/p_duct ~ 0.10`, far below the
///    `0.528` critical ratio, so the crack is choked:
///    `mdot = Cd*A*p0/sqrt(R*T0) * sqrt(gamma*(2/(gamma+1))^((gamma+1)/(gamma-1)))`
///    `= 1.054e-4 * 970000/411.5 * 0.6847 = 0.170 kg/s`.
/// 4. **Heat.** What that air can give the bay is its sensible enthalpy
///    *above the bay's own air*, `mdot * cp * (T_duct - T_bay)`:
///    `0.170 * 1005 * (590 - 288) = 51.6 kW` into a 15 C bay, falling to
///    zero as the bay approaches 590 K. That last term is the one the old
///    fixed 40 kW could not express, and is why it could drive a bay hotter
///    than the air feeding it.
///
/// The same arithmetic at the conditions the *old* constant claimed as its
/// basis (200 C / 44 psig, i.e. 473 K and 405 kPa, downstream of the
/// precooler) gives `0.0792 kg/s` and `14.7 kW` -- so 40 kW overstated even
/// its own stated source by a factor of 2.7. Both figures are checked
/// against this function in the tests below.
///
/// Never negative: a bay hotter than the duct is not cooled back through
/// the crack here. The escaping mass flow is small next to the bay's own
/// ventilation, so a reverse-sense term would be a rounding error dressed
/// up as physics.
fn pylon_bleed_leak_heat_w(severity: f64, duct_pa: f64, duct_k: f64, bay_air_k: f64, bay_pa: f64) -> f64 {
    let severity = severity.clamp(0.0, 1.0);
    if severity <= 0.0 {
        return 0.0;
    }
    let bore_area_m2 = std::f64::consts::PI / 4.0 * PYLON_BLEED_DUCT_BORE_M * PYLON_BLEED_DUCT_BORE_M;
    let crack_area_m2 = severity * PYLON_BLEED_LEAK_AREA_FRACTION_OF_BORE * bore_area_m2;
    let mdot = orifice_mass_flow_kg_s(BLEED_LEAK_DISCHARGE_COEFFICIENT, crack_area_m2, duct_pa, duct_k, bay_pa);
    mdot * super::network::CP_AIR_J_PER_KG_K * (duct_k - bay_air_k).max(0.0)
}

fn f(ata: u16, n: u16) -> u64 {
    failure_id(RegArea::ThermalZones, ata, n)
}

/// The three variable names one zone publishes, built once at
/// construction (the `Area` trait publishes by `&str`, and a zone's name
/// never changes).
struct ZoneVars {
    temperature_c: String,
    structure_temperature_c: String,
    smoke_concentration: String,
}

pub struct ThermalZonesLive {
    a380: A380Thermal,
    zone_vars: Vec<ZoneVars>,
    damage_vars: Vec<String>,
    /// Per gear bay (nose, wing, body): the ventilation-link health the
    /// door was at when its jam failure first engaged, `None` while the
    /// door is free. A jam freezes the door where it is; it does not move
    /// it (`registry.rs`, ATA 32).
    gear_door_jammed_at: [Option<f64>; 3],
}

impl Default for ThermalZonesLive {
    fn default() -> Self {
        Self::new()
    }
}

impl ThermalZonesLive {
    pub fn new() -> Self {
        let a380 = topology_a380::build();
        let zone_vars = a380
            .network
            .zones
            .iter()
            .map(|z| {
                let up = z.name.to_uppercase();
                ZoneVars {
                    temperature_c: format!("THERMAL_ZONE_{up}_TEMPERATURE_C"),
                    structure_temperature_c: format!("THERMAL_ZONE_{up}_STRUCTURE_TEMPERATURE_C"),
                    smoke_concentration: format!("THERMAL_ZONE_{up}_SMOKE_CONCENTRATION"),
                }
            })
            .collect();
        let damage_vars = a380.damage.components.iter().map(|c| format!("THERMAL_COMPONENT_{}_DAMAGE", c.name.to_uppercase())).collect();
        Self { a380, zone_vars, damage_vars, gear_door_jammed_at: [None; 3] }
    }

    /// Whether the electric ventilation fans have a bus to run on
    /// (module doc).
    fn fan_power_fraction(truth: &Truth) -> f64 {
        if truth.ac_bus_volts.iter().any(|&v| v >= MIN_FAN_BUS_VOLTS) {
            1.0
        } else {
            0.0
        }
    }

    fn outside_air(truth: &Truth) -> OutsideAir {
        OutsideAir {
            static_temp_c: truth.environment.sat_c,
            mach: truth.environment.mach(),
            true_airspeed_m_s: truth.environment.tas_ms,
        }
    }

    /// ATA 21: the five electric extract fans plus the two ram-air paths
    /// (`registry::ventilation_zones`, in its own order).
    fn apply_ventilation_failures(&mut self, faults: &Faults, fan_power: f64) {
        let v = &self.a380.vents;
        let electric = [
            (v.main_avionics_fan, f(21, 1)),
            (v.upper_avionics_fan, f(21, 2)),
            (v.cargo_fwd_fan, f(21, 3)),
            (v.cargo_aft_fan, f(21, 4)),
            (v.cargo_bulk_fan, f(21, 5)),
        ];
        for (link, id) in electric {
            let health = fan_power * (1.0 - faults.get(id));
            self.a380.network.set_ventilation_health(link, health);
        }
        // Ram-air scoop/drain paths: no electrical supply of their own.
        self.a380.network.set_ventilation_health(v.belly_pack_bay_vent, 1.0 - faults.get(f(21, 6)));
        self.a380.network.set_ventilation_health(v.apu_compartment_vent, 1.0 - faults.get(f(21, 7)));
    }

    /// ATA 26: cargo, nacelle and APU compartment fires -- heat and smoke
    /// into the zone the fire is in.
    fn apply_fire_failures(&mut self, faults: &Faults) {
        let z = &self.a380.zones;
        let cargo = [(z.cargo_fwd, f(26, 1)), (z.cargo_aft, f(26, 2)), (z.cargo_bulk, f(26, 3))];
        for (zone, id) in cargo {
            let severity = faults.get(id);
            if severity > 0.0 {
                self.a380.network.inject_heat_w(zone, severity * CARGO_FIRE_MAX_HEAT_W);
                self.a380.network.inject_smoke_kg_s(zone, severity * CARGO_FIRE_MAX_SMOKE_KG_S);
            }
        }
        for engine in 0..4usize {
            let severity = faults.get(f(26, 4 + engine as u16));
            if severity > 0.0 {
                let zone = z.nacelle_cowl[engine];
                self.a380.network.inject_heat_w(zone, severity * NACELLE_FIRE_MAX_HEAT_W);
                self.a380.network.inject_smoke_kg_s(zone, severity * NACELLE_FIRE_MAX_SMOKE_KG_S);
            }
        }
        let apu = faults.get(f(26, 8));
        if apu > 0.0 {
            self.a380.network.inject_heat_w(z.apu_compartment, apu * APU_FIRE_MAX_HEAT_W);
            self.a380.network.inject_smoke_kg_s(z.apu_compartment, apu * APU_FIRE_MAX_SMOKE_KG_S);
        }
    }

    /// ATA 30: wing/nacelle anti-ice duct leaks (heat) and nacelle vent
    /// scoop ice blockage (ventilation health).
    fn apply_ice_and_duct_failures(&mut self, faults: &Faults) {
        let z = &self.a380.zones;
        for (zone, id) in [(z.wing_le_left, f(30, 1)), (z.wing_le_right, f(30, 2))] {
            let leak = faults.get(id);
            if leak > 0.0 {
                self.a380.network.inject_heat_w(zone, leak * WING_DUCT_LEAK_MAX_HEAT_W);
            }
        }
        for engine in 0..4usize {
            let leak = faults.get(f(30, 3 + engine as u16));
            if leak > 0.0 {
                self.a380.network.inject_heat_w(z.nacelle_cowl[engine], leak * NACELLE_DUCT_LEAK_MAX_HEAT_W);
            }
        }
        for engine in 0..4usize {
            let blockage = faults.get(f(30, 7 + engine as u16));
            let link = self.a380.vents.nacelle_vent[engine];
            self.a380.network.set_ventilation_health(link, 1.0 - blockage);
        }
    }

    /// ATA 32: a jammed bay door stops following its commanded position
    /// and stays where it was. `commanded_open` (`[nose, left, right]`) is
    /// now `truth.controls.gear_door_commanded_open`, FlyByWire's own real
    /// (undamaged) door-actuator output -- previously this had no real
    /// input and every door held `topology_a380`'s own resting closed
    /// state forever, so a jam had nothing to diverge from; the latch
    /// logic itself is unchanged.
    fn apply_gear_door_failures(&mut self, faults: &Faults, commanded_open: [f64; 3]) {
        let doors = [
            (0usize, self.a380.vents.nose_gear_door, f(32, 1)),
            (1, self.a380.vents.wing_gear_door, f(32, 2)),
            (2, self.a380.vents.body_gear_door, f(32, 3)),
        ];
        for (i, link, id) in doors {
            let commanded = commanded_open[i].clamp(0.0, 1.0);
            let jam = faults.get(id);
            if jam > 0.0 {
                let stuck_at = *self.gear_door_jammed_at[i].get_or_insert_with(|| self.a380.network.ventilation_links[link].health);
                // A partial jam still partly follows the commanded
                // position; a full jam holds `stuck_at` outright.
                let health = commanded + (stuck_at - commanded) * jam;
                self.a380.network.set_ventilation_health(link, health);
            } else {
                self.gear_door_jammed_at[i] = None;
                self.a380.network.set_ventilation_health(link, commanded);
            }
        }
    }

    /// ATA 36/49: bleed duct runs through the pylons and the tail cone.
    ///
    /// The pylon runs carry the real physics ([`pylon_bleed_leak_heat_w`]):
    /// the crack's own choked mass flow at the engine's real bleed port
    /// condition, times its enthalpy above the bay's *current* air
    /// temperature. The tail-cone APU run is still the old fixed-wattage
    /// placeholder -- `Truth` carries `apu_bleed_pressure_pa` but no APU
    /// bleed *temperature*, so the same derivation cannot be done for it
    /// without inventing one (see `PROGRESS.md`).
    fn apply_bleed_duct_failures(&mut self, faults: &Faults, truth: &Truth) {
        for engine in 0..4usize {
            let leak = faults.get(f(36, 1 + engine as u16));
            if leak > 0.0 {
                let zone = self.a380.zones.pylon[engine];
                let bay_air_k = self.a380.network.air_temp_c(zone) + 273.15;
                let heat_w = pylon_bleed_leak_heat_w(
                    leak,
                    truth.engine_bleed_pressure_pa[engine],
                    truth.engine_bleed_temp_k[engine],
                    bay_air_k,
                    truth.environment.ambient_pressure_pa,
                );
                self.a380.network.inject_heat_w(zone, heat_w);
            }
        }
        let apu_duct = faults.get(f(49, 1));
        if apu_duct > 0.0 {
            self.a380.network.inject_heat_w(self.a380.zones.tail_cone, apu_duct * APU_DUCT_LEAK_MAX_HEAT_W);
        }
    }

    /// ATA 53: the crown insulation blanket's condition is the zone's own
    /// `insulation_effectiveness` (1 = intact .. 0 = missing).
    fn apply_insulation_failures(&mut self, faults: &Faults) {
        let zone = self.a380.zones.crown_area;
        self.a380.network.zones[zone].insulation_effectiveness = 1.0 - faults.get(f(53, 1));
    }

    /// Read access for tests and for anything that wants the network
    /// itself rather than its published variables.
    pub fn network(&self) -> &super::network::ThermalNetwork {
        &self.a380.network
    }
}

impl crate::deep::live::Area for ThermalZonesLive {
    fn name(&self) -> &'static str {
        "thermal_zones"
    }

    fn tick(&mut self, truth: &Truth, faults: &Faults) {
        let fan_power = Self::fan_power_fraction(truth);
        self.apply_ventilation_failures(faults, fan_power);
        self.apply_ice_and_duct_failures(faults);
        self.apply_gear_door_failures(faults, truth.controls.gear_door_commanded_open);
        self.apply_insulation_failures(faults);
        // Heat/smoke sources are accumulated per tick and consumed by the
        // step below, so they are injected last, immediately before it.
        self.apply_fire_failures(faults);
        self.apply_bleed_duct_failures(faults, truth);

        let outside = Self::outside_air(truth);
        self.a380.network.step(truth.dt_s, &outside, solar_flux_w_m2(truth));
        self.a380.damage.update(&self.a380.network, truth.dt_s);
    }

    fn publish(&self, out: &mut dyn FnMut(&str, f64)) {
        for (i, vars) in self.zone_vars.iter().enumerate() {
            out(&vars.temperature_c, self.a380.network.air_temp_c(i));
            out(&vars.structure_temperature_c, self.a380.network.structure_temp_c(i));
            out(&vars.smoke_concentration, self.a380.network.smoke_concentration(i));
        }
        for (i, name) in self.damage_vars.iter().enumerate() {
            out(name, self.a380.damage.damage_fraction(i));
        }
    }
}

pub fn live_system() -> Box<dyn crate::deep::live::Area> {
    Box::new(ThermalZonesLive::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn powered_ground_truth() -> Truth {
        Truth { dt_s: 1.0, ac_bus_volts: [115.0; 4], ..Truth::default() }
    }

    fn published(area: &dyn crate::deep::live::Area) -> BTreeMap<String, f64> {
        let mut map = BTreeMap::new();
        area.publish(&mut |name, value| {
            map.insert(name.to_string(), value);
        });
        map
    }

    fn run(area: &mut dyn crate::deep::live::Area, truth: &Truth, faults: &Faults, ticks: usize) {
        for _ in 0..ticks {
            area.tick(truth, faults);
        }
    }

    #[test]
    fn every_variable_the_registry_triggers_on_is_actually_published() {
        // The whole point of this file: an ECAM trigger reading a variable
        // nobody publishes can never fire. Every name below is read by a
        // trigger or contribution in `registry.rs`.
        let area = live_system();
        let map = published(area.as_ref());
        let required = [
            "THERMAL_ZONE_CARGOFWD_SMOKE_CONCENTRATION",
            "THERMAL_ZONE_CARGOAFT_SMOKE_CONCENTRATION",
            "THERMAL_ZONE_CARGOBULK_SMOKE_CONCENTRATION",
            "THERMAL_ZONE_NACELLECOWL1_TEMPERATURE_C",
            "THERMAL_ZONE_NACELLECOWL2_TEMPERATURE_C",
            "THERMAL_ZONE_NACELLECOWL3_TEMPERATURE_C",
            "THERMAL_ZONE_NACELLECOWL4_TEMPERATURE_C",
            "THERMAL_ZONE_APUCOMPARTMENT_TEMPERATURE_C",
            "THERMAL_ZONE_WINGLELEFT_TEMPERATURE_C",
            "THERMAL_ZONE_WINGLERIGHT_TEMPERATURE_C",
            "THERMAL_ZONE_BELLYFAIRINGPACKS_TEMPERATURE_C",
            "THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C",
            "THERMAL_COMPONENT_MAINAVIONICSWIRINGBUNDLE_DAMAGE",
        ];
        for name in required {
            assert!(map.contains_key(name), "{name} is read by an ECAM trigger but never published");
        }
        assert_eq!(map.len(), 26 * 3 + 5, "26 zones x 3 variables plus the 5 registered thermal components");
    }

    #[test]
    fn a_cargo_fire_raises_the_published_smoke_concentration_past_what_the_detectors_see() {
        // Failure 11_026_001 (CargoFwd cargo compartment fire), effect:
        // "CargoFwd air temperature and smoke concentration rise". The
        // contribution to CARGO_SMOKE_FWD triggers above 2e-4.
        let truth = powered_ground_truth();
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(26, 1), 1.0)]);
        run(area.as_mut(), &truth, &armed, 300);
        let hot = published(area.as_ref());

        let mut healthy = live_system();
        run(healthy.as_mut(), &truth, &Faults::default(), 300);
        let cold = published(healthy.as_ref());

        let smoke = hot["THERMAL_ZONE_CARGOFWD_SMOKE_CONCENTRATION"];
        assert!(smoke > 0.0002, "a full-severity cargo fire must put the bay past the detectors' 2e-4 threshold, got {smoke}");
        assert_eq!(cold["THERMAL_ZONE_CARGOFWD_SMOKE_CONCENTRATION"], 0.0);
        assert!(
            hot["THERMAL_ZONE_CARGOFWD_TEMPERATURE_C"] > cold["THERMAL_ZONE_CARGOFWD_TEMPERATURE_C"] + 50.0,
            "the fire must dominate the bay's own temperature: {} vs {}",
            hot["THERMAL_ZONE_CARGOFWD_TEMPERATURE_C"],
            cold["THERMAL_ZONE_CARGOFWD_TEMPERATURE_C"]
        );
    }

    #[test]
    fn arming_the_main_avionics_fan_failure_heats_the_bay_and_damages_its_wiring() {
        // Failure 11_021_001, effect: "MainAvionics loses cooling/purge
        // airflow; its steady-state air temperature rises ... and any
        // registered thermal component there accrues damage faster."
        // AVIONICS_VENT_FAULT triggers above 70 C or on any wiring damage.
        let truth = powered_ground_truth();
        let mut failed = live_system();
        let mut healthy = live_system();
        let armed = Faults::from_pairs([(f(21, 1), 1.0)]);
        run(failed.as_mut(), &truth, &armed, 20_000);
        run(healthy.as_mut(), &truth, &Faults::default(), 20_000);

        let failed_vars = published(failed.as_ref());
        let healthy_vars = published(healthy.as_ref());
        let hot = failed_vars["THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C"];
        let cool = healthy_vars["THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C"];
        assert!(hot > cool + 5.0, "a failed extract fan must leave the bay hotter: {hot} vs {cool}");
        assert!(hot > 70.0, "it must reach the AVIONICS VENT FAULT trigger temperature, got {hot}");
        assert!(
            failed_vars["THERMAL_COMPONENT_MAINAVIONICSWIRINGBUNDLE_DAMAGE"] > 0.0,
            "the wiring bundle registered in that bay must start accruing damage once it runs over its 70 C limit"
        );
        assert_eq!(healthy_vars["THERMAL_COMPONENT_MAINAVIONICSWIRINGBUNDLE_DAMAGE"], 0.0);
    }

    #[test]
    fn losing_every_ac_bus_stops_the_extract_fans_exactly_as_a_fan_failure_does() {
        let unpowered = Truth { dt_s: 1.0, ..Truth::default() }; // ac_bus_volts all 0
        let powered = powered_ground_truth();
        let mut dark = live_system();
        let mut live = live_system();
        run(dark.as_mut(), &unpowered, &Faults::default(), 20_000);
        run(live.as_mut(), &powered, &Faults::default(), 20_000);
        let dark_temp = published(dark.as_ref())["THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C"];
        let live_temp = published(live.as_ref())["THERMAL_ZONE_MAINAVIONICS_TEMPERATURE_C"];
        assert!(dark_temp > live_temp + 5.0, "unpowered fans must leave the bay hotter: {dark_temp} vs {live_temp}");
    }

    #[test]
    fn a_crown_insulation_failure_lets_the_crown_track_a_cold_outside_faster() {
        // Failure 11_053_001, effect: "CrownArea's structure tracks the
        // outside recovery temperature much more closely (colder at
        // altitude ...)".
        let truth = Truth {
            dt_s: 1.0,
            environment: crate::deep::integration::weather_truth::EnvironmentTruth { sat_c: -50.0, tas_ms: 230.0, ambient_pressure_pa: 25_000.0, ..Truth::default().environment },
            altitude_ft: 35_000.0,
            on_ground: false,
            ac_bus_volts: [115.0; 4],
            ..Truth::default()
        };
        let mut damaged = live_system();
        let mut intact = live_system();
        let armed = Faults::from_pairs([(f(53, 1), 1.0)]);
        run(damaged.as_mut(), &truth, &armed, 600);
        run(intact.as_mut(), &truth, &Faults::default(), 600);
        let damaged_c = published(damaged.as_ref())["THERMAL_ZONE_CROWNAREA_STRUCTURE_TEMPERATURE_C"];
        let intact_c = published(intact.as_ref())["THERMAL_ZONE_CROWNAREA_STRUCTURE_TEMPERATURE_C"];
        assert!(damaged_c < intact_c - 2.0, "a damaged blanket must chill faster: {damaged_c} vs {intact_c}");
    }

    #[test]
    fn a_commanded_open_gear_door_ventilates_the_bay_toward_outside_air_faster_than_closed() {
        // Before this pass `truth.controls.gear_door_commanded_open` had
        // no effect at all: every door held `topology_a380`'s own resting
        // closed state forever (module doc). Wiring it in must let a
        // genuinely commanded-open door ventilate its bay, the same
        // coupling `topology_a380.rs`'s own direct-`ThermalNetwork` test
        // proves the underlying link already supports.
        let cold_air = Truth {
            dt_s: 1.0,
            environment: crate::deep::integration::weather_truth::EnvironmentTruth { sat_c: -50.0, tas_ms: 230.0, ambient_pressure_pa: 25_000.0, ..Truth::default().environment },
            altitude_ft: 35_000.0,
            on_ground: false,
            ac_bus_volts: [115.0; 4],
            ..Truth::default()
        };
        let mut open_truth = cold_air.clone();
        open_truth.controls.gear_door_commanded_open = [0.0, 1.0, 0.0]; // wing gear door commanded open
        let closed_truth = cold_air;

        let mut open = live_system();
        let mut closed = live_system();
        run(open.as_mut(), &open_truth, &Faults::default(), 300);
        run(closed.as_mut(), &closed_truth, &Faults::default(), 300);
        let open_c = published(open.as_ref())["THERMAL_ZONE_WINGGEARWELL_TEMPERATURE_C"];
        let closed_c = published(closed.as_ref())["THERMAL_ZONE_WINGGEARWELL_TEMPERATURE_C"];
        assert!(closed_c > open_c + 5.0, "a door truly commanded open must ventilate the bay toward the cold outside air faster than one held closed: open {open_c} vs closed {closed_c}");
    }

    #[test]
    fn a_high_sun_elevation_heats_a_sun_exposed_zones_structure_more_than_no_sun_at_all() {
        // Before this pass every zone received a flat 0 W/m^2 regardless
        // of `truth.sun_elevation_deg` (module doc's own old "not a
        // modelling claim that the sun is never up"). CrownArea carries
        // the highest `sun_exposure_fraction` (0.8) of any zone
        // (`topology_a380::build`).
        let mut high_sun = powered_ground_truth();
        high_sun.sun_elevation_deg = 60.0;
        let mut no_sun = powered_ground_truth();
        no_sun.sun_elevation_deg = -10.0; // below the horizon: zero flux
        let mut high = live_system();
        let mut low = live_system();
        run(high.as_mut(), &high_sun, &Faults::default(), 3000);
        run(low.as_mut(), &no_sun, &Faults::default(), 3000);
        let hot = published(high.as_ref())["THERMAL_ZONE_CROWNAREA_STRUCTURE_TEMPERATURE_C"];
        let cold = published(low.as_ref())["THERMAL_ZONE_CROWNAREA_STRUCTURE_TEMPERATURE_C"];
        assert!(hot > cold + 1.0, "a high sun elevation must warm a sun-exposed zone's structure more than no sun at all: {hot} vs {cold}");
    }

    #[test]
    fn a_below_horizon_sun_never_produces_a_negative_or_nonzero_flux() {
        assert_eq!(solar_flux_w_m2(&Truth { sun_elevation_deg: -5.0, ..Truth::default() }), 0.0);
        assert_eq!(solar_flux_w_m2(&Truth { sun_elevation_deg: 0.0, ..Truth::default() }), 0.0);
        assert!(solar_flux_w_m2(&Truth { sun_elevation_deg: 90.0, ..Truth::default() }) > 0.0);
    }

    #[test]
    fn a_nacelle_fire_drives_that_cowl_past_its_overheat_trigger_and_leaves_the_others_alone() {
        // Failure 11_026_005 (engine 2 nacelle fire) raises ENG 2 NAC OVHT
        // (trigger: THERMAL_ZONE_NACELLECOWL2_TEMPERATURE_C > 150 C).
        let truth = powered_ground_truth();
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(26, 5), 1.0)]);
        run(area.as_mut(), &truth, &armed, 200);
        let map = published(area.as_ref());
        assert!(map["THERMAL_ZONE_NACELLECOWL2_TEMPERATURE_C"] > 150.0, "got {}", map["THERMAL_ZONE_NACELLECOWL2_TEMPERATURE_C"]);
        assert!(map["THERMAL_ZONE_NACELLECOWL1_TEMPERATURE_C"] < 150.0, "engine 1's cowl has no fire");
    }

    #[test]
    fn a_blocked_nacelle_vent_scoop_makes_the_same_duct_leak_hotter() {
        // Failure 11_030_007 (engine 1 vent scoop ice blockage), effect:
        // "NacelleCowl1 loses its large ram-air ventilation term, so any
        // heat present (engine proximity, a duct leak) accumulates faster".
        let truth = powered_ground_truth();
        let leak_only = Faults::from_pairs([(f(30, 3), 1.0)]);
        let leak_and_blockage = Faults::from_pairs([(f(30, 3), 1.0), (f(30, 7), 1.0)]);
        let mut vented = live_system();
        let mut blocked = live_system();
        run(vented.as_mut(), &truth, &leak_only, 600);
        run(blocked.as_mut(), &truth, &leak_and_blockage, 600);
        let vented_c = published(vented.as_ref())["THERMAL_ZONE_NACELLECOWL1_TEMPERATURE_C"];
        let blocked_c = published(blocked.as_ref())["THERMAL_ZONE_NACELLECOWL1_TEMPERATURE_C"];
        assert!(blocked_c > vented_c + 10.0, "blocked {blocked_c} vs vented {vented_c}");
    }

    #[test]
    fn an_unarmed_cold_aircraft_publishes_finite_values_and_no_smoke() {
        let mut area = live_system();
        let truth = Truth::default();
        run(area.as_mut(), &truth, &Faults::default(), 100);
        for (name, value) in published(area.as_ref()) {
            assert!(value.is_finite(), "{name} went non-finite");
            if name.ends_with("_SMOKE_CONCENTRATION") {
                assert_eq!(value, 0.0, "{name} must be clean with nothing burning");
            }
        }
    }

    #[test]
    fn a_zero_length_frame_changes_nothing() {
        let mut area = live_system();
        let truth = Truth { dt_s: 0.0, ..powered_ground_truth() };
        area.tick(&truth, &Faults::default());
        let before = published(area.as_ref());
        area.tick(&truth, &Faults::default());
        assert_eq!(before, published(area.as_ref()));
    }

    // -----------------------------------------------------------------
    // ATA 36: the pylon bleed duct leak.
    //
    // These pin `pylon_bleed_leak_heat_w`'s derivation (its own doc
    // comment carries the arithmetic) and the two behaviours the fixed
    // 40 kW constant it replaced could not have: no heat from a duct that
    // is not pressurised, and no bay hotter than the air leaking into it.
    // -----------------------------------------------------------------

    /// Trent 972 IP8 bleed port at take-off power, Pa and K: the duct
    /// condition `Truth::engine_bleed_pressure_pa`/`_temp_k` carries at
    /// the pylon. Derived in [`pylon_bleed_leak_heat_w`]'s doc comment
    /// (fan hub PR ~1.75 x IPC PR ~5.5 = ~9.6x ambient; the same ratio
    /// through a ~0.90 polytropic efficiency gives 590 K).
    const TAKEOFF_IP8_PA: f64 = 970_000.0;
    const TAKEOFF_IP8_K: f64 = 590.0;

    fn takeoff_truth() -> Truth {
        Truth {
            dt_s: 1.0,
            ac_bus_volts: [115.0; 4],
            engine_running: [true; 4],
            engine_n1_frac: [1.0; 4],
            engine_bleed_pressure_pa: [TAKEOFF_IP8_PA; 4],
            engine_bleed_temp_k: [TAKEOFF_IP8_K; 4],
            ..Truth::default()
        }
    }

    #[test]
    fn a_pylon_leak_carries_the_cracks_own_choked_flow_and_nothing_more() {
        // Steps 3-4 of the derivation, at the two duct conditions it works
        // through. A 14 mm crack (2% of a 4 in bore) choked from the
        // take-off IP8 port passes 0.170 kg/s, worth 51.6 kW above a 15 C
        // bay; the same crack at the conditions the old 40 kW constant
        // claimed as its basis (200 C / 44 psig, downstream of the
        // precooler) passes 0.0792 kg/s, worth 14.7 kW -- which is what
        // makes 40 kW wrong by 2.7x against its own stated source, in the
        // opposite direction from "a real leak is hotter than we thought".
        let bay_k = 288.15;
        let ambient = 101_325.0;

        let takeoff = pylon_bleed_leak_heat_w(1.0, TAKEOFF_IP8_PA, TAKEOFF_IP8_K, bay_k, ambient);
        assert!((takeoff - 51_600.0).abs() < 300.0, "take-off IP8 port: expected ~51.6 kW from the derivation, got {takeoff}");

        let precooled = pylon_bleed_leak_heat_w(1.0, 404_694.0, 473.15, bay_k, ambient);
        assert!((precooled - 14_730.0).abs() < 200.0, "200 C / 44 psig: expected ~14.7 kW from the derivation, got {precooled}");
        assert!(precooled < 40_000.0 / 2.5, "the constant this replaced claimed these very conditions as its basis and was 2.7x larger than they give");

        // Severity is the crack's area, and choked flow is linear in it,
        // so at a fixed bay temperature the heat is linear in severity.
        let half = pylon_bleed_leak_heat_w(0.5, TAKEOFF_IP8_PA, TAKEOFF_IP8_K, bay_k, ambient);
        assert!((half - takeoff / 2.0).abs() < 1.0, "half the crack area must pass half the flow: {half} vs {takeoff}");
    }

    #[test]
    fn a_leak_from_an_unpressurised_duct_delivers_nothing() {
        // The failure this replaced injected 40 kW into the pylon of a
        // shut-down engine, because a fixed wattage has no way to know
        // there is nothing in the duct. `Truth::default()` is a cold
        // aircraft: bleed port at ambient pressure and temperature.
        let cold = Truth::default();
        let heat = pylon_bleed_leak_heat_w(1.0, cold.engine_bleed_pressure_pa[0], cold.engine_bleed_temp_k[0], 288.15, cold.environment.ambient_pressure_pa);
        assert_eq!(heat, 0.0, "a duct at ambient pressure has nothing to leak");

        let mut area = live_system();
        let armed = Faults::from_pairs([(f(36, 1), 1.0)]);
        run(area.as_mut(), &cold, &armed, 2000);
        let leaking = published(area.as_ref());
        let mut healthy_area = live_system();
        run(healthy_area.as_mut(), &cold, &Faults::default(), 2000);
        let healthy = published(healthy_area.as_ref());
        assert!(
            (leaking["THERMAL_ZONE_PYLONENGINE1_TEMPERATURE_C"] - healthy["THERMAL_ZONE_PYLONENGINE1_TEMPERATURE_C"]).abs() < 0.01,
            "a full-severity leak on a shut-down engine must not warm its pylon at all"
        );
    }

    #[test]
    fn a_pylon_bay_never_gets_hotter_than_the_air_leaking_into_it() {
        // The conservation property the fixed-wattage form does not have,
        // and the reason it had to go: `WING_DUCT_LEAK_MAX_HEAT_W` still
        // settles WingLeLeft at 853 C from a duct whose air is at 200 C
        // (measured; PROGRESS.md). Here the driving term is
        // mdot*cp*(T_duct - T_bay), so it closes off as the bay
        // approaches the duct, whatever the duct is. Run at a
        // deliberately cool duct, where a fixed wattage would sail
        // straight past it.
        let cool_duct_k = 350.0;
        let truth = Truth { engine_bleed_temp_k: [cool_duct_k; 4], ..takeoff_truth() };
        let mut area = live_system();
        let armed = Faults::from_pairs([(f(36, 1), 1.0)]);
        run(area.as_mut(), &truth, &armed, 6000);
        let bay_c = published(area.as_ref())["THERMAL_ZONE_PYLONENGINE1_TEMPERATURE_C"];
        assert!(bay_c < cool_duct_k - 273.15, "the bay reached {bay_c} C from a duct at {} C", cool_duct_k - 273.15);
        assert!(bay_c > 20.0, "and it must still be a real, substantial leak, not a rounding error: {bay_c} C");
    }

    #[test]
    fn a_pylon_leak_heats_only_its_own_bay_and_by_how_much_the_vent_flow_allows() {
        // End-to-end, with `deep::pneumatic_ducts` in the frame so its own
        // overheat loops read this area's published bay temperature the
        // way they do in the plugin.
        //
        // The leak is real and localised: engine 1's pylon runs ~73 K hot
        // at take-off port conditions while engine 2's, on the same wing,
        // does not move. But the bay's steady rise is set almost entirely
        // by its 0.5 kg/s ram vent (502 W/K, against ~25 W/K through
        // structure), so the steady balance is
        //
        //     mdot_leak*cp*(T_duct - T_bay) = mdot_vent*cp*(T_bay - T_out)
        //
        // and with mdot_leak = 0.170 kg/s, T_duct = 590 K and
        // T_out = 288 K that puts T_bay at ~361 K, i.e. ~73 K above
        // ambient. Reaching the 100 K margin
        // `pneumatic_ducts::odls::OverheatDetectionLoop::
        // THRESHOLD_ABOVE_AMBIENT_K` confirms on would need
        // mdot_leak = mdot_vent * 100/202 = 0.248 kg/s -- half the bay's
        // entire ventilation flow, from a 17 mm crack rather than a 14 mm
        // one. That is a number chosen to make a detector fire, not a
        // number derived from a duct, so it is not taken: the threshold is
        // what is wrong, and the recommendation is in PROGRESS.md. This
        // test asserts the physics rather than the neighbouring area's
        // trip flag, so it stays true once that recommendation is acted
        // on.
        let mut deep = crate::deep::live::Deep::new()
            .with_area(crate::deep::pneumatic_ducts::live::live_system())
            .with_area(live_system());
        let armed = Faults::from_pairs([(f(36, 1), 1.0)]);
        let truth = takeoff_truth();
        let mut published = BTreeMap::new();
        for _ in 0..3000 {
            deep.tick(truth.clone(), &armed, &mut |name, value| {
                published.insert(name.to_string(), value);
            });
        }
        let leaking = published["THERMAL_ZONE_PYLONENGINE1_TEMPERATURE_C"];
        let untouched = published["THERMAL_ZONE_PYLONENGINE2_TEMPERATURE_C"];
        assert!(leaking - untouched > 60.0, "a full-severity pylon leak at take-off port conditions must heat its own bay substantially: {leaking} C vs {untouched} C");
        assert!(leaking < TAKEOFF_IP8_K - 273.15, "and never past the duct feeding it: {leaking} C");
        assert!(
            leaking - untouched < 100.0,
            "the ram-vented bay cannot reach the 100 K margin pneumatic_ducts::odls confirms on -- if this ever exceeds 100 K, the leak's mass flow has been inflated past what the crack passes: {} K",
            leaking - untouched
        );
    }

}

