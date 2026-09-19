//! The shared dispatcher: wraps every model's own event/outcome type in
//! one [`EnvironmentEvent`], and [`route`] turns each into the plain,
//! consumer-facing output structs the five owning systems actually read
//! (engines, sensors, structure, electrical buses, thermal zones) --
//! instead of every consumer needing to know all seven environment
//! models' individual output types. Nothing new is modelled here: every
//! number below already comes from the cited/GENERIC physics in the
//! module it is read from; this file only re-shapes it per consumer.
//! `registry.rs` documents which failures/components each field maps to;
//! see this file's own doc comments for the one genuinely new coupling it
//! introduces (lightning's nacelle/radome arc-root heat pulse).
//!
//! `wind_shear`'s F-factor/turbulence and `weather_cells`' derived
//! intensities are *not* wrapped in `EnvironmentEvent`: they are
//! continuous kinematic inputs the flight model/PWS reads directly from
//! those modules' own `step`, the same way they work on the real
//! aircraft (see `registry.rs`'s header), not discrete events with a
//! component consequence.

use super::{bird_strike, hail, ice_crystal_icing, lightning, runway_contamination, volcanic_ash};
#[cfg(test)]
use super::rng::Rng;

/// One occurrence from any of this directory's discrete/per-step hazard
/// models, tagged with whatever index its own model needs (an engine
/// number for the per-engine continuous models, none for the others since
/// their own outcome already carries a `target`).
#[derive(Clone, Debug)]
pub enum EnvironmentEvent {
    BirdStrike(bird_strike::StrikeOutcome),
    Lightning(lightning::LightningEvent),
    Hail(hail::HailOutcome),
    /// `volcanic_ash::VolcanicAshState` is stepped once per engine (its
    /// `AshInputs::core_mass_flow_kg_s` is that engine's own flow); its
    /// airframe-wide fields (pitot/windshield/cabin) are only meaningful
    /// once and are forwarded by `route` from `engine_index == 0` only,
    /// to avoid quadruplicating the same effect.
    VolcanicAsh { engine_index: usize, outputs: volcanic_ash::AshOutputs },
    IceCrystalIcing { engine_index: usize, outputs: ice_crystal_icing::IceCrystalOutputs },
    RunwayContamination(runway_contamination::RunwayFrictionOutput),
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EngineEffect {
    pub engine_index: usize,
    pub fan_damage_frac: f64,
    pub core_ingestion_frac: f64,
    pub flow_capacity_loss_frac: f64,
    pub compressor_efficiency_loss_frac: f64,
    pub rollback_risk_frac: f64,
    pub flameout_risk_frac: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SensorId {
    /// Pitot/static/AoA/TAT probe or small external antenna, 0-based.
    AirDataProbe(u8),
    StandbyCompass,
}

#[derive(Clone, Copy, Debug)]
pub struct SensorEffect {
    pub sensor: SensorId,
    /// Blockage/damage fraction for a probe, heading error in degrees for
    /// the compass -- whichever that sensor's own registered parameter
    /// means.
    pub magnitude: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StructureLocation {
    Windshield(u8),
    Radome,
    WingLeadingEdge(u8),
    NoseGear,
    /// Wingtip/tail composite skin (lightning attach/exit points not
    /// otherwise named).
    CompositeExtremity,
    /// Engine inlet lip/cowl, 0-based by engine.
    Nacelle(u8),
    /// Not a single aircraft part: the runway surface state every gear's
    /// braking model reads (see `registry.rs`'s `32_env.runway_friction`).
    RunwayFriction,
}

#[derive(Clone, Copy, Debug)]
pub struct StructureEffect {
    pub location: StructureLocation,
    pub damage_frac: f64,
    pub drag_delta_cd: f64,
    /// `Radome` only.
    pub wxr_attenuation_frac: f64,
    /// `Windshield` only.
    pub window_heat_fault: bool,
    pub visibility_loss_frac: f64,
    pub leak_area_m2: f64,
    /// `WingLeadingEdge` only.
    pub clmax_delta: f64,
    pub slat_jam_risk_frac: f64,
}

impl StructureEffect {
    fn blank(location: StructureLocation) -> Self {
        Self { location, damage_frac: 0.0, drag_delta_cd: 0.0, wxr_attenuation_frac: 0.0, window_heat_fault: false, visibility_loss_frac: 0.0, leak_area_m2: 0.0, clmax_delta: 0.0, slat_jam_risk_frac: 0.0 }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ElectricalBusEffect {
    pub bus: lightning::BusId,
    pub transient_volts: f64,
    pub upset_likely: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThermalZone {
    EngineNacelle(u8),
    Radome,
}

#[derive(Clone, Copy, Debug)]
pub struct ThermalZoneEffect {
    pub zone: ThermalZone,
    pub heat_pulse_j: f64,
}

/// Every consumer-facing effect one [`EnvironmentEvent`] produced. More
/// than one entry can land on the same engine/location/bus in one
/// `ConsumerEffects` (e.g. ash and ice both reaching engine 2 the same
/// tick): consumers should combine them (sum wear-type fields, take the
/// max of instantaneous risk fractions) rather than assume one entry per
/// target.
#[derive(Clone, Debug, Default)]
pub struct ConsumerEffects {
    pub engines: Vec<EngineEffect>,
    pub sensors: Vec<SensorEffect>,
    pub structure: Vec<StructureEffect>,
    pub electrical: Vec<ElectricalBusEffect>,
    pub thermal: Vec<ThermalZoneEffect>,
    /// Cabin/environmental-system (ATA 21) odour intensity: fits none of
    /// the five categories above (not an engine, sensor, structural,
    /// electrical or thermal-zone effect), so it is passed through
    /// directly instead of forced into an unrelated one.
    pub cabin_odor_intensity: f64,
}

/// GENERIC: a simplified proxy for arc-root heating at a lightning
/// attachment/exit point, J per kA of peak current. Standard aircraft
/// direct-effects analyses model arc-root heating as an anode/cathode
/// voltage drop (order 10-100 V) times the charge transferred; this
/// module does not integrate the full current waveform for charge, so it
/// scales directly off peak current instead, calibrated only to put a
/// severe (200 kA) strike's heat pulse in the broad range those analyses
/// use, not to reproduce one.
const ARC_ROOT_J_PER_KA: f64 = 40.0;

pub fn route(event: &EnvironmentEvent) -> ConsumerEffects {
    let mut out = ConsumerEffects::default();
    match event {
        EnvironmentEvent::BirdStrike(o) => route_bird_strike(o, &mut out),
        EnvironmentEvent::Lightning(e) => route_lightning(e, &mut out),
        EnvironmentEvent::Hail(o) => route_hail(o, &mut out),
        EnvironmentEvent::VolcanicAsh { engine_index, outputs } => route_ash(*engine_index, outputs, &mut out),
        EnvironmentEvent::IceCrystalIcing { engine_index, outputs } => route_ice(*engine_index, outputs, &mut out),
        EnvironmentEvent::RunwayContamination(o) => route_runway(o, &mut out),
    }
    out
}

fn route_bird_strike(o: &bird_strike::StrikeOutcome, out: &mut ConsumerEffects) {
    match o.target {
        bird_strike::ImpactTarget::EngineInlet(i) => {
            out.engines.push(EngineEffect { engine_index: i as usize, fan_damage_frac: o.fan_damage_frac, core_ingestion_frac: o.core_ingestion_frac, ..Default::default() });
        }
        bird_strike::ImpactTarget::Windshield(i) => {
            let mut s = StructureEffect::blank(StructureLocation::Windshield(i));
            s.damage_frac = if o.windshield_penetrated { 1.0 } else if o.windshield_crack { 0.6 } else { 0.0 };
            out.structure.push(s);
        }
        bird_strike::ImpactTarget::Radome => {
            let mut s = StructureEffect::blank(StructureLocation::Radome);
            s.damage_frac = o.radome_damage_frac;
            out.structure.push(s);
        }
        bird_strike::ImpactTarget::WingLeadingEdge(i) => {
            let mut s = StructureEffect::blank(StructureLocation::WingLeadingEdge(i));
            s.drag_delta_cd = o.leading_edge_dent_drag_delta_cd;
            out.structure.push(s);
        }
        bird_strike::ImpactTarget::NoseGear => {
            let mut s = StructureEffect::blank(StructureLocation::NoseGear);
            s.damage_frac = o.nose_gear_damage_frac;
            out.structure.push(s);
        }
        bird_strike::ImpactTarget::PitotAoaProbe(i) => {
            out.sensors.push(SensorEffect { sensor: SensorId::AirDataProbe(i), magnitude: if o.probe_blocked { 1.0 } else { 0.0 } });
        }
    }
}

fn route_lightning(e: &lightning::LightningEvent, out: &mut ConsumerEffects) {
    if e.radome_damage_frac > 0.0 {
        let mut s = StructureEffect::blank(StructureLocation::Radome);
        s.damage_frac = e.radome_damage_frac;
        out.structure.push(s);
    }
    if e.structure_damage_frac > 0.0 {
        let mut s = StructureEffect::blank(StructureLocation::CompositeExtremity);
        s.damage_frac = e.structure_damage_frac;
        out.structure.push(s);
    }
    for t in &e.transients {
        out.electrical.push(ElectricalBusEffect { bus: t.bus, transient_volts: t.peak_volts, upset_likely: t.upset_likely });
    }
    if e.compass_error_deg > 0.0 {
        out.sensors.push(SensorEffect { sensor: SensorId::StandbyCompass, magnitude: e.compass_error_deg });
    }
    // A genuinely new coupling not modelled inside lightning.rs itself:
    // an attachment/exit right at an engine nacelle or the radome puts a
    // localised heat pulse into that bay's thermal state (GENERIC, see
    // `ARC_ROOT_J_PER_KA`), on top of the skin damage already accounted
    // for above. Not yet given its own `registry.rs` failure/component
    // (time-boxed out of this pass); today it rides on the same
    // already-registered "Lightning composite extremity burn"/"Lightning
    // radome damage" failures for its magnitude.
    for p in [e.entry, e.exit] {
        if let lightning::AttachPoint::EngineNacelle(i) = p {
            out.thermal.push(ThermalZoneEffect { zone: ThermalZone::EngineNacelle(i), heat_pulse_j: ARC_ROOT_J_PER_KA * e.peak_current_ka });
        } else if p == lightning::AttachPoint::NoseRadome {
            out.thermal.push(ThermalZoneEffect { zone: ThermalZone::Radome, heat_pulse_j: ARC_ROOT_J_PER_KA * e.peak_current_ka });
        }
    }
}

fn route_hail(o: &hail::HailOutcome, out: &mut ConsumerEffects) {
    match o.target {
        hail::ImpactTarget::Radome => {
            let mut s = StructureEffect::blank(StructureLocation::Radome);
            s.damage_frac = o.damage_frac;
            s.drag_delta_cd = o.drag_delta_cd;
            s.wxr_attenuation_frac = o.wxr_attenuation_frac;
            out.structure.push(s);
        }
        hail::ImpactTarget::Windshield(i) => {
            let mut s = StructureEffect::blank(StructureLocation::Windshield(i));
            s.damage_frac = o.damage_frac;
            s.window_heat_fault = o.window_heat_fault;
            s.visibility_loss_frac = o.visibility_loss_frac;
            s.leak_area_m2 = o.leak_area_m2;
            out.structure.push(s);
        }
        hail::ImpactTarget::WingLeadingEdge(i) => {
            let mut s = StructureEffect::blank(StructureLocation::WingLeadingEdge(i));
            s.damage_frac = o.damage_frac;
            s.drag_delta_cd = o.drag_delta_cd;
            s.clmax_delta = o.clmax_delta;
            s.slat_jam_risk_frac = o.slat_jam_risk_frac;
            out.structure.push(s);
        }
        hail::ImpactTarget::EngineInlet(i) => {
            out.engines.push(EngineEffect {
                engine_index: i as usize,
                fan_damage_frac: o.fan_damage_frac,
                compressor_efficiency_loss_frac: o.compressor_efficiency_loss_frac,
                flameout_risk_frac: o.flameout_risk_frac,
                ..Default::default()
            });
        }
        hail::ImpactTarget::Nacelle(i) => {
            let mut s = StructureEffect::blank(StructureLocation::Nacelle(i));
            s.damage_frac = o.damage_frac;
            s.drag_delta_cd = o.drag_delta_cd;
            out.structure.push(s);
        }
        hail::ImpactTarget::Probe(i) => {
            out.sensors.push(SensorEffect { sensor: SensorId::AirDataProbe(i), magnitude: o.damage_frac });
        }
    }
}

fn route_ash(engine_index: usize, outputs: &volcanic_ash::AshOutputs, out: &mut ConsumerEffects) {
    out.engines.push(EngineEffect {
        engine_index,
        flow_capacity_loss_frac: outputs.flow_capacity_loss_frac,
        compressor_efficiency_loss_frac: outputs.compressor_efficiency_loss_frac,
        flameout_risk_frac: outputs.flameout_risk_frac,
        ..Default::default()
    });
    if engine_index == 0 {
        out.sensors.push(SensorEffect { sensor: SensorId::AirDataProbe(0), magnitude: outputs.pitot_blockage_frac });
        let mut s = StructureEffect::blank(StructureLocation::Windshield(0));
        s.visibility_loss_frac = outputs.windshield_visibility_loss_frac;
        out.structure.push(s);
        out.cabin_odor_intensity = outputs.cabin_odor_intensity;
    }
}

fn route_ice(engine_index: usize, outputs: &ice_crystal_icing::IceCrystalOutputs, out: &mut ConsumerEffects) {
    out.engines.push(EngineEffect {
        engine_index,
        flow_capacity_loss_frac: outputs.flow_capacity_loss_frac,
        rollback_risk_frac: outputs.rollback_risk_frac,
        flameout_risk_frac: outputs.flameout_risk_frac,
        ..Default::default()
    });
}

fn route_runway(o: &runway_contamination::RunwayFrictionOutput, out: &mut ConsumerEffects) {
    let mut s = StructureEffect::blank(StructureLocation::RunwayFriction);
    // Matches the GENERIC dry-reference scaling already registered for
    // this failure (`registry.rs`'s "Runway contamination friction/
    // hydroplaning loss").
    s.damage_frac = 1.0 - o.mu_effective / 0.40;
    out.structure.push(s);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn empty_bird(target: bird_strike::ImpactTarget) -> bird_strike::StrikeOutcome {
        bird_strike::StrikeOutcome {
            target,
            bird: bird_strike::BirdClass::Medium,
            bird_count: 1,
            impact_speed_ms: 200.0,
            impact_energy_j: 1000.0,
            fan_damage_frac: 0.4,
            core_ingestion_frac: 0.1,
            windshield_crack: true,
            windshield_penetrated: false,
            radome_damage_frac: 0.5,
            leading_edge_dent_drag_delta_cd: 0.01,
            nose_gear_damage_frac: 0.3,
            probe_blocked: true,
        }
    }

    #[test]
    fn bird_strike_on_an_engine_routes_to_engine_effects_only() {
        let effects = route(&EnvironmentEvent::BirdStrike(empty_bird(bird_strike::ImpactTarget::EngineInlet(2))));
        assert_eq!(effects.engines.len(), 1);
        assert_eq!(effects.engines[0].engine_index, 2);
        assert_eq!(effects.engines[0].fan_damage_frac, 0.4);
        assert!(effects.structure.is_empty() && effects.sensors.is_empty());
    }

    #[test]
    fn bird_strike_on_a_probe_routes_to_sensors() {
        let effects = route(&EnvironmentEvent::BirdStrike(empty_bird(bird_strike::ImpactTarget::PitotAoaProbe(3))));
        assert_eq!(effects.sensors.len(), 1);
        assert_eq!(effects.sensors[0].sensor, SensorId::AirDataProbe(3));
        assert_eq!(effects.sensors[0].magnitude, 1.0);
    }

    #[test]
    fn hail_on_a_windshield_carries_every_windshield_consequence() {
        let mut damage = hail::HailDamageState::new();
        let outcome = damage.strike(hail::ImpactTarget::Windshield(1), 30.0, 200.0, 15, 1.0);
        let effects = route(&EnvironmentEvent::Hail(outcome));
        assert_eq!(effects.structure.len(), 1);
        let s = &effects.structure[0];
        assert_eq!(s.location, StructureLocation::Windshield(1));
        assert!(s.window_heat_fault);
        assert!(s.leak_area_m2 > 0.0);
        assert!(s.visibility_loss_frac > 0.0);
    }

    #[test]
    fn hail_on_an_engine_feeds_the_same_engine_effect_fields_bird_strike_uses() {
        let mut damage = hail::HailDamageState::new();
        let outcome = damage.strike(hail::ImpactTarget::EngineInlet(0), 20.0, 100.0, 5, 0.2);
        let effects = route(&EnvironmentEvent::Hail(outcome));
        assert_eq!(effects.engines.len(), 1);
        assert!(effects.engines[0].fan_damage_frac > 0.0);
        assert!(effects.engines[0].compressor_efficiency_loss_frac > 0.0);
        assert!(effects.engines[0].flameout_risk_frac > 0.0);
    }

    #[test]
    fn hail_on_a_probe_routes_to_sensors_like_a_bird_strike_does() {
        let mut damage = hail::HailDamageState::new();
        let outcome = damage.strike(hail::ImpactTarget::Probe(2), 10.0, 150.0, 3, 1.0);
        let effects = route(&EnvironmentEvent::Hail(outcome));
        assert_eq!(effects.sensors.len(), 1);
        assert_eq!(effects.sensors[0].sensor, SensorId::AirDataProbe(2));
    }

    #[test]
    fn lightning_at_a_nacelle_raises_a_thermal_effect_and_electrical_transients() {
        let mut rng = Rng::new(1);
        let event = lightning::resolve(lightning::AttachPoint::EngineNacelle(1), lightning::AttachPoint::VStabTip, 180.0, &mut rng);
        let effects = route(&EnvironmentEvent::Lightning(event));
        assert!(effects.thermal.iter().any(|t| t.zone == ThermalZone::EngineNacelle(1) && t.heat_pulse_j > 0.0));
        assert!(!effects.electrical.is_empty());
    }

    #[test]
    fn runway_contamination_routes_to_a_dedicated_structure_location() {
        let out = runway_contamination::friction(runway_contamination::Contaminant::Ice, 200.0, 30.0);
        let effects = route(&EnvironmentEvent::RunwayContamination(out));
        assert_eq!(effects.structure.len(), 1);
        assert_eq!(effects.structure[0].location, StructureLocation::RunwayFriction);
        assert!(effects.structure[0].damage_frac > 0.0);
    }
}
