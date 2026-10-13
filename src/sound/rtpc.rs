use std::collections::{BTreeSet, HashMap};

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use super::triggers::{RtpcInput, Trigger};
use crate::xp::DataRef;
use crate::Vars;

const KNOTS_PER_MS: f64 = 1.943_844;
const SLUG_FT3_PER_KG_M3: f64 = 0.001_940_32;

#[derive(Clone, Copy, Debug)]
enum Read {
    Var { id: VariableIdentifier, scale: f64, offset: f64 },
    Deadband { id: VariableIdentifier, dead: f64, scale: f64 },
    Dref { dref: DataRef, index: Option<usize>, scale: f64 },
    GearMean(DataRef),
}

struct Input {
    game_param: u32,
    read: Option<Read>,
    derived: bool,
    prev: Option<f64>,
}

pub struct Inputs {
    per_trigger: Vec<Vec<Input>>,
    pub unfed: Vec<String>,
}

fn with_index(variable: &str, index: Option<u32>) -> String {
    match (variable.rsplit_once(':'), index) {
        (Some((base, n)), Some(i)) if n.trim().parse::<u32>().is_ok() => format!("{base}:{i}"),
        _ => variable.to_owned(),
    }
}

pub fn inferred(trigger: &Trigger, used: &BTreeSet<u32>, known: &HashMap<u32, RtpcInput>) -> Vec<RtpcInput> {
    let fed: BTreeSet<u32> = trigger.rtpcs.iter().map(|r| r.game_param).collect();
    let index = trigger.index();
    used.iter()
        .filter(|p| !fed.contains(p))
        .filter_map(|p| known.get(p))
        .map(|g| {
            let mut r = g.clone();
            if r.is_local {
                r.variable = with_index(&r.variable, index);
            } else if r.index > 0 {
                if let Some(i) = index {
                    r.index = i;
                }
            }
            r
        })
        .collect()
}

pub fn known_inputs(triggers: &[Trigger]) -> HashMap<u32, RtpcInput> {
    let mut out = HashMap::new();
    for t in triggers {
        for r in &t.rtpcs {
            out.entry(r.game_param).or_insert_with(|| r.clone());
        }
    }
    out
}

const RATTLE_DEADBAND_DEG_S: f64 = 0.5;

fn deadband(v: f64, dead: f64) -> f64 {
    v.signum() * (v.abs() - dead).max(0.)
}

fn simvar_read(vars: &mut Vars, name: &str, index: u32, units: &str) -> Option<Read> {
    let xplm = vars.xplm;
    let n = name.trim().to_ascii_uppercase();
    let u = units.trim().to_ascii_uppercase();
    let i = index.max(1) as usize;
    let var = |vars: &mut Vars, v: String, scale: f64, offset: f64| Some(Read::Var { id: vars.get(v), scale, offset });
    let dref = |path: &str, index: Option<usize>, scale: f64| xplm.find(path).map(|dref| Read::Dref { dref, index, scale });
    match n.as_str() {
        "TURB ENG N1" => var(vars, format!("A32NX_ENGINE_N1:{i}"), 1., 0.),
        "TURB ENG N2" => var(vars, format!("A32NX_ENGINE_N2:{i}"), 1., 0.),
        "GENERAL ENG COMBUSTION SOUND PERCENT" => var(vars, format!("ENG COMBUSTION:{i}"), if u == "PERCENT" { 100. } else { 1. }, 0.),
        "AIRSPEED TRUE" | "AIRSPEED INDICATED" | "PLANE ALT ABOVE GROUND" | "VERTICAL SPEED" | "SIM ON GROUND" | "AMBIENT TEMPERATURE"
        | "AMBIENT PRECIP RATE" | "SURFACE TYPE" | "CABIN SEATBELTS ALERT SWITCH" => var(vars, n.clone(), 1., 0.),
        "AMBIENT DENSITY" => dref("sim/weather/rho", None, SLUG_FT3_PER_KG_M3),
        "GROUND VELOCITY" => dref("sim/flightmodel/position/groundspeed", None, if u.starts_with("KNOT") { KNOTS_PER_MS } else { 1. }),
        "WHEEL RPM" => var(vars, format!("WHEEL RPM:{}", index), 1., 0.),
        "GEAR TOTAL PCT EXTENDED" => xplm.find("sim/flightmodel2/gear/deploy_ratio").map(Read::GearMean),
        "GEAR ANIMATION POSITION" => dref("sim/flightmodel2/gear/deploy_ratio", Some(index as usize), 100.),
        "ELEVATOR POSITION" => var(vars, "A32NX_HYD_ELEV_LEFT_INWARD_DEFLECTION".into(), 200., -100.),
        "AILERON POSITION" => var(vars, "A32NX_HYD_AIL_LEFT_INWARD_DEFLECTION".into(), 200., -100.),
        "RUDDER POSITION" => var(vars, "A32NX_HYD_UPPER_RUD_DEFLECTION".into(), 200., -100.),
        "RUDDER PEDAL POSITION" => var(vars, "A32NX_RUDDER_PEDAL_POSITION".into(), 1., 0.),
        "ELEVATOR TRIM POSITION" => dref("sim/cockpit2/controls/elevator_trim", None, 100.),
        "YOKE X POSITION" => dref("sim/joystick/yoke_roll_ratio", None, 100.),
        "YOKE Y POSITION" => dref("sim/joystick/yoke_pitch_ratio", None, 100.),
        "ROTATION VELOCITY BODY X" | "ROTATION VELOCITY BODY Y" | "ROTATION VELOCITY BODY Z" => {
            Some(Read::Deadband { id: vars.get(n.clone()), dead: RATTLE_DEADBAND_DEG_S, scale: std::f64::consts::PI / 180. * 100. })
        }
        "LEADING EDGE FLAPS LEFT PERCENT" => var(vars, "A32NX_LEFT_SLATS_POSITION_PERCENT".into(), 1., 0.),
        "LEADING EDGE FLAPS RIGHT PERCENT" => var(vars, "A32NX_RIGHT_SLATS_POSITION_PERCENT".into(), 1., 0.),
        "CIRCUIT ON" => var(vars, format!("CIRCUIT SWITCH ON:{index}"), 1., 0.),
        "CIRCUIT POWER SETTING" => var(vars, format!("CIRCUIT POWER SETTING:{index}"), 1., 0.),
        "ELECTRICAL MAIN BUS VOLTAGE" => var(vars, "A32NX_ELEC_DC_1_BUS_IS_POWERED".into(), 28., 0.),
        _ if index > 0 => var(vars, format!("{n}:{index}"), 1., 0.),
        _ => var(vars, n.clone(), 1., 0.),
    }
}

fn resolve(vars: &mut Vars, r: &RtpcInput) -> Option<Read> {
    if r.is_local {
        return Some(Read::Var { id: vars.get(r.variable.clone()), scale: 1., offset: 0. });
    }
    simvar_read(vars, &r.variable, r.index, &r.units)
}

fn read_value(vars: &mut Vars, read: Read) -> f64 {
    match read {
        Read::Var { id, scale, offset } => vars.read(&id) * scale + offset,
        Read::Deadband { id, dead, scale } => deadband(vars.read(&id), dead) * scale,
        Read::Dref { dref, index: None, scale } => vars.xplm.get_f(dref) as f64 * scale,
        Read::Dref { dref, index: Some(i), scale } => {
            let mut buf = [0f32; 16];
            let i = i.min(15);
            let n = vars.xplm.get_vf(dref, &mut buf[..=i]);
            if n > i { buf[i] as f64 * scale } else { 0. }
        }
        Read::GearMean(dref) => {
            let mut buf = [0f32; 5];
            let n = vars.xplm.get_vf(dref, &mut buf).max(1);
            buf[..n].iter().map(|&v| v as f64).sum::<f64>() / n as f64 * 100.
        }
    }
}

impl Inputs {
    pub fn new(vars: &mut Vars, per_trigger: Vec<Vec<RtpcInput>>) -> Self {
        let mut unfed = Vec::new();
        let per_trigger = per_trigger
            .into_iter()
            .map(|list| {
                list.into_iter()
                    .map(|r| {
                        let read = resolve(vars, &r);
                        if read.is_none() {
                            unfed.push(format!("{} ({})", r.name, r.variable));
                        }
                        Input { game_param: r.game_param, read, derived: r.derived, prev: None }
                    })
                    .collect()
            })
            .collect();
        unfed.sort();
        unfed.dedup();
        Self { per_trigger, unfed }
    }

    pub fn values(&mut self, trigger: usize, vars: &mut Vars, dt: f64) -> HashMap<u32, f32> {
        let mut out = HashMap::new();
        let Some(list) = self.per_trigger.get_mut(trigger) else { return out };
        for input in list.iter_mut() {
            let Some(read) = input.read else { continue };
            let v = read_value(vars, read);
            let value = if input.derived {
                let d = match input.prev {
                    Some(p) if dt > 1e-6 => (v - p) / dt,
                    _ => 0.,
                };
                input.prev = Some(v);
                d
            } else {
                v
            };
            out.insert(input.game_param, value as f32);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_cockpit_rattle_ignores_rotation_below_half_a_degree_a_second() {
        assert_eq!(deadband(0.3, RATTLE_DEADBAND_DEG_S), 0.);
        assert_eq!(deadband(-0.49, RATTLE_DEADBAND_DEG_S), 0.);
        assert!((deadband(2.0, RATTLE_DEADBAND_DEG_S) - 1.5).abs() < 1e-12);
        assert!((deadband(-2.0, RATTLE_DEADBAND_DEG_S) + 1.5).abs() < 1e-12);
    }

    fn input(name: &str, variable: &str, is_local: bool, index: u32) -> RtpcInput {
        RtpcInput { game_param: crate::sound::wwise::short_id(name), name: name.into(), variable: variable.into(), is_local, index, units: String::new(), derived: false }
    }

    #[test]
    fn an_unfed_parameter_borrows_another_entrys_input_for_its_own_engine() {
        let n1 = input("SIMVAR_TURB_ENG_N1", "A32NX_ENGINE_N1:1", true, 0);
        let tas = input("SIMVAR_AIRSPEED_TRUE", "AIRSPEED TRUE", false, 1);
        let known: HashMap<u32, RtpcInput> = [(n1.game_param, n1.clone()), (tas.game_param, tas.clone())].into_iter().collect();
        let xml = r#"<SoundInfo><SimVarSounds>
            <Sound WwiseEvent="Engine_3_FF" NodeName="SOUND_ENGINE_3" LocalVar="A32NX_ENGINE_N2:3"><Range LowerBound="1"/></Sound>
        </SimVarSounds></SoundInfo>"#;
        let t = &super::super::triggers::parse(xml).unwrap().triggers[0];
        let used: BTreeSet<u32> = [n1.game_param, tas.game_param, 12345].into_iter().collect();
        let got = inferred(t, &used, &known);
        assert_eq!(got.len(), 2);
        assert!(got.iter().any(|r| r.variable == "A32NX_ENGINE_N1:3"), "{got:?}");
        assert!(got.iter().any(|r| r.variable == "AIRSPEED TRUE" && r.index == 3));
    }
}
