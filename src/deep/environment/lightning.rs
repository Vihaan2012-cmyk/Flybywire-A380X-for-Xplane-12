//! Lightning strike: attachment/exit point selection, the conducted current
//! waveform's severity, which buses/computers see an induced transient,
//! standby-compass deviation, and radome/structure damage.
//!
//! ## Sources
//! - Zoning (`Zone1`/`Zone2`/`Zone3`) and the general attachment pattern
//!   (initial attachment at a forward extremity -- nose, wingtips -- with
//!   the arc then swept aft by the airflow and a second attachment/exit
//!   nearer the tail) are from SAE ARP5414 ("Aircraft Lightning Zoning")
//!   and the classic reference Fisher & Plumer, *Lightning Protection of
//!   Aircraft* (NASA RP-1008, public).
//! - Current severity: SAE ARP5412 ("Aircraft Lightning Environment and
//!   Related Test Waveforms") component A (first return stroke): 200 kA
//!   peak, 2x10^6 A^2s action integral -- the certification design value,
//!   not a typical strike. Typical strike currents are much lower: Berger
//!   et al.'s widely cited ground-based lightning current statistics give
//!   a median first-stroke peak of about 30 kA; used below as the median
//!   of a GENERIC log-normal peak-current distribution whose extreme tail
//!   reaches the 200 kA design value.
//! - "A commercial aircraft is struck by lightning on the order of once a
//!   year" is a widely repeated industry figure (Boeing/FAA public
//!   guidance); used to calibrate the GENERIC random-mode base rate as an
//!   order of magnitude, not a fitted rate.
//! - Bus/computer exposure factors, the induced-voltage scaling constant,
//!   the upset threshold, radome diverter-strip coverage and the standby-
//!   compass deviation model are all `GENERIC`, derived as documented at
//!   each constant: no per-type A380 lightning qualification report is
//!   public. The qualitative shape (shielded interior computers are far
//!   less exposed than nacelle- or extremity-mounted equipment; a direct
//!   composite-skin hit without its diverter strip does far more damage
//!   than a well-protected one) is standard and well documented; the
//!   numbers are order-of-magnitude placeholders for it.

use super::rng::Rng;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttachPoint {
    NoseRadome,
    WingtipLeft,
    WingtipRight,
    VStabTip,
    HStabTipLeft,
    HStabTipRight,
    EngineNacelle(u8),
}

/// A bus or computer the electrical/avionics side owns; this module only
/// says which ones see a transient and how big it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BusId {
    Prim(u8),
    Sec(u8),
    Fmgc(u8),
    Adirs(u8),
    StandbyInstruments,
    EngineFadec(u8),
    Ife,
}

/// GENERIC coupling factor, 0..1: how much of the strike current's field
/// reaches this bus's wiring, reflecting mounting location and shielding
/// (well-shielded avionics-bay computers on a well-bonded aluminium
/// fuselage couple weakly; equipment mounted out on the pylon/nacelle, or
/// wiring runs through the composite wingtip/tail extremities, couple much
/// more strongly -- the qualitative ordering is standard aircraft
/// lightning-protection practice, the exact figures are not).
fn exposure_factor(bus: BusId) -> f64 {
    match bus {
        BusId::Prim(_) | BusId::Sec(_) | BusId::Fmgc(_) => 0.06,
        BusId::Adirs(_) => 0.08,
        BusId::StandbyInstruments => 0.15,
        BusId::EngineFadec(_) => 0.35,
        BusId::Ife => 0.25,
    }
}

/// GENERIC: induced open-circuit transient, volts per kA of strike peak
/// current, at `exposure_factor` 1.0.
const INDUCED_V_PER_KA: f64 = 1.0;
/// GENERIC: typical avionics equipment's DO-160-style conducted-transient
/// immunity, volts, above which an upset (reset, data corruption) is
/// judged likely.
const UPSET_THRESHOLD_V: f64 = 50.0;

#[derive(Clone, Copy, Debug)]
pub struct BusTransient {
    pub bus: BusId,
    pub peak_volts: f64,
    pub upset_likely: bool,
}

fn transient_for(bus: BusId, peak_current_ka: f64) -> BusTransient {
    let v = peak_current_ka.max(0.0) * exposure_factor(bus) * INDUCED_V_PER_KA;
    BusTransient { bus, peak_volts: v, upset_likely: v > UPSET_THRESHOLD_V }
}

/// All buses this module reports on, for a full-aircraft sweep each strike.
fn all_buses() -> [BusId; 17] {
    [
        BusId::Prim(1),
        BusId::Prim(2),
        BusId::Prim(3),
        BusId::Sec(1),
        BusId::Sec(2),
        BusId::Sec(3),
        BusId::Fmgc(1),
        BusId::Fmgc(2),
        BusId::Adirs(1),
        BusId::Adirs(2),
        BusId::Adirs(3),
        BusId::StandbyInstruments,
        BusId::Ife,
        BusId::EngineFadec(0),
        BusId::EngineFadec(1),
        BusId::EngineFadec(2),
        BusId::EngineFadec(3),
    ]
}

/// What a strike did.
#[derive(Clone, Debug)]
pub struct LightningEvent {
    pub entry: AttachPoint,
    pub exit: AttachPoint,
    pub peak_current_ka: f64,
    /// SAE ARP5412 action integral scaling: `2e6 A^2s * (peak/200kA)^2`.
    pub action_integral_a2s: f64,
    pub transients: Vec<BusTransient>,
    /// Added to the standby magnetic compass reading until the next
    /// compass swing, degrees.
    pub compass_error_deg: f64,
    pub radome_damage_frac: f64,
    pub structure_damage_frac: f64,
}

const ARP5412_PEAK_KA: f64 = 200.0;
const ARP5412_ACTION_INTEGRAL_A2S: f64 = 2.0e6;
/// GENERIC log-normal median peak current, kA (Berger et al. first-stroke
/// median, see module doc).
const MEDIAN_PEAK_KA: f64 = 30.0;
/// GENERIC log-normal shape parameter, chosen so the ARP5412 200 kA design
/// value sits a little above the extreme (~99.9th percentile) tail.
const PEAK_KA_SIGMA: f64 = 0.7;
/// GENERIC: fraction of nose-radome strikes that land on the protective
/// diverter strip network rather than raw composite.
const DIVERTER_COVERAGE: f64 = 0.9;
/// GENERIC ceiling on post-strike standby-compass deviation before a
/// compass swing, degrees (maintenance guidance calls for a swing after
/// any suspected strike; no public figure gives a magnitude distribution).
const MAX_COMPASS_ERROR_DEG: f64 = 10.0;

fn sample_peak_current_ka(rng: &mut Rng) -> f64 {
    (MEDIAN_PEAK_KA.ln() + PEAK_KA_SIGMA * rng.normal()).exp()
}

fn pick_entry(rng: &mut Rng) -> AttachPoint {
    // GENERIC weighting: the nose/radome is the most frequently reported
    // first-attachment point (Fisher & Plumer), wingtips next.
    let r = rng.unit();
    if r < 0.5 {
        AttachPoint::NoseRadome
    } else if r < 0.75 {
        AttachPoint::WingtipLeft
    } else {
        AttachPoint::WingtipRight
    }
}

fn pick_exit(entry: AttachPoint, rng: &mut Rng) -> AttachPoint {
    // The channel is swept aft by the airflow relative to the entry point,
    // so the exit is drawn from the aft extremities (or the opposite
    // wingtip/an engine, for a wing entry).
    let candidates: &[AttachPoint] = match entry {
        AttachPoint::NoseRadome => &[AttachPoint::VStabTip, AttachPoint::HStabTipLeft, AttachPoint::HStabTipRight, AttachPoint::EngineNacelle(0), AttachPoint::EngineNacelle(3)],
        AttachPoint::WingtipLeft => &[AttachPoint::HStabTipLeft, AttachPoint::VStabTip, AttachPoint::EngineNacelle(1)],
        AttachPoint::WingtipRight => &[AttachPoint::HStabTipRight, AttachPoint::VStabTip, AttachPoint::EngineNacelle(2)],
        _ => &[AttachPoint::VStabTip],
    };
    candidates[rng.below(candidates.len())]
}

fn radome_damage(peak_current_ka: f64, hit_diverter: bool) -> f64 {
    let ratio = (peak_current_ka / ARP5412_PEAK_KA).min(1.5);
    if hit_diverter {
        (0.05 * ratio).min(1.0)
    } else {
        ratio.min(1.0)
    }
}

fn structure_damage(entry: AttachPoint, exit: AttachPoint, peak_current_ka: f64) -> f64 {
    // GENERIC: a well-bonded metal skin conducts a strike with negligible
    // direct damage (the reason metal aircraft are largely tolerant of
    // lightning); composite extremities (wingtips, tail, and the radome,
    // handled separately) carry copper mesh/foil protection but still see
    // a small paint/mesh burn scaling with current.
    let ratio = (peak_current_ka / ARP5412_PEAK_KA).min(1.5);
    let hit_composite_extremity = |p: AttachPoint| !matches!(p, AttachPoint::NoseRadome);
    let mut d: f64 = 0.0;
    if hit_composite_extremity(entry) {
        d = d.max(0.05 * ratio);
    }
    if hit_composite_extremity(exit) {
        d = d.max(0.05 * ratio);
    }
    d
}

/// `pub(super)` (rather than private) so `dispatch.rs`'s tests can build a
/// `LightningEvent` directly without re-deriving one through the full
/// `LightningModel` trigger machinery.
pub(super) fn resolve(entry: AttachPoint, exit: AttachPoint, peak_current_ka: f64, rng: &mut Rng) -> LightningEvent {
    let peak_current_ka = peak_current_ka.max(0.0);
    let action_integral_a2s = ARP5412_ACTION_INTEGRAL_A2S * (peak_current_ka / ARP5412_PEAK_KA).powi(2);
    let transients = all_buses().iter().map(|&b| transient_for(b, peak_current_ka)).collect();
    let hit_diverter = if entry == AttachPoint::NoseRadome || exit == AttachPoint::NoseRadome { rng.chance(DIVERTER_COVERAGE) } else { true };
    let radome_damage_frac = if entry == AttachPoint::NoseRadome || exit == AttachPoint::NoseRadome { radome_damage(peak_current_ka, hit_diverter) } else { 0.0 };
    let structure_damage_frac = structure_damage(entry, exit, peak_current_ka);
    // GENERIC: the compass (mounted near the cockpit, close to the nose)
    // sees the full effect only when the strike path runs through the
    // nose; otherwise a reduced, still-nonzero, coupling.
    let nose_involved = entry == AttachPoint::NoseRadome || exit == AttachPoint::NoseRadome;
    let proximity = if nose_involved { 1.0 } else { 0.4 };
    let compass_error_deg = MAX_COMPASS_ERROR_DEG * (peak_current_ka / ARP5412_PEAK_KA).min(1.0) * proximity;
    LightningEvent { entry, exit, peak_current_ka, action_integral_a2s, transients, compass_error_deg, radome_damage_frac, structure_damage_frac }
}

/// GENERIC background strike rate, per second, at `convective_intensity =
/// 1.0` (embedded in an active convective cell): calibrated to the
/// industry order-of-magnitude figure of roughly one strike per year per
/// aircraft, i.e. very rare outside convective weather and not attempted
/// to be more precise than that.
const BASE_RATE_PER_S_AT_FULL_INTENSITY: f64 = 1.0 / (200.0 * 60.0);

#[derive(Default)]
pub struct LightningModel {
    armed: Option<(Option<AttachPoint>, Option<f64>)>,
    pub random_mode: bool,
}

impl LightningModel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Arms a manual strike, fired on the next `step`. `entry`/`peak_current_ka`
    /// `None` are randomised as in the background model.
    pub fn trigger(&mut self, entry: Option<AttachPoint>, peak_current_ka: Option<f64>) {
        self.armed = Some((entry, peak_current_ka));
    }

    /// `convective_intensity` (0..1) is a weather-model input: how deep
    /// into active convective weather the aircraft currently is.
    pub fn step(&mut self, convective_intensity: f64, dt_s: f64, rng: &mut Rng) -> Option<LightningEvent> {
        if let Some((entry, peak)) = self.armed.take() {
            let entry = entry.unwrap_or_else(|| pick_entry(rng));
            let exit = pick_exit(entry, rng);
            let peak = peak.unwrap_or_else(|| sample_peak_current_ka(rng));
            return Some(resolve(entry, exit, peak, rng));
        }
        if self.random_mode {
            let p = BASE_RATE_PER_S_AT_FULL_INTENSITY * convective_intensity.clamp(0.0, 1.0) * dt_s.max(0.0);
            if rng.chance(p) {
                let entry = pick_entry(rng);
                let exit = pick_exit(entry, rng);
                let peak = sample_peak_current_ka(rng);
                return Some(resolve(entry, exit, peak, rng));
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_nan_or_negative_at_zero_current() {
        let mut rng = Rng::new(1);
        let e = resolve(AttachPoint::NoseRadome, AttachPoint::VStabTip, 0.0, &mut rng);
        assert_eq!(e.peak_current_ka, 0.0);
        assert_eq!(e.action_integral_a2s, 0.0);
        assert!(e.transients.iter().all(|t| t.peak_volts == 0.0 && !t.upset_likely));
        assert!(!e.compass_error_deg.is_nan() && e.compass_error_deg >= 0.0);
    }

    #[test]
    fn action_integral_scales_with_the_square_of_peak_current() {
        let mut rng = Rng::new(2);
        let low = resolve(AttachPoint::WingtipLeft, AttachPoint::VStabTip, 100.0, &mut rng);
        let high = resolve(AttachPoint::WingtipLeft, AttachPoint::VStabTip, 200.0, &mut rng);
        assert!((high.action_integral_a2s / low.action_integral_a2s - 4.0).abs() < 1e-6);
        assert!((high.action_integral_a2s - ARP5412_ACTION_INTEGRAL_A2S).abs() < 1e-6);
    }

    #[test]
    fn fadec_and_ife_are_more_exposed_than_shielded_flight_control_computers() {
        let mut rng = Rng::new(3);
        let e = resolve(AttachPoint::EngineNacelle(0), AttachPoint::VStabTip, 150.0, &mut rng);
        let fadec = e.transients.iter().find(|t| matches!(t.bus, BusId::EngineFadec(_))).unwrap().peak_volts;
        let prim = e.transients.iter().find(|t| matches!(t.bus, BusId::Prim(_))).unwrap().peak_volts;
        assert!(exposure_factor(BusId::EngineFadec(0)) > exposure_factor(BusId::Prim(1)));
        assert!(exposure_factor(BusId::Ife) > exposure_factor(BusId::Sec(1)));
        assert!(fadec > prim);
    }

    #[test]
    fn a_severe_strike_upsets_at_least_one_bus_a_mild_one_upsets_none() {
        let mut rng = Rng::new(4);
        let severe = resolve(AttachPoint::NoseRadome, AttachPoint::VStabTip, 200.0, &mut rng);
        assert!(severe.transients.iter().any(|t| t.upset_likely));
        let mild = resolve(AttachPoint::NoseRadome, AttachPoint::VStabTip, 10.0, &mut rng);
        assert!(mild.transients.iter().all(|t| !t.upset_likely));
    }

    #[test]
    fn missing_the_diverter_strip_does_far_more_radome_damage() {
        assert!(radome_damage(200.0, false) > radome_damage(200.0, true) * 5.0);
    }

    #[test]
    fn compass_error_peaks_when_the_nose_is_involved_and_current_is_high() {
        let mut rng = Rng::new(5);
        let nose = resolve(AttachPoint::NoseRadome, AttachPoint::VStabTip, 200.0, &mut rng);
        let wing = resolve(AttachPoint::WingtipLeft, AttachPoint::VStabTip, 200.0, &mut rng);
        assert!((nose.compass_error_deg - MAX_COMPASS_ERROR_DEG).abs() < 1e-9);
        assert!(wing.compass_error_deg < nose.compass_error_deg);
    }

    #[test]
    fn entry_and_exit_are_always_different_points() {
        let mut rng = Rng::new(6);
        for _ in 0..1000 {
            let entry = pick_entry(&mut rng);
            let exit = pick_exit(entry, &mut rng);
            assert_ne!(entry, exit);
        }
    }

    #[test]
    fn manual_trigger_fires_exactly_once_with_the_requested_entry() {
        let mut model = LightningModel::new();
        model.trigger(Some(AttachPoint::EngineNacelle(2)), Some(180.0));
        let mut rng = Rng::new(7);
        let first = model.step(0.0, 1.0, &mut rng);
        assert!(first.is_some());
        assert_eq!(first.unwrap().entry, AttachPoint::EngineNacelle(2));
        assert!(model.step(0.0, 1.0, &mut rng).is_none());
    }

    #[test]
    fn random_mode_needs_convective_weather() {
        let mut model = LightningModel::new();
        model.random_mode = true;
        let mut rng = Rng::new(8);
        let mut any = false;
        for _ in 0..1_000_000 {
            if model.step(0.0, 1.0, &mut rng).is_some() {
                any = true;
            }
        }
        assert!(!any, "no lightning outside convective weather");
    }

    #[test]
    fn random_mode_eventually_strikes_in_a_strong_cell() {
        let mut model = LightningModel::new();
        model.random_mode = true;
        let mut rng = Rng::new(9);
        let mut any = false;
        for _ in 0..2_000_000 {
            if model.step(1.0, 1.0, &mut rng).is_some() {
                any = true;
                break;
            }
        }
        assert!(any);
    }
}
