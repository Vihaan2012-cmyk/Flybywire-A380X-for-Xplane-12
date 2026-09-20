//! The breaker table: one entry per consumer `deep::electrical::loads.rs`
//! defines (same id, name, ATA, bus, wattage/power-factor -- read in full
//! this session and transcribed 1:1, group by group, matching its own
//! `ata21`/`ata26`/.../`ata36_bleed` structure exactly) plus the A380's
//! other real, publicly-known breaker-protected equipment by ATA chapter.
//!
//! A gap-closing pass gave 125 of the 128 group-2/group-3 entries below
//! (batteries' own charge-limiter/contactor-control circuits, recorders,
//! oxygen, the APU controller, engine FADEC/ignition, fire bottle squibs,
//! every position-indication/excitation circuit, ...) a matching load in
//! `deep::electrical::loads.rs`, so they now carry `protected_load:
//! Some(id)` like every group-1 entry. Only three -- BATTERY 1/2 and APU
//! BATTERY (`ata24_power_sources`) -- still honestly carry `protected_load:
//! None`: a battery's own output breaker protects a *source's* output, not
//! a consumer's demand, and there is no `Load` to point at without
//! double-counting against that battery's own `Source` branch (see that
//! function's own comment). That remaining trio is the same "real named
//! breaker, no modelled consumer" convention `src/breakers.rs`'s own
//! `gates_none` already established as precedent in this codebase.
//!
//! Self-contained per `docs/deep/BRIEF.md` hard rule 2: `Bus` here is this
//! module's own re-derivation (the 14 A380 bus names this catalogue's own
//! entries actually use, same 115 V AC / 28 V
//! DC nominal split `deep::electrical::network::BusId` and
//! `crate::physics::electrical::nominal_bus_voltage` both already use as
//! precedent), not an import -- `deep::electrical` is read for its load
//! ids/wattages/buses at authoring time, never referenced from this
//! module's code.
//!
//! ## Rating: load power / bus voltage, rounded up to a standard size
//! Every rating starts from the load's own real (or GENERIC, cited)
//! wattage the same way `deep::electrical::loads.rs::rated_current` derives
//! a breaker's rating: `I = P / (V * pf) * 1.25` -- the extra 25% is the
//! usual "breaker sized above steady load" margin (typical thermal-breaker
//! sizing practice, the identical margin `loads.rs` already applies, so a
//! healthy load never nuisance-trips its own protection). That raw current
//! is then rounded *up* to the next value in the real, published
//! AS39019/MIL-PRF-39019 (formerly MIL-C-5809/MIL-C-39019) standard
//! aircraft circuit-breaker ampere-rating series -- manufacturers only
//! actually produce parts at those discrete sizes, so a rating is never an
//! arbitrary computed number ([`standard_size`]).
//!
//! ## Type: thermal vs SSPC
//! The real A380 Electrical Load Management System distributes the large
//! majority of its lower-current circuits through solid-state power
//! controllers (SSPCs) housed in Primary/Secondary Power Centres, with
//! remote reset from the Cockpit Display System -- real, publicly
//! documented for this aircraft generation (Airbus/TE Connectivity/Data
//! Device Corporation solid-state power distribution literature). Published
//! SSPC product lines for this class of aircraft commonly cover roughly
//! 2.5-25 A remote-controlled circuits; higher-current feeder/motor/heater
//! breakers remain conventional thermal-magnetic. This module uses 25 A as
//! a GENERIC, representative (not per-part-number) split: [`kind_for`].
//!
//! ## Panel location
//! SSPC entries are placed at the Primary Power Centre matching their own
//! generator-side bus (PPC1-4 for AC1-4/DC1/DC2) or a Secondary Power
//! Centre for the essential/shed/cabin-and-cargo buses -- the real A380
//! ELMS architecture (Airbus solid-state power distribution literature).
//! Thermal entries for the classic pilot-facing ATA groups sit on the
//! overhead panel (forward half: engine/fuel/APU/hydraulic/bleed; aft half:
//! electrical/lighting/ice/gear/fire -- a GENERIC forward/aft split
//! following the A380 FCOM's own overhead-panel photos' general grouping,
//! not each breaker's exact published row/column); every other thermal
//! entry (an LRU with no cockpit pushbutton of its own) sits in the
//! avionics bay, matching where such LRUs actually live: [`panel_for`].

use super::trip::BreakerKind;

// ---------------------------------------------------------------------
// Bus.

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Bus {
    Ac1,
    Ac2,
    Ac3,
    Ac4,
    AcEss,
    AcEssShed,
    AcGndFltSvc,
    Dc1,
    Dc2,
    DcEss,
    DcBat,
    DcHot1,
    DcHot2,
    DcApu,
    /// A bus/feeder this module needs that has no counterpart in the 14
    /// above (e.g. an engine's own FADEC-channel supply) -- name and
    /// nominal voltage given directly, same escape hatch
    /// `crate::breakers::Bus::Named` already establishes as precedent.
    Named(&'static str, f64),
}

impl Bus {
    pub const fn is_ac(self) -> bool {
        matches!(self, Bus::Ac1 | Bus::Ac2 | Bus::Ac3 | Bus::Ac4 | Bus::AcEss | Bus::AcEssShed | Bus::AcGndFltSvc)
    }

    /// 115 V AC (three-phase equivalent) / 28 V DC nominal, the same split
    /// `deep::electrical::network::BusId::nominal_voltage` and
    /// `crate::physics::electrical::nominal_bus_voltage` both already use
    /// (real, FBW-sourced: `EngineGenerator::RATED_VOLTAGE_VOLT` / TRU-fed
    /// DC), independently re-derived here.
    pub fn nominal_voltage(self) -> f64 {
        match self {
            Bus::Named(_, v) => v,
            b if b.is_ac() => 115.0,
            _ => 28.0,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Bus::Ac1 => "AC1",
            Bus::Ac2 => "AC2",
            Bus::Ac3 => "AC3",
            Bus::Ac4 => "AC4",
            Bus::AcEss => "AC_ESS",
            Bus::AcEssShed => "AC_ESS_SHED",
            Bus::AcGndFltSvc => "AC_GND_FLT_SVC",
            Bus::Dc1 => "DC1",
            Bus::Dc2 => "DC2",
            Bus::DcEss => "DC_ESS",
            Bus::DcBat => "DC_BAT",
            Bus::DcHot1 => "DC_HOT1",
            Bus::DcHot2 => "DC_HOT2",
            Bus::DcApu => "DC_APU",
            Bus::Named(n, _) => n,
        }
    }
}

// ---------------------------------------------------------------------
// Panel location.

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Panel {
    OverheadFwd,
    OverheadAft,
    AvionicsBay,
    PrimaryPowerCentre1,
    PrimaryPowerCentre2,
    PrimaryPowerCentre3,
    PrimaryPowerCentre4,
    SecondaryPowerCentreFwd,
    SecondaryPowerCentreAft,
}

impl Panel {
    /// Short code used to build a `PanelPosition::label` prefix and by the
    /// Study/Breakers page to group a panel's own grid -- not a real
    /// Airbus panel-plate code (none of those are public per breaker),
    /// GENERIC mnemonic.
    pub fn code(self) -> &'static str {
        match self {
            Panel::OverheadFwd => "OHP-FWD",
            Panel::OverheadAft => "OHP-AFT",
            Panel::AvionicsBay => "EE-BAY",
            Panel::PrimaryPowerCentre1 => "PPC1",
            Panel::PrimaryPowerCentre2 => "PPC2",
            Panel::PrimaryPowerCentre3 => "PPC3",
            Panel::PrimaryPowerCentre4 => "PPC4",
            Panel::SecondaryPowerCentreFwd => "SPC-FWD",
            Panel::SecondaryPowerCentreAft => "SPC-AFT",
        }
    }
}

/// Where a breaker sits on its own panel's grid, for the Study/Breakers
/// page to draw a real-looking panel layout instead of a flat list.
/// [`assign_positions`] fills this in a deterministic second pass once the
/// whole catalogue exists (row/column depend on how many other breakers
/// share the same [`Panel`]), grouped by ATA chapter then id so the layout
/// is stable across rebuilds and roughly clusters one system's breakers
/// together the way a real panel module does. GENERIC grid: no photographed
/// real A380 panel-plate diagram is public at per-breaker resolution, so
/// this is an authored, deterministic arrangement, not a copied one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct PanelPosition {
    /// 1-based row within the breaker's own `Panel`.
    pub row: u32,
    /// 1-based column within the breaker's own `Panel`.
    pub column: u32,
    /// The short cap legend text a real breaker's own printed label would
    /// show (<=14 chars, truncated from the breaker's full `name`) --
    /// distinct from `name`/`consumer`, which stay full length for the
    /// Study page's tooltip/detail view.
    pub label: &'static str,
}

/// Breakers per row within one panel's own grid -- GENERIC, a typical
/// physical CB-panel module width class, not a measured real dimension.
const PANEL_COLUMNS: u32 = 12;

/// Truncate a breaker's full name to a real CB cap's own legend width.
/// GENERIC formatting (no photographed real legend text is public), kept
/// deterministic and lossless-prefix so the same breaker always gets the
/// same label across rebuilds.
fn cap_label(name: &str) -> String {
    if name.chars().count() <= 14 {
        name.to_string()
    } else {
        name.chars().take(14).collect()
    }
}

/// Second pass over the whole catalogue: assigns each breaker's row/column
/// within its own panel (grouped by `Panel`, ordered by ATA then id for a
/// stable, deterministic layout) and its cap label. Called once from
/// `build_catalog` after every `push_electrical`/`push_extra` call has run.
fn assign_positions(v: &mut [BreakerDef]) {
    use std::collections::HashMap;
    let mut order: Vec<usize> = (0..v.len()).collect();
    // `Panel` has no derived `Ord`; sort by its `code()` string instead so
    // the layout is deterministic without adding an arbitrary enum-variant
    // ordering just for this.
    order.sort_by(|&a, &b| (v[a].panel.code(), v[a].ata, v[a].id).cmp(&(v[b].panel.code(), v[b].ata, v[b].id)));
    let mut next_index: HashMap<Panel, u32> = HashMap::new();
    for i in order {
        let idx = next_index.entry(v[i].panel).or_insert(0);
        let row = *idx / PANEL_COLUMNS + 1;
        let column = *idx % PANEL_COLUMNS + 1;
        *idx += 1;
        let label: &'static str = Box::leak(cap_label(v[i].name).into_boxed_str());
        v[i].position = PanelPosition { row, column, label };
    }
}

fn panel_for(ata: u16, kind: BreakerKind, bus: Bus) -> Panel {
    if kind == BreakerKind::Sspc {
        return match bus {
            Bus::Ac1 | Bus::Dc1 => Panel::PrimaryPowerCentre1,
            Bus::Ac2 | Bus::Dc2 => Panel::PrimaryPowerCentre2,
            Bus::Ac3 => Panel::PrimaryPowerCentre3,
            Bus::Ac4 => Panel::PrimaryPowerCentre4,
            Bus::AcEss | Bus::AcEssShed | Bus::DcEss => Panel::SecondaryPowerCentreFwd,
            _ => Panel::SecondaryPowerCentreAft,
        };
    }
    match ata {
        21 | 28 | 29 | 36 | 49 => Panel::OverheadFwd,
        24 | 26 | 30 | 32 | 33 => Panel::OverheadAft,
        _ => Panel::AvionicsBay,
    }
}

// ---------------------------------------------------------------------
// Rating.

/// AS39019 (formerly MIL-C-5809/MIL-PRF-39019) standard aircraft
/// circuit-breaker ampere ratings -- the real, published discrete series
/// manufacturers actually produce parts in. Every catalogue rating is the
/// first series value at or above the load's own margined current.
const STANDARD_SIZES_A: [f64; 29] = [
    1.0, 2.0, 3.0, 4.0, 5.0, 7.5, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 125.0, 150.0, 175.0, 200.0, 225.0, 250.0, 300.0, 400.0, 500.0, 600.0,
];

/// Round a raw current up to the next real, manufactured standard size.
/// Above the published series (large feeder/generator-class current
/// limiters), rounds up to the next 100 A -- a coarser but still discrete
/// step, GENERIC beyond the mil-spec's own published table.
pub fn standard_size(current_a: f64) -> f64 {
    STANDARD_SIZES_A.iter().copied().find(|&s| s >= current_a).unwrap_or_else(|| (current_a / 100.0).ceil() * 100.0)
}

/// `P / (V * pf) * 1.25` -- the same formula and 25% margin
/// `deep::electrical::loads.rs::rated_current` already uses, independently
/// re-derived here per BRIEF hard rule 2.
fn margined_current(power_w: f64, voltage: f64, power_factor: f64) -> f64 {
    (power_w / (voltage * power_factor.max(0.1))) * 1.25
}

/// GENERIC 25 A split between the two technologies -- see this module's own
/// doc comment ("Type: thermal vs SSPC") for the citation.
fn kind_for(rating_a: f64) -> BreakerKind {
    if rating_a <= 25.0 {
        BreakerKind::Sspc
    } else {
        BreakerKind::Thermal
    }
}

fn avionics_pf(bus: Bus) -> f64 {
    if bus.is_ac() {
        0.95
    } else {
        1.0
    }
}

// ---------------------------------------------------------------------
// BreakerDef.

#[derive(Clone, Copy)]
pub struct BreakerDef {
    pub id: &'static str,
    pub name: &'static str,
    pub ata: u16,
    pub bus: Bus,
    pub rated_power_w: f64,
    pub power_factor: f64,
    /// `P / (V * pf) * 1.25`, before rounding to a standard size.
    pub raw_current_a: f64,
    /// [`standard_size`] of `raw_current_a` -- the breaker's real rating.
    pub rating_a: f64,
    pub basis: &'static str,
    pub consumer: &'static str,
    /// `deep::electrical::loads.rs`'s own load id this breaker protects,
    /// matched exactly; `None` for real A380 equipment with no load model
    /// anywhere in this codebase yet (documented, not fabricated).
    pub protected_load: Option<&'static str>,
    pub panel: Panel,
    pub kind: BreakerKind,
    /// Filled by [`assign_positions`] once the whole catalogue exists;
    /// `PanelPosition::default()` (row/column 0, empty label) until then --
    /// never the final value while a `push_electrical`/`push_extra` call
    /// site is still constructing this entry.
    pub position: PanelPosition,
}

fn push_electrical(v: &mut Vec<BreakerDef>, id: &'static str, name: &'static str, ata: u16, bus: Bus, watts: f64, pf: f64, consumer: &'static str, basis: &'static str) {
    let raw = margined_current(watts, bus.nominal_voltage(), pf);
    let rating = standard_size(raw);
    let kind = kind_for(rating);
    v.push(BreakerDef { id, name, ata, bus, rated_power_w: watts, power_factor: pf, raw_current_a: raw, rating_a: rating, basis, consumer, protected_load: Some(id), panel: panel_for(ata, kind, bus), kind, position: PanelPosition::default() });
}

fn push_extra(v: &mut Vec<BreakerDef>, id: &'static str, name: &'static str, ata: u16, bus: Bus, watts: f64, pf: f64, consumer: &'static str, basis: &'static str) {
    let raw = margined_current(watts, bus.nominal_voltage(), pf);
    let rating = standard_size(raw);
    let kind = kind_for(rating);
    v.push(BreakerDef { id, name, ata, bus, rated_power_w: watts, power_factor: pf, raw_current_a: raw, rating_a: rating, basis, consumer, protected_load: None, panel: panel_for(ata, kind, bus), kind, position: PanelPosition::default() });
}

/// Suffix `push_electrical_dual` uses for one feed's own breaker id --
/// matches `deep::electrical::loads.rs::add_dual`'s own `<id>-normal-bkr`/
/// `<id>-2nd-bkr` breaker ids exactly, so a real dual-fed load's two feed
/// breakers here carry the identical ids that catalogue's own internal
/// `Network::breakers` uses for the same load.
fn push_electrical_feed(v: &mut Vec<BreakerDef>, load_id: &'static str, feed_suffix: &'static str, name: &'static str, ata: u16, bus: Bus, watts: f64, pf: f64, consumer: &'static str, basis: &'static str) {
    let id: &'static str = Box::leak(format!("{load_id}-{feed_suffix}").into_boxed_str());
    let raw = margined_current(watts, bus.nominal_voltage(), pf);
    let rating = standard_size(raw);
    let kind = kind_for(rating);
    v.push(BreakerDef { id, name, ata, bus, rated_power_w: watts, power_factor: pf, raw_current_a: raw, rating_a: rating, basis, consumer, protected_load: Some(load_id), panel: panel_for(ata, kind, bus), kind, position: PanelPosition::default() });
}

/// A real dual-fed A380 LRU (`deep::electrical::loads.rs::add_dual`'s own
/// doc: flight-control/nav computers, ADIRUs, LGCIUs, ... each take power
/// from two separate buses through two separate breakers, OR-ed internally
/// so losing either single feed alone does not lose the unit): pushes
/// *two* breakers, `<load_id>-normal-bkr` and `<load_id>-2nd-bkr`, both
/// `protected_load: Some(load_id)` -- the same one load, two independent
/// feed breakers, matching the coordinator's "one breaker per FEED, not
/// per unit" instruction and `add_dual`'s own breaker-id scheme exactly.
fn push_electrical_dual(v: &mut Vec<BreakerDef>, load_id: &'static str, name: &'static str, ata: u16, normal_bus: Bus, second_bus: Bus, watts: f64, pf: f64, consumer: &'static str, basis: &'static str) {
    let normal_name: &'static str = Box::leak(format!("{name} NORMAL FEED").into_boxed_str());
    let second_name: &'static str = Box::leak(format!("{name} 2ND FEED").into_boxed_str());
    push_electrical_feed(v, load_id, "normal-bkr", normal_name, ata, normal_bus, watts, pf, consumer, basis);
    push_electrical_feed(v, load_id, "2nd-bkr", second_name, ata, second_bus, watts, pf, consumer, basis);
}

// =======================================================================
// Group 1: every `deep::electrical::loads.rs` consumer, one breaker each,
// transcribed group-by-group in the same order as that file's own `ata*`
// functions (read in full this session).
// =======================================================================

fn ata21(v: &mut Vec<BreakerDef>) {
    const FANS: [(&str, &str, Bus); 4] = [("cab-fan-1", "CAB FAN 1", Bus::Ac1), ("cab-fan-2", "CAB FAN 2", Bus::Ac2), ("cab-fan-3", "CAB FAN 3", Bus::Ac3), ("cab-fan-4", "CAB FAN 4", Bus::Ac4)];
    for (id, name, bus) in FANS {
        push_electrical(v, id, name, 21, bus, 500.0, 0.85, "cabin recirculation fan motor", "deep::electrical::loads.rs::ata21 CAB FAN 1-4 (500 W typical large-transport recirculation fan motor)");
    }
    push_electrical(v, "hotair-1", "HOT AIR VALVE 1", 21, Bus::AcEss, 50.0, 0.8, "hot air valve actuator", "deep::electrical::loads.rs::ata21 HOT AIR VALVE 1");
    push_electrical(v, "hotair-2", "HOT AIR VALVE 2", 21, Bus::AcEss, 50.0, 0.8, "hot air valve actuator", "deep::electrical::loads.rs::ata21 HOT AIR VALVE 2");
    push_electrical(v, "fwd-isol-valve", "FWD CARGO ISOL VALVE", 21, Bus::Dc2, 50.0, 0.8, "forward cargo isolation valve actuator", "deep::electrical::loads.rs::ata21 FWD CARGO ISOL VALVE (VCM Fwd DC2 channel)");
    push_electrical(v, "fwd-extract-fan", "FWD CARGO EXTRACT FAN", 21, Bus::Dc2, 150.0, 0.85, "forward cargo extraction fan motor", "deep::electrical::loads.rs::ata21 FWD CARGO EXTRACT FAN");
    push_electrical(v, "bulk-isol-valve", "BULK CARGO ISOL VALVE", 21, Bus::DcEss, 50.0, 0.8, "bulk cargo isolation valve actuator", "deep::electrical::loads.rs::ata21 BULK CARGO ISOL VALVE (VCM Aft DC_ESS channel)");
    push_electrical(v, "bulk-extract-fan", "BULK CARGO EXTRACT FAN", 21, Bus::DcEss, 150.0, 0.85, "bulk cargo extraction fan motor", "deep::electrical::loads.rs::ata21 BULK CARGO EXTRACT FAN");
    push_electrical(v, "cargo-heater", "BULK CARGO HEATER", 21, Bus::Ac2, 1000.0, 1.0, "bulk cargo heater element", "deep::electrical::loads.rs::ata21 BULK CARGO HEATER (AirHeater::new(AC2))");

    const FDAC: [(&str, &str, Bus); 4] = [("fdac-1a", "FDAC 1 CHANNEL 1", Bus::AcEss), ("fdac-1b", "FDAC 1 CHANNEL 2", Bus::Ac2), ("fdac-2a", "FDAC 2 CHANNEL 1", Bus::AcEss), ("fdac-2b", "FDAC 2 CHANNEL 2", Bus::Ac4)];
    for (id, name, bus) in FDAC {
        push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "FDAC channel", "deep::electrical::loads.rs::ata21 FDAC (FullDigitalAGUController)");
    }
    const TADD: [(&str, &str, Bus); 2] = [("tadd-1", "TADD CHANNEL 1", Bus::Ac2), ("tadd-2", "TADD CHANNEL 2", Bus::Ac4)];
    for (id, name, bus) in TADD {
        push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "trim air drive device channel", "deep::electrical::loads.rs::ata21 TADD (TrimAirDriveDevice)");
    }
    const VCM: [(&str, &str, Bus); 4] = [("vcm-fwd-1", "VCM FWD CHANNEL 1", Bus::Dc2), ("vcm-fwd-2", "VCM FWD CHANNEL 2", Bus::DcEss), ("vcm-aft-1", "VCM AFT CHANNEL 1", Bus::Dc2), ("vcm-aft-2", "VCM AFT CHANNEL 2", Bus::DcEss)];
    for (id, name, bus) in VCM {
        push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "ventilation control module channel", "deep::electrical::loads.rs::ata21 VCM (VentilationControlModule)");
    }
    const OCSM_AP: [(&str, &str, Bus); 4] = [("ocsm-1-ap", "OCSM 1 AUTO PARTITION", Bus::Dc1), ("ocsm-2-ap", "OCSM 2 AUTO PARTITION", Bus::Dc1), ("ocsm-3-ap", "OCSM 3 AUTO PARTITION", Bus::Dc2), ("ocsm-4-ap", "OCSM 4 AUTO PARTITION", Bus::Dc2)];
    for (id, name, bus) in OCSM_AP {
        push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "outflow valve control module auto-partition logic", "deep::electrical::loads.rs::ata21 OCSM auto-partition (OutflowValveControlModule)");
    }
    const OCSM_CH: [(&str, &str, Bus); 8] = [
        ("ocsm-1a", "OCSM 1 CHANNEL 1", Bus::Dc1),
        ("ocsm-1b", "OCSM 1 CHANNEL 2", Bus::DcEss),
        ("ocsm-2a", "OCSM 2 CHANNEL 1", Bus::Dc1),
        ("ocsm-2b", "OCSM 2 CHANNEL 2", Bus::DcEss),
        ("ocsm-3a", "OCSM 3 CHANNEL 1", Bus::Dc2),
        ("ocsm-3b", "OCSM 3 CHANNEL 2", Bus::DcEss),
        ("ocsm-4a", "OCSM 4 CHANNEL 1", Bus::Dc2),
        ("ocsm-4b", "OCSM 4 CHANNEL 2", Bus::DcEss),
    ];
    for (id, name, bus) in OCSM_CH {
        push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "outflow valve control module channel", "deep::electrical::loads.rs::ata21 OCSM channel");
    }
    let cpiom_bus = [Bus::Dc1, Bus::DcEss, Bus::DcEss, Bus::Dc2];
    for app in ["AGS", "TCS", "VCS", "CPCS"] {
        for k in 0..4usize {
            let id: &'static str = Box::leak(format!("cpiom-b{}-{}", k + 1, app.to_lowercase()).into_boxed_str());
            let name: &'static str = Box::leak(format!("CPIOM B{} {} APP", k + 1, app).into_boxed_str());
            let bus = cpiom_bus[k];
            push_electrical(v, id, name, 21, bus, 50.0, avionics_pf(bus), "CPIOM B application (AGS/TCS/VCS/CPCS)", "deep::electrical::loads.rs::ata21 CPIOM B bus map");
        }
    }
    for pack in 1..=2u32 {
        for side in 1..=2u32 {
            let id: &'static str = Box::leak(format!("pack-{pack}-flow-valve-{side}").into_boxed_str());
            let name: &'static str = Box::leak(format!("PACK {pack} FLOW VALVE {side}").into_boxed_str());
            push_electrical(v, id, name, 21, Bus::DcEss, 50.0, 0.8, "pack flow valve actuator", "deep::electrical::loads.rs::ata21 PACK FLOW VALVE (pneumatic.rs PackComplex ElectroPneumaticValve, DC_ESS)");
        }
    }
    // Four avionics-bay cooling fans, matching
    // `deep::electrical::loads.rs::ata21`'s own four one for one: fans 1/2 on
    // the essential pair, fans 3/4 on AC 1 / AC 2 so bay ventilation survives
    // the loss of the essential channel.
    for (id, name, bus) in [
        ("avionics-fan-1", "AVIONICS BAY FAN 1", Bus::AcEss),
        ("avionics-fan-2", "AVIONICS BAY FAN 2", Bus::AcEssShed),
        ("avionics-fan-3", "AVIONICS BAY FAN 3", Bus::Ac1),
        ("avionics-fan-4", "AVIONICS BAY FAN 4", Bus::Ac2),
    ] {
        push_electrical(v, id, name, 21, bus, 300.0, 0.85, "avionics-bay cooling fan motor", "deep::electrical::loads.rs::ata21 avionics-bay cooling fans (GENERIC, not individually named in src/breakers.rs)");
    }
}

fn ata26(v: &mut Vec<BreakerDef>) {
    let zones = ["eng1", "eng2", "eng3", "eng4", "apu", "mlgbay"];
    let titles = ["ENG1", "ENG2", "ENG3", "ENG4", "APU", "MLGBAY"];
    for (zone, title) in zones.iter().zip(titles.iter()) {
        for loop_name in ["A", "B"] {
            let id: &'static str = Box::leak(format!("fire-loop-{zone}-{loop_name}").into_boxed_str());
            let name: &'static str = Box::leak(format!("FIRE DET {title} LOOP {loop_name}").into_boxed_str());
            push_electrical(v, id, name, 26, Bus::DcEss, 20.0, avionics_pf(Bus::DcEss), "fire detection loop (redundant with its own A/B pair)", "deep::electrical::loads.rs::ata26 fire detection loop (20 W typical controller electronics; DC_ESS/DC_HOT1)");
        }
    }
}

fn ata27(v: &mut Vec<BreakerDef>) {
    let entries: [(&str, &str, u16); 11] = [
        ("rollout", "ROLLOUT", 22),
        ("fcu-1", "FCU 1", 22),
        ("fcu-2", "FCU 2", 22),
        ("prim-1", "PRIM 1", 27),
        ("prim-2", "PRIM 2", 27),
        ("prim-3", "PRIM 3", 27),
        ("sec-1", "SEC 1", 27),
        ("sec-2", "SEC 2", 27),
        ("sec-3", "SEC 3", 27),
        ("fcdc-1", "FCDC 1", 27),
        ("fcdc-2", "FCDC 2", 27),
    ];
    for (k, (id, name, ata)) in entries.into_iter().enumerate() {
        // deep::electrical::loads.rs::ata27 (session 2 re-sync): real dual
        // feed -- normal bus alternates Dc1/Dc2 by lane, DC ESS is the
        // universal backup feed, OR-ed internally (add_dual). Two breakers
        // per unit now, not one.
        let normal_bus = if k % 2 == 0 { Bus::Dc1 } else { Bus::Dc2 };
        push_electrical_dual(v, id, name, ata, normal_bus, Bus::DcEss, 100.0, 1.0, "flight-control/autoflight computer", "deep::electrical::loads.rs::ata27 flight-control/autoflight computer (100 W typical FCC-class LRU); real dual feed, normal DC1/DC2 bus + DC ESS backup, each on its own breaker, OR-ed internally");
    }
}

fn ata32(v: &mut Vec<BreakerDef>) {
    // deep::electrical::loads.rs::ata32 (session 2 re-sync): real dual-lane
    // feed, each LGCIU's own normal bus unchanged from session 1, DC2/DC ESS
    // now the cross-wired backup feed (add_dual). Two breakers per unit.
    push_electrical_dual(v, "lgciu-1", "LGCIU 1", 32, Bus::DcEss, Bus::Dc2, 50.0, 1.0, "Landing Gear Control and Interface Unit 1", "deep::electrical::loads.rs::ata32 LGCIU 1; real dual feed, DC ESS normal + DC2 backup, each on its own breaker");
    push_electrical_dual(v, "lgciu-2", "LGCIU 2", 32, Bus::Dc2, Bus::DcEss, 50.0, 1.0, "Landing Gear Control and Interface Unit 2", "deep::electrical::loads.rs::ata32 LGCIU 2; real dual feed, DC2 normal + DC ESS backup, each on its own breaker");

    const PUMPS: [(&str, &str, Bus); 4] = [("hyd-epump-ga", "HYD GREEN ELEC PUMP A", Bus::Ac3), ("hyd-epump-gb", "HYD GREEN ELEC PUMP B", Bus::Ac4), ("hyd-epump-ya", "HYD YELLOW ELEC PUMP A", Bus::AcEss), ("hyd-epump-yb", "HYD YELLOW ELEC PUMP B", Bus::Ac2)];
    for (id, name, bus) in PUMPS {
        push_electrical(v, id, name, 29, bus, 75.0 * 28.0, 0.85, "electric hydraulic pump motor", "deep::electrical::loads.rs::ata32 electric hydraulic pump (FBW ELECTRIC_PUMP_MAX_CURRENT_AMPERE = 75 A, hydraulic/mod.rs:1750, real/FBW-sourced; 28 V-equivalent power)");
        // deep::electrical::loads.rs::ata32 (session 2 re-sync): each pump
        // also has its own new "<id>-coil" load -- a real, physically
        // distinct low-power DC line-contactor holding-coil supply, not
        // the motor's own high-current feed.
        let coil_id: &'static str = Box::leak(format!("{id}-coil").into_boxed_str());
        let coil_name: &'static str = Box::leak(format!("{name} CONTACTOR COIL").into_boxed_str());
        push_electrical(v, coil_id, coil_name, 29, Bus::DcEss, 20.0, 1.0, "pump motor line-contactor holding-coil supply", "deep::electrical::loads.rs::ata32 pump contactor coil (GENERIC ~20 W DC line-contactor holding coil, the low-power control circuit that energises the pump motor's own contactor, distinct from the motor's own high-current feed)");
    }
    push_electrical(v, "autobrake-disarm-sol", "AUTOBRAKE DISARM SOLENOID", 32, Bus::Dc2, 56.0, 1.0, "autobrake knob disarm solenoid", "deep::electrical::loads.rs::ata32 AUTOBRAKE DISARM SOLENOID (56 W typical small solenoid valve; autobrakes.rs DC2)");

    const SENSORS: [&str; 12] = [
        "prox-uplock-gear-nose-1",
        "prox-downlock-gear-nose-2",
        "prox-uplock-gear-right-1",
        "prox-downlock-gear-right-2",
        "prox-uplock-gear-left-2",
        "prox-downlock-gear-left-1",
        "prox-uplock-door-nose-1",
        "prox-downlock-door-nose-2",
        "prox-uplock-door-right-2",
        "prox-downlock-door-right-1",
        "prox-uplock-door-left-2",
        "prox-downlock-door-left-1",
    ];
    for id in SENSORS {
        let name: &'static str = Box::leak(id.replace('-', " ").to_uppercase().into_boxed_str());
        push_electrical(v, id, name, 32, Bus::DcEss, 5.0, avionics_pf(Bus::DcEss), "gear/door uplock-downlock proximity sensor", "deep::electrical::loads.rs::ata32 proximity sensor (5 W typical target/pickup; LGCIU's own DC_ESS supply)");
    }
    const ACTUATORS: [&str; 6] = ["gear-actuator-nose", "gear-actuator-left", "gear-actuator-right", "gear-door-actuator-nose", "gear-door-actuator-left", "gear-door-actuator-right"];
    for id in ACTUATORS {
        let name: &'static str = Box::leak(id.replace('-', " ").to_uppercase().into_boxed_str());
        push_electrical(v, id, name, 32, Bus::DcEss, 75.0 * 28.0, 0.85, "gear/gear-door hydraulic actuator control", "deep::electrical::loads.rs::ata32 gear/door actuator control (same order of magnitude as the electric hydraulic pumps)");
    }
}

fn ata34(v: &mut Vec<BreakerDef>) {
    const RAS: [(&str, &str, Bus); 3] = [("ra-sys-a", "RA SYS A", Bus::Ac1), ("ra-sys-b", "RA SYS B", Bus::Ac2), ("ra-sys-c", "RA SYS C", Bus::AcEss)];
    for (id, name, bus) in RAS {
        push_electrical(v, id, name, 34, bus, 50.0, avionics_pf(bus), "radio altimeter transceiver", "deep::electrical::loads.rs::ata34 radio altimeter transceiver (A380RadioAltimeters)");
    }
    for (n, bus) in [(1, Bus::Ac1), (2, Bus::Ac2), (3, Bus::AcEss)] {
        let id: &'static str = Box::leak(format!("ra-ant-interrupt-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("RA {n} ANTENNA INTERRUPT").into_boxed_str());
        push_electrical(v, id, name, 34, bus, 10.0, avionics_pf(bus), "radio altimeter antenna feed", "deep::electrical::loads.rs::ata34 antenna-coupling network (10 W class)");
        let id2: &'static str = Box::leak(format!("ra-ant-coupling-{n}").into_boxed_str());
        let name2: &'static str = Box::leak(format!("RA {n} ANTENNA DIRECT COUPLING").into_boxed_str());
        push_electrical(v, id2, name2, 34, bus, 10.0, avionics_pf(bus), "radio altimeter antenna feed", "deep::electrical::loads.rs::ata34 antenna-coupling network (10 W class)");
    }
    push_electrical(v, "egpwc", "EGPWC (TAWS)", 34, Bus::AcEss, 100.0, avionics_pf(Bus::AcEss), "Enhanced Ground Proximity Warning Computer (TAWS/terrain display)", "deep::electrical::loads.rs::ata34 EGPWC (100 W typical flight-warning-class LRU; real AC_ESS bus, enhanced_gpwc/mod.rs)");
}

fn ata28_fuel(v: &mut Vec<BreakerDef>) {
    let pump_buses = [Bus::Ac1, Bus::Ac2, Bus::Ac3, Bus::Ac4, Bus::AcEss];
    for i in 0..25usize {
        let id: &'static str = Box::leak(format!("fuel-pump-{i}").into_boxed_str());
        let name: &'static str = Box::leak(format!("FUEL PUMP {i}").into_boxed_str());
        let bus = pump_buses[i % pump_buses.len()];
        push_electrical(v, id, name, 28, bus, 600.0, 0.85, "fuel boost/transfer/jettison pump motor", "deep::electrical::loads.rs::ata28_fuel CIRCUIT_FUEL_PUMP (25 real pumps, 600 W each, circuits.rs precedent figure)");
    }
    let valve_buses = [Bus::Dc1, Bus::Dc2, Bus::DcEss, Bus::DcBat];
    for i in 0..60usize {
        let id: &'static str = Box::leak(format!("fuel-valve-{i}").into_boxed_str());
        let name: &'static str = Box::leak(format!("FUEL VALVE {i}").into_boxed_str());
        let bus = valve_buses[i % valve_buses.len()];
        push_electrical(v, id, name, 28, bus, 50.0, 0.8, "fuel shutoff/transfer/crossfeed/isolation valve actuator", "deep::electrical::loads.rs::ata28_fuel CIRCUIT_FUEL_VALVE (60 real valves, 50 W each, circuits.rs precedent figure)");
    }
}

fn ata33_lighting(v: &mut Vec<BreakerDef>) {
    const LIGHTS: [(&str, &str, Bus, f64); 12] = [
        ("light-landing", "LANDING LIGHTS", Bus::Ac1, 600.0),
        ("light-taxi", "TAXI LIGHTS", Bus::Ac2, 250.0),
        ("light-nav", "NAV LIGHTS", Bus::AcEssShed, 40.0),
        ("light-beacon", "BEACON LIGHTS", Bus::AcEssShed, 100.0),
        ("light-strobe", "STROBE LIGHTS", Bus::Ac3, 300.0),
        ("light-logo", "LOGO LIGHTS", Bus::Ac4, 150.0),
        ("light-wing", "WING LIGHTS", Bus::Ac1, 150.0),
        ("light-recognition", "RECOGNITION LIGHTS", Bus::DcBat, 40.0),
        ("light-cabin", "CABIN LIGHTS", Bus::AcGndFltSvc, 200.0),
        ("light-panel", "PANEL LIGHTS", Bus::DcEss, 30.0),
        ("light-pedestal", "PEDESTAL LIGHTS", Bus::DcEss, 20.0),
        ("light-glareshield", "GLARESHIELD LIGHTS", Bus::DcEss, 20.0),
    ];
    for (id, name, bus, watts) in LIGHTS {
        push_electrical(v, id, name, 33, bus, watts, 1.0, "cockpit/cabin/exterior light circuit", "deep::electrical::loads.rs::ata33_lighting lumped per circuit type (circuits.rs CIRCUIT_LIGHT_* precedent figure)");
    }
}

fn ata30_ice_protection(v: &mut Vec<BreakerDef>) {
    for (n, bus) in [(1, Bus::Ac1), (2, Bus::Ac2)] {
        let id: &'static str = Box::leak(format!("windshield-heat-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("WINDSHIELD HEAT {n}").into_boxed_str());
        push_electrical(v, id, name, 30, bus, 2000.0, 1.0, "windshield electric anti-ice heating element", "deep::electrical::loads.rs::ata30_ice_protection GENERIC wide-body windshield anti-ice heater (2000 W/side)");
    }
    for (n, bus) in [(1, Bus::Ac1), (2, Bus::Ac2), (3, Bus::AcEss)] {
        let id: &'static str = Box::leak(format!("pitot-heat-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("PITOT HEAT {n}").into_boxed_str());
        push_electrical(v, id, name, 30, bus, 600.0, 1.0, "pitot tube heater element", "deep::electrical::loads.rs::ata30_ice_protection GENERIC precedent, physics::electrical.rs rated_watts(\"CIRCUIT_PITOT_HEAT\") = 600 W");
    }
    for (name_txt, id, bus) in [("AOA HEAT 1", "aoa-heat-1", Bus::Dc1), ("AOA HEAT 2", "aoa-heat-2", Bus::Dc2), ("TAT PROBE HEAT", "tat-heat", Bus::DcEss)] {
        let name: &'static str = Box::leak(name_txt.to_string().into_boxed_str());
        push_electrical(v, id, name, 30, bus, 150.0, 1.0, "probe (AOA vane / TAT) heating element", "deep::electrical::loads.rs::ata30_ice_protection GENERIC small-probe heater, an order of magnitude below a pitot tube's own heater");
    }
}

fn ata25_galleys(v: &mut Vec<BreakerDef>) {
    const GALLEYS: [(&str, &str, Bus, f64); 6] = [
        ("galley-fwd-upper", "FWD UPPER GALLEY", Bus::Ac1, 8000.0),
        ("galley-aft-upper", "AFT UPPER GALLEY", Bus::Ac2, 8000.0),
        ("galley-fwd-main", "FWD MAIN DECK GALLEY", Bus::Ac3, 10000.0),
        ("galley-mid-main", "MID MAIN DECK GALLEY", Bus::Ac4, 10000.0),
        ("galley-aft-main", "AFT MAIN DECK GALLEY", Bus::Ac1, 10000.0),
        ("galley-lower", "LOWER DECK GALLEY LIFT", Bus::Ac2, 3000.0),
    ];
    for (id, name, bus, watts) in GALLEYS {
        push_electrical(v, id, name, 25, bus, watts, 1.0, "galley complex load (ovens, water heaters, chillers)", "deep::electrical::loads.rs::ata25_galleys GENERIC wide-body galley complex load, no public per-zone A380 figure");
    }
}

fn ata44_ife(v: &mut Vec<BreakerDef>) {
    const ZONES: [(&str, &str, Bus, u32); 5] = [
        ("ife-upper-deck", "IFE UPPER DECK ZONE", Bus::AcGndFltSvc, 90),
        ("ife-main-fwd", "IFE MAIN DECK FWD ZONE", Bus::AcGndFltSvc, 120),
        ("ife-main-mid", "IFE MAIN DECK MID ZONE", Bus::AcGndFltSvc, 150),
        ("ife-main-aft", "IFE MAIN DECK AFT ZONE", Bus::AcGndFltSvc, 130),
        ("ife-server", "IFE SERVER RACK", Bus::Ac2, 1),
    ];
    for (id, name, bus, seats) in ZONES {
        let per_seat_w = if seats > 1 { 30.0 } else { 4000.0 };
        let watts = per_seat_w * seats as f64;
        push_electrical(v, id, name, 44, bus, watts, avionics_pf(bus), "IFE seat-box zone / server rack", "deep::electrical::loads.rs::ata44_ife GENERIC ~30 W/seat x zone seat count, or server-rack figure for the head-end");
    }
}

fn avionics_misc(v: &mut Vec<BreakerDef>) {
    // deep::electrical::loads.rs::avionics_misc (session 2 re-sync):
    // flight-critical boxes (FMS, ADIRU, TCAS) are now real dual-fed LRUs
    // (normal + ESS/backup bus, OR-ed internally, add_dual) -- two
    // breakers per unit; radios/transponders/weather radar stay
    // conventionally single-fed with a manual transfer switch, unchanged.
    const DUAL_BOXES: [(&str, &str, u16, Bus, Bus, f64); 7] = [
        ("fms-1", "FMS 1", 34, Bus::Dc1, Bus::DcEss, 60.0),
        ("fms-2", "FMS 2", 34, Bus::Dc2, Bus::DcEss, 60.0),
        ("fms-3", "FMS 3", 34, Bus::Dc1, Bus::DcEss, 60.0),
        ("adirs-1", "ADIRU 1", 34, Bus::AcEss, Bus::DcEss, 80.0),
        ("adirs-2", "ADIRU 2", 34, Bus::Ac2, Bus::DcEss, 80.0),
        ("adirs-3", "ADIRU 3", 34, Bus::AcEssShed, Bus::DcEss, 80.0),
        ("tcas", "TCAS COMPUTER", 34, Bus::DcEss, Bus::Dc2, 70.0),
    ];
    for (id, name, ata, normal_bus, second_bus, watts) in DUAL_BOXES {
        push_electrical_dual(v, id, name, ata, normal_bus, second_bus, watts, avionics_pf(normal_bus), "avionics LRU", "deep::electrical::loads.rs::avionics_misc GENERIC dual-fed avionics LRU class figure; real dual feed (normal + ESS/backup bus), each on its own breaker, OR-ed internally");
    }
    const SINGLE_BOXES: [(&str, &str, u16, Bus, f64); 5] = [
        ("xpdr-1", "TRANSPONDER 1", 34, Bus::Dc1, 50.0),
        ("xpdr-2", "TRANSPONDER 2", 34, Bus::Dc2, 50.0),
        ("vhf-1", "VHF 1", 23, Bus::Dc1, 40.0),
        ("vhf-2", "VHF 2", 23, Bus::Dc2, 40.0),
        ("wxr", "WEATHER RADAR", 34, Bus::Ac1, 150.0),
    ];
    for (id, name, ata, bus, watts) in SINGLE_BOXES {
        push_electrical(v, id, name, ata, bus, watts, avionics_pf(bus), "avionics LRU", "deep::electrical::loads.rs::avionics_misc GENERIC typical avionics LRU class figure; conventionally single-fed with a manual transfer switch, not an automatic OR");
    }
}

fn ata36_bleed(v: &mut Vec<BreakerDef>) {
    for n in 1..=4u32 {
        let bus = if n <= 2 { Bus::Dc1 } else { Bus::Dc2 };
        let id: &'static str = Box::leak(format!("bleed-eng-{n}").into_boxed_str());
        let name: &'static str = Box::leak(format!("BLEED ENG {n} VALVES").into_boxed_str());
        push_electrical(v, id, name, 36, bus, 150.0, 0.8, "engine bleed HP/pressure-regulating/fan-air valve set", "deep::electrical::loads.rs::ata36_bleed one shared feed per engine (real Airbus-style single bleed CB)");
    }
}

// =======================================================================
// Group 2: other real A380 breaker-protected equipment by ATA chapter.
// Gap-closing pass: all but the three battery-output breakers in
// `ata24_power_sources` now have a matching `deep::electrical::loads.rs`
// load (`push_electrical`, `protected_load: Some(id)`); the battery trio
// stays `push_extra`/`protected_load: None`, honestly documented rather
// than a fabricated gate, for the reason given in that function's comment.
// =======================================================================

fn ata24_power_sources(v: &mut Vec<BreakerDef>) {
    // Two main batteries + one APU battery: a real A380/large-transport
    // electrical architecture (public FCOM/AMM knowledge for this aircraft
    // class); GENERIC 150 A continuous-discharge breaker/current-limiter
    // class figure (no public per-battery A380 figure), each on its own
    // DC hot bus.
    //
    // These three deliberately stay `push_extra`/`protected_load: None`
    // (deep::electrical session re-sync, gap-closing pass): a battery's own
    // output/current-limiter breaker protects a *source's* output current,
    // not a consumer's demand, and `deep::electrical::network::Load` (the
    // only thing a `protected_load` id can point at) models demand only.
    // `deep::electrical::sources::Wiring::build` already gives each battery
    // its own `Source` under this same "bat-N" id family, in a different
    // list (`Network::sources`, not `Network::loads`); pointing this
    // breaker at a fabricated `Load` of the same id would double-count that
    // current against the battery's own `Source` branch, not model a real,
    // separate consumer. See `deep::electrical::loads.rs`'s own header
    // comment on this section for the same reasoning from the other side.
    push_extra(v, "bat-1", "BATTERY 1", 24, Bus::DcHot1, 150.0 * 28.0, 1.0, "main battery 1 output", "GENERIC: typical large-transport main aircraft battery continuous-discharge current-limiter class (150 A), no public A380 per-battery figure");
    push_extra(v, "bat-2", "BATTERY 2", 24, Bus::DcHot2, 150.0 * 28.0, 1.0, "main battery 2 output", "GENERIC: same class as BATTERY 1");
    push_extra(v, "bat-apu", "APU BATTERY", 24, Bus::DcApu, 100.0 * 28.0, 1.0, "APU battery output", "GENERIC: a smaller dedicated APU-start battery, same current-limiter class scaled down");
    // The remaining three in this group are real, small control circuits
    // (not a source's own output) and now have a matching load in
    // `deep::electrical::loads.rs::ata24_power_sources_extra`.
    push_electrical(v, "ext-pwr-contactor", "EXTERNAL POWER CONTACTOR CONTROL", 24, Bus::DcHot1, 20.0, 1.0, "external power contactor control coil", "GENERIC: typical small contactor-coil control circuit, real A380 external power system, no public per-part figure");
    push_electrical(v, "bat-charge-limiter-1", "BATTERY 1 CHARGE LIMITER", 24, Bus::DcEss, 20.0, 1.0, "battery 1 charge-limiter control circuit", "GENERIC: typical small charge-controller LRU control circuit");
    push_electrical(v, "bat-charge-limiter-2", "BATTERY 2 CHARGE LIMITER", 24, Bus::DcEss, 20.0, 1.0, "battery 2 charge-limiter control circuit", "GENERIC: typical small charge-controller LRU control circuit");
}

// The six functions below (ATA23/31/35/49/7x/26/29/52/33) now all have a
// matching `deep::electrical::loads.rs` load, one function each, added in
// this pass -- `push_electrical` rather than `push_extra`, same id/bus/
// watts/pf/basis as before.
fn ata23_comms(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "satcom", "SATCOM", 23, Bus::DcEss, 100.0, avionics_pf(Bus::DcEss), "satellite communication transceiver", "GENERIC: typical wide-body SATCOM transceiver LRU, real A380 equipment class, no public per-box figure");
    push_electrical(v, "hf-1", "HF 1", 23, Bus::Dc1, 100.0, avionics_pf(Bus::Dc1), "HF radio transceiver 1", "GENERIC: typical HF transceiver LRU");
    push_electrical(v, "hf-2", "HF 2", 23, Bus::Dc2, 100.0, avionics_pf(Bus::Dc2), "HF radio transceiver 2", "GENERIC: typical HF transceiver LRU");
    push_electrical(v, "acars-mu", "ACARS MU", 23, Bus::DcEss, 50.0, avionics_pf(Bus::DcEss), "ACARS management unit", "GENERIC: typical avionics LRU class figure");
    push_electrical(v, "pa-amplifier", "PA AMPLIFIER", 23, Bus::Ac1, 200.0, 0.9, "cabin passenger address amplifier", "GENERIC: typical wide-body PA amplifier power stage");
    push_electrical(v, "interphone", "INTERPHONE", 23, Bus::DcEss, 50.0, avionics_pf(Bus::DcEss), "crew interphone system", "GENERIC: typical avionics LRU class figure");
}

fn ata31_recorders(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "dfdr", "DFDR", 31, Bus::DcEss, 50.0, avionics_pf(Bus::DcEss), "Digital Flight Data Recorder", "GENERIC: typical avionics LRU class figure, real mandatory A380 equipment");
    push_electrical(v, "cvr", "CVR", 31, Bus::DcEss, 50.0, avionics_pf(Bus::DcEss), "Cockpit Voice Recorder", "GENERIC: typical avionics LRU class figure, real mandatory A380 equipment");
    push_electrical(v, "qar", "QAR", 31, Bus::Dc2, 30.0, avionics_pf(Bus::Dc2), "Quick Access Recorder", "GENERIC: typical small avionics LRU class figure");
}

fn ata35_oxygen(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "crew-o2-shutoff", "CREW OXYGEN SHUTOFF VALVE", 35, Bus::Dc1, 50.0, 0.8, "crew oxygen supply shutoff valve actuator", "GENERIC: typical motor/solenoid-operated shutoff valve actuator, real A380 crew oxygen system");
    push_electrical(v, "pax-o2-gen-ctl", "PAX OXYGEN GENERATOR CONTROL", 35, Bus::DcEss, 30.0, avionics_pf(Bus::DcEss), "passenger chemical oxygen generator deployment/control circuit", "GENERIC: typical small control-circuit LRU figure");
    push_electrical(v, "o2-pressure-xducer", "OXYGEN PRESSURE TRANSDUCER", 35, Bus::DcEss, 5.0, avionics_pf(Bus::DcEss), "crew oxygen bottle pressure transducer", "GENERIC: typical small pressure-transducer power draw");
}

fn ata49_apu(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "apu-ecu-a", "APU ECU CHANNEL A", 49, Bus::DcApu, 60.0, avionics_pf(Bus::DcApu), "APU electronic control unit channel A", "GENERIC: typical dual-channel engine/APU controller LRU class figure (real PW980 APU has its own FADEC-class controller)");
    push_electrical(v, "apu-ecu-b", "APU ECU CHANNEL B", 49, Bus::DcEss, 60.0, avionics_pf(Bus::DcEss), "APU electronic control unit channel B", "GENERIC: same class as channel A, redundant bus feed");
    push_electrical(v, "apu-fuel-shutoff-valve", "APU FUEL SHUTOFF VALVE", 49, Bus::Dc1, 50.0, 0.8, "APU fuel shutoff valve actuator", "GENERIC: typical motor/solenoid-operated shutoff valve actuator");
    // Transit-only (only energised through the APU's own start sequence):
    // still a real, modelled load, `deep::electrical::live` is what gates
    // its `commanded_on`, not this catalogue.
    push_electrical(v, "apu-start-contactor", "APU START CONTACTOR", 49, Bus::Dc1, 20.0, 1.0, "APU starter-generator start contactor control coil", "GENERIC: typical contactor-coil control circuit");
}

fn ata7x_engine(v: &mut Vec<BreakerDef>) {
    // FADEC channels A/B per engine (ATA73, fuel/control): real dual-lane
    // architecture on any modern turbofan FADEC, GENERIC per-channel power
    // figure (no public Trent 972B-84 FADEC electrical figure).
    for n in 1..=4u32 {
        let bus_a = if n <= 2 { Bus::DcEss } else { Bus::Dc1 };
        let bus_b = if n <= 2 { Bus::Dc2 } else { Bus::DcEss };
        let id_a: &'static str = Box::leak(format!("fadec-{n}a").into_boxed_str());
        let name_a: &'static str = Box::leak(format!("FADEC {n} CHANNEL A").into_boxed_str());
        push_electrical(v, id_a, name_a, 73, bus_a, 80.0, avionics_pf(bus_a), "engine FADEC channel A", "GENERIC: typical dual-lane FADEC-class controller channel, real Trent 972B-84 architecture, no public per-channel electrical figure");
        let id_b: &'static str = Box::leak(format!("fadec-{n}b").into_boxed_str());
        let name_b: &'static str = Box::leak(format!("FADEC {n} CHANNEL B").into_boxed_str());
        push_electrical(v, id_b, name_b, 73, bus_b, 80.0, avionics_pf(bus_b), "engine FADEC channel B", "GENERIC: same class as channel A, redundant bus feed");
    }
    // Ignition exciters A/B per engine (ATA74): a real high-energy ignition
    // exciter draws on the order of a couple hundred watts pulsed, GENERIC
    // (no public per-part figure for this engine). Transit-only (a real
    // exciter only fires during engine start / continuous ignition, not
    // throughout the flight) -- still a real, modelled load,
    // `deep::electrical::live` is what gates its `commanded_on`.
    for n in 1..=4u32 {
        let bus_a = if n % 2 == 1 { Bus::Dc1 } else { Bus::Dc2 };
        let bus_b = if n % 2 == 1 { Bus::Dc2 } else { Bus::Dc1 };
        let id_a: &'static str = Box::leak(format!("ignition-{n}a").into_boxed_str());
        let name_a: &'static str = Box::leak(format!("IGNITION {n} EXCITER A").into_boxed_str());
        push_electrical(v, id_a, name_a, 74, bus_a, 250.0, 0.9, "engine ignition exciter A", "GENERIC: typical high-energy ignition exciter unit pulsed power class (~250 W), no public per-part figure");
        let id_b: &'static str = Box::leak(format!("ignition-{n}b").into_boxed_str());
        let name_b: &'static str = Box::leak(format!("IGNITION {n} EXCITER B").into_boxed_str());
        push_electrical(v, id_b, name_b, 74, bus_b, 250.0, 0.9, "engine ignition exciter B", "GENERIC: same class as exciter A, redundant lane on the opposite DC bus");
    }
}

fn ata26_extinguishing(v: &mut Vec<BreakerDef>) {
    // Two engine fire-extinguisher bottles, each with 2 pyrotechnic squibs
    // (cross-feed capable to any of the 4 engines on a real wide-body fire
    // extinguishing system), plus one APU bottle -- real A380 architecture
    // class, GENERIC per-squib pulse power (a real pyrotechnic squib firing
    // circuit is a low-energy one-shot, small breaker). One-shot/transit-
    // only -- still a real, modelled load, `deep::electrical::live` is what
    // gates its `commanded_on`.
    for bottle in 1..=2u32 {
        for squib in 1..=2u32 {
            let bus = if bottle == 1 { Bus::Dc1 } else { Bus::Dc2 };
            let id: &'static str = Box::leak(format!("eng-fire-bottle-{bottle}-squib-{squib}").into_boxed_str());
            let name: &'static str = Box::leak(format!("ENG FIRE BOTTLE {bottle} SQUIB {squib}").into_boxed_str());
            push_electrical(v, id, name, 26, bus, 20.0, 1.0, "engine fire-extinguisher bottle pyrotechnic squib", "GENERIC: typical one-shot pyrotechnic squib firing circuit, real wide-body cross-feed fire-extinguishing architecture");
        }
    }
    for squib in 1..=2u32 {
        let id: &'static str = Box::leak(format!("apu-fire-bottle-squib-{squib}").into_boxed_str());
        let name: &'static str = Box::leak(format!("APU FIRE BOTTLE SQUIB {squib}").into_boxed_str());
        push_electrical(v, id, name, 26, Bus::DcApu, 20.0, 1.0, "APU fire-extinguisher bottle pyrotechnic squib", "GENERIC: same class as the engine bottle squibs");
    }
}

fn ata29_hydraulics_extra(v: &mut Vec<BreakerDef>) {
    // Transit-only, but unlike the ignition/squib set above
    // `deep::electrical::live` has a real signal for this one (its own
    // emergency-config/`rat_deployed` state) rather than a permanent off.
    push_electrical(v, "rat-deploy-solenoid", "RAT DEPLOY SOLENOID", 29, Bus::DcHot2, 100.0, 1.0, "Ram Air Turbine deployment solenoid", "GENERIC: typical deployment solenoid, hot-bus fed so it works with both engines/APU/main batteries down (real RAT deployment logic requirement)");
    push_electrical(v, "ptu-control-valve", "PTU CONTROL VALVE", 29, Bus::DcEss, 50.0, 0.8, "Power Transfer Unit control valve actuator", "GENERIC: typical motor/solenoid-operated valve actuator, real green/yellow hydraulic power-transfer-unit architecture");
}

fn ata52_doors(v: &mut Vec<BreakerDef>) {
    // Transit-only (draws only while the door is actually moving) -- still
    // a real, modelled load, `deep::electrical::live` is what gates its
    // `commanded_on`.
    push_electrical(v, "cargo-door-fwd-actuator-ctl", "FWD CARGO DOOR ACTUATOR CONTROL", 52, Bus::Dc1, 100.0, 0.8, "forward cargo door electric actuator control", "GENERIC: typical powered cargo door actuator control circuit");
    push_electrical(v, "cargo-door-aft-actuator-ctl", "AFT CARGO DOOR ACTUATOR CONTROL", 52, Bus::Dc2, 100.0, 0.8, "aft cargo door electric actuator control", "GENERIC: same class as the forward cargo door");
}

fn ata33_emergency_lighting(v: &mut Vec<BreakerDef>) {
    push_electrical(v, "emer-lighting-charger-1", "EMER LIGHTING BATTERY CHARGER 1", 33, Bus::DcHot1, 100.0, 1.0, "emergency lighting battery pack charger", "GENERIC: typical NiCd/Li-ion emergency-lighting pack charger circuit");
    push_electrical(v, "emer-lighting-charger-2", "EMER LIGHTING BATTERY CHARGER 2", 33, Bus::DcHot2, 100.0, 1.0, "emergency lighting battery pack charger", "GENERIC: same class as charger 1");
    push_electrical(v, "ext-service-lighting", "EXTERIOR SERVICE LIGHTING", 33, Bus::AcGndFltSvc, 100.0, 1.0, "exterior ground-service lighting circuit", "GENERIC: typical ground-service floodlight circuit");
}

// =======================================================================
// Group 3: control/excitation supplies -- the coordinator's explicit "one
// breaker per FEED, not per unit" and "include control/excitation supplies
// (relay coils, valve actuator supplies, sensor excitation) as their own
// protected circuits where real" instruction. On a real large-transport
// aircraft, a motor/solenoid-operated valve's *actuator* power and its
// *position-indication* microswitch/LVDT excitation are commonly two
// separate small circuits on two separate CBs (the position signal must
// keep working even if maintenance pulls the actuator's own breaker to
// safe the valve, and vice versa) -- real practice this catalogue's own
// existing valve-actuator entries (`ata21`, `ata28_fuel`, `ata29_hydraulics
// _extra`, `ata35_oxygen`, `ata36_bleed`, `ata49_apu`, `ata52_doors`) did
// not yet split out. Every entry here pairs with exactly one of those
// existing breakers by id (`<parent-id>-pos-ind`). Gap-closing pass:
// `deep::electrical::loads.rs::position_indication_supplies` now gives
// each of these 77 its own small (5 W) excitation-circuit `Load`, separate
// from its parent valve's own actuator `Load` -- a real position-indication
// microswitch/LVDT genuinely is its own circuit, not a second tap on the
// actuator's demand -- so every entry here is `protected_load: Some(id)`.
// =======================================================================

fn push_position_excitation(v: &mut Vec<BreakerDef>, parent_id: &'static str, parent_name: &'static str, ata: u16, bus: Bus, basis_suffix: &'static str) {
    let id: &'static str = Box::leak(format!("{parent_id}-pos-ind").into_boxed_str());
    let name: &'static str = Box::leak(format!("{parent_name} POSITION IND").into_boxed_str());
    let basis: &'static str = Box::leak(format!("GENERIC: typical position-indication microswitch/LVDT excitation circuit for a motor-operated valve, separate small CB from its own actuator power circuit (real large-transport fuel/pneumatic-system practice); {basis_suffix}").into_boxed_str());
    // Gap-closing pass: every one of these 77 now has a matching load in
    // `deep::electrical::loads.rs::position_indication_supplies`, same id,
    // same 5 W/unity-pf figure this function has always used.
    push_electrical(v, id, name, ata, bus, 5.0, 1.0, "position-indication microswitch/LVDT excitation circuit", basis);
}

fn ata_control_excitation_supplies(v: &mut Vec<BreakerDef>) {
    // Fuel valves: same per-valve bus cycling as `ata28_fuel`'s own actuator
    // breakers, so each position-indication circuit rides the same feeder
    // class as its own valve.
    let valve_buses = [Bus::Dc1, Bus::Dc2, Bus::DcEss, Bus::DcBat];
    for i in 0..60usize {
        let parent_id: &'static str = Box::leak(format!("fuel-valve-{i}").into_boxed_str());
        let parent_name: &'static str = Box::leak(format!("FUEL VALVE {i}").into_boxed_str());
        push_position_excitation(v, parent_id, parent_name, 28, valve_buses[i % valve_buses.len()], "pairs with this catalogue's own FUEL VALVE actuator breaker");
    }
    push_position_excitation(v, "hotair-1", "HOT AIR VALVE 1", 21, Bus::AcEss, "pairs with HOT AIR VALVE 1's own actuator breaker");
    push_position_excitation(v, "hotair-2", "HOT AIR VALVE 2", 21, Bus::AcEss, "pairs with HOT AIR VALVE 2's own actuator breaker");
    push_position_excitation(v, "fwd-isol-valve", "FWD CARGO ISOL VALVE", 21, Bus::Dc2, "pairs with FWD CARGO ISOL VALVE's own actuator breaker");
    push_position_excitation(v, "bulk-isol-valve", "BULK CARGO ISOL VALVE", 21, Bus::DcEss, "pairs with BULK CARGO ISOL VALVE's own actuator breaker");
    for pack in 1..=2u32 {
        for side in 1..=2u32 {
            let parent_id: &'static str = Box::leak(format!("pack-{pack}-flow-valve-{side}").into_boxed_str());
            let parent_name: &'static str = Box::leak(format!("PACK {pack} FLOW VALVE {side}").into_boxed_str());
            push_position_excitation(v, parent_id, parent_name, 21, Bus::DcEss, "pairs with its own PACK FLOW VALVE actuator breaker");
        }
    }
    for n in 1..=4u32 {
        let bus = if n <= 2 { Bus::Dc1 } else { Bus::Dc2 };
        let parent_id: &'static str = Box::leak(format!("bleed-eng-{n}").into_boxed_str());
        let parent_name: &'static str = Box::leak(format!("BLEED ENG {n} VALVES").into_boxed_str());
        push_position_excitation(v, parent_id, parent_name, 36, bus, "pairs with the shared BLEED ENG valve-set actuator breaker");
    }
    push_position_excitation(v, "ptu-control-valve", "PTU CONTROL VALVE", 29, Bus::DcEss, "pairs with PTU CONTROL VALVE's own actuator breaker");
    push_position_excitation(v, "apu-fuel-shutoff-valve", "APU FUEL SHUTOFF VALVE", 49, Bus::Dc1, "pairs with APU FUEL SHUTOFF VALVE's own actuator breaker");
    push_position_excitation(v, "crew-o2-shutoff", "CREW OXYGEN SHUTOFF VALVE", 35, Bus::Dc1, "pairs with CREW OXYGEN SHUTOFF VALVE's own actuator breaker");
    push_position_excitation(v, "cargo-door-fwd-actuator-ctl", "FWD CARGO DOOR", 52, Bus::Dc1, "pairs with the forward cargo door's own actuator-control breaker");
    push_position_excitation(v, "cargo-door-aft-actuator-ctl", "AFT CARGO DOOR", 52, Bus::Dc2, "pairs with the aft cargo door's own actuator-control breaker");
}

// =======================================================================

fn build_catalog() -> Vec<BreakerDef> {
    let mut v = Vec::with_capacity(400);
    // Group 1: every deep::electrical::loads.rs consumer.
    ata21(&mut v);
    ata26(&mut v);
    ata27(&mut v);
    ata32(&mut v);
    ata34(&mut v);
    ata28_fuel(&mut v);
    ata33_lighting(&mut v);
    ata30_ice_protection(&mut v);
    ata25_galleys(&mut v);
    ata44_ife(&mut v);
    avionics_misc(&mut v);
    ata36_bleed(&mut v);
    // Group 2: other real A380 equipment (mostly gap-closed, see above).
    ata24_power_sources(&mut v);
    ata23_comms(&mut v);
    ata31_recorders(&mut v);
    ata35_oxygen(&mut v);
    ata49_apu(&mut v);
    ata7x_engine(&mut v);
    ata26_extinguishing(&mut v);
    ata29_hydraulics_extra(&mut v);
    ata52_doors(&mut v);
    ata33_emergency_lighting(&mut v);
    // Group 3: control/excitation supplies paired with an existing
    // valve-actuator breaker above.
    ata_control_excitation_supplies(&mut v);
    assign_positions(&mut v);
    v
}

static CATALOG: std::sync::OnceLock<Vec<BreakerDef>> = std::sync::OnceLock::new();

/// The full breaker catalogue, built once. Safe to call from any thread.
pub fn all() -> &'static [BreakerDef] {
    CATALOG.get_or_init(build_catalog)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_id_is_unique() {
        let mut ids: Vec<&str> = all().iter().map(|d| d.id).collect();
        let before = ids.len();
        ids.sort_unstable();
        ids.dedup();
        assert_eq!(ids.len(), before, "duplicate breaker id in the catalogue");
    }

    #[test]
    fn the_catalogue_is_a_substantial_expansion_of_the_265_entry_legacy_catalogue() {
        assert!(all().len() > 390, "expected a substantial catalogue, got {}", all().len());
    }

    #[test]
    fn position_indication_entries_pair_with_a_real_parent_breaker_and_now_carry_their_own_load() {
        // Gap-closing pass: these 77 used to honestly carry no modelled
        // load; `deep::electrical::loads.rs::position_indication_supplies`
        // now gives every one of them a real 5 W excitation-circuit load,
        // same id, so `protected_load` must now be `Some(that same id)`,
        // not `None`.
        let ids: std::collections::HashSet<&str> = all().iter().map(|d| d.id).collect();
        let mut found_any = false;
        for def in all() {
            if let Some(parent_id) = def.id.strip_suffix("-pos-ind") {
                found_any = true;
                assert!(ids.contains(parent_id), "{} has no parent breaker {parent_id}", def.id);
                assert_eq!(def.protected_load, Some(def.id), "{} should now protect its own matching load", def.id);
                assert!(def.basis.contains("GENERIC"), "{} should cite GENERIC", def.id);
            }
        }
        assert!(found_any, "expected at least one position-indication entry");
    }

    #[test]
    fn every_breaker_gets_a_unique_position_on_its_own_panels_grid() {
        use std::collections::HashSet;
        let mut seen: HashSet<(&str, u32, u32)> = HashSet::new();
        for def in all() {
            assert!(def.position.row >= 1, "{} has row {}", def.id, def.position.row);
            assert!(def.position.column >= 1 && def.position.column <= PANEL_COLUMNS, "{} has column {}", def.id, def.position.column);
            assert!(!def.position.label.is_empty(), "{} has no cap label", def.id);
            assert!(def.position.label.chars().count() <= 14, "{} label too long for a real CB cap: {}", def.id, def.position.label);
            let key = (def.panel.code(), def.position.row, def.position.column);
            assert!(seen.insert(key), "{} collides with another breaker at panel {:?} row {} column {}", def.id, def.panel, def.position.row, def.position.column);
        }
    }

    #[test]
    fn every_electrical_group_entry_protects_itself_or_its_own_dual_feed_load() {
        // Every `push_electrical`/`push_electrical_dual`-derived entry was
        // transcribed from `deep::electrical::loads.rs` (read in full this
        // session, including session 2's dual-feed split via `add_dual`)
        // using the load's own id as `protected_load` -- either directly
        // (`def.id == load`, single-feed) or via one of the two real feed-
        // breaker id suffixes `push_electrical_dual` uses
        // (`<load>-normal-bkr` / `<load>-2nd-bkr`), matching `add_dual`'s
        // own breaker-id scheme exactly. `deep::breakers::integration_test`
        // (compiled once the lead wires both areas in) checks this against
        // `deep::electrical`'s actual live catalogue, not just structurally.
        for def in all() {
            if let Some(load) = def.protected_load {
                let self_or_feed = def.id == load || def.id == format!("{load}-normal-bkr") || def.id == format!("{load}-2nd-bkr");
                assert!(self_or_feed, "{} claims to protect {} but is neither that id nor one of its own dual-feed breaker ids", def.id, load);
            }
        }
    }

    #[test]
    fn rating_is_at_or_above_the_loads_own_current_the_standard_size_series_only_rounds_up() {
        for def in all() {
            let plain_current = def.rated_power_w / (def.bus.nominal_voltage() * def.power_factor.max(0.1));
            assert!(def.rating_a >= plain_current - 1e-9, "{}: rating {} A below plain load current {} A", def.id, def.rating_a, plain_current);
            assert!(def.rating_a >= def.raw_current_a - 1e-9, "{}: rating {} A below its own margined current {} A", def.id, def.rating_a, def.raw_current_a);
            let recomputed_raw = margined_current(def.rated_power_w, def.bus.nominal_voltage(), def.power_factor);
            assert!((recomputed_raw - def.raw_current_a).abs() < 1e-6, "{}: stored raw_current_a does not match its own power/voltage/pf", def.id);
            assert!((standard_size(def.raw_current_a) - def.rating_a).abs() < 1e-9, "{}: rating_a is not the standard-size rounding of its own raw current", def.id);
        }
    }

    #[test]
    fn standard_size_only_returns_a_published_series_value_and_never_rounds_down() {
        for &raw in &[0.5, 1.0, 1.1, 24.9, 25.0, 25.1, 99.0, 251.0, 999.0] {
            let s = standard_size(raw);
            assert!(s >= raw, "{s} A rounded below its own raw current {raw} A");
        }
        assert_eq!(standard_size(0.5), 1.0);
        assert_eq!(standard_size(25.0), 25.0);
        assert_eq!(standard_size(999.0), 1000.0);
    }

    #[test]
    fn every_ata_chapter_used_is_a_real_a380_chapter_and_matches_its_own_group() {
        // Every chapter this catalogue actually uses, each a real ATA 100
        // chapter with a real A380 system behind it: 21 air conditioning,
        // 22 auto flight, 23 communications, 24 electrical power, 25
        // equipment/furnishings, 26 fire protection, 27 flight controls,
        // 28 fuel, 29 hydraulic power, 30 ice and rain protection, 31
        // indicating/recording, 32 landing gear, 33 lights, 34 navigation,
        // 35 oxygen, 36 pneumatic, 44 cabin systems, 49 APU, 52 doors
        // (the cargo/passenger door actuator controls), 73 engine fuel and
        // control, 74 ignition. Kept as a whitelist, not a range, so a
        // mistyped chapter on a new entry still fails here.
        const VALID: [u16; 21] = [21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 44, 49, 52, 73, 74];
        for def in all() {
            assert!(VALID.contains(&def.ata), "{} has an unexpected ATA chapter {}", def.id, def.ata);
        }
    }

    #[test]
    fn breaker_kind_follows_its_own_rating_split_and_sspc_entries_land_in_a_power_centre() {
        for def in all() {
            assert_eq!(kind_for(def.rating_a), def.kind, "{} kind does not match its own rating", def.id);
            if def.kind == BreakerKind::Sspc {
                assert!(
                    matches!(def.panel, Panel::PrimaryPowerCentre1 | Panel::PrimaryPowerCentre2 | Panel::PrimaryPowerCentre3 | Panel::PrimaryPowerCentre4 | Panel::SecondaryPowerCentreFwd | Panel::SecondaryPowerCentreAft),
                    "{} is SSPC but not in a power centre",
                    def.id
                );
            }
        }
    }

    #[test]
    fn the_three_battery_output_breakers_have_no_fabricated_load_and_are_honestly_documented() {
        // Gap-closing pass: every other group-2/group-3 id this test used to
        // sample (satcom, dfdr, crew-o2-shutoff, apu-ecu-a, fadec-1a,
        // eng-fire-bottle-1-squib-1, rat-deploy-solenoid,
        // cargo-door-fwd-actuator-ctl, emer-lighting-charger-1) now has a
        // real load in `deep::electrical::loads.rs` and is asserted
        // `protected_load: Some(..)` elsewhere (see
        // `breakers_protecting_no_modelled_load_are_a_known_named_gap` and
        // `deep::electrical::loads::tests::every_closed_gap_id_is_a_real_load`).
        // Only the three battery-output breakers remain a genuine,
        // architectural gap (see `ata24_power_sources`'s own comment).
        let unmodelled_ids = ["bat-1", "bat-2", "bat-apu"];
        for def in all() {
            if unmodelled_ids.contains(&def.id) {
                assert!(def.protected_load.is_none(), "{} should honestly carry no modelled load", def.id);
                assert!(def.basis.contains("GENERIC"), "{} should cite GENERIC since it has no real per-part figure", def.id);
            }
        }
    }

    #[test]
    fn every_entry_has_a_positive_rating_and_a_nonempty_basis_and_consumer() {
        for def in all() {
            assert!(def.rating_a > 0.0, "{} has no positive rating", def.id);
            assert!(!def.basis.is_empty(), "{} has no basis citation", def.id);
            assert!(!def.consumer.is_empty(), "{} has no consumer description", def.id);
        }
    }
}
