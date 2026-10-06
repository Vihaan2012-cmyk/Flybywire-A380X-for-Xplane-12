//! Weight and balance: the payload and fuel FlyByWire's systems load, put on
//! X-Plane's aircraft with the centre of gravity MSFS would compute.
//!
//! In MSFS the aircraft's mass is `empty_weight` at `empty_weight_CG_position`
//! plus every payload station's `PAYLOAD STATION WEIGHT:n` at its
//! `station_load` position plus every fuel tank's weight at its `Position`,
//! all in feet from the reference datum (flight_model.cfg:16-57 and the
//! `[FUEL_SYSTEM]` tanks, lines 142-159). FlyByWire's payload aspect keeps the
//! station weights in pounds (a380_systems_wasm payload.rs:10-83, ported in
//! aspects.rs), and fuel.rs keeps `FUELSYSTEM TANK WEIGHT:1-16` in pounds.
//!
//! X-Plane's side (DataRefs.txt):
//! - the payload mass goes to `sim/flightmodel/weight/m_fixed` (kg), or, since
//!   that is "sum of all stations if airplane has stations", to
//!   `m_stations` when the .acf defines stations
//!   (`sim/aircraft/weight/acf_m_station_max` not all zero). The converter
//!   groups the cfg's 19 stations into X-Plane's nine at their weighted arms
//!   ([`station_groups`], the same grouping), and each group's weight goes
//!   to its station. The balance below still uses every cfg station's own
//!   arm.
//! - the centre of gravity goes to `sim/flightmodel2/misc/cg_offset_z`
//!   (metres, "including payload, and including internal and external
//!   fuel"), not the deprecated `sim/flightmodel/misc/cgz_ref_to_default`,
//!   which is the zero fuel CG. It is an offset from the aircraft's reference
//!   point, `sim/aircraft/weight/acf_cgZ_original` (feet). The converter
//!   writes that point from the cfg's empty weight CG in X-Plane's axes by
//!   default (z aft, z = -(MSFS z + datum z), msfs2xp-aircraft acf.rs's
//!   `acf_point`) -- but `--cg-z` can override it with a certificated figure
//!   instead when the cfg's own is known wrong (the installed A380 uses
//!   `--cg-z -8`, not the cfg's -16 ft: main.rs's own doc comment on that
//!   flag, and `acf/_cgZ -8.000000000` in the installed .acf). `corrected_empty`
//!   re-derives the empty mass this module sums from whichever one X-Plane
//!   actually reports, every tick, so an override is never silently
//!   cancelled out of the CG this module writes (see its own doc comment for
//!   the failure mode when it was).
//! - X-Plane's own fuel sits in its nine tanks, two of which merge MSFS's
//!   outer tanks into the outer feed tanks; the offset written here is the
//!   whole aircraft's MSFS centre of gravity, eleven tanks at their own
//!   positions included, so it does not matter where X-Plane puts its fuel.
//!
//! Lateral balance: the A380's stations and tanks are symmetrical but for the
//! 1 lb crew stations, and X-Plane's `cg_offset_x` can only be set on aircraft
//! with stations off the centreline, so only the longitudinal CG is written.
//! X-Plane has no vertical CG offset dataref.

use systems::simulation::{SimulatorReaderWriter, VariableIdentifier, VariableRegistry};

use crate::published::{self, Published, Value};
use crate::xp::{DataRef, Xplm};
use crate::Vars;

pub use crate::mass_balance::*;

pub struct WeightBalance {
    balance: Balance,
    groups: Vec<Vec<usize>>,
    stations: Vec<VariableIdentifier>,
    tanks: Vec<VariableIdentifier>,
    m_fixed: Option<DataRef>,
    m_stations: Option<DataRef>,
    station_max: Option<DataRef>,
    cg_offset_z: Option<DataRef>,
    reference_z: Option<DataRef>,
    _published: Published,
    payload_kg: Value,
    gross_kg: Value,
    /// Last CG offset handed to X-Plane, so the log speaks on a real move.
    last_offset_m: f64,
    /// Last total weight reported, so the log speaks when the load changes.
    last_total_lb: f64,
    /// Whether FlyByWire has ever published a payload, so its silent zero is
    /// never mistaken for an empty aircraft.
    payload_reported: bool,
    cg_z_ft: Value,
}

impl WeightBalance {
    pub fn new(vars: &mut Vars, xplm: &Xplm) -> Self {
        let balance = parse(FLIGHT_MODEL_CFG);
        let stations: Vec<VariableIdentifier> =
            (1..=balance.stations.len()).map(|n| vars.get(format!("PAYLOAD STATION WEIGHT:{n}"))).collect();
        let tanks = (1..=balance.tanks.len()).map(|n| vars.get(format!("FUELSYSTEM TANK WEIGHT:{n}"))).collect();
        // The crew stations (18 and 19, "CAPTAIN" and "FIRST OFFICER") are
        // no payload station of FlyByWire's; they keep the cfg's weight, as
        // MSFS loads it.
        // Every station starts at the cfg's own `station_load`, as MSFS
        // loads it. Stations 18 and 19 ("CAPTAIN"/"FIRST OFFICER") are no
        // payload station of FlyByWire's and simply keep theirs; the rest
        // are FlyByWire's to change, and its payload module syncs its
        // boarding L:Vars from these A:Vars, so a seed here is what the EFB
        // then shows and edits rather than something it fights.
        //
        // Seeding only the crew left the aircraft permanently empty -- a
        // 2 lb payload on a 300 t airframe -- because nothing else ever
        // wrote them: FlyByWire boards to what the EFB asks for, and the
        // EFB had asked for nothing.
        for (id, station) in stations.iter().zip(&balance.stations) {
            vars.write(id, station.pounds);
        }
        let mut p = Published::default();
        let payload_kg = p.number("fbw/wb/payload_kg", 0., false);
        let gross_kg = p.number("fbw/wb/gross_weight_kg", 0., false);
        let cg_z_ft = p.number("fbw/wb/cg_z_ft", 0., false);
        Self {
            stations,
            tanks,
            m_fixed: xplm.find("sim/flightmodel/weight/m_fixed"),
            m_stations: xplm.find("sim/flightmodel/weight/m_stations"),
            station_max: xplm.find("sim/aircraft/weight/acf_m_station_max"),
            cg_offset_z: xplm.find("sim/flightmodel2/misc/cg_offset_z"),
            reference_z: xplm.find("sim/aircraft/weight/acf_cgZ_original"),
            groups: station_groups(&balance),
            balance,
            _published: p,
            payload_kg,
            gross_kg,
            cg_z_ft,
            last_offset_m: f64::NEG_INFINITY,
            last_total_lb: f64::NEG_INFINITY,
            payload_reported: false,
        }
    }

    /// After the systems, the aspects and the fuel have written this tick's
    /// weights.
    pub fn update(&mut self, vars: &mut Vars, xplm: &Xplm) {
        let b = &self.balance;
        // See `corrected_empty`'s doc comment: anchor the empty mass on the
        // .acf's own declared reference point, not the cfg's raw (and on
        // this A380, `--cg-z`-overridden) figure, or the override never
        // reaches X-Plane's applied CG.
        let reference_ft = self.reference_z.map(|r| xplm.get_f(r) as f64);
        let empty = corrected_empty(b.empty, b.datum[0], reference_ft);
        let stations: Vec<Mass> = self
            .stations
            .iter()
            .zip(&b.stations)
            .map(|(id, s)| Mass { pounds: vars.read(id).max(0.), position: s.position })
            .collect();
        let tanks: Vec<Mass> =
            self.tanks.iter().zip(&b.tanks).map(|(id, at)| Mass { pounds: vars.read(id).max(0.), position: *at }).collect();
        let payload_lb: f64 = stations.iter().map(|s| s.pounds).sum();
        let stations_lb: Vec<f64> = stations.iter().map(|s| s.pounds).collect();
        let (total_lb, cg) = centre_of_gravity(std::iter::once(empty).chain(stations).chain(tanks));

        let payload_kg = payload_lb * LB_TO_KG;
        let has_stations = self.station_max.is_some_and(|d| {
            let mut max = [0f32; 9];
            xplm.get_vf(d, &mut max);
            max.iter().any(|&m| m > 0.)
        });
        // This module is what crashes the converted A380 (with its writes
        // skipped the aircraft stands indefinitely; with them it falls over
        // 35-60 s after every load, gear and struts healthy throughout).
        // Report every mass it hands X-Plane, and which branch took it.
        if (total_lb - self.last_total_lb).abs() > 500. {
            self.last_total_lb = total_lb;
            let kg_preview: Vec<i64> = self
                .groups
                .iter()
                .map(|g| (g.iter().map(|&i| stations_lb[i]).sum::<f64>() * LB_TO_KG).round() as i64)
                .collect();
            crate::log(&format!(
                "weight/balance -> X-Plane: total {total_lb:.0} lb, payload {payload_lb:.0} lb, cg {:.2} ft, has_stations {has_stations}, m_stations {}, m_fixed {}, station kg {kg_preview:?}",
                cg[0],
                self.m_stations.is_some(),
                self.m_fixed.is_some()
            ));
        }
        let skip_stations = crate::xp_writes_skip("weight-stations");
        let skip_cg = crate::xp_writes_skip("weight-cg");
        // FlyByWire's payload variables are among the ones nothing feeds in
        // this port, so before the loadsheet has been applied they all read
        // zero -- and zero here does not mean "an empty aircraft", it means
        // "nobody has said yet". Stamping that onto X-Plane's own payload
        // every tick is what crashes the converted A380 35-60 s after every
        // load: with this write skipped it stands indefinitely, with it the
        // aircraft falls over while its gear and struts measure healthy
        // throughout (bisected against every other write the plugin makes).
        //
        // So the payload is only driven once FlyByWire has published one.
        // Until then X-Plane keeps whatever the .acf and the user's own
        // loadsheet put there. The latch stays set for the session on the
        // first real figure, so unloading to genuinely empty still passes
        // through afterwards.
        if !self.payload_reported && payload_lb > MIN_REPORTED_PAYLOAD_LB {
            self.payload_reported = true;
            crate::log(&format!("weight/balance: FlyByWire's payload is live ({payload_lb:.0} lb); driving X-Plane's from here"));
        }
        let skip_stations = skip_stations || !self.payload_reported;
        // ... and the centre of gravity waits on the same latch, for the
        // same reason, which the payload fix above missed.
        //
        // Withholding the payload leaves X-Plane holding whatever the .acf
        // and the user's loadsheet put there -- 55 tonnes of it, measured:
        // this module computed a total of 694 038 lb from an empty airframe,
        // no stations and the tanks, while X-Plane's own `TOTAL WEIGHT` read
        // 815 226 lb at the same moment. The centre of gravity below is
        // computed from *this* module's masses, so while the payload is
        // being withheld it is the centre of gravity of an aeroplane
        // carrying nobody, and it was being stamped onto an aeroplane
        // carrying fifty-five tonnes at the .acf's own arms.
        //
        // A balance point that does not describe the mass it is applied to
        // puts the weight in the wrong place: the aircraft rode its nose
        // gear down the whole take-off roll at two to three degrees nose
        // down, hammering the strut hard enough to throw +/-127 deg/s^2 of
        // pitch acceleration through the airframe, and would not rotate.
        //
        // So it is withheld together with the payload it belongs to. Until
        // FlyByWire publishes a loadsheet, X-Plane's own balance is the
        // consistent one -- its payload and its centre of gravity describe
        // the same aeroplane -- and a half-applied one is worse than none.
        let skip_cg = skip_cg || !self.payload_reported;
        // X-Plane takes the payload either as one total (`m_fixed`) or per
        // station (`m_stations`). The per-station array is what crashed the
        // converted A380 -- rewriting all nine every tick puts the aircraft
        // on its belly within a minute (bisected: with this write skipped it
        // stands indefinitely). The total is enough here, because the centre
        // of gravity is written explicitly below from every cfg station's
        // own arm, so nothing is lost by not spreading the mass across
        // X-Plane's own nine.
        let use_stations = false;
        match (use_stations && has_stations && !skip_stations, self.m_stations.filter(|_| !skip_stations), self.m_fixed.filter(|_| !skip_stations)) {
            (true, Some(d), _) => {
                // Each group in the .acf station the converter made for it.
                let mut kg = [0f32; XPLANE_STATIONS];
                for (slot, group) in kg.iter_mut().zip(&self.groups) {
                    *slot = (group.iter().map(|&i| stations_lb[i]).sum::<f64>() * LB_TO_KG) as f32;
                }
                xplm.set_vf(d, &kg);
            }
            (_, _, Some(d)) => xplm.set_f(d, payload_kg as f32),
            _ => {}
        }
        if let (Some(d), Some(reference), true) = (self.cg_offset_z.filter(|_| !skip_cg), reference_ft, total_lb > 0.) {
            let offset = xplane_cg_offset_z(cg[0], b.datum[0], reference);
            // This module is the one that crashes the converted A380: with
            // its writes skipped the aircraft sits indefinitely, with them
            // it falls over 35-60 s after every load, while the gear and
            // struts measure healthy throughout. Report what it hands
            // X-Plane the first time and whenever it moves appreciably.
            if (offset - self.last_offset_m).abs() > 0.05 {
                self.last_offset_m = offset;
                crate::log(&format!(
                    "weight/balance -> X-Plane: cg_offset_z {offset:.3} m (cg {:.2} ft, datum {:.2} ft, acf_cgZ_original {reference:.2} ft), total {:.0} lb, payload {:.0} lb, stations {:?}",
                    cg[0],
                    b.datum[0],
                    total_lb,
                    payload_lb,
                    stations_lb.iter().map(|v| v.round() as i64).collect::<Vec<_>>()
                ));
            }
            xplm.set_f(d, offset as f32);
        }
        published::set(self.payload_kg, payload_kg);
        published::set(self.gross_kg, total_lb * LB_TO_KG);
        published::set(self.cg_z_ft, cg[0]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_cfg_balance_is_read_whole() {
        let b = parse(FLIGHT_MODEL_CFG);
        assert_eq!(b.datum, [0., 0., 0.]);
        assert_eq!(b.empty, Mass { pounds: 661_403., position: [16., 0., 2.8] });
        // station_load.0 .. .18
        assert_eq!(b.stations.len(), 19);
        assert_eq!(b.stations[0], Mass { pounds: 5185.3, position: [75.7, 0., 7.1] });
        assert_eq!(b.stations[16], Mass { pounds: 5540., position: [-52.9, 0., -0.71] });
        assert_eq!(b.stations[18].position, [105.3, 1.95, 7.1]);
        assert_eq!((b.station_kinds[0], b.station_kinds[14], b.station_kinds[18]), (3, 6, 2));
        // The converter's .acf stations, in the same order.
        assert_eq!(
            station_groups(&b),
            vec![vec![0, 1, 10], vec![2, 3, 4], vec![5, 6, 7], vec![8, 9, 13], vec![11, 12], vec![14], vec![15], vec![16], vec![17, 18]]
        );
        // Tank.1 .. Tank.16
        assert_eq!(b.tanks.len(), 16);
        assert_eq!(b.tanks[0], [-25., -100., 8.5]);
        assert_eq!(b.tanks[10], [-87.14, 0., 12.1]);
    }

    #[test]
    fn the_cg_is_the_moment_over_the_mass() {
        let b = parse(FLIGHT_MODEL_CFG);
        let (w, cg) = centre_of_gravity([b.empty]);
        assert_eq!((w, cg[0]), (661_403., 16.));
        // 63000 lb in the forward hold at 67.4 ft moves it forward.
        let (w, cg) = centre_of_gravity([b.empty, Mass { pounds: 63_000., position: b.stations[14].position }]);
        assert_eq!(w, 724_403.);
        assert!((cg[0] - (661_403. * 16. + 63_000. * 67.4) / 724_403.).abs() < 1e-9);
        // Full trim tank (6260.3 gal at 6.699 lb/gal) moves it aft.
        let (_, aft) = centre_of_gravity([b.empty, Mass { pounds: 41_937., position: b.tanks[10] }]);
        assert!(aft[0] < 16.);
    }

    #[test]
    fn x_plane_measures_aft_of_its_reference_point() {
        // The empty CG is the reference: no offset.
        assert_eq!(xplane_cg_offset_z(16., 0., -16.), 0.);
        // One foot forward in MSFS is 0.3048 m forward (negative) in X-Plane.
        assert!((xplane_cg_offset_z(17., 0., -16.) + 0.3048).abs() < 1e-12);
    }

    #[test]
    fn the_empty_cg_follows_the_acfs_own_reference_point() {
        let empty = Mass { pounds: 661_403., position: [16., 0., 2.8] };
        // No override: the converter wrote `acf_cgZ_original` straight from
        // the cfg (`p[2] = -(lon + datum)`), so the corrected position is
        // the cfg's own -- unchanged behaviour.
        assert_eq!(corrected_empty(empty, 0., Some(-16.)), empty);
        // `--cg-z -8` (the installed A380's actual value, main.rs:79): the
        // corrected position is 8 ft, not the cfg's 16.
        assert_eq!(corrected_empty(empty, 0., Some(-8.)).position[0], 8.);
        // Lateral/vertical are untouched either way.
        assert_eq!(corrected_empty(empty, 0., Some(-8.)).position[1..], empty.position[1..]);
        // No reference dataref found: keep the cfg's own, as before this fix.
        assert_eq!(corrected_empty(empty, 0., None), empty);
    }

    #[test]
    fn a_cg_z_override_reaches_the_applied_centre_of_gravity() {
        // Before this fix, `centre_of_gravity` always anchored on the cfg's
        // raw empty CG (16 ft) regardless of the .acf's own reference point,
        // so `xplane_cg_offset_z` -- an offset *from* that reference --
        // algebraically cancelled it out of X-Plane's final applied CG
        // (reference_ft + offset == -(cg_msfs_ft + datum), independent of
        // reference_ft): a `--cg-z` override never reached the sim. With the
        // fix, an aircraft at exactly the empty weight lands exactly on the
        // .acf's own declared reference point instead of back on the cfg's.
        let empty = corrected_empty(Mass { pounds: 661_403., position: [16., 0., 2.8] }, 0., Some(-8.));
        let (_, cg) = centre_of_gravity([empty]);
        let offset_m = xplane_cg_offset_z(cg[0], 0., -8.);
        assert!(offset_m.abs() < 1e-9, "empty aircraft should land on the acf's own -8 ft reference, got offset {offset_m} m");
    }
}

#[cfg(test)]
mod latch_tests {
    /// The payload and the centre of gravity are one decision, not two.
    ///
    /// This module withholds the payload until FlyByWire publishes a real
    /// loadsheet, because its payload variables read zero before that and
    /// zero means "nobody has said yet", not "an empty aircraft". Stamping
    /// that zero onto X-Plane put the converted A380 on its belly within a
    /// minute.
    ///
    /// The centre of gravity is computed from the same masses. Withholding
    /// one and writing the other leaves X-Plane holding a payload this
    /// module does not know about -- 55 tonnes of it, measured -- under a
    /// balance point computed as though it were not there. Both are
    /// withheld together, and this asserts the two conditions are the same
    /// expression rather than two that happen to agree today.
    #[test]
    fn the_centre_of_gravity_is_withheld_whenever_the_payload_is() {
        let src = include_str!("weight_balance.rs");
        let payload = src
            .lines()
            .find(|l| l.contains("let skip_stations = skip_stations ||"))
            .expect("the payload latch");
        let cg = src.lines().find(|l| l.contains("let skip_cg = skip_cg ||")).expect("the centre-of-gravity latch");
        let condition = |line: &str| line.split("||").nth(1).map(|c| c.trim().trim_end_matches(';').to_owned());
        assert_eq!(
            condition(payload),
            condition(cg),
            "the payload and the centre of gravity must wait on the same condition:\n  {payload}\n  {cg}"
        );
    }
}
