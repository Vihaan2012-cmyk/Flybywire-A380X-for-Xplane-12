//! The deep-systems push: new physical models, each area in its own
//! directory, all registering through `api`.

pub mod api;
pub mod apu;
/// ATA 22: the FCU (AFS control panel), its two MFD backups, and the
/// TCAS/AP mode arbitration fault. New with the ECAM-completeness pass;
/// see `E-AIR-DESIGN.md`'s ATA 22 section for why this is a new area
/// rather than being folded into an existing one -- no `deep::` area
/// modelled the autoflight control panel before this.
pub mod autoflight;
pub mod avionics_network;
pub mod breakers;
pub mod cabin;
/// ATA 23: CIDS, the cockpit PTT switches, the ATSU/datalink router, and
/// the HF/SATCOM/VHF transceivers' own LRU faults. New with the ECAM-
/// completeness pass; see `E-AIR-DESIGN.md`'s ATA 23 section.
pub mod communications;
pub mod ecam;
pub mod electrical;
#[cfg(test)]
mod cost_profile;
#[cfg(test)]
mod electrical_crate_parity;
pub mod engine_accessories;
pub mod environment;
pub mod fire_ice;
pub mod flight_controls;
pub mod fuel;
pub mod gear_structure;
pub mod hydraulics;
pub mod integration;
pub mod frame;
pub mod live;
/// The variable bridge to a host simulator -- what the deep layer
/// publishes and reads, behind a `VarStore` trait so one mapping serves
/// X-Plane's datarefs and MSFS's LVars alike.
pub mod lvar_bridge;
/// Where `Truth`'s ten X-Plane-specific inputs come from in MSFS, with
/// their unit conversions and how far each has been verified.
pub mod msfs_inputs;
pub mod oxygen;
/// The plugin's own side of `live`: filling `Truth`, snapshotting `Faults`
/// and turning a published name into a `Vars` write. Kept out of `live`
/// itself so the contract stays free of `crate::Vars`/X-Plane.
pub mod plugin;
pub mod pneumatic_ducts;
pub mod sensors;
pub mod thermal_zones;
/// The weather capability `deep` needs from a host (X-Plane, MSFS),
/// behind a trait -- see the module doc for why this lives here rather
/// than borrowing `crate::xp`'s types.
pub mod weather;
pub mod wiring;

mod all_registry;
pub use all_registry::registry;

#[cfg(test)]
mod tests {

    #[test]
    fn every_area_registers_together_without_collisions_or_dangling_references() {
        let r = super::registry();
        let errors = r.validate();
        let mut by_area: std::collections::BTreeMap<String, usize> = Default::default();
        for f in &r.failures {
            *by_area.entry(format!("{:?}", f.area)).or_default() += 1;
        }
        println!("DEEP failures {} components {} ECAM alerts {}", r.failures.len(), r.components.len(), r.alerts.len());
        println!("DEEP by area {by_area:?}");
        for e in errors.iter().take(40) {
            println!("DEEP error: {e}");
        }
        assert!(errors.is_empty(), "{} registry errors", errors.len());
    }
}
