use super::topology::{a380_reference_topology, CpiomType, ModuleKind, NetworkSide, NetworkTopology};
use crate::deep::api::*;

const ATA_IMA: u16 = 42;

fn module_kind_str(kind: ModuleKind) -> String {
    match kind {
        ModuleKind::Cpiom(t) => format!(
            "CPIOM-{}",
            match t {
                CpiomType::A => "A",
                CpiomType::B => "B",
                CpiomType::C => "C",
                CpiomType::D => "D",
                CpiomType::E => "E",
                CpiomType::F => "F",
                CpiomType::G => "G",
            }
        ),
        ModuleKind::Iom => "IOM".to_string(),
    }
}

pub fn key_safe(name: &str) -> String {
    name.chars().map(|c| if c.is_ascii_alphanumeric() { c.to_ascii_uppercase() } else { '_' }).collect()
}

pub fn register(r: &mut Registry) {
    let t = a380_reference_topology();
    let mut n: u16 = 0;
    macro_rules! next {
        () => {{
            n += 1;
            n
        }};
    }

    let mut network_failure_ids: [Vec<u64>; 2] = [Vec::new(), Vec::new()];
    let mut cable_ids: Vec<u64> = Vec::new();

    for side in NetworkSide::BOTH {
        for (i, spec) in t.switches[side.index()].iter().enumerate() {
            let comp_id = format!("42_ima.switch_{}", spec.name);
            let mut failures_here = Vec::new();

            let switch_id = failure_id(Area::AvionicsNetwork, ATA_IMA, next!());
            r.failure(FailureDef {
                id: switch_id,
                area: Area::AvionicsNetwork,
                ata: ATA_IMA,
                name: format!("AFDX switch {} failure", spec.name),
                component: comp_id.clone(),
                model_field: "deep::avionics_network::faults::SwitchFaults.failure".into(),
                magnitude: "0 healthy .. 1 fully failed: fraction of frames the switch fails to relay; at 1.0 it also drops out of routing (graph::NetworkGraph treats it as down, same test as FlyByWire's AvionicsFullDuplexSwitch::is_available)".into(),
                effect: "every virtual link routed through this switch on this network side loses this fraction of its frames; at full failure the network side may partition around it, forcing every affected function onto its other network".into(),
            });
            failures_here.push(switch_id);
            network_failure_ids[side.index()].push(switch_id);

            for neighbour in t.switch_ports(side, i) {
                let port_id = failure_id(Area::AvionicsNetwork, ATA_IMA, next!());
                r.failure(FailureDef {
                    id: port_id,
                    area: Area::AvionicsNetwork,
                    ata: ATA_IMA,
                    name: format!("AFDX switch {} port to {:?} failure", spec.name, neighbour),
                    component: comp_id.clone(),
                    model_field: "deep::avionics_network::faults::SwitchFaults.port_failure[neighbour]".into(),
                    magnitude: "0 healthy .. 1 fully failed: fraction of frames lost on the one port facing this neighbour (connector/transceiver), independent of the switch's other ports".into(),
                    effect: "combines with the segment's own cable fault (graph::NetworkGraph::edge_pass_fraction) to reduce that one link's pass fraction; the switch's other ports are unaffected".into(),
                });
                failures_here.push(port_id);
                network_failure_ids[side.index()].push(port_id);
            }

            r.component(ComponentDef {
                id: comp_id,
                area: Area::AvionicsNetwork,
                ata: ATA_IMA,
                name: format!("AFDX Switch {}", spec.name),
                params: vec![
                    ParamDef { name: "failure".into(), meaning: "whole-switch failure fraction".into(), healthy: 0.0 },
                    ParamDef { name: "worst_port_failure".into(), meaning: "highest single-port failure fraction on this switch".into(), healthy: 0.0 },
                ],
                failures: failures_here,
            });
        }
    }

    for side in NetworkSide::BOTH {
        for (a, b) in t.edges(side) {
            let comp_id = format!("42_ima.link_{:?}_{:?}_{:?}", side, a, b);
            let link_id = failure_id(Area::AvionicsNetwork, ATA_IMA, next!());
            r.failure(FailureDef {
                id: link_id,
                area: Area::AvionicsNetwork,
                ata: ATA_IMA,
                name: format!("AFDX cable {:?}-{:?} ({:?}) failure", a, b, side),
                component: comp_id.clone(),
                model_field: "deep::avionics_network::faults::LinkFaults.open".into(),
                magnitude: "0 healthy (continuity) .. 1 fully severed (cable cut/connector pulled)".into(),
                effect: "combines with both endpoints' port faults to reduce this one segment's pass fraction; a full break can partition the graph if this is the only route between the two nodes".into(),
            });
            network_failure_ids[side.index()].push(link_id);
            cable_ids.push(link_id);
            r.component(ComponentDef {
                id: comp_id,
                area: Area::AvionicsNetwork,
                ata: ATA_IMA,
                name: format!("AFDX Cable {:?}-{:?} ({:?})", a, b, side),
                params: vec![ParamDef { name: "open".into(), meaning: "cable open-circuit fraction".into(), healthy: 0.0 }],
                failures: vec![link_id],
            });
        }
    }

    for es in &t.end_systems {
        let comp_id = format!("42_ima.module_{}", es.name);
        let mut failures_here = Vec::new();

        let hardware_id = failure_id(Area::AvionicsNetwork, ATA_IMA, next!());
        r.failure(FailureDef {
            id: hardware_id,
            area: Area::AvionicsNetwork,
            ata: ATA_IMA,
            name: format!("{} hardware failure", es.name),
            component: comp_id.clone(),
            model_field: "deep::avionics_network::faults::ModuleFaults.hardware_failure".into(),
            magnitude: "0 healthy .. 1 fully failed: at 1.0 the module (ModuleFaults::is_available) drops off both AFDX networks entirely".into(),
            effect: "reduces this module's send/receive pass fraction on every virtual link it sources or sinks; full failure loses every function hosted on it".into(),
        });
        failures_here.push(hardware_id);

        let config_id = failure_id(Area::AvionicsNetwork, ATA_IMA, next!());
        r.failure(FailureDef {
            id: config_id,
            area: Area::AvionicsNetwork,
            ata: ATA_IMA,
            name: format!("{} configuration table corruption", es.name),
            component: comp_id.clone(),
            model_field: "deep::avionics_network::faults::ModuleFaults.config_corruption".into(),
            magnitude: "0 healthy .. 1 fully corrupted: fraction of this module's transmitted frames whose payload is scrambled after their checksum is computed".into(),
            effect: "the receiver's CRC-32 integrity check (message::crc32_ieee) catches and discards the affected frames, reported as NoComputedData rather than lost in transit (NoData)".into(),
        });
        failures_here.push(config_id);

        let babble_id = failure_id(Area::AvionicsNetwork, ATA_IMA, next!());
        r.failure(FailureDef {
            id: babble_id,
            area: Area::AvionicsNetwork,
            ata: ATA_IMA,
            name: format!("{} babbling (unregulated transmission)", es.name),
            component: comp_id.clone(),
            model_field: "deep::avionics_network::faults::EndSystemFaults.babbling".into(),
            magnitude: "0 keeps to its virtual links' regulated Bandwidth Allocation Gap .. 1 floods its egress port at the raw 100 Mbit/s line rate regardless of any VL's schedule".into(),
            effect: "oversubscribes the port it attaches to (graph::NetworkGraph::port_load); every other virtual link sharing that port loses frames to queue overflow, not just this module's own traffic".into(),
        });
        failures_here.push(babble_id);

        let power_id = failure_id(Area::AvionicsNetwork, ATA_IMA, next!());
        r.failure(FailureDef {
            id: power_id,
            area: Area::AvionicsNetwork,
            ata: ATA_IMA,
            name: format!("{} loss of bus power", es.name),
            component: comp_id.clone(),
            model_field: "deep::avionics_network::faults::ModuleFaults.powered".into(),
            magnitude: "interface from the electrical model: 0 unpowered, 1 powered (not a continuous fraction — a bus is either energised or not)".into(),
            effect: "an unpowered module (ModuleFaults::is_available false) drops off both AFDX networks entirely, exactly as a full hardware failure does".into(),
        });
        failures_here.push(power_id);

        let mut partition_params = Vec::new();
        for part in &es.partitions {
            let pid = failure_id(Area::AvionicsNetwork, ATA_IMA, next!());
            r.failure(FailureDef {
                id: pid,
                area: Area::AvionicsNetwork,
                ata: ATA_IMA,
                name: format!("{} partition {} failure", es.name, part),
                component: comp_id.clone(),
                model_field: "deep::avionics_network::faults::PartitionFaults.failure".into(),
                magnitude: "0 healthy .. 1 fully down: this one ARINC 653 partition (application) crashed or hung, independent of the module's hardware or its other partitions (partition-level fault containment)".into(),
                effect: "consequences::FunctionAvailability for any function this partition performs reports it lost even while the module's own AFDX interface and every other partition on it keep running".into(),
            });
            failures_here.push(pid);
            partition_params.push(ParamDef { name: format!("partition_{}_failure", key_safe(part)), meaning: format!("{part} (ARINC 653 partition) failure fraction"), healthy: 0.0 });

            if es.name == "CPIOM-C1" && *part == "FWS" {
                let custom_id = failure_id(Area::AvionicsNetwork, ATA_IMA, next!());
                r.failure(FailureDef {
                    id: custom_id,
                    area: Area::AvionicsNetwork,
                    ata: ATA_IMA,
                    name: format!("{} partition {} customization database rejected", es.name, part),
                    component: comp_id.clone(),
                    model_field: "deep::avionics_network::faults::PartitionFaults.customization_db_rejected".into(),
                    magnitude: "inherently boolean (a software validation step either passed or failed, like a failed checksum): 0 accepted, >= 0.5 rejected -- this crate's own discrete BITE-flag convention, not a fabricated continuous threshold".into(),
                    effect: "the FWS function rejected the loaded airline-customization database at startup (314800001 FWS AIRLINE CUSTOMIZATION REJECTED)".into(),
                });
                failures_here.push(custom_id);
                partition_params.push(ParamDef { name: "fws_customization_db_rejected".into(), meaning: "FWS partition airline-customization database rejected at startup: 0 accepted, 1 rejected".into(), healthy: 0.0 });

                let atqc_id = failure_id(Area::AvionicsNetwork, ATA_IMA, next!());
                r.failure(FailureDef {
                    id: atqc_id,
                    area: Area::AvionicsNetwork,
                    ata: ATA_IMA,
                    name: format!("{} partition {} ATQC database rejected", es.name, part),
                    component: comp_id.clone(),
                    model_field: "deep::avionics_network::faults::PartitionFaults.atqc_db_rejected".into(),
                    magnitude: "inherently boolean, same convention as fws_customization_db_rejected above: 0 accepted, >= 0.5 rejected".into(),
                    effect: "the FWS function rejected the loaded ATQC database at startup (314800005 FWS ATQC DATABASE REJECTED)".into(),
                });
                failures_here.push(atqc_id);
                partition_params.push(ParamDef { name: "fws_atqc_db_rejected".into(), meaning: "FWS partition ATQC database rejected at startup: 0 accepted, 1 rejected".into(), healthy: 0.0 });
            }
        }

        let mut params = vec![
            ParamDef { name: "hardware_failure".into(), meaning: "module hardware failure fraction".into(), healthy: 0.0 },
            ParamDef { name: "config_corruption".into(), meaning: "loaded routing/virtual-link configuration table corruption fraction".into(), healthy: 0.0 },
            ParamDef { name: "babbling".into(), meaning: "unregulated egress traffic fraction (BAG shaping defeated)".into(), healthy: 0.0 },
            ParamDef { name: "powered".into(), meaning: "bus energised (interface from the electrical model): 0 unpowered, 1 powered".into(), healthy: 1.0 },
            ParamDef { name: "overheat_trip_frac".into(), meaning: "avionics bay overheat trip (interface from this module's bay's fan/valve failures, registered below under ATA 21): 0 within limits, 1 tripped off".into(), healthy: 0.0 },
        ];
        params.extend(partition_params);

        r.component(ComponentDef { id: comp_id, area: Area::AvionicsNetwork, ata: ATA_IMA, name: format!("{} ({})", es.name, module_kind_str(es.kind)), params, failures: failures_here });
    }

    let both_ids: Vec<u64> = network_failure_ids[0].iter().chain(network_failure_ids[1].iter()).copied().collect();
    r.alert(
        EcamAlert::new(
            "AVNCS_NETWORK_AB_FAULT",
            ATA_IMA,
            "AVIONICS NETWORK ALL SWITCHES FAULT",
            Level::Caution,
            all(vec![
                var("AFDX_NETWORK_A_AVAILABLE").eq(0.0),
                var("AFDX_NETWORK_B_AVAILABLE").eq(0.0),
                var("AVNCS_ANY_MODULE_POWERED").eq(1.0),
            ]),
        )
        .confirm(1.0)
        .raised_by(&both_ids),
    );
    r.alert(
        EcamAlert::new(
            "AVNCS_MULTIPLE_CABLE_FAULT",
            ATA_IMA,
            "AVIONICS NETWORK MULTIPLE CABLE FAULT",
            Level::Advisory,
            var("AFDX_FAILED_CABLE_COUNT").ge(2.0),
        )
        .confirm(1.0)
        .raised_by(&cable_ids),
    );


    register_ventilation(r, &t);
    register_supplemental_cooling(r);
}

fn register_supplemental_cooling(r: &mut Registry) {
    const ATA_COOLING: u16 = 21;
    for (i, n) in [1u16, 2].into_iter().zip([90u16, 92]) {
        let comp = format!("21_vent.avionics_cooling_system_{i}");
        let overheat_id = failure_id(Area::AvionicsNetwork, ATA_COOLING, n);
        let prot_id = failure_id(Area::AvionicsNetwork, ATA_COOLING, n + 1);
        r.component(ComponentDef {
            id: comp.clone(),
            area: Area::AvionicsNetwork,
            ata: ATA_COOLING,
            name: format!("Avionics cooling system {i} (liquid loop + heat exchanger)"),
            params: vec![
                ParamDef { name: "overheat".into(), meaning: "the cooling loop itself overheated, 0 healthy .. 1 fully faulted".into(), healthy: 0.0 },
                ParamDef { name: "protection_fault".into(), meaning: "the loop's own overheat-detection/protection monitor lost, 0 healthy .. 1 fully faulted".into(), healthy: 0.0 },
            ],
            failures: vec![overheat_id, prot_id],
        });
        r.failure(FailureDef {
            id: overheat_id,
            area: Area::AvionicsNetwork,
            ata: ATA_COOLING,
            name: format!("Avionics cooling system {i} overheat"),
            component: comp.clone(),
            model_field: "avionics_network::live::LiveAvionicsNetwork.coolg_overheat_id".into(),
            magnitude: "0 healthy .. 1 fully faulted -- a component-broken flag; this network has no cooling-loop thermal-cycle physics, so there is no real magnitude to degrade and no threshold to invent".into(),
            effect: format!("FlyByWire's own {} VENT COOLG SYS {i} OVHT", if i == 1 { "212800019" } else { "212800020" }),
        });
        r.failure(FailureDef {
            id: prot_id,
            area: Area::AvionicsNetwork,
            ata: ATA_COOLING,
            name: format!("Avionics cooling system {i} protection fault"),
            component: comp,
            model_field: "avionics_network::live::LiveAvionicsNetwork.coolg_prot_fault_id".into(),
            magnitude: "0 healthy .. 1 fully faulted".into(),
            effect: "FlyByWire's own 212800021 VENT COOLG SYS PROT FAULT (one id covers either system's protection monitor, OR'd)".into(),
        });
    }
}

fn register_ventilation(r: &mut Registry, t: &NetworkTopology) {
    const ATA_COOLING: u16 = 21;
    let mut n: u16 = 0;
    macro_rules! next {
        () => {{
            n += 1;
            n
        }};
    }

    let mut bays: Vec<&'static str> = t.end_systems.iter().map(|e| e.bay).collect();
    bays.sort_unstable();
    bays.dedup();

    for bay in bays {
        let modules_here: Vec<&str> = t.end_systems.iter().filter(|e| e.bay == bay).map(|e| e.name).collect();
        let mut all_ids_for_bay = Vec::new();

        for fan_role in ["PRIMARY", "STANDBY"] {
            let comp_id = format!("21_vent.{bay}_fan_{fan_role}");
            let fid = failure_id(Area::AvionicsNetwork, ATA_COOLING, next!());
            r.failure(FailureDef {
                id: fid,
                area: Area::AvionicsNetwork,
                ata: ATA_COOLING,
                name: format!("{bay} {fan_role} extraction fan failure"),
                component: comp_id.clone(),
                model_field: "deep::avionics_network::ventilation::FanFaults.failure".into(),
                magnitude: "0 healthy .. 1 fully failed (seized bearing/burnt winding): fraction less air the fan moves than commanded".into(),
                effect: "the bay's airflow fraction is the *better* of its fans (ventilation::Bay::step takes the max, since bays commonly run both fans together for margin); only losing every fan in the bay collapses it to natural convection".into(),
            });
            all_ids_for_bay.push(fid);
            r.component(ComponentDef {
                id: comp_id,
                area: Area::AvionicsNetwork,
                ata: ATA_COOLING,
                name: format!("{bay} {fan_role} Extraction Fan"),
                params: vec![ParamDef { name: "failure".into(), meaning: "fan failure fraction".into(), healthy: 0.0 }],
                failures: vec![fid],
            });
        }

        let valve_comp_id = format!("21_vent.{bay}_extract_valve");
        let valve_id = failure_id(Area::AvionicsNetwork, ATA_COOLING, next!());
        r.failure(FailureDef {
            id: valve_id,
            area: Area::AvionicsNetwork,
            ata: ATA_COOLING,
            name: format!("{bay} extract valve stuck closed"),
            component: valve_comp_id.clone(),
            model_field: "deep::avionics_network::ventilation::ExtractValveFaults.stuck_closed".into(),
            magnitude: "0 healthy (follows command) .. 1 stuck fully closed regardless of command".into(),
            effect: "blocks the forced draught even with healthy fans (fan and valve are in series in the one duct); the bay collapses to natural convection exactly as losing every fan would".into(),
        });
        all_ids_for_bay.push(valve_id);
        r.component(ComponentDef {
            id: valve_comp_id,
            area: Area::AvionicsNetwork,
            ata: ATA_COOLING,
            name: format!("{bay} Extract Valve"),
            params: vec![ParamDef { name: "stuck_closed".into(), meaning: "extract valve stuck-closed fraction".into(), healthy: 0.0 }],
            failures: vec![valve_id],
        });

        let _ = (&modules_here, &all_ids_for_bay);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers_without_duplicate_or_dangling_ids() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate_area();
        assert!(errors.is_empty(), "{errors:?}");
    }

    #[test]
    fn every_failure_id_carries_its_own_ata_chapter() {
        let mut r = Registry::default();
        register(&mut r);
        for f in &r.failures {
            assert_eq!(f.id / 1_000 % 1_000, f.ata as u64);
            assert!(f.ata == ATA_IMA || f.ata == 21, "unexpected ATA chapter {}", f.ata);
        }
        assert!(r.failures.iter().any(|f| f.ata == ATA_IMA));
        assert!(r.failures.iter().any(|f| f.ata == 21));
    }

    #[test]
    fn every_end_system_gets_a_component() {
        let mut r = Registry::default();
        register(&mut r);
        let t: NetworkTopology = a380_reference_topology();
        for es in &t.end_systems {
            assert!(r.components.iter().any(|c| c.id == format!("42_ima.module_{}", es.name)), "missing component for {}", es.name);
        }
    }
}
