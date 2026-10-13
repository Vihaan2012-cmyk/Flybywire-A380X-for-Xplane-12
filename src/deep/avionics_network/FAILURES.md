# Failures — avionics_network

One line per genuinely distinct physical fault. Every fault is also registered in code
(`registry.rs`, `pub fn register`) with its exact model field, sourced constants and the ECAM
alert(s) it can raise; this file is the flat summary the brief asks for.

| ATA | proposed name | model element it acts on | magnitude meaning (0..1) | effect |
|---|---|---|---|---|
| 42 | AFDX switch failure | `faults::SwitchFaults.failure` | fraction of frames the switch fails to relay; 1.0 removes it from routing (`graph::NetworkGraph::shortest_path`/`reachable`) | every VL routed through it on that network side loses this fraction; full failure can partition the side around it |
| 42 | AFDX switch port failure | `faults::SwitchFaults.port_failure[neighbour]` | fraction of frames lost on the one port facing a given neighbour, independent of the switch's other ports | combines with the segment's own cable fault to reduce that one link's pass fraction; other ports unaffected |
| 42 | AFDX cable/link (segment) failure | `faults::LinkFaults.open` | open-circuit fraction of one physical segment | reduces that segment's pass fraction; a full break can partition the graph if it is the only route |
| 42 | Babbling end system | `faults::EndSystemFaults.babbling` | fraction of the raw 100 Mbit/s line rate flooded regardless of any virtual link's Bandwidth Allocation Gap | oversubscribes the port it attaches to (`graph::NetworkGraph::port_load`); every other VL sharing that port loses frames to queue overflow, not just its own traffic |
| 42 | CPIOM/IOM hardware failure | `faults::ModuleFaults.hardware_failure` | module hardware degradation; 1.0 makes `ModuleFaults::is_available` false | reduces (or, at 1.0, zeroes) this module's send/receive pass fraction on every VL it sources or sinks |
| 42 | CPIOM/IOM configuration table corruption | `faults::ModuleFaults.config_corruption` | fraction of transmitted frames whose payload is scrambled after their checksum would have been computed | the receiver's CRC-32 check (`message::crc32_ieee`) catches and discards them: `DataStatus::NoComputedData`, not lost in transit |
| 42 | CPIOM/IOM partition (application) failure | `faults::PartitionFaults.failure` | one ARINC 653 partition crashed/hung, independent of its module's hardware and its sibling partitions | any function that partition performs is lost (`consequences`) even while the module's AFDX interface and other partitions keep running |
| 42 | CPIOM/IOM loss of bus power (interface) | `faults::ModuleFaults.powered` | boolean, from the electrical model | module drops off both AFDX networks entirely, same as a full hardware failure |
| 42 | Avionics module overheat trip (consequence, not directly injectable) | `faults::ModuleFaults.overheat_trip_frac`, driven by `ventilation::OverheatTrip::step` from the module's bay temperature | 0 within limits .. 1 tripped off | takes the module off both networks exactly as a full hardware failure does; caused by the two ATA 21 faults below, not injected directly |
| 21 | Avionics bay extraction fan failure | `ventilation::FanFaults.failure` | fraction less air the fan moves than commanded | bay airflow is the best of its fans (`ventilation::Bay::step` takes the max); only losing every fan in the bay collapses it to natural convection |
| 21 | Avionics bay extract valve stuck closed | `ventilation::ExtractValveFaults.stuck_closed` | fraction stuck closed regardless of command | blocks the forced draught even with healthy fans (fan and valve are in series in the one duct); same consequence as losing every fan |
| 42 | ARINC 429 bus open circuit | `arinc429::BusFaults.open` | fraction of words that never reach the receiver | link goes stale, `DataStatus::NoData`, once nothing valid arrives within the staleness window |
| 42 | ARINC 429 bus short circuit | `arinc429::BusFaults.short` | fraction of words that arrive with the line held/glitching such that parity fails | receiver rejects them, `DataStatus::NoComputedData`; the word's fields (including its own SSM) are never trusted |

13 distinct faults, spanning the graph (switches/ports/segments), the end systems hosting
CPIOM/IOM applications, the avionics bay cooling those modules depend on, and the legacy ARINC
429 gateway. See `registry.rs` for per-instance registration (16 switches x ~5 ports, ~18
segments per network, 5 end systems x partitions, 2 bays x 2 fans + 1 valve, over the reference
topology in `topology::a380_reference_topology`).
