# Progress — avionics_network

- [done] Backlog 1 (network graph + redundancy + message delivery + integrity) — `topology.rs`,
  `graph.rs`, `message.rs` — dual A/B AFDX topology (reference set mirrors FlyByWire's public
  `avionics_data_communication_network.rs` switch adjacency, lines 145-154 as read, and its
  CPIOM/IOM switch attachment, lines 227-291), BFS reachability generalised from FlyByWire's
  `switches_reachable`, shortest-path + continuous pass-fraction for partial faults, bandwidth
  allocation gap check (`graph::PortLoad`, real BAG/frame-size arithmetic vs. the 100 Mbit/s AFDX
  line rate), first-valid-wins redundancy management across both networks, store-and-forward
  latency + bounded jitter, and a real CRC-32 (IEEE 802.3) integrity check.
- [done] Backlog 2 (faults) — `faults.rs` (switch/port/link/end-system/module/partition fault
  structs), exercised through `graph.rs`/`message.rs`; `ventilation.rs` (fan/valve faults feeding
  the overheat interface). All 13 distinct faults listed in `FAILURES.md`.
- [done] Backlog 3 (consequences) — `consequences.rs`: `Availability::{Normal,Degraded,Lost}`
  computed from a function's actual sources (`SourceStatus`), `FunctionMonitor` bundling the AFDX
  receivers a function depends on as one steppable/readable unit, `Combine::{All,Any}` for
  functions with independent redundant sources vs. functions needing every input.
- [done] Backlog 4 (ARINC 429) — `arinc429.rs`: 32-bit word (label/SDI/data/SSM/odd parity),
  low/high speed timing with the real minimum inter-word gap, bus open (stale) vs. short (parity
  failure) faults, reusing `message::DataStatus` for link delivery status while keeping the
  word's own SSM as the separate, source-claimed thing it really is.
- [done] Backlog 5 (ventilation) — `ventilation.rs`: per-bay thermal model (forced vs. natural
  convection depending on fan(s) + extract valve health, in series), per-module overheat trip
  (first-order ramp, no discontinuity) feeding `faults::ModuleFaults.overheat_trip_frac`.
- [done] Registration — `registry.rs`: every failure/component/ECAM alert above registered via
  `crate::deep::api::Registry` under `Area::AvionicsNetwork`, ATA 42 (AFDX/IMA) and ATA 21
  (avionics bay cooling). `Registry::validate()` exercised in `registry.rs`'s own tests.

## Vars this module will need published once wired into the simulation

Nothing in this module reads or writes a simulator Var yet (hard rule 2: self-contained, no
crate-internal dependencies). `registry.rs`'s ECAM alerts reference these by name for whoever
wires this module in:
- `AFDX_NETWORK_A_AVAILABLE`, `AFDX_NETWORK_B_AVAILABLE` — 1 if any end system can still reach
  any other on that network side (`graph::NetworkGraph::reachable` over every end-system pair, or
  cheaper: any switch on that side has `SwitchFaults::is_available()`), else 0.
- `AVNCS_MODULE_<NAME>_AVAILABLE` per end system in the topology actually wired in
  (`faults::ModuleFaults::is_available`).
- `AVNCS_<BAY>_AIRFLOW_FRAC` per avionics bay (`ventilation::BayState::airflow_frac`).

## Verification

The brief says no cargo/build in the shared crate; instead this module's 9 files plus a copy of
the real `src/deep/api.rs` were compiled standalone in a scratch directory (`rustc --edition 2021
--crate-type lib`, then `--test`) to catch what a read-through alone would miss. That caught and
fixed two real bugs: an unused `PendingFrame.sequence` field (removed), and a wrong assumption in
`graph::tests::a_fully_failed_switch_removes_it_from_reachability` (it failed a switch neither
end system actually attaches to, so reachability correctly survived — fixed to fail the specific
switch `CPIOM-A1` attaches through). Final state: clean build, 0 warnings, all 55 tests pass.

## Next (extending past the backlog)

- A proper AFDX multicast replication-tree model (currently each destination's path is computed
  independently; `graph::NetworkGraph::port_load` already de-duplicates by edge, but a shared
  switch-to-switch trunk carrying one VL to three destinations only truly sends one frame per
  hop, which the current per-destination BFS approximates rather than models exactly).
- Wire `consequences::FunctionMonitor` up to also take `arinc429::Arinc429Channel` sources
  (`consequences::SourceStatus::Arinc429` already exists for this; `FunctionMonitor` itself is
  currently AFDX-only for its owned/stepped receivers).
- A second, larger reference topology exercising more of FlyByWire's public CPIOM/IOM table
  (currently 5 of the ~22+8 real modules, chosen to cover every fault/consequence path at least
  once, not to be exhaustive).

- [done] Live system — `live.rs` (`live_system() -> Box<dyn deep::live::Area>`), `mod.rs` — `LiveAvionicsNetwork`
  owns both AFDX networks (8 switches and all cabling each), every switch port, every end system with its
  partitions, both avionics bays' fans/valve/thermal node, one overheat supervisor per module, and the reference
  function monitors. All 159 registered failure ids are consumed, resolved by failure name (`FaultIndex`).
  Publishes `AFDX_NETWORK_A/B_AVAILABLE`, `AVNCS_MODULE_<name>_AVAILABLE` and `AVNCS_<bay>_AIRFLOW_FRAC` (every var
  this area's ECAM triggers name) plus bay temperature, per-module overheat trip and pass fraction, and per-function
  availability/age. Bus allocation (modules across the two DC buses, each bay's two fans across two AC buses) is
  GENERIC — `Truth` has no avionics load allocation.
- [fix] `ventilation.rs` — `OverheatTrip::step` approached 1.0 asymptotically and so never satisfied
  `ModuleFaults::is_available`'s `< 1.0` test: a module baking in an uncooled bay could never actually drop off the
  network. It now snaps to its target within 1e-6, which is what a latching comparator does.
- [perf] `graph.rs` — `NetworkGraph` builds each side's adjacency once in `new` instead of rebuilding it inside
  every path search; a live network re-runs those searches for every virtual link on every frame.
