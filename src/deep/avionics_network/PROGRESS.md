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
- [done] sourced-constants pass — ventilation.rs, live.rs — `TRIP_SPAN_K` is no longer free: the overheat ramp now begins at RTCA DO-160 Temperature and Altitude Category A1's +55 C operating high temperature (pressurised, temperature-controlled location), i.e. exactly where the equipment stops being qualified. The 70 C fully-tripped end stays GENERIC (DO-160G Table 4-1's short-time figures are not publicly reproduced). The module-to-bus and fan-to-bus allocations stay GENERIC with the search recorded: the segregation property is real (CS 25.1309/25.1360), the allocation is not public and FBW model no electrical supply for CPIOMs or bay fans.
- [done] Redundancy made observable (deep failure audit: 93 of this area's 159 failures moved nothing published) —
  `ventilation.rs` (`Fan::output_frac` made `pub`), `live.rs` (`Snapshot`, `tick`, `publish`, tests). Root cause: the
  area only ever published a whole network side's boolean availability and two reference functions' rolled-up
  status, so a single switch/port/cable/partition/babbling/fan fault that never happened to flip one of those two
  coarse numbers was invisible, however correctly the model itself was applying it. New Vars, one per component the
  matching `registry.rs` failure already names (all direct reads of that failure's own model field, no new state):
  - `AVNCS_SWITCH_<name>_AVAILABLE` / `_HEALTH_FRAC` — per switch, both sides (`SwitchFaults.failure`).
  - `AVNCS_SWITCH_<name>_PORT_<neighbour>_HEALTH_FRAC` — per switch port (`SwitchFaults.port_failure[neighbour]`).
  - `AVNCS_CABLE_<a>_<b>_<A|B>_HEALTH_FRAC` — per physical segment (`LinkFaults.open`).
  - `AVNCS_MODULE_<name>_PARTITION_<part>_AVAILABLE` — per ARINC 653 partition (`PartitionFaults.failure`), independent
    of the module's own `_AVAILABLE` (partition-level fault containment).
  - `AVNCS_MODULE_<name>_PORT_LOAD_FRAC_<A|B>` — `graph::PortLoad::offered_bps/capacity_bps` at the module's own
    attach port, moved directly by `EndSystemFaults.babbling` even before it costs another VL a frame.
  - `AVNCS_<bay>_FAN_PRIMARY_HEALTH_FRAC` / `_STANDBY_HEALTH_FRAC` — each fan's own `output_frac()`, since
    `Bay::step` takes the *best* of the two and so never moves the bay's own `_AIRFLOW_FRAC` on a single fan failure.
  - `AVNCS_MODULE_<name>_NETWORK_A_REACHABLE` / `_NETWORK_B_REACHABLE` / `_NETWORKS_UP` — whether *this* end system
    (not the aircraft's network as a whole) can still reach another live one on each side, and the count 0..2. This
    is the actual "one fault from losing the function" signal the audit's problem statement asks for:
    `AFDX_NETWORK_<A|B>_AVAILABLE` only goes false once *every* pair is isolated on that side, so a healthy,
    fully-redundant network and one running every function on a single network both read the same `1`. `NETWORKS_UP
    == 1` is the state redundancy monitoring exists to catch.
  - `AVNCS_VL_<name>_PATHS_UP` / `_PATHS_DESIGNED` — per virtual link, how many of the two networks currently carry
    a real path from its source to every one of its destinations, against the two (`NetworkSide::BOTH.len()`) it is
    always designed for.
  Verified against the failure classes the audit named dead: switch (8), port (48), cable (24), partition (7) and
  babbling (3) failures now each move their own new variable directly, independent of whether they lie on either
  reference function's path (new tests: `a_single_port_failure_moves_that_ports_own_health_reading`,
  `a_cable_and_a_switch_failure_each_move_their_own_component_reading`,
  `a_partition_failure_moves_only_its_own_partition_variable`,
  `a_babbling_end_system_moves_its_own_egress_port_load_fraction`). Bay-fan failures (4): confirmed still not
  observable via `_AIRFLOW_FRAC` (by design — the bay genuinely does not care which fan is running) and now
  observable via the new per-fan health Var (`a_single_fan_failure_moves_its_own_health_reading_even_though_the_
  bays_airflow_does_not`). The redundancy pair the brief asked for explicitly —
  `cutting_both_sides_of_a_modules_attachment_shows_redundancy_loss_then_function_loss` — cuts one side of
  CPIOM-C1's own attachment cable (module and function both stay up, `NETWORKS_UP` drops 2 -> 1) and then the other
  side too (function now genuinely lost). `registry.rs`'s alert triggers were checked against every Var now
  published (`grep -n 'var(' registry.rs`): all four already read Vars this area publishes
  (`AFDX_NETWORK_A/B_AVAILABLE`, `AVNCS_MODULE_<name>_AVAILABLE`, `AVNCS_<bay>_AIRFLOW_FRAC`) — no orphaned trigger
  in this area. No new ECAM alert added for degraded-but-working redundancy: the real A380's ECAM does not
  annunciate single-switch/port/cable-level AFDX degradation (that is CMS/BITE-level maintenance information, not a
  crew message), and the existing NETWORK AFDX 1/2 FAULT alerts already cover the crew-relevant case (a whole
  network side actually gone). The new Vars are for the EFB Study page and for `consequences::FunctionMonitor` (or
  a future one) to reason about, not for a new crew-facing alert -- see the report handed back for this task for
  the full reasoning.
