# Failures — deep/hydraulics

One line per genuinely distinct physical fault type. Each is registered per
instance (per pump/circuit) in `registry.rs` — see that file for exact
component ids and failure ids (`Area::Hydraulics`, ATA 29).

| ATA | proposed name | model element it acts on | magnitude meaning (0..1) | effect |
|---|---|---|---|---|
| 29 | EDP displacement loss | `pump::EngineDrivenPump` via `PumpFaults.displacement_loss` | swash/valve-plate damage: 0 healthy .. 1 zero displacement at any pressure | pump delivers proportionally less flow at every pressure on its own compensator curve |
| 29 | EDP/electric pump seizure | `pump::EngineDrivenPump`/`ElectricPump` via `PumpFaults.seizure` | 0 free .. 1 fully seized shaft | zero flow and zero case drain; motor (electric pump) still spins against a jammed pump end |
| 29 | Check valve stuck open | `network::CheckValve` via `CheckValveFaults.stuck_open` | 0 healthy .. 1 fully jammed off its seat | loses its reverse-block function: a stopped pump's own manifold-side pressure can bleed backward through it |
| 29 | Check valve stuck shut | `network::CheckValve` via `CheckValveFaults.stuck_shut` | 0 healthy .. 1 fully jammed on its seat | throttles/blocks that pump's own delivery even while otherwise healthy |
| 29 | Fire shutoff valve stuck | `network::FireShutoffValve` via `EdpFaults.fire_sov_stuck` | 0 healthy .. 1 seized at last commanded position | firewall isolation for that pump no longer follows the FIRE handle (stuck open: no isolation; stuck shut: pump cannot be restored) |
| 29 | Priority valve stuck | `network::PriorityValve` via `CircuitFaults.priority_valve_stuck` | 0 healthy .. 1 seized at last position | stuck shut starves the whole non-essential branch (gear/brakes/steering/cargo doors/reversers); stuck open removes flight-controls priority under high total demand |
| 29 | Relief valve cracking low | `network::ReliefValve` via `CircuitFaults.relief_valve_crack_low` | 0 healthy 5400 psi crack .. 1 cracks at half that | circuit cannot reach normal regulated pressure; continuous dumping also raises fluid temperature |
| 29 | Accumulator precharge loss | `accumulator::Accumulator` via `AccumulatorFaults.precharge_loss` | 0 full rated N2 precharge .. 1 fully bled to ambient | accumulator still fills with fluid but returns almost no stored pressure on a transient demand or all-pump-loss scenario |
| 29 | Reservoir leak | `reservoir::Reservoir` via `ReservoirFaults.leak_area_m2` | leak orifice area, 0..20 mm^2 | reservoir fluid quantity falls over time; low level progressively unports every pump inlet on that circuit |
| 29 | Reservoir pressurisation loss | `reservoir::Reservoir` via `ReservoirFaults.pressurization_loss` | 0 healthy .. 1 no bootstrap air pressure at all | pump inlet gauge pressure collapses toward zero, cavitating every pump on the circuit even with a full reservoir |
| 29 | Return filter contamination | `network::Filter` via `CircuitFaults.filter_clog` | 0 clean .. 1 fully blocked | return-side pressure drop rises until the bypass valve cracks, admitting unfiltered flow (protects flow, loses filtration) |
| 29 | Branch supply line leak | `network::Line.leak_area_m2` via `CircuitFaults.line_leak_area_m2` (per branch: gear/brakes/steering/cargo doors/reversers) | leak orifice area, 0..20 mm^2 | fluid lost straight to the bay from that branch; drags the whole circuit's reservoir down over time |
| 29 | Air ingestion | `network::Node.air_fraction_at_1atm` (network-wide) via `CircuitFaults.air_ingestion` | 0 healthy .. 1 at 5% free air by volume at 1 atm (GENERIC ceiling) | entrained air softens the fluid's effective bulk modulus, making the whole circuit spongy/slow to pressurise, worst at low pressure |
