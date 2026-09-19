# Electrical area — failure catalogue

One line per genuinely distinct physical fault mechanism this area's models
support (`docs/deep/BRIEF.md`'s own format). Each mechanism is implemented
once, in `network.rs`/`sources.rs`, and applies to every real instance it is
attached to across the catalogue (see `registry.rs`, which expands each of
these into one `FailureDef` per real component instance — ~600 total, listed
there in code rather than duplicated here as ~600 near-identical lines).

ATA | name | model element it acts on | magnitude meaning (0..1) | effect
--- | --- | --- | --- | ---
(various, every catalogued consumer's own ATA — see `loads.rs`) | Load open circuit | `network::Load.faults.open_circuit` | fraction of the load's own internal path opened | draws proportionally less current, delivers proportionally less function; no current/no function at 1.0
(various) | Load short to ground | `network::Load.faults.short_to_ground` | short severity; fault current = severity × bus voltage / wiring resistance | unregulated extra current on top of the load's own demand; can overload its own breaker or, if that breaker fails to trip, its bus feeder
(various) | Load high-resistance / overheat | `network::Load.faults.high_resistance` | fraction of a 50% extra-current ceiling | draws more current for the same useful output, dissipated as heat inside the load
(various) | Load intermittent connection | `network::Load.faults.intermittent` | dropout rate, up to 2 Hz at 1.0 | drops the load off the bus at a rate proportional to severity, independent of breaker/bus health
24 | Breaker fails to trip | `network::Breaker.faults.fails_to_trip` | probability the trip attempt fails, re-evaluated on every attempt | stays closed under an overload/short that should have opened it, letting the fault continue to heat the bus/wiring downstream
24 | Breaker nuisance trip | `network::Breaker.faults.nuisance_trip` | continuous extra I²t heat added regardless of current; 1.0 trips in ~5 s with no real load | opens and de-energises its load/bus segment despite no genuine fault
24 | Contactor fails to close | `network::Contactor.faults.fails_to_close` | probability the close attempt fails, re-evaluated every tick commanded closed | the bus/path it should have connected stays de-energised
24 | Contactor welded closed | `network::Contactor.faults.welded_closed` | probability of the welded-shut state each tick | stays connected even when commanded open (e.g. a bus tie that should have isolated a fault)
24 | Diode fails open | `network::Diode.faults.open_circuit` | probability of the open-junction state each tick | the one-way path it provided is gone; whatever it fed can no longer be reached through it
24 | Bus short to structure | `network::Bus.faults.short_to_ground` (via `Network::set_bus_fault`) | short severity; conductance = severity / 0.03 Ω | collapses the bus's own voltage; overloads whatever feeder/tie breaker protects it
24 | VFG winding degradation | `sources::Vfg.faults.winding_degradation` (same field on `sources::ApuGenerator`) | series reactance grows toward 4× its healthy value | terminal voltage sags harder under load; a fully degraded machine may never reach rated voltage
24 | VFG/GCU regulator drift | `sources::Vfg.faults.regulator_drift` (same field on `sources::ApuGenerator`) | signed −1..1 magnitude of a regulation drift, scaled to ±8 V | no-load terminal voltage drifts away from 115 V, feeding an over/under-voltage condition to the whole bus
24 | TRU winding/diode-bridge degradation | `sources::Tru.faults.winding_degradation` | internal resistance grows from 0.0135 to 0.054 Ω | DC output sags harder under load; TRU runs hotter for the same delivered power
24 | Battery capacity fade | `sources::Battery.faults.capacity_fade` | fraction of a 70% capacity-loss ceiling | less usable charge before the battery reads empty; shorter time-to-empty under the same load
24 | Battery internal-resistance growth | `sources::Battery.faults.resistance_growth` | internal resistance grows toward 3× its healthy value | terminal voltage sags harder under load; lower max deliverable power before the source current-limits
24 | Static inverter efficiency loss | `sources::StaticInverter.faults.efficiency_loss` | efficiency interpolates from 0.85 down to a 0.30 floor | less real power deliverable to AC EMER for the same battery input; faster battery drain in an emergency configuration
24 | RAT jammed / fails to fully deploy | `sources::Rat.faults.jammed` | fraction of aerodynamic power lost | less (at 1.0, no) emergency electrical power from the RAT
24 | Ground power weak/miswired cart | `sources::GroundPower.faults.weak_cart` | series reactance grows toward 5× | ground-service bus voltage sags harder under load while on ground power
24 | Galley/commercial shed relay fails to shed | `shedding::ShedRelayFaults.fails_to_shed` | probability the shed command is not honoured this tick | the relay's own loads stay powered when they should have been shed (defeats the aircraft's own power-management budget)
24 | Galley/commercial shed relay sheds spuriously | `shedding::ShedRelayFaults.sheds_when_not_commanded` | probability of a spurious shed each tick with no command | the relay's own loads lose power with no real cause (nuisance galley/IFE power loss)

See `registry.rs` for the per-instance expansion (one `FailureDef`/`ComponentDef`
pair per real load, breaker, contactor, diode, bus and source, numbered
sequentially within its own ATA chapter) and its own tests, which assert the
full registered set validates cleanly and has no duplicate ids.
