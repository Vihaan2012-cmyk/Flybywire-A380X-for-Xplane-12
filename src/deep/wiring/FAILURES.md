# Wiring -- failures

One line per genuinely distinct physical fault mechanism (`src/deep/wiring/faults.rs`).
Each applies to any circuit/segment/zone in the routing catalogue (`routing.rs`) --
registered per zone in `registry.rs` (13 zones x 7 kinds = 91 `FailureDef`s, `ATA 91`,
`Area::Wiring`), not repeated below as 91 near-identical rows.

| ATA | proposed name | model element it acts on | magnitude meaning (0..1) | effect |
|---|---|---|---|---|
| 91 | Wire chafe | `bundle::Segment` (one circuit's conductor within it) | insulation breach depth: 0 intact insulation .. 1 conductor fully bared/bolted contact | `faults::chafe_effect`: intermittent high-resistance arcing contact (resistance falls toward the arc's own `V_arc/I` floor as the breach deepens) until, at full severity, a bolted short to structure or to a specific bundled neighbour |
| 91 | Bundle overheat / fire | every `bundle::Segment` in a `zones::Zone` | localized fire/overheat severity: 0 ambient .. 1 representative 400 C severe electrical fire | `faults::zone_overheat_effects`: every circuit whose own insulation temperature rating (`gauge::Insulation`) the fire's temperature exceeds is damaged (high-resistance leakage, then inter-conductor crosstalk short, then full open) -- higher-rated insulation in the very same bundle survives a less severe event that opens its neighbour |
| 91 | Connector corrosion | one circuit's connector within a `bundle::Segment` | oxide-film contact-resistance growth: 0 clean .. 1 worst modelled corrosion | `faults::connector_corrosion_effect`: added series contact resistance, up to 100 ohm at full severity |
| 91 | Water ingress | one circuit's connector/splice within a `bundle::Segment` | contamination/leakage-path severity: 0 dry .. 1 worst modelled leakage | `faults::water_ingress_effect`: a leakage resistance to structure through contaminated moisture, falling toward 2 kOhm as ingress worsens |
| 91 | Rodent damage | one circuit's conductor within a `bundle::Segment` | insulation/conductor bite-through progress: 0 none .. 1 fully bitten/bared | `faults::rodent_damage_effect`: a developing chafe-style contact that, at full severity, opens a thin (>=18 AWG) signal wire or bares/shorts-to-structure a heavy feeder, gauge-dependent |
| 91 | Maintenance damage | one circuit's conductor within a `bundle::Segment` | crush/pinch/cut severity: 0 none .. 1 crushed/severed | `faults::maintenance_damage_effect`: a developing chafe-style short to structure or to a bundled neighbour (a crush event shorts far more often than it cleanly opens) |
| 91 | Open wire (fatigue) | one circuit's conductor within a `bundle::Segment` | fatigue-crack cross-section fraction: 0 intact .. 1 fully separated | `faults::open_wire_effect`: series resistance rising as `1/(1-magnitude)` (remaining conductor area shrinking, textbook `R = rho*L/A`), snapping to a full open once the crack completes |

## Modifier, not a standalone fault

`arc::thermal_breaker_sees_ratio` is not itself a new failure -- it is the *consequence*
any of the above faults' arcing/high-resistance phase has on the protecting breaker's own
thermal (I^2t) trip element: an intermittent arc (low `duty`) can keep the breaker's own
RMS-equivalent heating current under its rated value indefinitely, so a real short-in-
progress may never trip a conventional thermal/magnetic breaker (the documented real-world
motivation for arc-fault-specific protection, cited in `arc.rs`). `duty` is supplied by
whatever fault-scenario harness drives a chafe/rodent/maintenance fault's own intermittency,
not modelled as its own magnitude here.
