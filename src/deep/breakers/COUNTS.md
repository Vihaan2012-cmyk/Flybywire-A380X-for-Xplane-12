# Circuit breakers — counts

**397 breakers total** (session 3 re-sync, see below) — up from 373 after
`deep::electrical::loads.rs` landed its real dual-feed split (`add_dual`):
245 single-feed loads protected 1:1 by id, 7 units now dual-fed (PRIM/SEC/
FCDC/ROLLOUT/FCU, LGCIU 1/2, FMS/ADIRU/TCAS — 18 loads × 2 breakers = 36
breakers via `push_electrical_dual`, ids `<load>-normal-bkr`/`<load>-2nd-
bkr`), 4 new hydraulic-pump contactor-coil loads (session 3), 51 other real
equipment with no load model yet, and 77 control/excitation supplies —
protected_load: None, GENERIC-cited. The ATA table below reflects the
session 3 re-sync exactly (recomputed by hand against the code, given a
hard time-box on this session); the panel/type tables below are still the
373-entry (session 2) breakdown and need a re-run once cargo is available
— see `PROGRESS.md`'s session 3 note.

## By ATA chapter

| ATA | Count | System |
|---|---|---|
| 21 | 63 | Air conditioning (fans, FDAC/TADD/VCM/OCSM channels, CPIOM B apps, pack flow valves, +8 position-indication) |
| 22 | 6 | Autoflight (ROLLOUT, FCU 1/2 — now dual-fed, 2 breakers each) |
| 23 | 8 | Communications (VHF 1/2 + SATCOM, HF 1/2, ACARS MU, PA amplifier, interphone) |
| 24 | 6 | Electrical power (batteries, ext-power contactor control, charge limiters) |
| 25 | 6 | Equipment/furnishings (galley zones) |
| 26 | 18 | Fire protection (12 detection-loop breakers + 6 extinguisher-bottle squibs) |
| 27 | 16 | Flight controls (PRIM/SEC/FCDC — now dual-fed, 2 breakers each) |
| 28 | 145 | Fuel (25 pumps + 60 valves + 60 valve position-indication) |
| 29 | 11 | Hydraulic power (4 electric pumps + 4 pump contactor coils + RAT solenoid + PTU valve + 1 position-indication) |
| 30 | 8 | Ice and rain protection (windshield/pitot/AOA/TAT heat) |
| 31 | 3 | Indicating/recording (DFDR, CVR, QAR) |
| 32 | 23 | Landing gear (LGCIU x2 — now dual-fed, 4 breakers — autobrake solenoid, 12 proximity sensors, 6 actuators) |
| 33 | 15 | Lights (12 lighting circuits + 2 emergency-lighting chargers + exterior service lighting) |
| 34 | 27 | Navigation (RA systems + antennas, EGPWC, FMS x3 + ADIRU x3 + TCAS now dual-fed, transponders + weather radar single-fed) |
| 35 | 4 | Oxygen (crew shutoff valve + its position-indication, pax generator control, pressure transducer) |
| 36 | 8 | Pneumatic (4 engine bleed valve sets + 4 position-indication) |
| 44 | 5 | Cabin systems / IFE (seat-box zones + server rack) |
| 49 | 5 | Airborne auxiliary power (APU ECU A/B, fuel shutoff valve + its position-indication, start contactor) |
| 52 | 4 | Doors (fwd/aft cargo door actuator control + their position-indication) |
| 73 | 8 | Engine fuel and control (FADEC channel A/B x4 engines) |
| 74 | 8 | Ignition (exciter A/B x4 engines) |

Total: 397 (sum of the column above).

## By panel location

| Panel | Count |
|---|---|
| Primary Power Centre 1 (AC1/DC1) | 76 |
| Primary Power Centre 2 (AC2/DC2) | 86 |
| Primary Power Centre 3 (AC3) | 7 |
| Primary Power Centre 4 (AC4) | 9 |
| Secondary Power Centre, forward (ESS/ESS SHED buses) | 131 |
| Secondary Power Centre, aft (cabin/cargo/ground-service/hot buses) | 40 |
| Overhead panel, forward half (engine/fuel/APU/hydraulic/bleed ATAs) | 4 |
| Overhead panel, aft half (electrical/fire/ice/gear/lighting ATAs) | 9 |
| Avionics bay (LRU with no cockpit pushbutton of its own) | 11 |

All 77 new control/excitation entries are 5 W GENERIC circuits, well under
the 25 A SSPC/thermal split, so they land entirely in the Primary/Secondary
Power Centres above (the SSPC side) — the overhead/avionics-bay/thermal
counts are unchanged from the previous 296-entry pass.

Every breaker also carries a `position: PanelPosition { row, column, label
}` (`catalog::assign_positions`) so the Study/Breakers page can draw an
actual grid per panel instead of a flat list — 12 breakers per row
(GENERIC), grouped by panel then ATA then id for a stable, deterministic
layout across rebuilds. Not a photographed real A380 panel-plate diagram
(none is public at per-breaker resolution); documented as authored/GENERIC
in `catalog.rs`.

## By type

| Type | Count |
|---|---|
| SSPC (solid-state power controller, remote reset/status/lockout via CDS/OIT) | 349 |
| Thermal (conventional bimetal, magnetic instantaneous, manual reset only) | 24 |

The SSPC-heavy split reflects the real A380 ELMS architecture (the large
majority of this aircraft's circuits are lower-current avionics/valve/
solenoid/sensor-excitation loads, which is exactly the class of circuit
this generation's solid-state power distribution targets); the 24 thermal
entries are the catalogue's highest-current items — the 4 electric
hydraulic pumps, the 6 gear/gear-door actuators, the 6 galley zones, engine
bleed valve sets, and windshield heat — each landing above the 25 A
GENERIC split point documented in `catalog.rs`. Every SSPC entry now
supports `trip.rs`'s own `remote_reset`/`remote_open`/`status`/
`maintenance_clear_lockout` (a real thermal breaker refuses all of these
with `RemoteControlError::NotRemoteCapable` — it has no CDS/OIT interface
at all, matching the real hardware split); an SSPC latches into lockout
after 3 trips within a rolling 5-minute window (GENERIC thresholds, real
documented SSPC application-literature behaviour) until a separate
ground-maintenance action clears it.

## Growth log

- Session 1: 296 (245 electrical-load matches + 51 other real equipment).
- Session 2 (this pass): +77 control/excitation supplies (valve
  position-indication circuits paired with an existing actuator breaker) →
  373. `deep::electrical::loads.rs` was re-read and had grown by 17 lines
  (a new `rated_frequency_hz` field / `frequency_sensitive_motor_spec`
  helper) but **no load ids/buses/wattages changed** in this pass, so no
  breaker-side re-sync was needed yet for the load-splitting work the
  coordinator flagged as in progress — see `PROGRESS.md` for the next
  re-sync checklist.
- Session 3 (hard-stop pass): `loads.rs` grew again (609 → 686 lines) and
  this time the real dual-feed split landed via a new `add_dual`/
  `add_triple` (`network::LoadFeed`) API: ROLLOUT/FCU 1-2/PRIM 1-3/SEC 1-3/
  FCDC 1-2 (11), LGCIU 1-2 (2), and FMS 1-3/ADIRU 1-3/TCAS (7) — 20 loads
  total — are now dual-fed (one `Load`, two independent feed breakers,
  OR-ed internally); plus 4 brand-new single-fed loads, one contactor-coil
  supply per electric hydraulic pump. Reconciled with a new
  `push_electrical_dual` helper producing `<load>-normal-bkr`/`<load>-2nd-
  bkr` breaker pairs (matching `add_dual`'s own breaker-id scheme exactly)
  and 4 new coil breakers → **397 total**. `add_triple` exists in
  `loads.rs` but has no call site yet (reserved for a future triple-fed
  unit, e.g. the FWS) — nothing to reconcile against it yet.
