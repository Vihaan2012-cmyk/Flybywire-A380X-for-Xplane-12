# What the A380 reference documents say, and what this port does

Twelve PDFs were supplied as source material. This is what they contain that
bears on this port: what they **confirm** (turning a figure this code had to
guess into a sourced one), what they **contradict**, and what they simply do
not have, so nobody spends another afternoon looking.

Every figure below is cited by document and page. Nothing here reproduces a
document: these are technical constants used the way any engineering work
uses a published figure. That distinction matters for two of the documents
(see **Provenance and what may be used**).

## The documents

| Document | Pages | What it is | Variant |
|---|---|---|---|
| `airbus-a380-fcom_compress.pdf` | 7156 | Flight Crew Operating Manual, full DSC/LIM/ECAM/SOP/FMS chapters | **KAL fleet**, 06 JUL 11 |
| `a380-general-familiarization-course_compress.pdf` | 908 | Airbus Training Centre Hamburg, General Familiarization | A380-800, 2004 |
| `Airbus-Aircraft-AC-A380.pdf` | 311 | Aircraft Characteristics — Airport and Maintenance Planning | A380-800 |
| `349573090-A380-Training-Notes.pdf` | 124 | Technical Training Manual, Maintenance Course T1/T2 | **RR / Metric** |
| `performance-analysis-of-airbus-a380_compress.pdf` | 58 | Third-party performance study | A380-800 |
| `373548503-a380.pdf` | 33 | Third-party crib sheet of limits | unstated |
| `TCDS_E.012_Issue_12.pdf` | 20 | **EASA Type Certificate Data Sheet, RB211 Trent 900** | correct engine |
| `A380-QRH.pdf` | 18 | Quick Reference Handbook, Fire and Smoke chapter only | **Emirates A380-861 (GP7200)** |
| `XPHFBW-verification-methodology*.pdf` (x3) | 2 each | This project's own verification write-up | n/a |
| `gq3.pdf` | 1 | No text layer | n/a |

All twelve were checked before use: every one begins `%PDF`, none is an
executable, none carries an embedded file or an OpenAction, and the `/JS`
byte sequences inside the FCOM's compressed streams are not JavaScript
actions (`/JavaScript` appears zero times).

### Provenance and what may be used

The **TCDS** is published by EASA. Its figures are freely usable and are the
best source here for anything engine-related: it is the certification
document for the exact engine this aircraft models.

The **QRH** carries an explicit restriction — supplied in confidence, not to
be reproduced in whole or in part without written permission. It is not
quoted here and must not be shipped in this repository in any form. It is
also the wrong variant: an A380-861 with GP7200 engines, where this port is
an -842 with Trent 900s.

The **FCOM** is a KAL fleet issue, so its *weights* are that operator's
certified figures and not universal: MTOW 1 234 588 lb = **560 t**, MLW
386 t, MZFW 361 t, max taxi 562 t, minimum weight 270 t. The third-party
crib sheet gives 569/391/366/571 t for the same rows. Both are plausible
A380-800 weight variants; neither is "the" answer, and this port should not
hard-code either.

## Confirmed: figures this code guessed, now sourced

These needed no change. Their value is that they stop being assumptions.

**Generator ratings.** `sources::Vfg::RATED_TRUE_POWER_W = 150_000`,
`Rat::MAX_POWER_W = 70_000`, and the APU's 120 kVA are each confirmed:
the course states VFGs 150 kVA, APU GENs 120 kVA, EMER GEN (RAT-driven)
70 kVA, STAT INV 2.5 kVA (GenFam p.162), and the FCOM independently gives
150 kVA per engine generator and 90 kVA per external power unit (FCOM
p.1636-1637). Three separately-derived constants, all correct.

**Variable frequency.** `deep::electrical::loads` derives fan power from the
affinity law across a 360–800 Hz band, noting that a motor on such a bus
must carry its nameplate at the top of the band. The course confirms the
band exactly: 115 VAC, 360–800 Hz, 3-phase, with the static inverter the
single-phase 400 Hz exception (GenFam p.162). The emergency network runs
600–800 Hz.

**115 V as the fuel pump's electrical basis.** `fuel.rs`'s
`FUEL_PUMP_VOLTAGE_V = 115.` is right: the Aircraft Characteristics document
gives the aircraft's AC power as three-phase 115 V 400 Hz (AC p.190), and the
FCOM places every feed pump on an AC bus (below).

**Trent 900 temperature limits.** The TCDS (p.13) gives trimmed TGT limits of
700 °C for ground starts below 50 % HP speed, 850 °C for in-flight relights,
**900 °C for take-off** (5 min), **850 °C maximum continuous**, and 920 °C
over-temperature for 20 s. FlyByWire's EWD already implements exactly this —
red at ≥900, amber above 850 only below the take-off detent — and the crib
sheet's independent table agrees. There is no 850 °C clamp anywhere in
`fadec.rs`'s EGT polynomial, so an engine sitting at 850 at take-off power is
the model's output, not a limiter.

**TGT is not EGT.** `deep/ecam/fbw/ata70.rs` declines to wire FlyByWire's EGT
OVER LIMIT procedure to this port's `DEEP_ENG_n_TGT_SENSED_C` on the grounds
that they are different gas-path stations. TCDS Note 6 settles it: TGT is
measured by thermocouples at the LP turbine's 1st-stage nozzle guide vane.
The refusal was right.

**Main and standby feed pumps.** The course describes each collector cell as
holding a main pump and a standby pump, the standby being redundancy for the
main (GenFam p.308) — the architecture this port's two-pumps-per-feed-tank
model already assumes.

## Contradicted or improvable

### 1. Feed pumps are on AC buses, not an always-powered bus

`breakers.rs`'s `absorbed_systems_cfg` puts every fuel pump on
`Bus::Named("INFINIBAT", 28.)`, because MSFS's `systems.cfg` says
`Connections:bus.1`. `fuel.rs` documents that bus as *always powered*. The
consequence is that **no electrical failure can stop a fuel pump today**.

The FCOM's fuel chapter gives the real supply (DSC-28-70, p.2053):

| Feed tank | Main pump | Standby pump |
|---|---|---|
| 1 | AC 4 | AC 2 |
| 2 | AC ESS | AC 3 |
| 3 | AC 3 | AC ESS |
| 4 | AC 2 | AC 4 |

with crossfeed valves and engine LP valves on DC 2 or DC ESS, the APU pump
and its isolation valve on DC ESS, and the trim line isolation valves on
DC 1 (forward gallery) and DC ESS (aft gallery). A footnote adds a real
behaviour worth modelling on its own: in electrical emergency configuration
the pumps are unpowered when the slats are extended, leaving gravity feed.

These map cleanly onto `MSFS_BUSES` (`AC_1`=2 … `AC_ESS`=6). One caveat:
FlyByWire's config names the pumps `Feed<n>TankPump1/2` with no main/standby
designation and identical 30 psi, so reading Pump1 as Main is an assumption
— but a low-consequence one, because swapping them preserves each tank's
*pair* ({AC 4, AC 2}, {AC ESS, AC 3}, …). "Losing AC 2 costs one pump in
feed 1 and one in feed 4" holds either way; only which index dies changes.

### 2. The APU starter duty cycle is guessed, and the real rule exists

`deep/apu/params.rs` states plainly: *"No PW980A-specific duty-cycle figure
is public"*, and uses a generic `STARTER_DUTY_LIMIT_S = 60.0` with a
`STARTER_COOLDOWN_TIME_CONSTANT_S = 180.0`.

The FCOM gives the operating rule (LIM-49-10, p.6343): after an aborted APU
start the crew waits **60 s** before another attempt (ECB inhibition logic),
and after **three consecutive attempts** must wait **60 minutes** to let the
starter cool. A 180-second cooldown constant is roughly twenty times faster
than the real constraint implies.

### 3. APU limits: four different quantities, and the crew's are absent

`params.rs` is careful to separate control limit, warning and protective
trip, and takes all three from FlyByWire's PW980A model: start fuel-schedule
anchor 900 °C, start warning 900 °C below FL250 and 982 °C above, running
warning 900 °C, trip 950 °C.

The FCOM adds a fourth category the port does not carry — the **crew
limitation** (LIM-49-10, p.6343): maximum indicated EGT 900 °C; maximum EGT
for APU start **700 °C on ground, 800 °C in flight**; maximum EGT for APU
running **900 °C for 30 s, or 935 °C for 0.5 s**; maximum N1 106 %, maximum
N2 102 %.

The running limits are *time-qualified*, which nothing in the port expresses.
These are not corrections to FlyByWire's ECB thresholds — they are a
different kind of number — but the start limits being 700/800 against the
model's 900/982 warning is a gap worth a decision rather than silence.

### 4. Feed pump boost pressure

The course says the operating pump boosts the collector cell above ambient
by *typically 25 psig* (GenFam p.308). FlyByWire's `flight_model.cfg` uses
`Pressure:30` for all eight feed pumps. A 20 % difference, on a figure that
feeds this port's whole `fuel_network` pressure solve.

### 5. Tank capacities run about 3 % high

The course tabulates tank masses at a stated density of 785 kg/m³ (GenFam
p.298). Converting FlyByWire's capacities (US gallons) at the same density:

| Tank | FBW gal | FBW kg @785 | Course kg | Diff |
|---|---|---|---|---|
| Outer (each) | 2731.5 | 8117 | 7477 | **+8.6 %** |
| Feed 1 / 4 (each) | 7299.6 | 21691 | 21175 | +2.4 % |
| Mid (each) | 9632.0 | 28622 | 28146 | +1.7 % |
| Inner (each) | 12189.4 | 36221 | 35571 | +1.8 % |
| Feed 2 / 3 (each) | 7753.2 | 23039 | 22082 | +4.3 % |

Totals: 323 546 L modelled against the course's 315 354 L usable without a
centre tank, **+2.6 %**. The outer tanks are the outlier. This is FlyByWire's
data, not this port's, and the course's figures may be usable rather than
total volume — but the gap is real and now measured.

## Not in these documents — stop looking

**Per-consumer electrical ratings, including the fuel pump's full-load
current.** This was the motivation for reading the FCOM and it is not there.
A flight crew manual carries bus assignments, not amperages; per-breaker and
per-motor ratings are AMM/ASM data. Searched: the FCOM's 131 pump pages, its
electrical chapter, the Aircraft Characteristics document and the training
notes. The ampere-looking matches across the FCOM are all false positives
("A380 FLEET", section codes such as `20A`).

Consequence: `breakers.rs` cannot rate an absorbed `sys-<n>` fuel pump
breaker against a real current, and the multiplier path it now uses remains
the honest choice. See the same-basis rule in `docs/physics/breakers.md`.

**Per-breaker ampere ratings** generally. The FCOM's C/B material describes
the C/B SD *page*, not a catalogue. The ~30 catalogue entries reading
"generic avionics LRU (50 W, typical/derived)" stay generic.

**Certified take-off and landing performance.** Not present, and the
licensing position would bar embedding it even if it were. The OIT's
`NOT AVAIL` fields keep their stated reason.

## New behaviour these documents enable

**The C/B SD page** (FCOM DSC-24-20, p.1683). Tripped breakers are listed
with the **last tripped at the top**; each shows its location and Functional
Identification Number, explicitly for maintenance use only; a **maximum of 18
tripped C/Bs appear per page** with **at most two pages**, the remainder
reached with the C/B key on the ECAM control panel; a distinct message
appears when the emergency C/B monitoring function is lost, meaning tripped
breakers on the electrical emergency network can no longer be shown; and the
NORMAL indication is replaced by NOT AVAIL when C/B monitoring is lost. This
port's breaker page can be checked against all five rules.

**A full limits set** for anything that wants one — gear extension ceiling
21 000 ft, slats/flaps 20 000 ft, VLE/VLO 250 kt/M 0.55 (gravity extension
220 kt/M 0.48), maximum ground speed 204 kt, maximum brake temperature for
take-off 300 °C, maximum positive differential pressure 9 psi and negative
−0.725 psi (the course's 8.78 psi is the *nominal operational* figure, and
cabin altitude 7 500 ft at FL430), crosswind 30 kt (15 kt CAT II/III),
headwind 38 kt, tailwind 10 kt, 40 kt for door operation, minimum runway
width 148 ft, minimum TAT −54 °C, VMO/MMO 340 kt/M 0.89, VMCL 120 kt and
VMCL-2 145 kt, load factors −1 g to +2.5 g clean and 0 g to +2 g otherwise.

**Refuelling rates**, if ground servicing is ever modelled: 255 000 L through
four nozzles in 48 min at 40 psi, 1 330 L/min/nozzle, 2 000 L/min/nozzle with
the rear step when passengers are aboard.
