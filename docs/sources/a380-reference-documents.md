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

The **FCOM** is a KAL fleet issue, so its weights are that operator's
certified figures. They turn out to be the baseline ones — see the weight
variants below.

## Weight variants: the port mixes two of them

The Aircraft Characteristics document carries the whole weight-variant table
(AC p.25), which settles a question this code has been guessing at:

| | **WV000** | WV001 | WV002 | WV003 | WV004 |
|---|---|---|---|---|---|
| MRW / MTW | 562 t | 512 t | 571 t | 512 t | 562 t |
| MTOW | **560 t** | 510 t | 569 t | 510 t | 560 t |
| MLW | **386 t** | 394 t | 391 t | 395 t | 391 t |
| MZFW | **361 t** | 372 t | 366 t | 373 t | 366 t |

WV000 — 560/386/361 — is corroborated twice over: Airbus's own Technical
Training Manual states MTOW 560 t, MLW 386 t, MZFW 361 t, and the KAL FCOM's
limitations chapter gives the same four figures including MTW 562 t. The
third-party crib sheet's 569/391/366/571 is WV002.

**This port uses two variants at once.** `deep::gear_structure::MTOW_KG` is
510 000 kg, which is WV001/WV003; `MLW_KG` (and `physics::damage`'s own copy)
is 386 000 kg, which is WV000. Both cite *"WV000"* — and the citation in
`damage.rs` states the variant's MTOW as 510 000 kg, which the table above
shows is not WV000's figure at all.

FlyByWire's own `flight_model.cfg` is a third combination: `max_gross_weight`
1 124 355 lb = 510 t with MLW 395 t and MZFW 373 t (all WV003), under a
comment giving MRW 562 t (WV000/WV004).

So there are three coherent choices and the code currently makes none of them:

* **WV000**, the Airbus baseline all three documents agree on — then
  `MTOW_KG` should be 560 000 and `MLW_KG` stays;
* **WV003**, the aircraft FlyByWire actually models and the one the converted
  `.acf` inherits its mass from — then `MTOW_KG` stays and `MLW_KG` becomes
  395 000;
* keep the present numbers deliberately, and say which variant each belongs
  to instead of citing one that matches neither.

It is not cosmetic. `MLW_KG` arms the overweight-landing failure (32_123) and
sets `overweight_landing_check`'s inspection tier, so against the aircraft
FlyByWire models, a legal 390 t landing is currently treated as overweight;
`MTOW_KG` sets the MTOW drop case in `strut.rs`'s certification limit load,
where the WV000 figure would size the struts about ten per cent stronger.

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

**Landing gear and brakes, in full.** The course (GenFam p.442) describes
two wing landing gears on four-wheel bogies, two body landing gears on
six-wheel bogies retracting rearwards, a two-wheel nose gear retracting
forward with twin steering actuators, all single-stage oleo-pneumatic; and,
of the main group's **20 wheels, 16 anti-skid brakes**, because each body
bogie's rear axle is steerable and unbraked. `deep::gear_structure` already
models precisely this — `BRAKED_WHEEL_COUNT = 16.0`, "16 braked of the 22
total", the body-bogie rear axle steered rather than braked, three steerable
positions. Tyres are one specification for both wing and body gear (1400 mm)
and a different one for the nose (1270 mm).

**Hydraulic system pressure.** 5000 psi nominal (GenFam p.348), which this
code uses throughout (`HYDRAULIC_NOMINAL_PA`, the accumulator model). The
course adds the context: the 5000 psi choice saves over 1000 kg, and the
A380's installed hydraulic power is 800 kW against 300 kW for the
A340-500/600, which is why two fuel/hydraulic heat exchangers per circuit
are fitted.

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

### Two documents with less in them than their size suggests

The **Technical Training Manual** (124 pages) is Level I **ATA 00 only** —
aircraft general introduction, stations and zoning, cockpit philosophy,
documentation, tools, safety precautions and handling. No system chapters at
all. Its one contribution is the weight table above. It also carries an
explicit no-reproduction notice.

The **performance analysis** (58 pages) is an academic study: wing span, wing
area, aspect ratio, sweep angle and published weights, then analysis built on
them. Useful only as a cross-check on figures that are better sourced
elsewhere.

### 6. The ground spoiler and speed brake schedule is fully specified

`deep::flight_controls::live` gives every spoiler one limit,
`SPOILER_MAX_DEG = 50.0`, from this project's own integration table. The FCOM
(DSC-27-10-10, p.1846-1848) specifies them per panel and per function:

*Speed brake, in flight* — spoilers 1 to 5 reach **20 °**, spoilers 6 to 8
reach **45 °**; the lever's 1/2 position gives 3/4 extension; deflection is
reduced in CONF 1+F, 2, 3 and FULL.

*Ground spoilers, partial extension* (after retard, at least one gear on
ground) — spoilers 1 and 2 to **10 °**, spoilers 3 to 8 to **15 °**, ailerons
not deflected, at **5 °/s**.

*Ground spoilers, full extension* (three main gears on ground) — spoilers 1
and 2 to **35 °**, spoilers 3 to 8 to **50 °**, ailerons **25 ° up**, at
**17 °/s**. The spoilers' roll function is inhibited below 110 kt while they
are serving as ground spoilers, leaving roll to the ailerons.

So the port's flat 50 ° is right only for spoilers 3-8 at full ground
extension: panels 1 and 2 stop at 35 ° there, and in flight nothing goes
beyond 45 °. None of the rates, nor the aileron-up-with-ground-spoilers
coupling, is modelled.

Two thresholds in `spoiler.rs` are worth singling out, because the file is
candid about both being guesses:

* `WHEEL_SPINUP_KT = 72.0`, commented *"a commonly cited order-of-magnitude
  transport wheel-spin-up threshold; not an A380-certified number"* — it is
  in fact exactly the A380's own figure. The FCOM's rejected-takeoff logic
  extends the ground spoilers fully "when the thrust levers are at idle and
  the speed is greater than **72 kt**" (p.1848). The guess was right and can
  now be cited.
* `SPINUP_RADIO_ALT_FT = 5.0`, its paired low-height gate, against the FCOM's
  auto-arm gate of **6 ft** radio height.

### 7. Timings and hydraulic figures the port does not carry

**Landing gear gravity extension takes approximately 70 s** (FCOM p.4846,
repeated across the abnormal procedures). `gear_structure::live` has a
`gravity_extend_commanded` input and no duration behind it.

**Rudder trim reset** drives the trim to zero at **3 °/s**, and the reset
pushbutton is inactive with the autopilot engaged (p.1858). The port's
`RudderTrimActuator::new_generic` is explicitly generic at ±20 ° authority;
the FCOM was searched for a trim authority figure and does not state one, so
that part stays generic.

**Hydraulic reservoirs** (DSC-29-10, p.2061): total volume **31.70 US gal**
each, located in the outer engine pylon — close to the pumps, which is what
prevents cavitation — pressurised by outer-engine HP air with aircraft bleed
as the fallback. This is not directly comparable with the 12 and 12.7 US gal
this port takes from FlyByWire, which are usable-fluid maxima rather than
total volume, but it is the Airbus figure and the two should be reconciled
deliberately rather than by accident. Also stated: reservoir fans run above
55 °C and stop below 35 °C, an overheat closes valves above **85 °C**, and
low reservoir air pressure annunciates below **21.8 psi**.

**Hydraulic architecture confirmed, no change needed**: each system is
pressurised by four engine-driven pumps, two per engine, with engines 1 and 2
on GREEN and 3 and 4 on YELLOW, two pumps being sufficient for the users, and
one fire shutoff valve per engine between the reservoir and its pumps.
`deep::hydraulics::live` already models exactly this, down to the
`green_edp_1a`/`1b`/`2a`/`2b` naming.

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
