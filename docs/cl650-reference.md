# Hot Start CL650 (Challenger 650) — Depth Reference

Purpose: a factual yardstick for the FBW A380X-for-X-Plane port. Every claim below is
sourced; where a claim could not be confirmed from a primary or reputable secondary
source it is explicitly flagged as **unconfirmed** rather than assumed. "Primary"
here means Hot Start/X-Aviation's own product pages, changelogs, or developer posts
on the official support forums (X-Pilot, X-Plane.org). No `docs.hotstartsim.com` or
equivalent hosted wiki could be located during this research — Hot Start does not
appear to publish a public systems manual outside the product's bundled PDF
documentation and forum changelogs, so several areas (hydraulics/pneumatics detail,
APU, fuel temperature, battery chemistry, brake/tyre wear) are thin in public sources
and are called out as gaps in the sourcing itself, not just gaps in the A380 port.

Product identity: Hot Start's "Take Command!: Challenger 650" (CL650) is sold through
X-Aviation for X-Plane 11/12 at $114.95, positioned as a business-jet study sim with a
simulated FBO environment
([X-Aviation product page](https://www.x-aviation.com/catalog/product_info.php/take-command-hot-start-challenger-650-p-212);
[Threshold coverage](https://www.thresholdx.net/news/hst650)).

---

## 1. System simulation depth

### Overall architecture claims (primary source)

Hot Start's own product page makes specific, checkable architecture claims rather
than generic "study level" marketing:

- "No shortcuts taken on the systems and avionics architecture."
- Over 300 simulated ARINC 429 data buses connecting systems, with realistic
  signaling delays, precision errors, and "potential for faults."
- Dedicated simulated data routers for multi-hop data flows between LRUs.
- Redundant Air Data Computers (ADCs) with transponder integration and automatic
  switching.
- Four simulated input-output concentrators (IOCs) aggregating sensor data.
- Multi-source fault tolerance modeled for pressure data specifically.
- "Well over 50 independent computers" each handling a distinct aircraft system.
- Physics-first modeling: engine thrust is derived from fuel-to-heat energy
  conversion (not a lookup table); the electrical system balances generator supply
  against demand rather than inventing power; generator electrical load imposes
  mechanical drag on the engine core, which in turn affects fuel consumption.

([X-Aviation feature page](https://www.x-aviation.com/catalog/iframes/index.html))

In their pre-release announcement, Hot Start developers described the target more
bluntly: "each subsystem will function like its real-world counterpart... there
won't be anything surface level about this aircraft," aiming for "a study level
recreation of the Challenger 650 in every aspect (unless limited by the sim)"
([Threshold preview](https://www.thresholdx.net/news/hscl65)).

### Electrical

Confirmed, sourced specifics are limited to bug-fix changelog entries, which
nonetheless confirm real functional depth (a bus/breaker model that can actually
fail and needs correct behavior, not a cosmetic switch):

- AC UTIL BUS 2 was previously mislabeled as UTIL BUS 1 (implies distinct, modeled
  AC utility buses).
- "Bus short circuit failures" are a modeled failure mode that was buggy and then
  fixed.
- The IAPS (Integrated Avionics Processor System) breaker could enter an
  unrecoverable pop/reset loop under FCC (Flight Control Computer) overload —
  i.e., breaker state and downstream system load are causally linked, not just a
  cosmetic pull.
- The FREQ CONV (frequency converter) breaker could get stuck permanently popped.

(All from the [v1.1.0 changelog coverage, Threshold](https://www.thresholdx.net/news/cll650)
and the [X-Pilot v1.1.0 update thread](https://forums.x-pilot.com/forums/topic/23133-take-command-hot-start-challenger-650-v110-update-released/).)

As of v1.8, the forward avionics/equipment bay is physically modeled and openable
from outside the aircraft, "including the main battery disconnect, TRUs
[transformer-rectifier units] and all CBs [circuit breakers]," letting the user
interact with the main battery disconnect directly rather than only through a menu
([FSElite v1.8 coverage](https://fselite.net/content/hot-start-updates-challenger-650-to-1-8/)).
The aft equipment bay was, as of that update, not yet modeled to the same depth.

**Unconfirmed / not found in public sources:** battery chemistry (the real CL650
uses NiCad main-ship batteries) and any battery-temperature modeling. No source
found describing this level of detail for Hot Start's implementation — flagged as a
gap in available documentation, not a confirmed absence.

### Hydraulics, pneumatics/bleed, air conditioning & pressurization

No primary source was found describing these systems' modeled depth in specific
terms (e.g., number of hydraulic systems, pump types, bleed source logic, pack
modeling). The only adjacent evidence is indirect: the "four IOCs" and "multi-source
fault tolerance... for pressure data" claims from the X-Aviation feature page above,
which imply pneumatic/pressure sensing is part of the simulated data-bus
architecture, and the wing/engine anti-ice bleed-air logic noted under Ice
Protection below. **This is a genuine documentation gap** — treat CL650 hydraulics/
pneumatics depth as plausible-but-unverified rather than as a confirmed benchmark.

### Fuel (temperature, freezing)

No source found. Not confirmed either way. Flagged as a documentation gap.

### Engines and FADEC (start, hot/hung starts, oil, vibration)

The product page's physics claims (thrust from fuel-to-heat conversion, generator
load creating engine drag) apply here
([X-Aviation feature page](https://www.x-aviation.com/catalog/iframes/index.html)).
Beyond that, no primary source with specifics on hot-start/hung-start modeling,
oil system simulation, or vibration modeling was found. The only related, confirmed
fact is that engine oil has a **manual replenishment feature** reachable through a
Ground Services menu (see Persistence, below) — this indicates an oil quantity value
exists and can be depleted/serviced, but not that oil temperature/pressure dynamics
or wear are modeled in depth.

### APU

No primary source with specifics found. Not confirmed. Flagged as a documentation
gap.

### Ice protection and ice accretion

The wing anti-ice system is modeled with real CAS (Crew Alerting System) tie-ins:
selecting Wing Anti-Ice OFF requires the WING L HEAT / WING R HEAT indicators and a
green "WING A/ICE ON" CAS message, and the logic depends on the aircraft's own
ice-detection system state and icing-conditions status. This description comes from
the bundled **CL650 Operations Reference**, one of the real-world-derived documents
Hot Start includes with the product
([CL650 Operations Reference, hosted copy](https://www.scribd.com/document/656523517/CL650-Operations-Reference);
documentation bundling confirmed at
[X-Pilot: CL650 Included Documentation](https://forums.x-pilot.com/forums/topic/22282-cl650-included-documentation/)).
Because this is operations-manual text rather than a developer statement about the
sim's code, it confirms the *procedure* is faithfully reproduced (real CAS message
logic gates the switch) but does not independently confirm visual ice accretion
rendering fidelity.

X-Plane 12 itself provides native airframe/engine-inlet/pitot/AOA-sensor ice
accretion and native rain-on-glass effects; Hot Start's v1.7 update explicitly
**removed CL650's own custom atmospheric model** in favor of X-Plane 12's native
temperature/atmosphere simulation, so cold-soak and icing-relevant atmospheric
behavior on XP12 is X-Plane's engine model, not a CL650-specific one
([Threshold v1.7 coverage](https://www.thresholdx.net/news/hst650)). No source
found describing a CL650-specific ice-accretion enhancement beyond what XP12
provides natively (contrast with Hot Start's own TBM 900, which is marketed with
"fully simulated icing and rain effects... on windows and airframe" as a named
feature — that claim is specific to the TBM 900 product page, not the CL650, and
should not be assumed to carry over).

### Flight controls, gear, brakes and tyres

Confirmed from changelogs: nosewheel steering and brakes were tuned for
responsiveness; a bug where "tires took too long to spin down on their own" was
fixed, implying tyre rotational-inertia/spin-down physics are modeled, not just a
binary rolling/stopped state; gear downlock assist springs and safety-pin fasteners
are modeled as 3D detail; physically simulated gear pins and pitot/AOA probe covers
must be removed before flight and otherwise block manipulation of the systems they
protect
([X-Pilot v1.1.0 thread](https://forums.x-pilot.com/forums/topic/23133-take-command-hot-start-challenger-650-v110-update-released/);
[X-Aviation feature page](https://www.x-aviation.com/catalog/iframes/index.html)).

**No source found** for brake or tyre *temperature* or *wear* modeling — and this
absence is corroborated by the maintenance-system finding below (Hot Start
deliberately did not build persistent component wear into the CL650), so brake/tyre
wear is very likely genuinely absent, not merely undocumented.

### Avionics: Collins Pro Line 21, FMS, CAS messages

The CL650 reproduces the **Rockwell Collins Pro Line 21 Advanced** suite with three
independent Flight Management Computers (FMCs), each communicating over simulated
serial/data buses and each driving its own CDU output, with displays rendered to
"pixel-level" fidelity
([X-Aviation feature page](https://www.x-aviation.com/catalog/iframes/index.html);
[Threshold preview](https://www.thresholdx.net/news/hscl65)).

Synthetic Vision (SVS) is implemented and is confirmed by a developer to be
"artificially constructed from a terrain database... SV mode in the sim works
exactly as it does in the real aircraft." Enhanced Vision (an infrared-camera
overlay) was considered but the developers state it could not be implemented "due
to limitations in X-Plane [11]"
([X-Pilot: Systems and Tech CL650](https://forums.x-pilot.com/forums/topic/23384-systems-and-tech-cl650/)).

CAS messages are functionally tied to system state, not decorative: examples found
in changelogs include FD 1/2 FAIL and AFCS 1/2 INOP messages driven by actual flight
control system failure states, and the WING A/ICE ON message gating the anti-ice
switch logic described above
([Threshold v1.7 coverage](https://www.thresholdx.net/news/hst650)).

### Fire and oxygen

Confirmed from the v1.1.0 changelog: an **oxygen refill feature** and an oxygen
service quantity gauge were added, and **fire extinguisher bottle servicing** was
implemented — both reachable through the aircraft's Ground Services menu, alongside
engine oil replenishment
([X-Pilot v1.1.0 thread](https://forums.x-pilot.com/forums/topic/23133-take-command-hot-start-challenger-650-v110-update-released/);
maintenance-menu location confirmed at
[X-Pilot: CL650 Maintenance system](https://forums.x-pilot.com/forums/topic/22489-cl650-maintenance-system/)).
These are consumable-quantity features (a number that depletes and can be
replenished), not confirmed to include fire-detection-loop or oxygen-system-leak
failure simulation depth beyond quantity tracking.

---

## 2. Circuit breakers

Circuit breakers are functionally modeled (not decorative): they can pop as a
consequence of real system faults (e.g., FCC overload popping the IAPS breaker,
short-circuit failures on a bus), and popped breakers block the systems they
protect until correctly reset — including at least one confirmed bug where a
breaker could get stuck in a "permanently popped" or an "inescapable" pop/reset
loop, which by definition means normal behavior is "pop under fault → reset once
fault clears"
([X-Pilot v1.1.0 thread](https://forums.x-pilot.com/forums/topic/23133-take-command-hot-start-challenger-650-v110-update-released/);
[Threshold v1.1.0 coverage](https://www.thresholdx.net/news/cll650)).

Presentation: circuit breakers are modeled as physical, clickable 3D objects in
the cockpit and, as of v1.8, in a fully modeled, openable **forward equipment bay**
reachable from outside the aircraft, which also houses the main battery disconnect
and TRUs
([FSElite v1.8 coverage](https://fselite.net/content/hot-start-updates-challenger-650-to-1-8/)).
There is **no evidence of a tablet/EFB "CB page"** — CL650 does not use a G3000-style
touchscreen EFB at all (see Section 6); breaker interaction is purely physical/3D,
consistent with the real Pro Line 21-era Challenger having no CB synoptic page.

**Not confirmed:** an exact total breaker count for the modeled aircraft. (For
context only — not a claim about Hot Start's implementation — the real Challenger
600-series airframe's CB panels are documented elsewhere as distributed across
overhead, left-side, right-side, glareshield, instrument, and center-console
locations with dozens of breakers per panel; this is real-aircraft panel-numbering
reference material, not a verified count of what Hot Start reproduces
([panel numbering reference, third-party](https://xplanecrj.wordpress.com/2013/07/21/circuit-breaker-panel-numbering/)).)

---

## 3. Failures: random, scheduled, wear-based, MTBF, propagation

This is the most important corrective finding for the gap analysis: **Hot Start did
not ship the CL650 with its own dedicated random/scheduled/MTBF failure engine.**

- The CL650 does have functional, state-driven faults that can occur as a
  consequence of system logic (bus short circuits, breaker overload trips, etc. —
  see Sections 1–2), and these clearly propagate (an FCC overload trips a specific
  breaker which then disables the systems it feeds).
- However, there is **no evidence of a built-in, MTBF-driven, statistically random
  failure system** comparable to what Hot Start's own TBM 900 is marketed as having
  ("custom failure engine" is a named TBM 900 feature on its own product page —
  that specific phrase was not found on the CL650's product page).
- The community filled this gap itself: a third-party FlyWithLua script,
  "Enable random failures for HotStart Challenger 650," adds configurable MTBF-based
  random failures (default `MTBF_hours = 10.0`, user-adjustable down to fractional
  hours for training-intensity failure rates, with an option to restrict failures to
  above 20 kt groundspeed)
  ([X-Pilot file listing](https://forums.x-pilot.com/files/file/1512-enable-random-failures-for-hotstart-challenger-650/)).
- A separate community "Challenger CL650 Failures manager" utility was also under
  development/distributed on the X-Plane.org forum, again as a third-party addition
  rather than a stock feature
  ([X-Plane.org thread](https://forums.x-plane.org/forums/topic/344887-challenger-cl650-failures-manager)).
- It is likely (though not confirmed by a developer statement found in this
  research) that the CL650 supports X-Plane's own native Special > Failures menu for
  manual/instructor-triggered failures, as most systems-deep X-Plane aircraft do by
  virtue of exposing standard datarefs — but no source explicitly confirms this for
  CL650.

**Conclusion for the yardstick:** do not assume CL650 offers a deep, wear-linked,
propagating MTBF failure system out of the box. Its strength is a systems model
detailed enough that faults *can* propagate realistically when triggered (by
X-Plane's native failures, by a third-party script, or by the aircraft's own
internal fault logic), not a bespoke scheduled/wear-based failure campaign layer.

---

## 4. Aircraft persistence and wear

This is the second major corrective finding. A Hot Start developer directly
addressed why the CL650 has **no persistent maintenance/wear system**, in contrast
to Hot Start's own TBM 900:

> "The idea behind the Challenger not having a maintenance system is to simulate or
> immerse you into being the pilot of the plane. As a corporate pilot you don't do
> much maintenance in the same way as you do if your an owner/operator of e.g. a
> TBM."

A user explicitly requested a TBM-style maintenance/wear tracking system (and even
proposed a "Maintenance and Servicing" FBO office) for the CL650; the developer's
response confirms this was a **deliberate design choice**, not an oversight
([X-Pilot: CL650 Maintenance system](https://forums.x-pilot.com/forums/topic/22489-cl650-maintenance-system/)).

What *does* exist, and is better described as **consumable servicing** than
persistent wear tracking:

- Manual engine oil replenishment, oxygen refill with a quantity gauge, and fire
  extinguisher bottle servicing, all via the Ground Services menu
  ([X-Pilot v1.1.0 thread](https://forums.x-pilot.com/forums/topic/23133-take-command-hot-start-challenger-650-v110-update-released/)).

**Not confirmed / evidence points to absent:** tyre wear, brake wear, component
degradation over time, an airframe log, or any state that persists and accumulates
*between separate flight sessions*. No source describes save-state persistence of
wear or servicing needs across sessions; the forum discussion explicitly frames the
CL650 as *not* having this category of system at all.

---

## 5. Ground handling and services

Ground services are a headline feature of the product, framed around an FBO
("fixed-base operator") experience rather than a systems-panel checklist:

- **Training mode** (quick setup) and **Career mode** (full corporate-pilot
  workflow: fuel orders, de-icing decisions, briefing-room flight planning, fueler
  interaction, and coordinating passenger arrival) are both offered
  ([X-Aviation feature page](https://www.x-aviation.com/catalog/iframes/index.html)).
- The player can physically **ride along in the de-icing truck** during a deice
  service
  ([Stormbirds preview coverage](https://stormbirds.blog/2022/01/06/hotstarts-cl650-comes-out-tomorrow-with-a-steep-price/)).
- Fuel truck service includes visible hoses/cables, and a "knock" interaction
  sequence if the refueler arrives at a closed door; Air Start Unit (ASU) hose
  auto-disconnect behavior was refined in updates
  ([X-Pilot v1.1.0 thread](https://forums.x-pilot.com/forums/topic/23133-take-command-hot-start-challenger-650-v110-update-released/)).
- Gear pins and pitot/AOA probe covers are physically simulated: fitted objects
  that must be removed during the walkaround/preflight and that otherwise block
  interaction with what they protect
  ([X-Pilot v1.1.0 thread](https://forums.x-pilot.com/forums/topic/23133-take-command-hot-start-challenger-650-v110-update-released/)).
- Chocks and main-door closure are part of the standard securing flow (confirmed via
  the third-party CrewPackXP companion tool's description of First-Officer-automated
  securing checks, which mirrors the manual procedure a solo pilot performs)
  ([CrewPackXP CL650 docs](https://crewpackxp.readthedocs.io/en/latest/CL650/)).
- GPU (ground power) interaction exists as part of cold-and-dark startup procedure
  (implied throughout changelog and preview material; no single citation isolates
  GPU alone, but it is a prerequisite step in every documented startup flow,
  e.g. [CrewPackXP CL650 docs](https://crewpackxp.readthedocs.io/en/latest/CL650/)).

**Not confirmed:** tow/pushback is not clearly described as a Hot Start-native
system in any source found; ground-equipment/pushback add-ons exist as separate,
generic third-party X-Plane utilities compatible with any aircraft (e.g. "Simple
Ground Equipment & Services"), which suggests pushback specifically may rely on
X-Plane's own ATC/ground-service pushback rather than a CL650-specific
implementation. Flagged as unconfirmed rather than assumed present.

---

## 6. The in-sim interface

**There is no tablet/EFB "Study"-style app** comparable to what G3000-based Hot
Start aircraft or many G1000-era add-ons use. The CL650 reproduces the real
Pro Line 21 Advanced cockpit directly:

- Interaction is through the **three physical CDUs** on the pedestal (each FMC has
  its own), the PFD/MFD bezel keys, and the CCP (Cursor Control Panel) which
  selects what's presented on the MFD (maps, systems synoptic pages, checklists,
  charts) — this mirrors the real aircraft's own interface rather than adding a
  companion tablet
  ([CrewPackXP CL650 docs](https://crewpackxp.readthedocs.io/en/latest/CL650/) for
  CDU/MFD workflow; DCP/CCP hardware panels being sold specifically for CL650 use
  confirm this interface model —
  [Avionique Simulation DCP/CCP product](https://avioniquesimulation.com/product/dcp-ccp-panels/)).
- v1.8 added hover-triggered semi-transparent popups over the PFD/MFD displays to
  make cockpit controls easier to hit without a physical panel
  ([FSElite v1.8 coverage](https://fselite.net/content/hot-start-updates-challenger-650-to-1-8/)).
- **Checklists** are electronic and displayable on the MFD (per the real Pro Line 21
  PM-side MFD capability referenced in CrewPackXP's documentation) and are also
  stored/editable as an XML file (`checklists.xml`) that a community project patches
  for corrections — confirming Hot Start ships a structured, data-driven checklist
  system rather than a static image
  ([CrewPackXP CL650 docs](https://crewpackxp.readthedocs.io/en/latest/CL650/);
  [community checklist-correction repo](https://github.com/intelfx/hotstart-cl650-checklists)).
- Ground Services / consumable servicing (oil, oxygen, fire bottles — Section 1) is
  exposed through a dedicated **Ground Services menu**, separate from the flight
  avionics
  ([X-Pilot: CL650 Maintenance system](https://forums.x-pilot.com/forums/topic/22489-cl650-maintenance-system/)).
- There is no in-house "maintenance panel" (Section 4 explains this is a deliberate
  omission).
- A "Status HUD" overlay exists, but it belongs to **CrewPackXP**, a widely-used
  third-party FlyWithLua companion addon (adds First-Officer automation, PTT, extra
  settings) layered on top of the stock CL650 — not a Hot Start-authored interface
  element
  ([CrewPackXP CL650 docs](https://crewpackxp.readthedocs.io/en/latest/CL650/)).

---

## 7. Environment coupling

- On X-Plane 12, Hot Start **removed the CL650's own custom atmospheric model**
  and switched to X-Plane 12's native atmosphere/temperature simulation, i.e. the
  aircraft explicitly defers cold-soak/ambient-temperature behavior to the sim
  engine rather than maintaining a proprietary model
  ([Threshold v1.7 coverage](https://www.thresholdx.net/news/hst650)).
- Ice-protection switch logic is gated on the aircraft's own ice-detection-system
  state and icing-condition status per the bundled Operations Reference (Section 1).
- No source found describing bespoke weather-driven system degradation (e.g.
  precipitation affecting specific sensors beyond X-Plane's native icing model, or a
  proprietary hail/contamination model). Treated as unconfirmed.

---

## 8. Performance and flight model fidelity claims

In their pre-release announcement, Hot Start committed to a flight model that would
"very closely represent its real-life counterpart in handling (within a few
percent)... this will be tested and verified by real Challenger pilots"
([Threshold preview](https://www.thresholdx.net/news/hscl65)). Concrete,
changelog-confirmed refinements since release include: yoke control-input scaling
increased ~15% for better feel, ground-effect tuning adjustments, and more gradual
throttle management in FLC descent mode
([X-Pilot v1.1.0 thread](https://forums.x-pilot.com/forums/topic/23133-take-command-hot-start-challenger-650-v110-update-released/);
[FSElite v1.8 coverage](https://fselite.net/content/hot-start-updates-challenger-650-to-1-8/)).
As noted in Section 1, thrust is derived from a fuel-to-heat energy model rather
than a static table, and generator electrical load imposes real mechanical drag on
the engine core, coupling the electrical and propulsion models together
([X-Aviation feature page](https://www.x-aviation.com/catalog/iframes/index.html)).

---

## 9. Sound, cockpit interaction model, and other notable points

- **Sound**: Hot Start's pre-release description states the CL650 "uses a custom
  sound engine to fully recreate the aircraft in different states/phases,
  sub-components are also fully accounted for here, each containing its own
  sounds," with sounds "sourced from real recordings of a Challenger 650"
  ([Threshold preview](https://www.thresholdx.net/news/hscl65)). Post-release,
  the "buzzsaw" (propeller/fan-tone) sound was specifically retuned in v1.8 "to be
  more audible and closer to the real airplane both outside and inside"
  ([FSElite v1.8 coverage](https://fselite.net/content/hot-start-updates-challenger-650-to-1-8/)).
  A third-party sound-replacement pack (BlueSkyStar) also exists for the CL650,
  implying the stock sound set has an active enthusiast market for further
  refinement ([BlueSkyStar CL650 sound pack](https://store.x-plane.org/BSS-CL650-Sound-Pack_p_1746.html)).
- **Cockpit interaction / detail**: micro-details such as gear downlock assist
  springs and safety-pin fasteners are explicitly called out as modeled; the
  interior/FBO environment is described by reviewers as feeling "alive," with a
  "mind-boggling level of detail" in switch and texture work
  ([X-Aviation feature page](https://www.x-aviation.com/catalog/iframes/index.html)).
- **Two-pilot shared cockpit**: implemented in v1.7, with its own quick-start guide,
  letting two networked users crew the aircraft together
  ([Threshold v1.7 coverage](https://www.thresholdx.net/news/hst650)).
- **Radome**: openable to inspect weather-radar antenna and nose lighting, added
  v1.8 ([FSElite v1.8 coverage](https://fselite.net/content/hot-start-updates-challenger-650-to-1-8/)).
- **Handling difficulty**: because the systems and automation are modeled
  faithfully rather than simplified, community/review commentary describes the
  aircraft as unforgiving of sloppy speed, descent-profile, flap, or gear
  management on approach — the autothrottle/FLC automation requires active pilot
  supervision rather than "big airliner" hands-off automation. (Summarized from
  search-engine-indexed content of the
  [Flight Simulator Blog CL650 review](https://flightsimulator.blog/hot-start-challenger-650-review/);
  this specific page could not be directly re-fetched during this research pass
  due to a connection error, so treat this characterization as secondary/
  corroborating rather than a verbatim primary quote.)

---

## What 75–80% of CL650 depth means for an A380X port

Ordered by how much each item contributes to the CL650 experience, with the
matching A380X system/area named for each line. Items marked **(gap in CL650
sourcing)** are areas where even Hot Start's own depth could not be confirmed from
public sources — treat those as lower-confidence targets, not as things the A380X
must definitely match feature-for-feature.

1. **A causally-linked systems core, not a switch simulator.** CL650's headline
   trait is that buses, breakers, generators, and engine thrust are computed from
   physical/logical relationships (ARINC 429 bus traffic with real delays/faults,
   generator load creating engine drag, thrust from fuel-energy conversion) so that
   pulling a breaker or losing a generator has real downstream consequences. →
   Maps to the A380's **electrical (AC/DC buses, generators, RAT, batteries),
   hydraulics (Green/Yellow/standby + local electric backup pumps), and FADEC/
   thrust model** — the single highest-leverage investment, since it's what makes
   every other failure and CB meaningful rather than cosmetic.

2. **A real avionics suite driving CAS/ECAM state from system logic**, not scripted
   messages. CL650 ties Pro Line 21 CAS messages (WING A/ICE ON, FD FAIL, AFCS
   INOP) to actual subsystem state. → Maps to the A380's **ECAM (E/WD + SD pages)
   and the FWC (Flight Warning Computer) logic** — messages, memos, and synoptic
   pages must be driven by the same state the systems core computes, not
   independently triggered.

3. **Physical, functional circuit breakers with propagating effects**, presented as
   3D cockpit/equipment-bay objects rather than a CB synoptic page. → Maps to the
   A380's **overhead/pedestal CB panels** (and, if modeled, the FBW ECAM-equivalent
   fault propagation) — CL650 shows this doesn't need a tablet CB page to be
   convincing; physical, consequence-bearing breakers are enough.

4. **A deliberately scoped-down maintenance/wear model** (this is a *lesson*, not
   just a feature to copy): CL650 explicitly ships consumable servicing (oil,
   oxygen, fire bottles) via a Ground Services menu but **no persistent wear,
   tyre/brake degradation, or airframe log**, and the developer justified this as
   matching the corporate-pilot role rather than an owner-operator role. → Maps to
   the A380's **ground-services/EFB "servicing" concept** — for a 75–80% target,
   replicate consumable servicing (oil, oxygen/fire-bottle quantities) but treat
   persistent component wear and an airframe log as explicitly out of scope,
   exactly as Hot Start scoped the CL650. This is the cheapest way to hit the
   stated depth target without over-building.

5. **A failure system that is present but not MTBF/wear-driven out of the box.**
   CL650's own scheduled/random-failure capability is thin — the community had to
   add MTBF-based random failures via FlyWithLua. → Maps to the A380's **failure/
   malfunction system**: matching CL650 depth means exposing enough datarefs/state
   for X-Plane's native Failures menu (and third-party MTBF scripts) to work
   against real system logic — it does **not** require building a bespoke
   statistical failure engine to hit parity with CL650, since CL650 itself doesn't
   have one as a stock feature.

6. **FBO-centric ground handling and services as an experience, not a checklist
   item**: interactive fuel truck with knock/hose sequences, rideable de-ice truck,
   physically-removable gear pins/probe covers, GPU/ASU hookup, Career-mode fuel
   orders and passenger coordination. → Maps to the A380's **ground-services/EFB
   ground-ops flow** — GPU, ground air, fuel truck, chocks, pins, covers, and
   pushback should be physically interactive objects tied to real preflight gating
   (can't remove covers you didn't fit, etc.), not menu toggles.

7. **A faithful native-cockpit interface instead of a bolt-on tablet.** CL650 has
   no G3000-style EFB; it drives everything through the real CDUs/MFD/CCP, with
   XML-defined electronic checklists on the MFD. → Maps to the A380's **FBW EFB and
   flight-deck displays**: if the A380 port already centers on an EFB (per the
   existing `docs/efb-interface.md`), CL650's example argues for making sure
   checklists and system pages are *data-driven and state-linked* (not static
   images) rather than for copying CL650's tablet-free approach verbatim — the
   A380 real aircraft does have an OIS/EFB, so this is a case of matching intent
   (state-driven checklists/pages) rather than form.

8. **Environment coupling deferred to the sim engine where reasonable.** CL650
   dropped its own atmospheric model in favor of X-Plane 12 native
   temperature/icing simulation. → Maps to the A380's **cold-soak/icing/ECS
   coupling** — don't over-invest in a bespoke atmospheric model; wire system
   logic (anti-ice CAS logic, pack/ECS behavior) to X-Plane 12's native
   temperature and icing state, the same way Hot Start does.

9. **Sound sourced from the real aircraft, with per-subcomponent layering.** →
   Maps to the A380's **audio model** — real-recording-based, state-driven sound
   per subsystem (not just per engine state) is a relatively cheap, high-perceived-
   value item once the systems core exists to drive it.

10. **Flight model tuned/verified against real pilots, "within a few percent."** →
    Maps to the A380's **flight dynamics model** — necessary but table-stakes for
    "study level" branding; sequence this after the systems core (item 1) since
    CL650's own marketing frames handling fidelity as a verification step layered
    on top of the systems work, not the primary differentiator.

11. **(gap in CL650 sourcing) Hydraulics/pneumatics/bleed/ECS depth, APU depth, fuel
    temperature/freezing, battery chemistry/temperature, brake/tyre
    temperature.** These could not be confirmed as CL650 features from public
    sources at all — they may exist but are undocumented, or may simply not be
    deep. Given finding #4 (CL650 deliberately scopes out wear), it is plausible
    CL650 also does not model brake/tyre temperature. **Recommendation:** don't
    treat these as confirmed 100%-CL650-parity targets; size them against what's
    independently justifiable for the A380 (e.g., A380 hydraulics fidelity should
    be driven by the FBW A380X's own real-aircraft documentation, not by an
    unconfirmed CL650 comparison).

---

### Source list (deduplicated)

- [X-Aviation product page](https://www.x-aviation.com/catalog/product_info.php/take-command-hot-start-challenger-650-p-212)
- [X-Aviation feature/marketing page](https://www.x-aviation.com/catalog/iframes/index.html)
- [Threshold: CL650 released for XP11](https://www.thresholdx.net/news/chl650)
- [Threshold: CL650 updated for XP11 & 12 (v1.7)](https://www.thresholdx.net/news/hst650)
- [Threshold: CL650 v1.1.0 update](https://www.thresholdx.net/news/cll650)
- [Threshold: pre-release announcement & previews](https://www.thresholdx.net/news/hscl65)
- [FSElite: CL650 v1.8 update](https://fselite.net/content/hot-start-updates-challenger-650-to-1-8/)
- [X-Pilot: v1.1.0 update thread](https://forums.x-pilot.com/forums/topic/23133-take-command-hot-start-challenger-650-v110-update-released/)
- [X-Pilot: CL650 Maintenance system thread](https://forums.x-pilot.com/forums/topic/22489-cl650-maintenance-system/)
- [X-Pilot: Systems and Tech CL650 thread](https://forums.x-pilot.com/forums/topic/23384-systems-and-tech-cl650/)
- [X-Pilot: CL650 Included Documentation thread](https://forums.x-pilot.com/forums/topic/22282-cl650-included-documentation/)
- [X-Pilot: random-failures FlyWithLua script listing](https://forums.x-pilot.com/files/file/1512-enable-random-failures-for-hotstart-challenger-650/)
- [X-Plane.org: Challenger CL650 Failures manager thread](https://forums.x-plane.org/forums/topic/344887-challenger-cl650-failures-manager)
- [CrewPackXP CL650 documentation](https://crewpackxp.readthedocs.io/en/latest/CL650/)
- [GitHub: community CL650 checklist corrections](https://github.com/intelfx/hotstart-cl650-checklists)
- [Stormbirds: CL650 pre-release coverage](https://stormbirds.blog/2022/01/06/hotstarts-cl650-comes-out-tomorrow-with-a-steep-price/)
- [Avionique Simulation DCP/CCP hardware panel (confirms interface model)](https://avioniquesimulation.com/product/dcp-ccp-panels/)
- [BlueSkyStar CL650 sound pack (confirms active sound-mod ecosystem)](https://store.x-plane.org/BSS-CL650-Sound-Pack_p_1746.html)
- [CL650 Operations Reference (bundled documentation, hosted copy)](https://www.scribd.com/document/656523517/CL650-Operations-Reference)
- [Flight Simulator Blog CL650 review (indexed content, not directly re-fetchable)](https://flightsimulator.blog/hot-start-challenger-650-review/)
- [Real-Challenger CB panel numbering reference, background only](https://xplanecrj.wordpress.com/2013/07/21/circuit-breaker-panel-numbering/)
