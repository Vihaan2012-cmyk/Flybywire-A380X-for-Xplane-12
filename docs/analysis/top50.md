# Stage 2.5: the top 50

These 50 come from docs/analysis/systems.md, ecam-instruments-coupling.md and cockpit-study-cbs.md (106 gaps in total), ranked against docs/cl650-reference.md, whose core is the causally linked systems, state-driven warnings, consumables servicing and pins/covers.

**Left out because they are already fixed:** XP-001 reversers, XP-004 fuel pump connections, XP-006 sim rate, XP-007 LightSync inputs, XP-008 performance warning, CPU-011 computer failures, STUDY-002 Failures page (added). **Left out as out of scope by CL650's own design:** STUDY-006 wear/maintenance, COMM-001/002, RA-001, WB-001, FCTL-001/008, CPU-001/009.

**Every fix must also be visible in the Study panel** (a page, or a field on the relevant page).

| # | ID(s) | Fix | Workstream |
|---:|---|---|---|
| 1 | XP-002 | FlyByWire's FADEC decides thrust, fuel flow and N1/N2 dynamics; X-Plane's engine only follows (throttle to the thrust FBW commands, or thrust override) | A coupling |
| 2 | XP-003 | Flight model from flight_model.cfg [AERODYNAMICS]/[FLIGHT_TUNING]/[FLAPS]/stall, checked against FBW performance data | A coupling |
| 3 | CTRL-002, CPU-006, FCTL-005 | FCU altitude knob, increment selector and value events (SPD/HDG/ALT/VS set, EFIS) reach FBW's FCU | C controls |
| 4 | CTRL-009 | Fire handles, guards and discharge wired into FBW's fire logic end to end | C controls |
| 5 | LIGHT-001, XP-005, LGT-001 | Exterior and cockpit lights powered from FBW's buses via systems.cfg light circuits | B electrical |
| 6 | CB-001 | The 10 reset-panel buttons (A32NX_RESET_PANEL_*) actually reset their systems as FBW defines | B electrical |
| 7 | CTRL-007 | Parking brake lever bound | C controls |
| 8 | CTRL-006 | Landing elevation and manual cabin V/S controls bound to FBW's CPCS | C controls |
| 9 | INST-005 | FMS takeoff speed check re-enabled (FmcAircraftInterface.ts:635) | D ECAM/instr |
| 10 | JS-010 | The 10 legacy React SD pages render correctly (verify, then fix the DOM) | F runtime |
| 11 | STUDY-003, OXY-001 (servicing) | Ground Services page: GPU, air start, chocks, gear pins and probe covers gating, fuel truck, plus consumables servicing (engine oil, oxygen, fire bottles), per the CL650 | Lead: Study |
| 12 | OXY-001 | Oxygen quantity model: crew and passenger bottles, consumption, low-pressure cautions, mask drop on decompression | E environment |
| 13 | ICE-001 | AMBIENT IN CLOUD fed from X-Plane cloud layers | E environment |
| 14 | FUEL-001 | Fuel jettison valves operable | E environment |
| 15 | FUEL-002 | Manual fuel pump clicks routed to the fuel network | E environment |
| 16 | FCDC-001..005 | FCDC stubs: aileron fault, speedbrake command, steering fault, autoland warning via FWS, EFCS status word | D ECAM/instr |
| 17 | CPU-003, CPU-004, FCTL-003 | Pitch and rudder trim switch discretes into the PRIMs | C controls |
| 18 | CPU-002, FCTL-002 | DME distance into the PRIMs' ILS | D ECAM/instr |
| 19 | ECAM-014 | FADEC discrete words (running/idle/starting) for the FWS | D ECAM/instr |
| 20 | ECAM-010 | GEN LO / bleed / reverser INOP from contactor, IDG, hydraulic and power discretes | D ECAM/instr |
| 21 | ECAM-001 | Most restrictive speed limitation | D ECAM/instr |
| 22 | ECAM-002 | Derated climb, MFP heating failed, soft GA lost conditions | D ECAM/instr |
| 23 | ECAM-003 | Door open with engine running; pack overheat | D ECAM/instr |
| 24 | ECAM-007 | Antiskid fault from power and fault signals | D ECAM/instr |
| 25 | ECAM-009 | MAX REVERSE callout suppressed when a reverser is INOP | D ECAM/instr |
| 26 | ECAM-017 | All ECAM aurals wired to the sound module | D ECAM/instr |
| 27 | INST-009 | PFD spoiler indication from FCDC and LGCIS | D ECAM/instr |
| 28 | STUDY-005 | Schematics for Bleed, Air Cond, Pressurisation, Gear/Brakes, Air Data, Fire | Lead: Study |
| 29 | CTRL-003 | EFIS baro knobs bound to FBW | C controls |
| 30 | CTRL-004, LGT-003 | Surveillance and EFIS button backlights powered | C controls |
| 31 | LGT-002 | Interior dimming knobs bound to FBW potentiometers | B electrical |
| 32 | LGT-004 | Cabin signage switches (EMER EXIT, NO SMOKING) | C controls |
| 33 | CTRL-008 | Audio selector, radar clutter/multiscan, DLS switches | C controls |
| 34 | ICE-002 | Window heat and wipers (heat as an electrical load, and rain removal) | E environment |
| 35 | FUEL-004 | Fuel temperature per tank from X-Plane ambient, with freeze point and FOB LO TEMP | E environment |
| 36 | ECAM-011/012 | ELEC GALLEY/PAX SYS contactor state separate from the pushbutton | D ECAM/instr |
| 37 | ECAM-015 | IR3 selection via CDS, SFCC switching, rudder fault logic | D ECAM/instr |
| 38 | ECAM-013 | FWS ground speed from the CDS source | D ECAM/instr |
| 39 | ECAM-008 | SURV SYS group INOP from real system health | D ECAM/instr |
| 40 | INST-010 | Pitch trim CG fallback (FQMS, then WBBC) | D ECAM/instr |
| 41 | JS-006 | CSS gradients | F runtime |
| 42 | JS-005 | SVG getCTM/getScreenCTM/getTotalLength | F runtime |
| 43 | JS-002, JS-004 | Canvas isPointInPath/isPointInStroke, getImageData | F runtime |
| 44 | JS-003 | Canvas drawImage from canvas sources | F runtime |
| 45 | JS-008 | animation-play-state: paused | F runtime |
| 46 | JS-001, JS-007 | Canvas patterns, CSS grid repeat() and tracks | F runtime |
| 47 | STUDY-001, CB-002 | Circuit breaker page groundwork (stage 3 builds 50+ on it) | B electrical + Lead |
| 48 | DOORS-001 | Upper deck door exterior animations (converter) | Lead |
| 49 | CTRL-005 | 13 overhead pushbuttons with no click code: give each FBW's documented function where one exists | C controls |
| 50 | STUDY-004 | Aircraft state saved between sessions (switch positions, fuel, oil, oxygen, doors) | Lead: Study |

**Rule for FBW TypeScript fixes (D, F):** keep FBW's source untouched. Fix through `SourcePatch` (src/js/msfs/mod.rs), or through patch files under tools/js-build/patches applied before FBW's build. Document each one.
