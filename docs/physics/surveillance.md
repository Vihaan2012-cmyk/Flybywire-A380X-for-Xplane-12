# Surveillance (ATA 34): AESS — findings and wiring spec

Status: research complete, no plugin code landed yet (ran out of the session's
time budget before a safe lib.rs integration). This is the handoff for
whoever continues the SURV workstream.

## What's actually there today

FBW's A380X pedestal weather-radar panel
(`fbw-a380x/.../behaviour/pedestal/pedestal.xml`, `Component ID="WeatherRadar"`)
defines eight push buttons through `FBW_A380X_BacklightIndicator_Button_Template`
with **no `LEFT_SINGLE_CODE`/handling code at all** — only backlighting. Their
`INDICATOR_CODE` is the same nonsense placeholder for all eight
(`(L:A32NX_OVHD_INTLT_ANN) 0 ==`, an overhead-panel light test var, not a SURV
state). Confirmed by the converter's cockpit-binding audit
(`D:\A380\fbw-build\conv-test\A380X (branch fs2020-master)\cockpit_bindings.txt`):

```
PUSH_SURV_GS_MODE:        unresolved (ASOBO_GT_Push_Button), keeps fbw/cockpit/PUSH_SURV_GS_MODE: template parameter never given: #BUTTON_CODE#
PUSH_SURV_TCAS_ABV:       unresolved ... keeps fbw/cockpit/PUSH_SURV_TCAS_ABV: ...
PUSH_SURV_TCAS_BLW:       unresolved ... keeps fbw/cockpit/PUSH_SURV_TCAS_BLW: ...
PUSH_SURV_TCAS_TAONLY:    unresolved ... keeps fbw/cockpit/PUSH_SURV_TCAS_TAONLY: ...
PUSH_SURV_WXR_TAWS_SYS1:  unresolved ... keeps fbw/cockpit/PUSH_SURV_WXR_TAWS_SYS1: ...
PUSH_SURV_WXR_TAWS_SYS2:  unresolved ... keeps fbw/cockpit/PUSH_SURV_WXR_TAWS_SYS2: ...
PUSH_SURV_XPDR_TCAS_SYS1: unresolved ... keeps fbw/cockpit/PUSH_SURV_XPDR_TCAS_SYS1: ...
PUSH_SURV_XPDR_TCAS_SYS2: unresolved ... keeps fbw/cockpit/PUSH_SURV_XPDR_TCAS_SYS2: ...
SWITCH_RADAR_GCS:         unresolved (ASOBO_GT_Interaction_LeftSingle_Code), keeps fbw/cockpit/SWITCH_RADAR_GCS: changes no FBW variable (MSFS-local state only)
SWITCH_RADAR_MULTISCAN:   unresolved ... keeps fbw/cockpit/SWITCH_RADAR_MULTISCAN: changes no FBW variable (MSFS-local state only)
SWITCH_RADAR_PWS:         toggles fbw/A32NX_SWITCH_RADAR_PWS_Position between 1 and 0   <- already wired (PWS is INOP in FBW's own tooltip, but at least round-trips)
```

Each unresolved button still gets a manipulator on its own model node (the
converter falls back to the *clip's own* manipulator kind — `Push` for the
eight SURV buttons — see `msfs2xp-aircraft/src/rig.rs:637-733`), so clicking
it in the cockpit already toggles a real, X-Plane-published dataref named
`fbw/cockpit/<NODE_ID>` exactly (e.g. `fbw/cockpit/PUSH_SURV_TCAS_TAONLY`).
`ATTR_manip_push` makes this **momentary**: it goes to 1 while held/clicked
and back to 0 on release, not a latching toggle. Anything reading these needs
to detect the 0→1 edge and do its own state-holding.

The knob/switch templates for tilt, gain and the mode selector
(`FBW_AIRBUS_WeatherRadar_Template`, referenced in `pedestal.xml:311`) are
**not defined anywhere in fbw-aircraft** — they come from Asobo's own SDK
template library that FBW references by name but that isn't in this
repository, so the converter has nothing to expand for `KNOB_RADAR_MODE`.
This needs its own audit pass (not done this session): find what dataref
name(s), if any, `cockpit_bindings.txt` assigns to `KNOB_RADAR_MODE` and
`SWITCH_RADAR_SYS`, or add a hand-written binding for them in the converter
if it left them out entirely.

## What FBW's own JS already does (good news — reuse it, don't replace it)

`fbw-a380x/src/systems/systems-host/Misc/tcas/components/LegacyTcasComputer.ts`
already implements full TA/RA traffic-computer logic (closure rate, sense
selection, aural alerts) reading:

- `L:A32NX_TCAS_MODE` (Enum: STBY/TA/TA-RA) — **no button drives it today**.
- `L:A32NX_TCAS_TA_ONLY` (bool) — this *is* the natural target for
  `PUSH_SURV_TCAS_TAONLY`; nothing sets it today.
- `L:A32NX_TRANSPONDER_SYSTEM` (0/1, which XPDR is active) — natural target
  for `PUSH_SURV_XPDR_TCAS_SYS1`/`SYS2`; nothing sets it today.
- `TRANSPONDER STATE:<n>` — X-Plane's own native transponder-state simvar,
  already the right home for STBY/AUTO/ON/ALT RPTG once system select works.
- Traffic itself comes from `GET_AIR_TRAFFIC`, already answered from real
  X-Plane TCAS targets by `src/mapdata/traffic.rs` (`sim/cockpit2/tcas/
  targets/...`, tested in that file). **Nothing to change here** — RA logic
  does not need reimplementing in Rust; it runs in FBW's TS once the mode
  and system-select vars are driven correctly.

No existing FBW L: var was found for TCAS display range (ABV/NORM/BLW) or for
G/S mode inhibit, WXR/TAWS system select (SYS1/SYS2), or FLAP mode — those
are genuinely new state that has no consumer in FBW's TS today. Making
`PUSH_SURV_TCAS_ABV/BLW` and `PUSH_SURV_GS_MODE` visibly do anything requires
either a plugin-side ND overlay (the plugin owns `src/mapdata` traffic/terrain
rendering, so it *can* just filter by the new range var itself rather than
waiting on FBW's ND code) or a host-side (JS bridge) addition, per the
workstream brief's "JS/TS behaviour FBW lacks is implemented in the plugin or
host."

## Exact wiring needed (for the converter lead, and for the next plugin pass)

Bindings the converter already emits correctly (usable as-is, no converter
change needed):
- `fbw/cockpit/PUSH_SURV_XPDR_TCAS_SYS1` / `_SYS2` (momentary, 0/1)
- `fbw/cockpit/PUSH_SURV_TCAS_TAONLY`, `_ABV`, `_BLW`, `_GS_MODE`,
  `_WXR_TAWS_SYS1`, `_WXR_TAWS_SYS2` (all momentary, 0/1)
- `fbw/cockpit/SWITCH_RADAR_GCS`, `fbw/cockpit/SWITCH_RADAR_MULTISCAN`
  (per `rig.rs`, `ASOBO_GT_Interaction_LeftSingle_Code` fallback — confirm at
  implementation time whether these round-trip a persistent state or are also
  momentary; the audit's "changes no FBW variable" wording suggests they may
  currently be pure MSFS-local toggles the converter mirrors 1:1, in which
  case they *are* usable as a latching 0/1 already, unlike the push buttons).

Still needed from the converter (not found in this session, needs its own
follow-up): a real binding for `KNOB_RADAR_MODE`, `SWITCH_RADAR_SYS`, and any
tilt/gain axis for the weather radar knob cluster — currently untraceable
because their template (`FBW_AIRBUS_WeatherRadar_Template`) isn't in
fbw-aircraft's source tree at all.

## Proposed plugin design (not yet implemented)

A new `src/surveillance.rs` module, constructed in `Plugin::new` after
`radios` (reuses its transponder plumbing) and before/alongside `mapdata`
(owns TCAS range filtering and WXR/TAWS system select), holding:

- Two `AessSystem` instances (1/2) each tracking: active/standby, which ADIRU/
  MMR feed it prefers (mirrors the "each with its own power supply and its
  own ADIRU/MMR inputs" requirement — needs a look at how ADIRS source
  selection already works elsewhere, e.g. `physics/adirs.rs`, before
  duplicating that logic).
- Edge-detectors on the eight `fbw/cockpit/PUSH_SURV_*` momentary datarefs
  (read via `vars.register_named`/`find`, compare against last-tick value),
  each driving one piece of latched state:
  - `XPDR_TCAS_SYS1/2` → `L:A32NX_TRANSPONDER_SYSTEM` (0/1) — exclusive select.
  - `TCAS_TAONLY` → `L:A32NX_TCAS_TA_ONLY` toggle, and derive
    `L:A32NX_TCAS_MODE` from the existing STBY/TA/TA-RA rotary if one exists
    (needs locating — not found this session) or add the rotary's own
    binding.
  - `TCAS_ABV/BLW` → new plugin-owned range enum (ABV/NORM/BLW), consumed by
    `src/mapdata/traffic.rs`'s filtering before targets reach the ND (BLW:
    -9900/+2700 ft, NORM: -2700/+2700 ft, ABV: -2700/+9900 ft, per FCOM).
  - `WXR_TAWS_SYS1/2` → exclusive select mirroring XPDR_TCAS's pattern, feeds
    which AESS lane's WXR/TAWS output reaches the ND (own power supply/ADIRU
    per system, so a lane failure should blank that ND's terrain/WX overlay).
  - `GS_MODE` → G/S mode inhibit for `src/wxr` — verify against `wxr/mod.rs`'s
    existing GPWS-adjacent logic before adding a duplicate flag.
- `SWITCH_RADAR_MULTISCAN` and `SWITCH_RADAR_GCS` driving `src/wxr`'s tilt/
  gain-clutter code directly (the brief: "the plugin has its own weather
  radar ... so those controls must drive our implementation" — this is a
  `src/wxr` change, not a `systems-host` one).

Tests to add once implemented (contract, not yet written):
- Edge detection only fires once per press, not every tick the momentary
  dataref reads 1.
- `XPDR_TCAS_SYS1`/`SYS2` are mutually exclusive (pressing 2 while 1 active
  deselects 1).
- TCAS ABV/BLW/NORM range actually changes which `mapdata::traffic::Target`s
  are handed to the ND (altitude filter boundaries exactly per FCOM figures
  above).
- WXR/TAWS SYS1/2 select changes which AESS lane's `src/wxr` output is live,
  and that a powered-off/failed lane blanks its own side only.

## Why nothing was committed this session

The session's time budget was consumed by tracing the actual control chain
end to end (pedestal.xml → converter's binding resolution in
`msfs2xp-aircraft/src/rig.rs`/`bind.rs` → generated `cockpit_bindings.txt` →
FBW's `LegacyTcasComputer.ts` consumers) because none of it was already
documented anywhere in this repo, and a hard 10-minute wrap-up deadline
landed before a `lib.rs`-integrated module could be written and tested
safely. Landing a half-wired `Plugin` field under that time pressure risked
leaving the shared build broken for the other 16 agents, which the
instructions rank above shipping something. The design above is scoped
tightly enough (one new file, one `Plugin::new` call, no edits to any other
workstream's files) to implement directly next session.
