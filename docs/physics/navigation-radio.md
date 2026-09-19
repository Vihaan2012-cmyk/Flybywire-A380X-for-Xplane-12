# Navigation receivers and communications (ATA 34 / ATA 23) — audit and plan

Session cut short by an early deadline. One change landed and verified
(compiles, tests pass); everything else below is audit/plan only, not yet
coded. This is the handoff for whoever picks up the NAV/COM receivers
workstream next.

## Landed this session: NAV VOR LATLONALT:1-4

`src/radios.rs` now triangulates each VOR/DME station's position from the
aircraft's own lat/lon plus what the receiver already measures — the true
bearing to the station (`nav_bearing_deg_mag` corrected for magnetic
variation) and its DME slant range — via a standard spherical
destination-point formula (`destination_point`, new `EARTH_RADIUS_NM`
constant). It only fires where `nav_type != 0` (a station is actually
received) and `has_dme != 0` (a range leg exists to triangulate with); no
DME means no fabricated position. Published as `fbw/radio/nav{1..4}/lat`
and `/lon` (plain numbers), computed once per tick in `feed_variables`
alongside the existing NAV feed.

Both `simvar.js` hosts (`src/js/msfs/simvar.js` and `app/js/msfs/simvar.js`)
now special-case `NAV VOR LATLONALT:1-4` in their `struct()` function,
composing the LLA from those two published values (altitude is left at sea
level — not knowable from the receiver alone, and unused by the ND's 2D
plot, which is the only consumer of this struct that FBW's `VorBusPublisher`
currently wires up). This removes the `"... is not supported here"` console
warning for `nav1Location`..`nav4Location` reads.

New test: `radios::tests::vor_station_position_triangulates_from_bearing_and_dme`
(pure geometry, checked against 1 nm ≈ 1/60 degree at the equator both
north and east, plus the zero-distance identity case). Verified: `cargo
check --features js` clean, `cargo test --lib radios::` — 6/6 pass,
including the new one.

**Left for next time:** `NAV GS LATLONALT:3` (the glideslope's position) is
not yet served — it needs the glideslope transmitter's position, which
isn't a simple bearing+DME triangulation the way a VOR is (the GS antenna
sits off the runway threshold, not at the station DME's phase center);
`sensors.rs`'s `Ils` already resolves the tuned frequency to a navaid-
database record for the raw glide slope angle (`glide_slope_angle`,
`sensors.rs:306-363`) and would be the natural place to also expose that
record's lat/lon — but `sensors.rs` was left untouched this session since
its ownership wasn't confirmed and it's plausible another workstream
touches it concurrently. Coordinate before extending it.

## What already exists (`src/radios.rs`, `src/sensors.rs`)

`src/radios.rs` (1191 lines) is solid for MSFS event → X-Plane dataref tuning:

- BCD16/BCD32/ADF-BCD32 codecs (`bcd32_to_hz`, `hz_to_bcd32`, `bcd16_to_hz`,
  `adf_bcd32_to_hz`), all round-tripped in `bcd_encodings_round_trip_the_way_msfs_packs_them`.
- Every MSFS radio key event (`NAVn_RADIO_SET`, `VORn_SET`, `ADFn_*`,
  `COMn_*`, whole/fract/carry steps, swaps) applied to a `Receivers` struct,
  then mirrored to X-Plane's `sim/cockpit2/radios/actuators/*` datarefs
  (`Radios::write_to_xplane`) and taken back when X-Plane's own cockpit or
  another plugin changes them first (`Follow`/`take_from_xplane`).
- NAV1/2/4 feed `NAV ACTIVE/STANDBY FREQUENCY`, `NAV OBS`, `NAV HAS NAV`,
  `NAV RELATIVE BEARING TO STATION`, `NAV LOCALIZER`, `NAV RADIAL ERROR`,
  `NAV MAGVAR`, `NAV HAS DME`/`NAV DME`, `NAV TOFROM`, volume/ident-sound.
  NAV3 (the MMR/ILS) is deliberately left to `sensors.rs`'s `Ils` struct,
  which does line-of-sight-correct loc/GS validity, hdef/vdef-to-degrees at
  FBW's PFD scale, DME, magvar, and a **real glide slope angle** looked up
  from X-Plane's nav database via `XPLMFindNavAid`/`XPLMGetNavAidInfo` when
  the on-frequency `nav1/2_slope_degt` dataref isn't available
  (`Ils::glide_slope_angle`, `sensors.rs:306-363`).
- ADF1/2: active/standby frequency, `ADF RADIAL` (relative bearing only —
  no quadrantal or night-effect error model), volume/ident-sound.
- COM1-3: active/standby, volume, receive/transmit select, `COM RECEIVE ALL`,
  marker beacon state/sound.
- The MFD's manual LS (ILS) frequency/course entry (`LsTuning`), including
  the approach-phase/700 ft tuning lock and `A32NX_FM_LS_COURSE` output —
  fully implemented and tested (`manual_ls_tuning_sends_what_the_navaid_tuner_sends`).

## Confirmed gaps, in priority order

### 1. NAV VOR LATLONALT / NAV GS LATLONALT — unserved struct simvars (the task's headline ask)

`app/js/msfs/simvar.js` and `src/js/msfs/simvar.js` (two copies, kept in
behavioural sync — see the comment at the top of the `app/js` one) both have
a `struct(name, unit)` function that special-cases exactly one `latlonalt`
name, `(A:)?PLANE POSITION`, composing it from three already-served scalar
simvars (`A:PLANE LATITUDE/LONGITUDE/ALTITUDE`) via `host.getVar`. Anything
else falls through to `once(...)` and the ND's `"... is not supported here"`
warning. `NAV VOR LATLONALT:n` and `NAV GS LATLONALT:n` are exactly this
case (confirmed against `@microsoft/msfs-sdk`'s built-in nav/GS/DME/ADF LLA
publisher, `msfssdk.js:10573-10575`, which the ND's raw-data provider pulls
from) and are explicitly the two FBW reads `radios.rs`'s own module doc
lists as **not served** (`radios.rs:56-58`) — `docs/briefs/msfs-interface.md`
lines 168-169 marking them "served: src/radios.rs" is stale/aspirational,
not actual.

**Plan** (not yet coded):
1. In `radios.rs`, add three `Published` scalar values per receiver that
   needs a position — `fbw/radio/nav{n}/lat`, `/lon`, `/alt_ft` for the VOR
   case (n = 1,2,3,4) and `fbw/radio/nav{n}/gs_lat` etc. for the glideslope
   case (n = 3 at least, matching what `Ils` already does for NAV3) —
   mirroring the existing `fbw/radio/nav{n}/ident` pattern
   (`radios.rs:743-746`).
2. Feed those values from the tuned frequency using a **navaid database
   lookup**, not raw FFI if avoidable: `src/navdata` (`apt.rs`, `dat.rs`,
   `nearest.rs`, `facility.rs`) already parses X-Plane's own `nav.dat`/CIFP
   data for other purposes and is the safer, already-tested route versus
   duplicating `sensors.rs`'s local `XPLMFindNavAid`/`XPLMGetNavAidInfo` FFI
   (that FFI works — `sensors.rs:165-197, 306-363` — but is per-file
   unsafe boilerplate that shouldn't be triplicated without an existing
   safe wrapper; check `navdata::nearest`/`navdata::dat` for a
   frequency+position query before reaching for raw FFI again). Whichever
   source, the VOR case should key off `nav.active_hz` per receiver and the
   GS case off the same frequency `sensors.rs`'s `Ils` already resolves for
   NAV3, so the two should probably share one lookup.
3. In both `simvar.js` files, extend `struct()`:
   ```js
   if (u === 'latlonalt' && /^(A:)?NAV VOR LATLONALT:([1-4])$/i.test(name)) {
     const n = RegExp.$1;
     return { lat: host.getVar(`fbw/radio/nav${n}/lat`, 'degrees'),
               long: host.getVar(`fbw/radio/nav${n}/lon`, 'degrees'),
               alt: host.getVar(`fbw/radio/nav${n}/alt_ft`, 'feet') };
   }
   if (u === 'latlonalt' && /^(A:)?NAV GS LATLONALT:([1-4])$/i.test(name)) { /* same, gs_* */ }
   ```
   (regex capture group access style to match this file's existing
   `PLANE POSITION` special case exactly; verify `RegExp.$1` behaves inside
   this QuickJS/host sandbox before relying on it — a captured-group
   parameter passed explicitly to a helper is the safer bet if not.)
4. Add a `#[cfg(test)]` for the new lookup function and update
   `docs/briefs/msfs-interface.md:168-169` to stop claiming this is already
   served.

This was the highest-value, most clearly-scoped item and was next up when
the session was cut short — no code for it landed.

### 2. Radio altimeters 1-3 — real gap, higher severity than expected

`prim.rs` (owned by the flight-controls/PRIM workstream, not this one) reads
`A32NX_RA_{1,2,3}_RADIO_ALTITUDE` as an ARINC-429 word
(`prim.rs:631`, `self.names.word(vars, ...)`). Grepping the whole `src/`
tree, **the only place this variable is ever written is the test-only
`world()` fixture** (`prim.rs:1834`, inside `#[cfg(test)] mod tests`). There
is no production writer. `sensors.rs`'s ILS code and `radios.rs`'s LS-lock
logic both read a single scalar,
`sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot`, directly
(`radios.rs:616`, `lib.rs:745`) — that's a plain float, not the ARINC word
the PRIMs actually consume, and it's only ever the pilot-side probe (no
independent RA2/RA3, no range limit/NCD above ~2500 ft, no terrain-return
distinct from a straight radar-altitude readout).

**This means the PRIMs currently run on an uninitialized/zero radio height
word in production**, which cascades into autoland warnings
(`autoland_warning_condition`, tested against synthetic `NavSimData` only),
flare law, and anything else gated on `radio_alt` in `extra_backend_fcdc.rs`.
This is a bigger finding than "range limit and terrain return are missing" —
the feed doesn't exist at all yet. Fixing it is squarely ATA 34 (radio
altimeter) scope, but the write target (`A32NX_RA_n_RADIO_ALTITUDE`, an
ARINC word) lives conceptually next to `prim.rs`'s word-packing helpers
(`prim.rs:257-286`, `BaseArinc429`, `SSM_NCD`/`SSM_NO`/`SSM_FT`). The
cleanest ownership split: **add the writer in `radios.rs`** (this
workstream), reusing `prim.rs`'s public `to_simvar`/`BaseArinc429`
encoding (check whether it's `pub` — if not, that's a one-line visibility
change, minimal and additive, worth flagging to the flight-controls agent
rather than editing `prim.rs` unreviewed) to write proper SSM_NCD-above-range
words for all three RAs from up to three distinct X-Plane probes if X-Plane
exposes per-antenna radio altimeter datarefs (only `_pilot` was seen this
session — `_copilot`/`_stby` variants need checking against X-Plane's
DataRefs.txt, not done this session).

**Do not attempt this without coordinating with whoever owns `prim.rs`** —
the fix needs to read `BaseArinc429`/`SSM_*`/`to_simvar` from that file or
duplicate them, and duplication of ARINC word packing is exactly the kind
of "faked numbers standing in for the real encoding" this project's rule
forbids.

### 3. Not yet investigated this session (flagged, not started)

- VOR cone of confusion, DME hold function.
- ADF quadrantal error / night effect.
- GPS satellite geometry / HIL / RAIM — need to check which X-Plane GPS
  datarefs are "real" (X-Plane does simulate GPS integrity in some builds)
  versus need a plugin-side model; nothing under this name found in
  `radios.rs`/`sensors.rs`/`lib.rs` this session.
- False glideslope lobes.
- VHF1-3/HF1-2/SATCOM, RMP1-3 (RMP3 state is being fixed by the FWS agent —
  coordinate by reading only, not done this session), ACP/AMU audio
  selection, SELCAL/CIDS: `grep -ril "RMP\|ACP\|SELCAL\|CIDS\|SATCOM\|VHF\|HF1\|HF2" src`
  found **no existing files for any of these** — this is greenfield, not a
  gap in existing code. Needs its own session with real time budget.
- Receivers failing when unpowered: `radios.rs`'s `Radios::update` never
  checks any electrical bus state before tuning/feeding — worth checking
  `breakers.rs`/`circuits.rs` for the relevant NAV/COM/ADF/RA buses and
  gating `write_to_xplane`/`feed_variables` on them, but not investigated
  this session.

## Session outcome

Read-only this session: audited `radios.rs`, `sensors.rs`, both
`simvar.js` copies, `VorBusPublisher.ts` (FBW A380X source, confirms
`NAV VOR LATLONALT:n` is read as `SimVarValueType.LLA`), and
`msfssdk.js` (confirms the ND's built-in raw-nav-data publisher also reads
`NAV GS LATLONALT`/`NAV DME LATLONALT`/`ADF LATLONALT` the same way — all
three are candidates for the same `struct()` extension, not just the two
named in the brief). No source files were changed; no patches to submit
against `D:\fbw-aircraft`. The tree is untouched and compiles as it did at
session start.
