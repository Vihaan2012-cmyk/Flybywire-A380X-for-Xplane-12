# Flight model calibration (XP-003)

How the converter (`D:\A380\msfs2xp-aircraft\src\acf.rs`) carries FlyByWire's
`flight_model.cfg` aerodynamic data into the .acf, and where X-Plane's
blade-element model cannot take an MSFS table directly and has to be
calibrated instead. Companion to the FADEC/thrust write-up in `fadec.rs`'s
own module doc (XP-002).

## What is set directly

`acf.rs`'s `aero_tuning()` and the speeds block in `build()` read
`flight_model.cfg` and set verified Plane Maker properties (checked against
a real template, `Airbus A330-300/A330.acf`, and against the converter's own
prior output, `X-Plane 12/Aircraft/FlyByWire A380X/FlyByWire A380X.acf`,
which had several of these still at the A330 template's own defaults before
this change):

| Plane Maker property | flight_model.cfg source | Notes |
|---|---|---|
| `acf/_Vso_kts` | `[REFERENCE SPEEDS] full_flaps_stall_speed` | already read before this change |
| `acf/_Vs_kts` | `[REFERENCE SPEEDS] flaps_up_stall_speed` | already read before this change |
| `acf/_Vfe1_kts` | `[REFERENCE SPEEDS] max_flaps_extended` | already read before this change |
| `acf/_Vle_kts` | `[REFERENCE SPEEDS] max_gear_extended` | already read before this change |
| `acf/_Vno_kts`, `acf/_Vne_kts` | the given VMO (`inp.vmo`), **not** `[REFERENCE SPEEDS] max_indicated_speed` | unchanged; see below |
| `acf/_Mmo` | the given MMO (`inp.mmo`), **not** `[REFERENCE SPEEDS] max_mach` | unchanged; see below |
| `acf/_flap1_cl` | `[AERODYNAMICS] lift_coef_flaps` | previously left at the A330 template's `1.1`; FBW's is `1.2694` |
| `acf/_flap1_cd` | `[AERODYNAMICS] drag_coef_flaps` | previously left at the template's `0.075`; FBW's is `0.270` |
| `acf/_stall_warn_aoa` | `[STALL PROTECTION] on_limit` | previously left at the template's `12` deg; FBW's alpha-protection trigger is `20` deg |
| `acf/_flap_ext_time`, `acf/_flap_ret_time` | `[FLAPS.*] extending-time` (slowest section) | previously left at the template's `16` sec; FBW's is up to 25 sec |

`_flap2_cl`/`_flap2_cd` (the leading-edge slats' family) are left at the
template's: `[AERODYNAMICS]` gives one combined lift/drag figure, not split
by leading/trailing edge, and using the same number for both would not be
sourced from anything — the honest choice is to leave the second family
alone rather than invent a split.

### Vno/Vne/Mmo stay separate from `[REFERENCE SPEEDS]`

`acf.rs`'s `Inputs::vmo`/`mmo` already carry the doc comment "Maximum
operating speed (kt) and Mach, which MSFS cfgs do not hold (their
max_indicated_speed and max_mach are overspeed damage limits)". That holds
for FlyByWire's A380X too: `[REFERENCE SPEEDS] max_indicated_speed` is 390 kt
and `max_mach` is 0.97, both well past the real aircraft's published VMO/MMO
of 330 kt/Mach 0.89 (which is what `inp.vmo`/`inp.mmo` are given as, and what
the converter's prior real output already has: `_Mmo 0.89`). Reading VNO/VNE
markings and MMO off `max_indicated_speed`/`max_mach` would put the ASI's
barber pole and the Mach warning at the airframe's damage threshold instead
of the normal operating limit — a regression, not a fix — so this item
leaves that path alone and only adds the genuinely unread sections
(`[AERODYNAMICS]`, `[FLIGHT_TUNING]`, `[FLAPS.*]`, `[STALL PROTECTION]`).

## What X-Plane cannot take directly, and how it is checked instead

X-Plane's blade-element model computes lift and drag from the converted
wing's own geometry (span, chord, sweep, dihedral, airfoil) plus the handful
of scalars in the table above. It has no equivalent of:

- `[FLIGHT_TUNING] induced_drag_scalar` (0.887), `parasite_drag_scalar` (1),
  `flap_induced_drag_scalar` (0.35) — X-Plane's induced and parasite drag
  come from the wing's geometry and its airfoil polar files, not a global
  multiplier Plane Maker exposes.
- `[AERODYNAMICS] drag_coef_zero_lift`, `drag_coef_zero_lift_mach_tab`,
  `lift_coef_aoa_table`'s full shape (only its peak, for `_stall_warn_aoa`,
  is used) — these are MSFS's own tabular Cl(alpha)/Cd(alpha) curves; X-Plane
  gets its curve from the airfoil files the wing surfaces reference, which
  the converter's earlier wing-slicing code already picks from the 3D model,
  not from this table.

These cannot be "set"; they can only be checked. `aero_tuning()` runs an
offline calibration check every conversion: the textbook 1g lift equation

```
Vs = sqrt(2 * W / (rho_sl * S * CLmax))
```

(`stall_speed_kt` in `acf.rs`) predicts a stall speed from
`[WEIGHT_AND_BALANCE] max_gross_weight`, `[AIRPLANE_GEOMETRY] wing_area` and
the clean wing's own CLmax (the peak of `[AERODYNAMICS] lift_coef_aoa_table`,
`lift_curve_peak()`), and the result is compared against
`[REFERENCE SPEEDS] full_flaps_stall_speed`/`flaps_up_stall_speed`. The
report gets a `stall speed check:` line every run, and a `WARNING:` line if
either prediction is over 25% off its reference — a signal that the
converted wing's area or airfoils (not this file) need a look.

### The numbers, for FlyByWire's A380X

| | MTOW 1,124,355 lb, wing area 9,096 sq ft | flight_model.cfg reference | Error |
|---|---|---|---|
| Landing (CLmax 1.70 + 1.2694 flap) | 110.9 kt | 115 kt | -3.6% |
| Clean (CLmax 1.70) | 146.6 kt | 171 kt | -14.3% |

Landing configuration lines up closely — CLmax and the flap increment both
come straight from `[AERODYNAMICS]`, and MTOW/wing area are exact cfg values,
so there is little room for the two numbers to diverge by more than the
formula's own idealisation (no ground effect, no compressibility, an
idealised 1g stall rather than a certified/buffet-onset speed).

Clean configuration is a cruder approximation, at -14%, still inside this
check's 20% tolerance but worth recording honestly rather than picking a
tolerance that hides it: `flaps_up_stall_speed` in `[REFERENCE SPEEDS]` looks
to carry more operational margin above the pure aerodynamic 1g stall than
the landing figure does (it is more of an "operable minimum speed" than a
literal Vs1g), and the clean-wing `lift_coef_aoa_table`'s peak (1.70 at 20.6
deg) is a single MSFS breakpoint, not a full three-dimensional CLmax. Both
numbers are still the right order of magnitude and both come from
`flight_model.cfg` itself, not an invented constant.

## What only X-Plane can verify

This offline check predicts a 1g, sea-level, wings-level stall speed from
weight, wing area and CLmax; it cannot see:

- Actual buffet onset, stall departure characteristics or post-stall
  behaviour, which come from the converted wing's real airfoil polars in
  X-Plane's own solver, not from this formula.
- Ground effect, load factor beyond 1g, bank angle, ice, or the flap-detent
  drag/pitch transients `[FLAPS.*]`'s per-detent `drag_scalar`/`pitch_scalar`
  describe (X-Plane's own flap animation and drag come from the deployed
  flap surface's geometry, not a per-detent scalar).
- Whether the .acf's actual in-sim stall speed (flown, not predicted) lands
  near flight_model.cfg's reference — this check only says the inputs to
  X-Plane's own aerodynamic solver are in the right place, not what that
  solver ultimately produces once the aircraft is flown in X-Plane.
- Climb gradient and L/D in cruise, which need a flown or X-Plane
  Plane-Maker-computed polar, not just CLmax/wing area.

## Study panel

- `acf/_Mmo`, `acf/_Vno_kts`, `acf/_Vne_kts`, `acf/_flap1_cl`, `acf/_flap1_cd`
  and `acf/_stall_warn_aoa` are static .acf properties, not datarefs — there
  is nothing live for the Study panel to show for XP-003 itself. The
  "stall speed check"/`WARNING:` report lines are conversion-time output
  the lead already surfaces wherever the converter's report is shown; no
  new Study panel field is needed for this item.
