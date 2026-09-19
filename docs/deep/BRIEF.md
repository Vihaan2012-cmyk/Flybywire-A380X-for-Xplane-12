# Deep systems push — shared brief (read fully before starting)

Project: FlyByWire A380X ported to X-Plane 12, study-level systems. Plugin crate:
`D:\fbw-xp-systems` (Rust). FlyByWire's own systems (read-only reference, never edit):
`D:\fbw-aircraft\fbw-a380x\src\wasm\systems\a380_systems\src` and
`D:\fbw-aircraft\fbw-common\src\wasm\systems\systems\src`.

Goal: CL650-level (or deeper) causal systems for the A380 — every failure acts on a
modelled physical element, and its consequences emerge from the physics, never from a
scripted symptom. You are one of 20 agents working in parallel for a fixed time window.

## Hard rules

1. **Write only inside your own directory** (given in your task), plus your own
   `PROGRESS.md` and `FAILURES.md` there. Debug agents: only the files named in their task.
   Never edit any other file. Never touch `D:\fbw-aircraft` (read only).
2. **No builds, no tests run, no cargo, no git.** The lead does one build when everyone is
   done. Write code that you are confident compiles: stable Rust 2021, `f64`, **std only**
   (no external crates) unless your task says otherwise. Keep modules self-contained: your
   directory's `mod.rs` declares your submodules; nothing else in the crate references your
   code yet, and your code must not depend on crate internals unless your task allows it.
3. **No fake values.** Every constant is either sourced (cite: document, standard, FBW file
   and line, textbook) or labelled `GENERIC` with how it was derived. Consequences come out of
   the model (mass/energy/momentum balances, flow laws, electrical laws), never scripted.
   No filler, no duplicated logic, no placeholder functions that do nothing.
4. **Tests with the code.** Each model gets `#[cfg(test)]` unit tests of real behaviour
   (conservation, limits, a failure changing the outcome, no NaN at zero/rest).
5. Do not read or copy the CL650 install or any proprietary manual. Public data only.
6. Never run whole-disk searches (`find /`, recursive listing of drive roots).

## Working style

- **Work as fast as possible and never stop to ask.** Work down your backlog in order; when
  it is done, extend it with the next most valuable items in your area and keep going until
  you are stopped. There is no deliverable size — go as far as you can.
- Finish one item at a time (code + tests), then append a line to `PROGRESS.md`
  (`- [done] item — files — notes`) before starting the next, so a hard stop loses nothing.
- In `FAILURES.md`, list every single failure your models support:
  `ATA | proposed name | model element it acts on | magnitude meaning (0..1) | effect`.
  One line per genuinely distinct physical fault; do not pad with renamings.

## Registering failures, components and ECAM alerts (every system agent, mandatory)

Register everything in CODE through the API in `D:bw-xp-systemssrcdeeppi.rs` (read it
fully: `Registry`, `FailureDef`, `ComponentDef`, `ParamDef`, `EcamAlert`, `line(...)`, `var(...)`,
`Cond`, `Level`, `Phase`, `failure_id`, `Area`). In your directory create `registry.rs` with

    use crate::deep::api::*;
    pub fn register(r: &mut Registry) { ... }

and declare it in your `mod.rs`. For every item you build:
- `r.component(ComponentDef { .. })` for each physical part with health parameters (0..1, with
  their physical meaning and healthy value), expanded per instance (×4 engines, L/R, green/yellow);
- `r.failure(FailureDef { id: failure_id(Area::<YourArea>, ata, n), .. })` for each single failure,
  naming its component and the exact model field it drives;
- `r.alert(EcamAlert::new(key, ata, "TITLE AS SHOWN", Level::.., trigger).confirm(s).inhibit(&[..])
  .step(line("ENG 2 MASTER", "OFF").done(var("ENGINE_MASTER:2").off())) ... .raised_by(&[ids]))`
  for each ECAM alert your failures raise on the real A380 — procedure lines complete from the
  variable the real cockpit control writes; use the plugin's existing Var names where they exist
  (grep src\ for them) and document any new Var your model must publish in PROGRESS.md.
Ids are yours alone (your area code); number `n` sequentially per ATA chapter. The lead calls
every area's `register` and runs `Registry::validate()`. This replaces CATALOGUE.md and ECAM.md
(delete those files if you made them, after moving their content into registry.rs).

## Conventions

- SI units internally (Pa, K, kg, kg/s, m^3, W, V, A, s). Convert only at edges, named
  `_psi`, `_c`, `_kt` etc.
- Each model is a plain struct with `new(...)`, a `step(&mut self, inputs, dt_s) -> outputs`
  (or equivalent), and fault inputs as fractions `0.0 = healthy .. 1.0 = fully failed`
  collected in a `...Faults` struct with `Default` = healthy.
- Numerically safe at rest and at dt = 0: no NaN, no division by zero, exact exponential
  steps for first-order lags, sub-stepping where stiff.
- Doc comments explain the physics and cite sources, matching the existing style in
  `D:\fbw-xp-systems\src\physics\engine\oil.rs` and `hot_section.rs` (read those as examples).
- Aircraft: A380-800, 4 × Rolls-Royce Trent 972B-84 (EASA TCDS E.012 is public), 2 hydraulic
  systems (green/yellow, 5000 psi) plus electrical backup (EHA/EBHA), 4 VFGs, APU PW980.
