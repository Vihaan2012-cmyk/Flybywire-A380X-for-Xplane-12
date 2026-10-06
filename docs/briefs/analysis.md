# Brief: stage 2 analysis (read-only)

This is a read-only analysis: don't edit any source file. Write only your report file in docs/analysis/.

**Do not start sub-agents** (no Agent tool). Do the reading yourself, efficiently: grep and targeted reads, not whole-tree dumps.

## Context
The FlyByWire A380X is being ported to X-Plane 12 as the plugin D:\A380\fbw-xp-systems (Rust). Read docs/team.md.
- **What runs:**
  - FBW's Rust systems (D:\fbw-aircraft fbw-a380x/src/wasm/systems/a380_systems, fbw-common/src/wasm/systems/systems);
  - compiled FBW C++ computers (src/fbw_cpp);
  - Rust ports (fadec.rs, prim.rs, flight_controls.rs, handling/, extra_backend*, sensors.rs, aspects.rs, ...);
  - FBW's TypeScript hosts and instruments, unmodified, in a JS runtime.
- **The goal is about 75-80% of Hot Start CL650 depth; docs/cl650-reference.md (being written) describes what the CL650 simulates, so use it as the yardstick when it exists.**

## Your job
Analyse **everything** in your scope and list **every** gap. Do not stop at a top N. However long the report gets is fine.

A gap is:
- anything the real A380 has that isn't simulated;
- any real physical or logical model replaced by a shortcut: a hard-coded state machine or if-chain standing in for physics, constants, `TODO`/unimplemented, "simplified", always-true/false stubs, magic timers, missing components (buses, contactors, valves, sensors), missing failure modes;
- inputs left at defaults or fed from proxies in the plugin glue;
- places where X-Plane's own physics or systems decide something FBW's model should.

## For every gap
- **ID:** a stable id, e.g. ELEC-017.
- **System and ATA chapter.**
- **Evidence:** file:line.
- **What the real aircraft does.** Cite FBW's own docs and code comments, or reputable public A380 references. Never invent values.
- **Concrete proposal** to simulate it properly.
- **Realism impact** (1-5) and **effort** (S/M/L/XL).

## Report layout
1. A summary table of all gaps sorted by impact, then effort.
2. Per-system sections.
3. At the end, a "Top 50 candidates" list: the 50 highest impact-per-effort gaps in your scope, with their ids. Stage 2.5 will fix the overall top 50, and each fix will also be shown in the plugin's Study panel.
