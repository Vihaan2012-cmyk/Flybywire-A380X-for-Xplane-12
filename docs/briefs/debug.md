# Whole-aircraft debug pass (2026-09-17)

The FBW A380X runs in X-Plane 12 through our plugin (D:\A380\fbw-xp-systems, Rust,
built with `CARGO_TARGET_DIR=/d/A380/fbw-build/target-main cargo +stable-x86_64-pc-windows-gnu build --release --features js`)
and a converted aircraft made by D:\A380\msfs2xp-aircraft (OBJ8 cockpit, SASL Lua
bindings, cockpit_bindings.txt). Installed aircraft:
`D:/Steam Games/steamapps/common/X-Plane 12/Aircraft/FlyByWire A380X`.
Latest sim log: `D:/Steam Games/steamapps/common/X-Plane 12/Log.txt`.
FBW sources: D:\fbw-aircraft (reference).

## Rules for every agent
- Do NOT start sub-agents. Do NOT launch or close X-Plane. Do NOT run tools/install.sh
  (X-Plane is running; the lead installs).
- Never edit FBW tracked sources under D:\fbw-aircraft. FBW Rust physics edits
  are saved as patches in patches/fbw-rust; TS fixes go via SourcePatch or
  tools/js-build/patches.
- Don't modify the Airbus House livery; never bundle SASL.
- No faked values: fix causes, not symptoms.
- Stay inside the files of your own area; if a fix belongs to another area,
  write it in your report instead of editing.
- Cargo shares one target dir; builds queue on its lock — that is expected.
- Run the relevant `cargo test --release --features js --lib <filter>` before finishing.
- Finish within about 25 minutes. Report: root causes found (file:line),
  what you changed, what is left, how the user should verify in the sim.

## Symptoms seen in the sim
1. Cold start at apron: the plugin logs "ground power connected for a cold start"
   (src/efb.rs ~761 writes EXT_PWR_AVAIL:1-4 and OVHD_ELEC_EXT_PWR_n_PB_IS_ON) but
   nothing powers up — no displays, no panel lights.
2. At start the log shows "failure 32123 (Overweight landing) activated" — a
   damage/random failure firing on a parked aircraft.
3. Earlier: cockpit buttons in the wrong state, annunciators all lit (e.g. RAM AIR
   light on), clicking buttons did not change the displays.
4. Lua logs 140 "A380 click: fbw/cockpit/..." lines at start without the user
   clicking (seat, knobs, pushbuttons set to 0.5/1.0).
5. JS errors: "[dom] submitDisplay() failed: Error: there is no screen",
   "Failed to fetch edition SyntaxError: Unexpected end of JSON input",
   "[dom] SCREEN_DU_EWD: TypeError: cannot read property 'push' of undefined at
   layoutFlex (dom/layout.js:820)", "ReferenceError: Facilities is not defined".
6. Visual: cockpit very dark (albedo matches MSFS; MSFS adds ambient that XP12
   lacks). Overhead pushbuttons show bright light-blue edge lines along one side of
   every button cap and the button caps look "lifted"/floating; lens decals may
   be lifted 1.5 mm by lift_decals() in msfs2xp-aircraft/src/main.rs.

## Storage rules (added)
- Build only with `CARGO_TARGET_DIR=/d/A380/fbw-build/target-main cargo +stable-x86_64-pc-windows-gnu ...`
  (one shared target dir). Never `cargo clean`, never make another target dir,
  never build the whole workspace with different feature sets.
- Do not copy aircraft/scenery folders, textures or logs anywhere; read them in place.
- No extra backups (the lead runs the backup script). Delete any scratch files you make.
- Fixes since the first round: cabin air waits for real ambient air (patch
  cabin-air-waits-for-ambient), ground power no longer drops on the settling
  ground-speed blip (efb.rs moving_for_s), damage.rs touchdown gate, JS shims
  (Facilities, layout absList, headless hosts), rig.rs ref() retries forever.
