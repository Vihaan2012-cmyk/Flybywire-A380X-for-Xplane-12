# Give FlyByWire everything it takes from MSFS

Goal: every simulator variable, environment variable, event, Coherent call and
runtime service FlyByWire's A380X uses (docs/briefs/msfs-interface.md) is
served with MSFS's meaning, units and timing, from X-Plane's real state or our
own simulation. Nothing reads a silent 0/NaN.

## Evidence to start from
- docs/briefs/msfs-interface.md: the full inventory with coverage columns
  (treat "partly"/"missing" as leads; our own hand tables in sensors.rs,
  prim.rs, key_events.rs, js_bridge.rs, start_state.rs are more precise).
- State dumps of a real cold apron start in X-Plane:
  D:\A380\fbw-build\state-dumps\session-*\dump-*.tsv (name, value, source, dataref).
  source 0 = nothing ever fed the variable. Simulator variables (names with
  spaces, e.g. "G FORCE") at source 0 are unserved inputs.
- Log: D:/Steam Games/steamapps/common/X-Plane 12/Log.txt ("has no source here").

## Rules
- Read docs/briefs/debug.md rules and storage rules; they all apply.
- No sub-agents. Don't launch/close X-Plane, don't run tools/install.sh.
- Never edit D:\fbw-aircraft tracked sources (TS fixes via SourcePatch or
  tools/js-build/patches only).
- No faked values: a variable comes from X-Plane's matching dataref (with
  correct units/sign/axes, DataRefs.txt at
  D:/Steam Games/steamapps/common/X-Plane 12/Resources/plugins/DataRefs.txt) or
  from a real computation; if X-Plane has no source and none can be computed,
  list it as a gap with the reason instead of inventing a value.
- Stay inside your own files. Build with the shared target dir. Add tests.
- FBW's Rust systems now run in a separate process (src/remote); they see the
  plugin's variables through the shared block, so serving a variable in the
  plugin's Vars serves them too.
- Report: what you served (name -> source), what remains and why, tests run.
