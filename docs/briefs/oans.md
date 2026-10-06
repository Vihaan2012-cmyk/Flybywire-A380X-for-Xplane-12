# Brief: OANS airport map (owner of src/oans/, docs/oans.md)

Read docs/team.md first. Earlier engineers were stopped part way: src/oans/mod.rs and plugin.rs exist. Read them and finish.

## Threads
Scripts run on a worker thread (src/js_worker.rs). Handlers the scripts reach must be thread-safe (static Mutex/Arc), with no `thread_local!` and no XPLM calls. Calls answered on any thread go in `crate::js_bridge::direct_call`; add one line there.

## Goal
FBW's A380X OANS (fbw-common/src/systems/instruments/src/OANC and the A380X ND/MFD pieces) working in X-Plane. The user will supply the AMDB data. Read a local folder in exactly the format FBW's Navigraph AMDB client receives, and answer the same requests FBW's code makes.

1. **Research, with file:line.** How the OANC gets AMDB data (fetch/HTTP, SimBridge or Coherent), the request/response shapes (layers, projection) and the Navigraph auth gating. Grep D:\fbw-aircraft for `Amdb`, `NavigraphAmdb`, `AmdbFeature`, `getAirportData`, `searchForAirports`.
2. **Provider.** The default folder is `<X-Plane>/Output/fbw-a380x/amdb/`, configurable in Output/preferences/fbw_a380x_oans.ini. Answer requests in the exact shapes, including search and nearby queries.
3. **Auth checks.** Prefer answering the client's own auth and subscription checks faithfully. Use a SourcePatch (src/js/msfs/mod.rs; examples in js_bridge.rs `native_ports`) only if unavoidable, and justify it.
4. **Documentation.** docs/oans.md: the exact folder layout and file format the user must supply, with a minimal valid example airport.
5. **Tests.** Parsing, request shapes, and search. Keep the full suite green; use CARGO_TARGET_DIR=D:\A380\fbw-build\target-oans.

## Report
Findings, data format, hooks, and test results.
