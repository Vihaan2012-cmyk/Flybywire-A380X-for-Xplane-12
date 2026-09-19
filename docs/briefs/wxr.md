# Brief: weather radar (owner of src/wxr/, docs/wxr.md)

Read docs/team.md first. Check src/wxr/ for any partial work.

## Threads
Scripts and screen tessellation run on a worker thread (src/js_worker.rs; src/display/mod.rs `Tess`). Native images are read from both threads, so keep yours behind a Mutex/Arc like `crate::mapdata::plugin::native_image`. Sample X-Plane's weather on the main thread in your own tick, in small budgeted slices, or on a background thread only if the SDK allows it.

## Goal
A weather radar picture on FBW's A380X ND from X-Plane 12.4.4's real weather. The SDK 4.3.0 headers are in D:\fbw-build\xpsdk\SDK\CHeaders (XPLM/XPLMWeather.h).

1. **Research, with citations.**
   - **FBW side:** how the A380X ND shows WXR and what it expects. Grep fbw-a380x and fbw-common for wxr, WXR, weather radar, and EfisTawsBridge (EfisTawsBridge.ts:456-457 marks the radars failed), plus the msfs-sdk weather modes. Look for an ND WXR layer, its inputs (panel modes, tilt and gain L:vars), and TERR/WXR selection.
   - **X-Plane side:** the XPLMGetWeatherAtLocation / XPLMWeatherInfo_t fields, cost and thread rules (header and developer.x-plane.com via WebFetch), and DataRefs.txt `sim/cockpit2/EFIS/EFIS_weather_*`.
2. **Radar model.** Sample precipitation along the beam on a polar grid (tilt, beam width, range) within a strict per-frame budget. Convert to returns at the green, amber, red and magenta levels. Include turbulence and clutter only if they are real.
3. **Image.** An RGBA native image with a generation counter, drawn with opcode 60 NATIVE_IMAGE (docs/display-stream.md). Add a minimal fallback to `crate::wxr::native_image` where src/display/mod.rs `draw_screen` resolves native images.
4. **Placement.** Show it where FBW's ND shows WXR. Otherwise compose it under the ND like terronnd (src/js/msfs/mod.rs NativeGauge), mutually exclusive with terrain per the selector L:vars. Decide on FBW's radar-failed flag; use a SourcePatch only if FBW's logic would hide a working radar, and justify it.
5. **Documentation and tests.** docs/wxr.md, plus tests for geometry, colour levels, polar-to-image, and a synthetic sampler.
6. **Build.** Keep the tree building; use CARGO_TARGET_DIR=D:\fbw-build\target-wxr.

## Report
Findings, what is real versus unavailable, performance, and hooks.
