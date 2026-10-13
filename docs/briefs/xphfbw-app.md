# XPHFBW: the aircraft's companion app

A Fenix-style desktop app that is the aircraft's host: it runs FlyByWire's
systems and FlyByWire's JavaScript instruments, and it is where the pilot
changes the aircraft's settings. The X-Plane plugin becomes the bridge to
X-Plane.

Decisions (user, 2026-09-17): Rust + CEF; started by the plugin when the
aircraft loads, closes with X-Plane; CPU (software) rendering for the
instruments, so no extra VRAM.

## Processes

```
X-Plane ── win.xpl (plugin) ──shared memory──  XPHFBW.exe (browser process)
                │                                  ├─ systems thread: FBW Simulation<A380> (src/remote/server.rs)
                │                                  ├─ settings window (CEF, windowed, app/ui/index.html)
                │                                  └─ one off-screen CEF browser per instrument view
                │                                        (MFD, PFD×2, ND×2, EWD, SD, FCU, RMP×3, ISIS, Clock, RTPI, BAT,
                │                                         SystemsHost, ExtrasHost)
                └─ cockpit screens (XPLM avionics devices) ◄── pixels ── OnPaint dirty rects
                                                  CEF renderer processes: V8 runs FBW's JS with the MSFS runtime
                                                  (SimVar/Coherent/VCockpit shims, later MSFS's own core JS)
```

## Shared memory blocks (all created by the plugin, named with a per-session tag)

1. **Systems block** (exists, wire.rs): the Rust systems' 2,119 variables, lockstep per frame.
2. **Variables block**: every plugin variable (names table + f64 values, ~5,000),
   published by the plugin each frame after the systems tick; a ring buffer of
   writes/events from JS back to the plugin (SetSimVarValue, K:/H: events,
   Coherent.call requests that need the plugin: nav data, map data, WXR).
   Renderer processes map it directly, so SimVar.GetSimVarValue is a memory
   read, as in MSFS.
3. **Screens block**: per screen a BGRA frame plus a dirty-rect list and a
   frame counter; the plugin uploads only dirty rects to the device texture.
4. **Input ring**: mouse/wheel/keyboard for a screen, plugin → app.

## Plugin changes

- Starts `XPHFBW.exe <tag> <pid>` (falls back to fbw_a380_systems_server.exe,
  then to in-process systems + the built-in QuickJS engine).
- Menu item "XPHFBW settings" asks the app to show its window.
- Display device draw callback reads the screens block instead of tessellating.

## Settings

Stored where FBW's instruments read them (FBW's NXDataStore keys, our stored
data JSON used by src/js/msfs), plus `xphfbw.json` for the app's own. Applied
live where FBW applies them live; "restart displays" for the rest.

Product configuration
- Auto crash report (collect Log.txt, state dumps and app log into a zip on a crash)
- Streamer mode (mask logon codes and IDs in the UI)
- ACARS service: None / Hoppie / SayIntentions (ACARS_PROVIDER)
- Hoppie ACARS logon code (CONFIG_HOPPIE_USERID); SayIntentions key (CONFIG_SAI_LOGON_KEY)
- Online features (CONFIG_ONLINE_FEATURES_STATUS)
- SimBridge: enabled (CONFIG_SIMBRIDGE_ENABLED: AUTO ON / PERM OFF), remote/local, IP, port
- SimBrief user ID override (CONFIG_OVERRIDE_SIMBRIEF_USERID), auto import
- Navigraph account link

Flight deck displays
- Use display safe mode / force CPU rendering (the app's CEF rendering mode)
- Display frame rate (60 / 30 / 20)
- DMC self test time (CONFIG_SELF_TEST_TIME, seconds)
- ISIS baro inHg (ISIS_BARO_UNIT_INHG), ISIS metric altitude (ISIS_METRIC_ALTITUDE)
- Initial baro unit (CONFIG_INIT_BARO_UNIT), lat/lon extended format (LATLON_EXT_FMT)
- MCDU keyboard timeout (CONFIG_MCDU_KB_TIMEOUT), ECL softkeys (CONFIG_A380X_SHOW_ECL_SOFTKEYS)
- FO EFIS sync (FO_SYNC_EFIS_ENABLED)

Aircraft
- Weight unit (EFB_PREFERRED_WEIGHT_UNIT), ADIRS align time (CONFIG_ALIGN_TIME)
- Boarding rate (CONFIG_BOARDING_RATE), refuel rate (REFUEL_RATE_SETTING)
- Thrust reduction / acceleration / engine-out acceleration altitudes
- Dynamic registration decal, pilot avatars, wheel chocks, cones, satcom
- Pause at T/D + distance, realistic tiller, pushback with controller input
- RMP 25 kHz spacing, radio receiver usage, FDR
- Light preset autoload (day / dawn-dusk / night)
- Sound: announcements, PTU audible in cockpit, interior engine, interior wind, exterior master

Simulation (XPHFBW)
- Systems in their own process
- Cold & dark start with ground power
- Random failures (enable, rate multiplier)
- Airframe wear and damage persistence (enable, reset airframe)
- State dumps (enable, every N frames, keep N)

System status: Restart all, Restart displays, Restart systems, Open logs,
Open state dumps; live status: X-Plane link, systems tick timing, displays fps.

Help & support: Report an issue, Open logs, Discord, changelog.

## Stages

1. Build tools; CEF hello world with the settings window (app/ui/index.html).
2. Systems thread moves into the app; plugin auto-starts it; tray; settings
   read/write (JSON + NXDataStore keys).
3. Variables block + V8 SimVar/Coherent shims; the PFD rendered off-screen
   onto its cockpit device, with mouse input.
4. All views including SystemsHost/ExtrasHost; plugin uses the app's screens;
   QuickJS kept only as the fallback.
