# Brief: sound (owner of src/sound/)

Read docs/team.md first. Earlier engineers were stopped part way through this; src/sound/ (mod.rs, triggers.rs, vorbis.rs, wwise.rs) has partial work, so read it and finish. lib.rs may reference a `sound` field; the tree must build.

## Threads
FlyByWire's JavaScript instruments run on a worker thread (src/js_worker.rs). Anything the scripts call, such as `play_instrument_sound`, runs there:
- Keep it thread-safe (queue into a Mutex).
- Never call XPLM from it; your main-thread tick makes the XPLM audio calls.
- Don't use `thread_local!`.

## Task
1. **Decoding test.** Load the real PCKs, list their events, and decode several Vorbis and PCM media with plausible lengths and rates.
   - The sounds are in D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842\SimObjects\AirPlanes\FlyByWire_A380_842\sound.
   - Read them at run time and never copy them.
   - Find the package path from MSFS UserCfg.opt `InstalledPackagesPath`, with an override `package=` in Output/preferences/fbw_a380x_sound.ini.
2. **Playback.**
   - Index the banks on a background thread.
   - Decode lazily on a worker, with a bounded cache.
   - Play with `crate::xp::play_pcm16_on_bus` on the interior bus, from the main thread.
   - Loop, stop and set volume per the Wwise actions and sound.xml.
3. **Triggers.** Evaluate sound.xml SimVarSounds/LocalVar triggers each tick from `Vars`. Continuous sounds loop while their condition holds; one-shots fire on entry. Leave out WwiseRTPC-driven entries and document that.
4. **Named sounds.** Add `pub fn play_instrument_sound(name: &str)`, thread-safe, mapped per sound.xml AvionicSounds. Route `Coherent.call('PLAY_INSTRUMENT_SOUND', name)` to it with one line in `crate::js_bridge::direct_call`.
5. **Wiring.** Add lib.rs slot lines for create, tick after the systems, and release. No `dead_code` allow, and no warnings in your files.
6. **Tests.** Run `cargo +stable-x86_64-pc-windows-gnu test --release --features js` with CARGO_TARGET_DIR=D:\A380\fbw-build\target-sound. Don't launch X-Plane.

## Report
Events mapped, triggers, what is left out and why, memory and CPU, and your lib.rs lines.
