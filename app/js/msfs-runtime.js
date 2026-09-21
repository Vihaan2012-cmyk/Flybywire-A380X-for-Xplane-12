// XPHFBW's MSFS runtime for a real Chromium page.
//
// Loaded as `xphfbw://runtime/msfs-runtime.js`, the very first element of
// <head> of every gauge page (agent G, docs/briefs/xphfbw-js-bridge.md
// section F). By the time this file runs, `window.__xphfbw` is already
// installed (agent E, on_context_created) with `view`, `screen`,
// `aircraftDir` and the native functions the contract lists:
// snapshot/getVar/setVar/getString/setString/sendEvent/call/poll/
// gameString/storedData/readFile/log/magVar/loaded.
//
// This file itself only builds the `__host` adapter every ported shim below
// talks to (matching the shape `globalThis.__host` had in our QuickJS
// engine, src/js/mod.rs's `Host` trait) and loads the rest of the runtime
// synchronously with it, then starts the per-frame tick loop. The actual
// MSFS-shaped API (SimVar, Coherent, ViewListener, BaseInstrument, ...)
// lives in msfs/*.js, ported from src/js/msfs/*.js — see each file's own
// header comment for what changed and why.
//
// Two files from the QuickJS port are *not* ported here at all, because a
// real Chromium page already natively provides everything they existed to
// fake:
//   - src/js/msfs/window.js: `window`/`self`/`parent`/`top`/`location`/`URL`/
//     `URLSearchParams` are all real already. In fact `location` is now
//     *more* correct than the QuickJS shim's fixed `VCockpit.html` stand-in,
//     since agent G navigates each view's browser to the gauge's own page,
//     so relative `Include.addScript`/`addImport` calls resolve against the
//     gauge's real directory.
//   - src/js/msfs/dom_standin.js: a real DOM (HTMLElement, customElements,
//     querySelector, ...) is already there; nothing needs a stand-in.
(() => {
  'use strict';

  const native = window.__xphfbw;
  if (!native) {
    console.error('MSFS runtime: window.__xphfbw is missing; this page is not hosted by XPHFBW (see app/js/test/runtime-test.html for a standalone fake).');
    return;
  }

  // ------------------------------------------------------------------
  // console.* -> __xphfbw.log, in addition to whatever devtools console
  // this browser may or may not have attached (XPHFBW's views are mostly
  // off-screen/headless; without this, every `console.error` FlyByWire's
  // bundle produces disappears with nothing to read it).
  // ------------------------------------------------------------------
  const format = (args) =>
    args
      .map((a) => {
        if (typeof a === 'string') return a;
        if (a instanceof Error) return a.stack || a.message;
        try {
          return JSON.stringify(a);
        } catch {
          return String(a);
        }
      })
      .join(' ');
  const realConsole = { log: console.log, info: console.info, warn: console.warn, error: console.error, debug: console.debug };
  for (const [name, level] of [['debug', 0], ['log', 0], ['info', 0], ['warn', 1], ['error', 2]]) {
    console[name] = (...args) => {
      realConsole[name].apply(console, args);
      try {
        native.log(level, format(args));
      } catch {
        // The bridge itself failed; there is nowhere left to report that.
      }
    };
  }
  window.addEventListener('error', (e) => native.log(2, `Uncaught ${e.message} (${e.filename}:${e.lineno}:${e.colno})`));
  window.addEventListener('unhandledrejection', (e) => native.log(2, `Unhandled rejection: ${e.reason && e.reason.stack ? e.reason.stack : e.reason}`));

  // NOT a real Node `process`: this is a real Chromium page, which never
  // had one, but two of FlyByWire's bundles (the OIT views' oit.js) read
  // `process.env.CLIENT_ID`/`process.env.CLIENT_SECRET` unconditionally at
  // the top of the script (fbw-common/src/systems/instruments/src/
  // navigraph.ts calling the Navigraph SDK's `initializeApp`), unlike the
  // rest of their dependencies, which all guard the same global behind
  // `typeof process !== 'undefined'` first. With no `process` at all, that
  // read throws `ReferenceError: process is not defined` before the
  // script's first line finishes, which `bootstrapGauge`'s catch below
  // turns straight into `host.loaded(false, ...)` — view 15/16 "failed to
  // load" in X-Plane's log. FlyByWire's own build normally substitutes
  // those two properties with real Navigraph client credentials from CI
  // secrets; we have none to substitute and must not invent any, so this
  // only supplies the shape (`process.env` as a plain object) and lets
  // `CLIENT_ID`/`CLIENT_SECRET` read back as `undefined`, exactly as an
  // unmodified copy of the bundle would behave under real Node with those
  // variables unset. Nothing else of a real `process` (argv, platform,
  // version, ...) is provided or should be added here.
  window.process = { env: {} };

  // Coherent GT draws no scrollbars and a cockpit screen never scrolls;
  // Chromium would draw one down the edge of any page taller than its view.
  const noScrollbars = document.createElement('style');
  // MSFS's core stylesheet also gives every gauge the `hidden` utility class,
  // which FlyByWire's FCU relies on to hide its LCD light-test images.
  noScrollbars.textContent = '::-webkit-scrollbar { display: none; } html, body { overflow: hidden; } .hidden { display: none !important; }';
  (document.head || document.documentElement).appendChild(noScrollbars);

  // ------------------------------------------------------------------
  // The __host adapter: the same shape src/js/msfs/*.js already expects
  // (SimVar.js's, coherent.js's and environment.js's `host.X()` calls),
  // built on top of `window.__xphfbw` instead of the QuickJS engine's
  // natives.
  // ------------------------------------------------------------------
  const once = (() => {
    const reported = new Set();
    return (what) => {
      if (!reported.has(what)) {
        reported.add(what);
        native.log(1, `MSFS runtime: ${what} is not supported here`);
      }
    };
  })();

  // Registered ids (SimVar.GetRegisteredId / the msfs-sdk fast path):
  // `__xphfbw` has no separate id-based read, so this only saves the
  // name/unit lookup on this side, not a round trip. Still worth keeping
  // 1:1 with the QuickJS port's behaviour, since FlyByWire's code paths
  // (msfs-sdk's SimVarPublisher family) assume registration always
  // succeeds and getters never fail once registered.
  let nextRegId = 1;
  const registered = new Map(); // id -> {name, unit}
  const registerVar = (name, unit) => {
    const id = nextRegId++;
    registered.set(id, { name: String(name), unit: String(unit) });
    return id;
  };

  // A variable write. H:/K: names are events (send them, don't set a slot);
  // everything else is a plain L:/A:/E: write. This is the same split
  // src/js/msfs/mod.rs's `ViewHost::set_var` made; the difference is that
  // *who else hears it* (other views' intercepts, other instruments'
  // onInteractionEvent) is now entirely the plugin's job
  // (xphfbw_host.rs, agent C) on the other side of `sendEvent`/`poll`,
  // not something this adapter fans out itself.
  const setVar = (name, unit, value) => {
    const n = String(name);
    if (n.startsWith('H:') || n.startsWith('K:')) {
      native.sendEvent(n, [Number(value)]);
    } else {
      native.setVar(n, String(unit), Number(value));
    }
  };
  const setString = (name, value) => {
    const n = String(name);
    if (n.startsWith('H:') || n.startsWith('K:')) {
      native.sendEvent(n, [0]);
      once(`setting ${n} through a string write (value ignored on the event path)`);
    } else {
      native.setString(n, String(value));
    }
  };

  const host = {
    getVar: (name, unit) => native.getVar(String(name), String(unit)),
    setVar,
    getString: (name) => native.getString(String(name)),
    setString,
    gameString: (name) => native.gameString(String(name)),

    registerVar,
    getReg: (id) => {
      const e = registered.get(id);
      return e ? native.getVar(e.name, e.unit) : 0;
    },
    setReg: (id, value) => {
      const e = registered.get(id);
      if (e) setVar(e.name, e.unit, value);
    },
    getRegString: (id) => {
      const e = registered.get(id);
      return e ? native.getString(e.name) : '';
    },
    setRegString: (id, value) => {
      const e = registered.get(id);
      if (e) setString(e.name, value);
    },

    log: (level, message) => native.log(level, message),

    call: (name, argsJson) => native.call(name, argsJson),
    // There is no fire-and-forget native `trigger`; everything that used to
    // reach the plugin's in-process router that way now goes through
    // `call()`, ignoring (but logging) whatever it settles with, since
    // Coherent.trigger never awaited a reply either.
    trigger: (name, argsJson) => {
      // A trigger nothing on the simulator side acts on was a silent no-op
      // in MSFS and in the plugin's own engine (src/js/msfs/mod.rs
      // `ViewHost::trigger`'s fallthrough); only other failures are news.
      native.call(name, argsJson).catch((e) => {
        if (!/nothing here answers/.test(String(e && e.message))) {
          once(`trigger ${name} rejected (${e && e.message})`);
        }
      });
    },

    readFile: (path) => native.readFile(path),
    storedData: (op, key, value) => native.storedData(op, key, value),
    getMagvar: (lat, lon) => native.magVar(lat, lon),
    loaded: (ok, text) => native.loaded(!!ok, String(text || '')),

    // Include.addScript: read the file and eval it in the global scope, the
    // same synchronous "read then run" the QuickJS engine's `run_script`
    // did (see msfs/environment.js's header comment for why this stays
    // synchronous instead of a real `<script src>` element).
    runScript: (absolutePath) => {
      const text = native.readFile(absolutePath);
      if (text === null || text === undefined) {
        throw new Error(`${absolutePath}: not found`);
      }
      try {
        // Indirect eval: runs in global scope, exactly like a parsed
        // <script> element would, so top-level `var`/function declarations
        // become globals the next included script (or this one's own
        // registerInstrument() call) can see.
        (0, eval)(text + `\n//# sourceURL=coui://html_ui${absolutePath}`);
      } catch (e) {
        native.log(2, `${absolutePath}: ${e && e.stack ? e.stack : e}`);
        throw e;
      }
    },
  };
  globalThis.__host = host;

  // ------------------------------------------------------------------
  // Load the rest of the runtime synchronously (document.write, while this
  // script's own parser-blocking execution still owns the insertion point,
  // so every one of these runs — and finishes running — before the parser
  // reaches the gauge's own <script type="text/html" import-script> entries
  // further down the page). Classic head-loader technique; still fully
  // supported by Chromium/CEF.
  // ------------------------------------------------------------------
  // Resolved against this very script's own URL, not hardcoded to
  // `xphfbw://runtime/`, so the same file also works loaded relatively (see
  // app/js/test/runtime-test.html, which loads it as `../msfs-runtime.js`
  // from a plain browser tab with no XPHFBW scheme behind it).
  const selfSrc = document.currentScript && document.currentScript.src;
  const base = selfSrc ? selfSrc.replace(/[^/]*$/, '') : 'xphfbw://runtime/';
  const RUNTIME_FILES = ['msfs/coherent.js', 'msfs/simvar.js', 'msfs/environment.js', 'msfs/instrument.js'];
  for (const file of RUNTIME_FILES) {
    document.write(`<script src="${base}${file}"><\/script>`);
  }

  // ------------------------------------------------------------------
  // The tick loop: one requestAnimationFrame callback, registered before
  // any instrument's own (BaseInstrument.createMainLoop() only runs once
  // its element connects, well after this), so it is always first in each
  // frame's callback queue (rAF callbacks run in registration order; this
  // one re-registers itself as the very first thing it does, before any
  // instrument's own callback for the same frame gets a chance to run and
  // register the *next* frame's).
  //
  // Per tick: take the seqlock snapshot every getVar/getString this frame
  // will read from (docs/briefs/xphfbw-js-bridge.md rule 1 — getVar would
  // take one lazily anyway, but every instrument in this view should still
  // see one consistent frame, taken once), then drain this view's downlink
  // and turn it back into the same Coherent events the QuickJS port
  // delivered through __msfsDeliver.
  // ------------------------------------------------------------------
  const drainPoll = () => {
    let items;
    try {
      items = native.poll();
    } catch (e) {
      native.log(2, `poll(): ${e && e.stack ? e.stack : e}`);
      return;
    }
    for (const item of items) {
      const [kind, name, payload] = item;
      switch (kind) {
        case 'h':
          // A cockpit click/command (H: event): every instrument's
          // onInteractionEvent hears it, target '' (broadcast), same shape
          // src/js/msfs/mod.rs's `h_event()` delivered.
          globalThis.__coherentDeliver('OnInteractionEvent', ['', [name]]);
          break;
        case 'provider': {
          // A named event the plugin is fanning out on behalf of another
          // view or one of its own providers (navdata, mapdata, wxr, sound,
          // the cross-view GENERIC_DATA/HTML_EVENT/TO_ALL_SUBSCRIBERS
          // triggers coherent.js sends via `call`). `payload` is that
          // event's JSON-encoded argument array.
          let args = [];
          if (payload) {
            try {
              args = JSON.parse(payload);
            } catch {
              args = [payload];
            }
          }
          globalThis.__coherentDeliver(name, args);
          break;
        }
        case 'game':
          // GAME: string values are cached and served by `gameString()`
          // itself (agent E); nothing else here currently reacts to a raw
          // change notification, so this is only logged once as a marker
          // that the path is live. Flagged for agent H: if some FBW code
          // ends up needing a *push* rather than gameString()'s pull/cache,
          // this is the place to add it.
          // Already cached by the native for gameString(); nothing to do.
          break;
        case 'string':
          // A string variable changed under another view (StringValue
          // downlink record). The one documented cross-view consumer for
          // this shape is FlyByWire's own datastore sync
          // (Coherent.on('FBW_NXDATASTORE_UPDATE', key => ...),
          // msfs-interface.md Layer 5.2); re-deliver it as that event.
          // Flagged for agent H to confirm against the real plugin-side
          // producer once xphfbw_host.rs exists.
          globalThis.__coherentDeliver('FBW_NXDATASTORE_UPDATE', [name]);
          break;
        default:
          once(`poll() item of kind "${kind}"`);
      }
    }
  };

  const tick = () => {
    requestAnimationFrame(tick);
    try {
      native.snapshot();
    } catch (e) {
      native.log(2, `snapshot(): ${e && e.stack ? e.stack : e}`);
    }
    drainPoll();
  };
  requestAnimationFrame(tick);
})();
