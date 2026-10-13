// Coherent: the bridge between an MSFS view's JavaScript and the simulator.
// Ported from src/js/msfs/coherent.js (our QuickJS shim, already verified
// against FlyByWire's JS) to run on top of `window.__xphfbw` in a real
// Chromium page instead of our own in-process "plugin simulates MSFS"
// engine. See docs/briefs/xphfbw-js-bridge.md section F for the contract
// and docs/briefs/msfs-interface.md Layer 5 for what FBW expects here.
//
// Coherent.on/off register handlers for events the simulator (or another
// view) sends this view; Coherent.trigger sends an event, to this view's own
// handlers first and then to the simulator; Coherent.call asks the simulator
// for something and returns a Promise.
//
// What changed from the QuickJS shim, and why:
//  - `host.call` used to return a synchronous "<status><payload>" string
//    (0=resolved, 1=rejected, 2=pending-id), with late replies delivered
//    through `__msfsDeliver`'s ["resolve", id, ok, value] items. XPHFBW's
//    `window.__xphfbw.call()` is a real Promise, so that whole waiting-map
//    dance is gone; `Coherent.call` just returns it (wrapped to keep this
//    layer's own error semantics, e.g. an empty string decodes to undefined).
//  - `host.trigger` had no synchronous void-returning native to call
//    anymore (the __xphfbw contract only has `call`, not a fire-and-forget
//    `trigger`); triggers that must reach the simulator go through
//    `__xphfbw.call()` too, firing it and not waiting on the answer. The
//    cross-view fan-out these triggers used to get for free by running in
//    the plugin's own in-process "Shared" router (src/js/msfs/mod.rs) is now
//    the plugin's xphfbw_host.rs job (agent C): it re-delivers provider
//    events to every view's downlink, which `msfs-runtime.js`'s poll loop
//    turns back into local `emit()`s here (kind "provider").
//  - ADD_VIEW_LISTENER/REMOVE_VIEW_LISTENER and ALL_INSTRUMENTS_LOADED never
//    left the *same* view in the old plugin-side router either (mod.rs's
//    `ViewHost::trigger` pushed their acks with `To::View(view)`, i.e. back
//    to the sender only) — so here they are answered locally, with no round
//    trip to the plugin at all. This is behaviour-preserving, not a
//    shortcut: it only removes a hop that never crossed a view boundary.
//  - There is no `__msfsDeliver` anymore; `msfs-runtime.js`'s tick loop
//    drains `__xphfbw.poll()` each frame and calls `__coherentDeliver`
//    (exposed below) to re-emit those items as Coherent events.
(() => {
  const host = globalThis.__host;
  const handlers = new Map();
  const reported = new Set();

  const once = (what) => {
    if (!reported.has(what)) {
      reported.add(what);
      host.log(1, `MSFS runtime: ${what} is not supported here`);
    }
  };

  const emit = (name, args) => {
    const list = handlers.get(name);
    if (list === undefined || list.length === 0) {
      return;
    }
    for (const entry of list.slice()) {
      if (!entry.alive) {
        continue;
      }
      try {
        entry.fn.apply(entry.context, args);
      } catch (e) {
        console.error(e);
      }
    }
  };
  // The tick loop (msfs-runtime.js) re-emits what `__xphfbw.poll()` drains
  // (kinds "h" and "provider") through this, outside the IIFE's closure.
  globalThis.__coherentDeliver = emit;

  const remove = (name, entry) => {
    entry.alive = false;
    const list = handlers.get(name);
    if (list !== undefined) {
      const i = list.indexOf(entry);
      if (i >= 0) {
        list.splice(i, 1);
      }
    }
  };

  // Arguments cross to the plugin as JSON; undefined becomes null, as it
  // does crossing into Coherent's C++.
  const encode = (args) => JSON.stringify(args, (_k, v) => (v === undefined ? null : v));

  // Calls this view answers itself: variable writes, which reach the
  // plugin's variables directly through the bridge's snapshot/uplink, not
  // through a round trip.
  const localCalls = Object.assign(Object.create(null), {
    setValueReg_Number: (id, value) => host.setReg(Number(id), Number(value)),
    setValueReg_Bool: (id, value) => host.setReg(Number(id), value ? 1 : 0),
    setValueReg_String: (id, value) => host.setRegString(Number(id), String(value)),
    setValue_Number: (name, unit, value) => host.setVar(String(name), String(unit), Number(value)),
    setValue_Bool: (name, value) => host.setVar(String(name), 'bool', value ? 1 : 0),
    setValue_String: (name, value) => host.setString(String(name), String(value)),
    getArrayValues: (count, index, names, units) => {
      const out = [];
      for (let i = 0; i < names.length; i++) {
        out.push(SimVar.GetSimVarValue(names[i], units[i]));
      }
      return out;
    },
    // Shared globals (JS_LISTENER_SHAREDGLOBAL, msfs-sdk SharedGlobal): in
    // MSFS one object seen by every view. Views here are separate browser
    // pages that cannot share an object (same reason as the QuickJS port:
    // separate engines/processes), so each view owns its own. FlyByWire
    // shares only its facility cache this way (navdata FacilityCache.ts),
    // which works the same per view, loading what it needs itself.
    CREATE_SHARED_GLOBAL: (name) => {
      const key = String(name);
      if (typeof globalThis[key] !== 'object' || globalThis[key] === null) {
        globalThis[key] = {};
        sharedGlobalAttached(key);
      }
    },
    REQUEST_SHARED_GLOBAL: (name) => {
      const key = String(name);
      if (typeof globalThis[key] === 'object' && globalThis[key] !== null) {
        sharedGlobalAttached(key);
      }
    },
  });
  const sharedGlobalAttached = (name) =>
    setTimeout(() => emit('EVENT_FROM_VIEW_LISTENER', ['JS_LISTENER_SHAREDGLOBAL', 'SharedGlobalAttached', name, true]), 0);

  // Flow events the simulator acts on, to the plugin's call names (agent C's
  // xphfbw_host.rs is expected to fan these out to the views they target).
  const flowEvents = {
    ON_MOUSERECT_HTMLEVENT: (...args) => host.trigger('HTML_EVENT', encode([args.map(String)])),
    ON_HTMLEVENT_TO_ALL_VIEWS: (...args) => host.trigger('HTML_EVENT', encode([args.map(String)])),
    ON_HTMLEVENT_TO_SPECIFIC_VIEW: (target, ...args) => host.trigger('HTML_EVENT_TO', encode([String(target), args.map(String)])),
    ON_HTMLEVENT_TO_MULTIPLE_VIEWS: (targets, ...args) => host.trigger('HTML_EVENT_TO', encode([String(targets), args.map(String)])),
    ON_VCOCKPIT_INSTRUMENT_INITIALIZED: (guid, identifier) => host.trigger('INSTRUMENT_INITIALIZED', encode([String(guid), String(identifier)])),
  };

  const listenerTriggers = {
    JS_LISTENER_GENERICDATA: {
      SEND: (key, json) => host.trigger('GENERIC_DATA', encode([String(key), String(json)])),
    },
  };

  const nativeTriggers = {
    // Same-view acks (see the file doc above): answered locally, one tick
    // later, exactly as mod.rs's ViewHost::trigger already only echoed them
    // back to the sending view.
    ADD_VIEW_LISTENER: (name) => setTimeout(() => emit('VIEW_LISTENER_REGISTERED', [String(name).toUpperCase()]), 0),
    REMOVE_VIEW_LISTENER: () => {},
    // instrument.js's bootstrap triggers this once this view's (single)
    // instrument has connected; mod.rs's "ALL_INSTRUMENTS_LOADED" handler
    // only ever echoed `OnAllInstrumentsLoaded` back to the sending view
    // too, so this never needed to cross a view boundary.
    ALL_INSTRUMENTS_LOADED: () => setTimeout(() => emit('OnAllInstrumentsLoaded', []), 0),
    EVENT_TO_VIEW_LISTENER: (listener, event, ...args) => {
      const table = listenerTriggers[String(listener).toUpperCase()];
      const fn = table && table[event];
      if (fn === undefined) {
        once(`${listener} event ${event}`);
        return;
      }
      fn(...args);
    },
    TRIGGER_EVENT_TO_ALL_SUBSCRIBERS: (listener, event, ...jsonArgs) =>
      host.trigger('TO_ALL_SUBSCRIBERS', encode([String(listener).toUpperCase(), String(event), ...jsonArgs.map(String)])),
    LAUNCH_FLOW_EVENT_FROM_VIEW: (event, ...args) => {
      const fn = flowEvents[event];
      if (fn === undefined) {
        once(`flow event ${event}`);
        return;
      }
      fn(...args);
    },
  };

  globalThis.Coherent = {
    isViewLoaded: true,
    isAttached: true,

    on(name, fn, context) {
      const entry = { fn, context, alive: true };
      let list = handlers.get(name);
      if (list === undefined) {
        list = [];
        handlers.set(name, list);
      }
      list.push(entry);
      return { clear: () => remove(name, entry) };
    },

    off(name, fn, context) {
      const list = handlers.get(name);
      if (list === undefined) {
        return;
      }
      for (const entry of list.slice()) {
        if (entry.fn === fn && (context === undefined || entry.context === context)) {
          remove(name, entry);
        }
      }
    },

    trigger(name, ...args) {
      emit(name, args);
      const fn = nativeTriggers[name];
      if (fn !== undefined) {
        fn(...args);
      } else {
        host.trigger(String(name), encode(args));
      }
    },

    call(name, ...args) {
      const fn = localCalls[name];
      if (fn !== undefined) {
        try {
          return Promise.resolve(fn(...args));
        } catch (e) {
          return Promise.reject(e);
        }
      }
      return host.call(String(name), encode(args)).then((text) => (text === '' || text === undefined ? undefined : JSON.parse(text)));
    },
  };
})();
