// Coherent: the bridge between an MSFS view's JavaScript and the simulator.
//
// Coherent.on/off register handlers for events the simulator (or another
// view) sends this view; Coherent.trigger sends an event, to this view's own
// handlers first and then to the simulator; Coherent.call asks the simulator
// for something and returns a Promise.
//
// The simulator side is the plugin's (src/js/msfs/mod.rs): calls go to
// __host.call, which answers now, rejects with the reason (a call nothing
// answers is rejected, never left hanging), or answers on a later tick;
// triggers the simulator acts on go to __host.trigger under the names below.
// The plugin delivers events and late answers through __msfsDeliver(batch).
(() => {
  const host = globalThis.__host;
  const handlers = new Map();
  const reported = new Set();
  const waiting = new Map();

  const once = (what) => {
    if (!reported.has(what)) {
      reported.add(what);
      console.warn(`MSFS runtime: ${what} is not supported here`);
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
  // plugin's variables directly.
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
    // MSFS one object seen by every view. Views here are separate engines
    // that cannot share an object, so each view owns its own. FlyByWire
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

  // Flow events the simulator acts on, to the plugin's trigger names.
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
    ADD_VIEW_LISTENER: (name) => host.trigger('ADD_VIEW_LISTENER', encode([String(name).toUpperCase()])),
    REMOVE_VIEW_LISTENER: () => {},
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
      return new Promise((resolve, reject) => {
        try {
          const fn = localCalls[name];
          if (fn !== undefined) {
            resolve(fn(...args));
            return;
          }
          const reply = host.call(String(name), encode(args));
          const status = reply[0];
          const payload = reply.slice(1);
          if (status === '0') {
            resolve(payload === '' ? undefined : JSON.parse(payload));
          } else if (status === '2') {
            waiting.set(Number(payload), { resolve, reject });
          } else {
            reject(new Error(payload));
          }
        } catch (e) {
          reject(e);
        }
      });
    },
  };

  // Deliveries from the plugin: ["event", name, args], ["resolve", id, ok,
  // value] and ["screen", screen, kind, x, y, button, delta] (mouse input
  // for the DOM's __screenEvent).
  globalThis.__msfsDeliver = (batchJson) => {
    for (const item of JSON.parse(batchJson)) {
      if (item[0] === 'event') {
        emit(item[1], item[2]);
      } else if (item[0] === 'screen') {
        if (typeof globalThis.__screenEvent === 'function') {
          try {
            globalThis.__screenEvent(item[1], item[2], item[3], item[4], item[5], item[6]);
          } catch (e) {
            console.error(e);
          }
        }
      } else if (item[0] === 'resolve') {
        const call = waiting.get(item[1]);
        if (call !== undefined) {
          waiting.delete(item[1]);
          if (item[2]) {
            call.resolve(item[3]);
          } else {
            call.reject(new Error(item[3]));
          }
        }
      }
    }
  };
})();
