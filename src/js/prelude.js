// The host environment scripts expect, built on the few functions the
// plugin gives QuickJS (globalThis.__host). Time is the simulator's: timers
// and animation frames advance when the plugin ticks the engine.
(() => {
  const host = globalThis.__host;
  let now = 0;
  let nextId = 1;
  const timers = new Map();
  const frames = new Map();

  const describe = (value) => {
    if (typeof value === 'string') return value;
    if (value instanceof Error) return value.stack ? `${value}\n${value.stack}` : String(value);
    try {
      const json = JSON.stringify(value);
      return json === undefined ? String(value) : json;
    } catch {
      return String(value);
    }
  };
  const format = (args) => args.map(describe).join(' ');

  globalThis.console = {
    log: (...a) => host.log(0, format(a)),
    info: (...a) => host.log(0, format(a)),
    debug: (...a) => host.log(0, format(a)),
    warn: (...a) => host.log(1, format(a)),
    error: (...a) => host.log(2, format(a)),
  };

  globalThis.setTimeout = (fn, ms = 0, ...args) => {
    const id = nextId++;
    timers.set(id, { at: now + Math.max(0, Number(ms) || 0), fn, args, every: null });
    return id;
  };
  globalThis.setInterval = (fn, ms = 0, ...args) => {
    const id = nextId++;
    const every = Math.max(1, Number(ms) || 0);
    timers.set(id, { at: now + every, fn, args, every });
    return id;
  };
  globalThis.clearTimeout = (id) => { timers.delete(id); };
  globalThis.clearInterval = (id) => { timers.delete(id); };
  globalThis.requestAnimationFrame = (fn) => {
    const id = nextId++;
    frames.set(id, fn);
    return id;
  };
  globalThis.cancelAnimationFrame = (id) => { frames.delete(id); };
  globalThis.queueMicrotask = (fn) => { Promise.resolve().then(fn); };

  globalThis.performance = { now: () => now };

  // MSFS's variable API, as instruments call it.
  globalThis.SimVar = {
    GetSimVarValue: (name, unit) => host.getVar(String(name), unit === undefined ? '' : String(unit)),
    SetSimVarValue: (name, unit, value) => {
      host.setVar(String(name), unit === undefined ? '' : String(unit), Number(value));
      return Promise.resolve();
    },
  };

  globalThis.__tick = (ms) => {
    now = ms;
    const due = [...timers.entries()].filter(([, t]) => t.at <= now).sort((a, b) => a[1].at - b[1].at);
    for (const [id, t] of due) {
      if (!timers.has(id)) continue;
      if (t.every === null) {
        timers.delete(id);
      } else {
        t.at += t.every;
        if (t.at <= now) t.at = now + t.every;
      }
      try {
        t.fn(...t.args);
      } catch (e) {
        console.error(e);
      }
    }
    const pending = [...frames.values()];
    frames.clear();
    for (const fn of pending) {
      try {
        fn(now);
      } catch (e) {
        console.error(e);
      }
    }
  };
})();
