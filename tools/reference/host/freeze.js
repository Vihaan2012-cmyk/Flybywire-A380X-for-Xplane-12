// Determinism: a virtual clock, installed before any page script.
//
// Date, performance.now, setTimeout/setInterval, requestAnimationFrame and
// requestIdleCallback all run on simulated time that only moves when the
// runner calls __refStep(ms). One step is one simulator frame: due timers run
// in time order (each followed by a real macrotask, so promise chains settle
// between callbacks), then the frame's animation callbacks run. Playwright's
// page.clock is not used: it yields a clamped real setTimeout after every
// fake timer, which makes an FSComponent instrument take minutes per
// simulated frame.
//
// CSS animations and transitions run on the compositor's real clock. After
// every step the runner calls __refNoteAnimations(), which stamps each new
// Animation with the virtual time; __refFreezeAnimations() pins each one to
// "virtual now - stamp" before the screenshot, the same phase every run.
(function () {
  'use strict';
  const cfg = window.__REFERENCE__ || {};
  const RealDate = Date;
  const realPerfNow = performance.now.bind(performance);
  const start = typeof cfg.clockStart === 'number' ? cfg.clockStart : RealDate.now();
  const perfOrigin = realPerfNow();
  let now = start;

  function FakeDate(...args) {
    if (!new.target) return new RealDate(now).toString();
    return args.length ? new RealDate(...args) : new RealDate(now);
  }
  FakeDate.prototype = RealDate.prototype;
  FakeDate.now = () => now;
  FakeDate.UTC = RealDate.UTC;
  FakeDate.parse = RealDate.parse;
  Object.defineProperty(FakeDate.prototype, 'constructor', { value: FakeDate, writable: true, configurable: true });
  window.Date = FakeDate;
  // Seeded, so random boot delays (CdsDisplayUnit.tsx setTimer) repeat.
  let seed = 0x2f6b3a1d;
  Math.random = () => {
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  performance.now = () => perfOrigin + (now - start);

  // Timers, ordered by due time then creation.
  let nextId = 1;
  const timers = new Map(); // id -> {id, at, fn, args, interval}
  const frames = new Map(); // id -> fn (rAF)
  const idles = new Map();
  const call = (fn, args) => {
    if (typeof fn === 'function') fn.apply(window, args);
    else (0, eval)(String(fn));
  };
  window.setTimeout = (fn, delay, ...args) => {
    const id = nextId++;
    timers.set(id, { id, at: now + Math.max(0, Number(delay) || 0), fn, args, interval: null });
    return id;
  };
  window.setInterval = (fn, delay, ...args) => {
    const id = nextId++;
    const interval = Math.max(1, Number(delay) || 0);
    timers.set(id, { id, at: now + interval, fn, args, interval });
    return id;
  };
  window.clearTimeout = window.clearInterval = (id) => {
    timers.delete(id);
  };
  window.requestAnimationFrame = (fn) => {
    const id = nextId++;
    frames.set(id, fn);
    return id;
  };
  window.cancelAnimationFrame = (id) => {
    frames.delete(id);
  };
  window.requestIdleCallback = (fn) => {
    const id = nextId++;
    idles.set(id, fn);
    return id;
  };
  window.cancelIdleCallback = (id) => {
    idles.delete(id);
  };

  // A real macrotask: lets microtasks, fetches and script loads progress.
  const channel = new MessageChannel();
  const waiting = [];
  channel.port1.onmessage = () => waiting.shift()();
  const yieldTask = () =>
    new Promise((resolve) => {
      waiting.push(resolve);
      channel.port2.postMessage(0);
    });

  const errors = [];
  const guard = (fn, args) => {
    try {
      call(fn, args);
    } catch (e) {
      errors.push(String(e && e.stack ? e.stack : e));
      console.error(e);
    }
  };

  let busy = false;
  const stats = { steps: 0, timers: 0, runaway: 0 };
  window.__refStep = async (ms) => {
    if (busy) throw new Error('__refStep re-entered');
    busy = true;
    try {
      const target = now + ms;
      // A zero-delay timer that re-arms itself never lets time move; cap the
      // number of timers one step may run.
      let budget = 20000;
      for (;;) {
        let first = null;
        for (const t of timers.values()) {
          if (t.at <= target && (!first || t.at < first.at || (t.at === first.at && t.id < first.id))) first = t;
        }
        if (!first) break;
        if (--budget < 0) {
          stats.runaway++;
          break;
        }
        now = Math.max(now, first.at);
        if (first.interval) first.at += first.interval;
        else timers.delete(first.id);
        stats.timers++;
        guard(first.fn, first.args);
        await yieldTask();
      }
      now = target;
      const due = [...frames.values()];
      frames.clear();
      const ts = performance.now();
      for (const fn of due) guard(fn, [ts]);
      await yieldTask();
      const idle = [...idles.values()];
      idles.clear();
      for (const fn of idle) guard(fn, [{ didTimeout: false, timeRemaining: () => 0 }]);
      stats.steps++;
      await yieldTask();
    } finally {
      busy = false;
    }
  };
  // Lets real-time work (script and font loading) progress without moving the clock.
  window.__refIdle = async (n = 5) => {
    for (let i = 0; i < n; i++) await yieldTask();
  };
  window.__refClockStats = () => ({ now, start, pendingTimers: timers.size, pendingFrames: frames.size, errors: errors.slice(0, 50), ...stats });

  const starts = new WeakMap();
  const usage = new Map();
  window.__refNoteAnimations = () => {
    for (const a of document.getAnimations()) {
      if (!starts.has(a)) {
        starts.set(a, now);
        const key = `${a.constructor.name}:${a.animationName || a.transitionProperty || ''}`;
        usage.set(key, (usage.get(key) || 0) + 1);
      }
    }
  };
  window.__refFreezeAnimations = () => {
    window.__refNoteAnimations();
    let n = 0;
    for (const a of document.getAnimations()) {
      try {
        a.pause();
        a.currentTime = Math.max(0, now - starts.get(a));
        n++;
      } catch (e) {
        /* an animation without a timeline cannot be pinned */
      }
    }
    return n;
  };
  window.__refAnimationUsage = () => Object.fromEntries(usage);
})();
