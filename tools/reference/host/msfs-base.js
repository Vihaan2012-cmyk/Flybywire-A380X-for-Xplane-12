// The MSFS VCockpit environment FBW's instruments expect, reimplemented for
// a plain Chromium page. Behaviour follows MSFS's documented/observable
// host API (FBW's typings in fbw-common/src/typings/fs-base-ui, the
// msfs-sdk's use of the native `simvar` binding, and the VCockpit panel
// lifecycle); no Asobo source is copied.
//
// Configuration comes from window.__REFERENCE__ (injected by
// src/reference.mjs before any page script runs):
//   { screen: {name, width, height, gauges:[{url,x,y,w,h}]},
//     state: {vars: {KEY: {value, unit}}, storage: {...}, time: {...}} }
(function () {
  'use strict';

  const CFG = window.__REFERENCE__ || { screen: null, state: { vars: {}, storage: {} } };
  const units = window.__refUnits;

  // ---------------------------------------------------------------------------
  // Access log: every variable, storage key, Coherent call and view listener
  // the instruments touch. Read back by reference.mjs after the run.
  // ---------------------------------------------------------------------------
  const log = {
    reads: new Map(), // key -> {key, units:Set, count, found, writtenBefore, value}
    writes: new Map(), // key -> {key, units:Set, count, last}
    unitMismatch: new Map(),
    storage: new Map(), // key -> {count, found}
    storageWrites: new Map(),
    coherentCalls: new Map(),
    coherentOn: new Map(),
    coherentTriggers: new Map(),
    viewListeners: new Map(),
    keyEvents: new Map(),
    gameVars: new Map(),
    errors: [],
    missingGlobals: new Map(),
  };
  const bump = (map, key, extra) => {
    let e = map.get(key);
    if (!e) {
      e = Object.assign({ key, count: 0 }, extra || {});
      map.set(key, e);
    }
    e.count++;
    return e;
  };

  // ---------------------------------------------------------------------------
  // Variable store
  // ---------------------------------------------------------------------------
  // Keys are normalised: "L:NAME", "A:NAME[:index]", "E:NAME", "GAME:NAME".
  // A-vars without a prefix are A-vars. A/E names are case-insensitive in MSFS.
  function normKey(name) {
    let s = String(name).trim();
    const m = s.match(/^([A-Za-z]{1,4}):(.*)$/);
    let prefix = 'A';
    if (m && (m[1].length === 1 || m[1].toUpperCase() === 'GAME')) {
      prefix = m[1].toUpperCase();
      s = m[2];
    }
    if (prefix === 'A' || prefix === 'E') s = s.toUpperCase().replace(/\s+/g, ' ');
    return `${prefix}:${s}`;
  }

  const store = new Map(); // key -> {value, unit, fromState}
  const storeLower = new Map();
  for (const [k, v] of Object.entries((CFG.state && CFG.state.vars) || {})) {
    const key = normKey(k);
    store.set(key, { value: v.value, unit: v.unit ?? '', fromState: true });
    storeLower.set(key.toLowerCase(), key);
  }

  function lookupVar(key) {
    let e = store.get(key);
    if (!e) {
      const alt = storeLower.get(key.toLowerCase());
      if (alt) e = store.get(alt);
    }
    // "A:NAME" is the same variable as "A:NAME:1" for indexed A-vars? No: MSFS
    // treats a missing index as index 0/default. Keep exact keys only.
    return e;
  }

  function readVar(name, unit) {
    const key = normKey(name);
    const e = lookupVar(key);
    const entry = bump(log.reads, key, { units: new Set(), found: false, fromState: false, writtenBefore: false });
    entry.units.add(String(unit ?? ''));
    if (!e) {
      return undefined;
    }
    if (e.fromState) entry.fromState = true;
    else entry.writtenBefore = true;
    entry.found = true;
    if (typeof e.value !== 'number') return e.value;
    const r = units.convert(e.value, e.unit, unit);
    if (!r.ok) bump(log.unitMismatch, `${key} [${e.unit} -> ${unit}]`);
    return r.value;
  }

  function writeVar(name, unit, value) {
    const key = normKey(name);
    const w = bump(log.writes, key, { units: new Set() });
    w.units.add(String(unit ?? ''));
    w.last = value;
    if (key.startsWith('K:')) {
      bump(log.keyEvents, key.slice(2));
      return;
    }
    let stored = value;
    if (typeof value === 'boolean') stored = value ? 1 : 0;
    store.set(key, { value: stored, unit: String(unit ?? ''), fromState: false });
    storeLower.set(key.toLowerCase(), key);
  }

  // ---------------------------------------------------------------------------
  // Geometry value types MSFS provides globally (Types.js).
  // ---------------------------------------------------------------------------
  class LatLong {
    constructor(lat = 0, long = 0) {
      if (typeof lat === 'object' && lat) {
        this.lat = lat.lat;
        this.long = lat.long;
      } else {
        this.lat = lat;
        this.long = long;
      }
    }
    toStringFloat() {
      return `${this.lat}, ${this.long}`;
    }
    toString() {
      return this.toStringFloat();
    }
  }
  class LatLongAlt {
    constructor(lat = 0, long = 0, alt = 0) {
      if (typeof lat === 'object' && lat) {
        this.lat = lat.lat;
        this.long = lat.long;
        this.alt = lat.alt;
      } else {
        this.lat = lat;
        this.long = long;
        this.alt = alt;
      }
    }
    toLatLong() {
      return new LatLong(this.lat, this.long);
    }
  }
  class PitchBankHeading {
    constructor(p = 0, b = 0, h = 0) {
      if (typeof p === 'object' && p) Object.assign(this, { pitchDegree: p.pitchDegree, bankDegree: p.bankDegree, headingDegree: p.headingDegree });
      else Object.assign(this, { pitchDegree: p, bankDegree: b, headingDegree: h });
    }
  }
  class LatLongAltPBH {
    constructor(lla, pbh) {
      if (lla && lla.lla) {
        this.lla = new LatLongAlt(lla.lla);
        this.pbh = new PitchBankHeading(lla.pbh);
      } else {
        this.lla = new LatLongAlt(lla);
        this.pbh = new PitchBankHeading(pbh);
      }
    }
  }
  class PID_STRUCT {
    constructor(o) {
      Object.assign(this, o || {});
    }
  }
  class XYZ {
    constructor(x = 0, y = 0, z = 0) {
      if (typeof x === 'object' && x) Object.assign(this, { x: x.x, y: x.y, z: x.z });
      else Object.assign(this, { x, y, z });
    }
  }
  class Vec2 {
    constructor(x = 0, y = 0) {
      this.x = x;
      this.y = y;
    }
  }
  class Vec3 {
    constructor(x = 0, y = 0, z = 0) {
      this.x = x;
      this.y = y;
      this.z = z;
    }
  }
  Object.assign(window, { LatLong, LatLongAlt, PitchBankHeading, LatLongAltPBH, PID_STRUCT, XYZ, Vec2, Vec3 });

  // ---------------------------------------------------------------------------
  // The native `simvar` binding and the SimVar namespace.
  // ---------------------------------------------------------------------------
  const registered = []; // id -> {name, unit, dataSource}
  const registeredIndex = new Map();
  function register(name, unit, dataSource) {
    const k = `${name}|${unit}|${dataSource ?? ''}`;
    let id = registeredIndex.get(k);
    if (id === undefined) {
      id = registered.length;
      registered.push({ name: String(name), unit: String(unit ?? ''), dataSource: dataSource ?? '' });
      registeredIndex.set(k, id);
    }
    return id;
  }

  const asNumber = (v) => {
    if (v === undefined || v === null) return 0;
    if (typeof v === 'number') return v;
    if (typeof v === 'boolean') return v ? 1 : 0;
    const n = Number(v);
    return Number.isFinite(n) ? n : 0;
  };
  const structOf = (name, Ctor, fallback) => {
    const v = readVar(name, 'struct');
    return v && typeof v === 'object' ? v : fallback;
  };

  window.simvar = {
    getValueReg: (id) => asNumber(readVar(registered[id].name, registered[id].unit)),
    getValueReg_String: (id) => {
      const v = readVar(registered[id].name, registered[id].unit);
      return v === undefined || v === null ? '' : String(v);
    },
    getValue: (name, unit) => asNumber(readVar(name, unit)),
    getValue_String: (name) => {
      const v = readVar(name, 'string');
      return v === undefined || v === null ? '' : String(v);
    },
    getValue_LatLongAlt: (name) => structOf(name, LatLongAlt, { lat: 0, long: 0, alt: 0 }),
    getValue_LatLongAltPBH: (name) =>
      structOf(name, LatLongAltPBH, { lla: { lat: 0, long: 0, alt: 0 }, pbh: { pitchDegree: 0, bankDegree: 0, headingDegree: 0 } }),
    getValue_PBH: (name) => structOf(name, PitchBankHeading, { pitchDegree: 0, bankDegree: 0, headingDegree: 0 }),
    getValue_PID_STRUCT: (name) => structOf(name, PID_STRUCT, {}),
    getValue_XYZ: (name) => structOf(name, XYZ, { x: 0, y: 0, z: 0 }),
  };

  const SimVar = (window.SimVar = window.SimVar || {});
  SimVar.IsReady = () => true;
  SimVar.GetRegisteredId = (name, unit, dataSource) => register(name, unit, dataSource);
  SimVar.GetSimVarValue = (name, unit, dataSource) => {
    const u = String(unit ?? '');
    if (/latlonaltpbh/i.test(u)) return new LatLongAltPBH(window.simvar.getValue_LatLongAltPBH(name));
    if (/latlonalt/i.test(u)) return new LatLongAlt(window.simvar.getValue_LatLongAlt(name));
    if (/^pbh$/i.test(u)) return new PitchBankHeading(window.simvar.getValue_PBH(name));
    if (/pid_struct/i.test(u)) return new PID_STRUCT(window.simvar.getValue_PID_STRUCT(name));
    if (/^xyz$/i.test(u)) return new XYZ(window.simvar.getValue_XYZ(name));
    const id = register(name, unit, dataSource);
    if (/string/i.test(u)) return window.simvar.getValueReg_String(id);
    return window.simvar.getValueReg(id);
  };
  SimVar.GetSimVarValueFast = SimVar.GetSimVarValue;
  SimVar.GetSimVarValueFastReg = (id) => window.simvar.getValueReg(id);
  SimVar.GetSimVarValueFastRegString = (id) => window.simvar.getValueReg_String(id);
  SimVar.SetSimVarValue = (name, unit, value) => {
    if (value === undefined || value === null) return Promise.resolve();
    writeVar(name, unit, value);
    return Promise.resolve();
  };
  SimVar.GetGlobalVarValue = (name, unit) => asNumber(readVar(`GAME:${name}`, unit));
  SimVar.GetGameVarValue = (name, unit) => {
    bump(log.gameVars, `${name} [${unit}]`);
    const v = readVar(`GAME:${name}`, unit);
    return /string/i.test(String(unit)) ? (v ?? '') : asNumber(v);
  };
  SimVar.GetGameVarValueFast = SimVar.GetGameVarValue;
  SimVar.GetRegisteredGameVarId = (name, unit) => register(`GAME:${name}`, unit, '');
  SimVar.GetGameVarValueFastReg = (id) => window.simvar.getValueReg(id);
  SimVar.SetGameVarValue = (name, unit, value) => {
    writeVar(`GAME:${name}`, unit, value);
    return Promise.resolve();
  };
  SimVar.SimVarBatch = class SimVarBatch {
    constructor(count, index) {
      this.simVarCount = count;
      this.simVarIndex = index;
      this.wantedNames = [];
      this.wantedUnits = [];
      this.wantedTypes = [];
    }
    add(name, unit, type = 'number') {
      this.wantedNames.push(name);
      this.wantedUnits.push(unit);
      this.wantedTypes.push(type);
    }
    getCount() {
      return this.simVarCount;
    }
    getIndex() {
      return this.simVarIndex;
    }
    getNames() {
      return this.wantedNames;
    }
    getUnits() {
      return this.wantedUnits;
    }
    getTypes() {
      return this.wantedTypes;
    }
  };
  SimVar.GetSimVarArrayValues = (batch, callback) => {
    const n = asNumber(readVar(batch.getCount(), 'number'));
    const out = [];
    for (let i = 0; i < n; i++) {
      out.push(batch.getNames().map((nm, j) => SimVar.GetSimVarValue(`${nm}:${i}`, batch.getUnits()[j])));
    }
    setTimeout(() => callback(out), 0);
  };
  SimVar.LogSimVarValueHistory = () => {};
  SimVar.LogSimVarValueHistoryByTimePerFrame = () => {};

  // ---------------------------------------------------------------------------
  // Coherent: event emitter plus async calls into the (absent) native side.
  // ---------------------------------------------------------------------------
  const handlers = new Map();
  const callHandlers = new Map();
  const Coherent = (window.Coherent = {
    isAttached: false,
    IsAttached: false,
    forceEnableMocking: false,
    on(name, callback, context) {
      bump(log.coherentOn, name);
      if (!handlers.has(name)) handlers.set(name, []);
      const h = { callback, context };
      handlers.get(name).push(h);
      return {
        clear: () => {
          const list = handlers.get(name) || [];
          const i = list.indexOf(h);
          if (i >= 0) list.splice(i, 1);
        },
      };
    },
    off(name, callback, context) {
      const list = handlers.get(name) || [];
      const i = list.findIndex((h) => h.callback === callback && (context === undefined || h.context === context));
      if (i >= 0) list.splice(i, 1);
    },
    trigger(name, ...args) {
      bump(log.coherentTriggers, name);
      for (const h of [...(handlers.get(name) || [])]) {
        try {
          h.callback.apply(h.context, args);
        } catch (e) {
          console.error(e);
        }
      }
    },
    call(name, ...args) {
      bump(log.coherentCalls, name);
      const h = callHandlers.get(name);
      try {
        return Promise.resolve(h ? h(...args) : undefined);
      } catch (e) {
        return Promise.reject(e);
      }
    },
    mock() {},
    translate: (t) => t,
    reloadLocalization() {},
    showOverlay() {},
    hideOverlay() {},
    events: {},
  });
  window.engine = Object.assign(Coherent, {
    beginProfileEvent() {},
    endProfileEvent() {},
  });

  // Calls the native side answers. Setters write the store, the rest resolve
  // with nothing (no navdata, no weather, no map rendering in the reference).
  callHandlers.set('setValueReg_Number', (id, v) => writeVar(registered[id].name, registered[id].unit, Number(v)));
  callHandlers.set('setValueReg_Bool', (id, v) => writeVar(registered[id].name, registered[id].unit, v ? 1 : 0));
  callHandlers.set('setValueReg_String', (id, v) => writeVar(registered[id].name, registered[id].unit, String(v)));
  for (const n of ['setValue_LatLongAlt', 'setValue_LatLongAltPBH', 'setValue_PBH', 'setValue_PID_STRUCT', 'setValue_XYZ']) {
    callHandlers.set(n, (name, v) => writeVar(name, 'struct', v));
  }
  callHandlers.set('TRIGGER_KEY_EVENT', (key) => {
    bump(log.keyEvents, String(key));
  });
  callHandlers.set('INTERCEPT_KEY_EVENT', () => undefined);
  callHandlers.set('PLAY_INSTRUMENT_SOUND', () => undefined);

  window.LaunchFlowEvent = (...args) => bump(log.coherentTriggers, `FlowEvent:${args[0]}`);

  // ---------------------------------------------------------------------------
  // View listeners (JS_LISTENER_*). MSFS calls the ready callback once the
  // native listener is bound, asynchronously.
  // ---------------------------------------------------------------------------
  class ViewListener {
    constructor(name) {
      this.name = name;
      this.subs = [];
      this.connected = true;
    }
    on(event, cb, ctx) {
      const sub = Coherent.on(event, cb, ctx);
      this.subs.push(sub);
      return sub;
    }
    off(event, cb, ctx) {
      Coherent.off(event, cb, ctx);
    }
    trigger(event, ...args) {
      bump(log.coherentTriggers, `${this.name}:${event}`);
    }
    triggerToAllSubscribers(event, ...args) {
      Coherent.trigger(event, ...args);
    }
    // JS_LISTENER_GENERICDATA: data other views send under a key. One page
    // holds one screen, so nothing arrives from other instruments.
    send(key, data) {
      bump(log.coherentTriggers, `${this.name}:send:${key}`);
    }
    call(name, ...args) {
      bump(log.coherentCalls, `${this.name}:${name}`);
      return Promise.resolve();
    }
    onDataReceived(key, cb) {
      bump(log.coherentOn, `${this.name}:onDataReceived:${key}`);
    }
    unregister() {
      this.subs.forEach((s) => s.clear());
      this.connected = false;
    }
  }
  window.ViewListener = { ViewListener };
  window.RegisterViewListener = (name, callback, requiresSingleton) => {
    bump(log.viewListeners, name);
    const l = new ViewListener(name);
    if (typeof callback === 'function') setTimeout(() => callback(), 0);
    return l;
  };
  window.RegisterGenericDataListener = (callback) => window.RegisterViewListener('JS_LISTENER_GENERICDATA', callback);

  // ---------------------------------------------------------------------------
  // Small globals from MSFS's common/avionics scripts that bundles call.
  // ---------------------------------------------------------------------------
  const kebab = (s) => s.replace(/[A-Z]/g, (c) => `-${c.toLowerCase()}`);
  window.StyleProperty = new Proxy({}, { get: (_, p) => (typeof p === 'string' ? kebab(p) : undefined) });
  window.diffAndSetAttribute = (el, name, value) => {
    if (el && el.getAttribute(name) !== String(value)) el.setAttribute(name, value);
  };
  window.diffAndSetStyle = (el, prop, value) => {
    if (el && el.style.getPropertyValue(prop) !== String(value)) el.style.setProperty(prop, value);
  };
  window.diffAndSetText = (el, text) => {
    if (el && el.textContent !== String(text)) el.textContent = text;
  };
  window.diffAndSetHTML = (el, html) => {
    if (el && el.innerHTML !== String(html)) el.innerHTML = html;
  };
  window.diffAndAddClass = (el, c) => el && el.classList.add(c);
  window.diffAndRemoveClass = (el, c) => el && el.classList.remove(c);
  window.fastToFixed = (v, n) => Number(v).toFixed(n);
  window.EmptyCallback = { Void: () => {}, Boolean: () => {}, Number: () => {}, String: () => {} };
  window.checkAutoload = () => {};
  window.EDITION_MODE = () => false;
  window.DEBUG = () => false;
  window.RELEASE = () => true;
  window.MASTER = () => true;
  window.SUBMISSION = () => false;
  window.GetUIEditionMode = () => '';
  window.UI_USE_DATA_FOLDER = () => false;
  window.bInitialized = true;

  const DEG2RAD = Math.PI / 180;
  window.Utils = {
    Clamp: (v, lo, hi) => Math.min(Math.max(v, lo), hi),
    Loop: (n, lo, hi) => {
      const range = hi - lo;
      return ((((n - lo) % range) + range) % range) + lo;
    },
    Translate: (key) => (key && key.startsWith('TT:') ? key.slice(3) : key),
    pad: (num, len) => String(num).padStart(len, '0'),
    isNumeric: (v) => !isNaN(parseFloat(v)) && isFinite(v),
    strToBool: (v) => String(v).toLowerCase() === 'true',
    DisplayTimeToSeconds: () => 0,
    RemoveAllChildren: (el) => {
      while (el && el.firstChild) el.removeChild(el.firstChild);
    },
    generateGUID: () => '00000000-0000-4000-8000-000000000000',
    SmoothPow: (o, d, f, dt) => o + (d - o) * Math.min(1, 1 - Math.pow(f, dt)),
    SmoothLinear: (o, d, f, dt) => o + Math.sign(d - o) * Math.min(Math.abs(d - o), f * dt),
    SmoothSin: (o, d) => d,
    formatNumber: (v) => String(v),
    formatInteger: (v) => String(Math.round(v)),
    inIframe: () => false,
    toArray: (a) => Array.from(a),
    getScreenRatio: () => window.innerWidth / window.innerHeight,
    getVirtualHeight: () => window.innerHeight,
    getVh: (p) => `${p}vh`,
    getSize: (px) => px,
    getVhNumber: (p) => (p * window.innerHeight) / 100,
    SecondsToDisplayTime: (s) => String(s),
    SecondsToDisplayDuration: (s) => String(s),
    timeToString: (h, m, s) => `${h}:${m}:${s}`,
  };

  window.Avionics = {
    SVG: { NS: 'http://www.w3.org/2000/svg' },
    Utils: {
      DEG2RAD,
      RAD2DEG: 180 / Math.PI,
      METER2NAV: 1 / 1852,
      NAV2METER: 1852,
      FEET2METER: 0.3048,
      METER2FEET: 1 / 0.3048,
      fmod: (a, b) => a - b * Math.floor(a / b),
      clampAngle: (a) => ((a % 360) + 360) % 360,
      diffAngle: (a, b) => {
        let d = b - a;
        while (d > 180) d -= 360;
        while (d <= -180) d += 360;
        return d;
      },
      computeGreatCircleHeading(from, to) {
        const lat1 = from.lat * DEG2RAD;
        const lat2 = to.lat * DEG2RAD;
        const dl = (to.long - from.long) * DEG2RAD;
        const y = Math.sin(dl) * Math.cos(lat2);
        const x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dl);
        return ((Math.atan2(y, x) / DEG2RAD) + 360) % 360;
      },
      computeGreatCircleDistance(from, to) {
        const lat1 = from.lat * DEG2RAD;
        const lat2 = to.lat * DEG2RAD;
        const dlat = lat2 - lat1;
        const dl = (to.long - from.long) * DEG2RAD;
        const a = Math.sin(dlat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dl / 2) ** 2;
        return (2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a)) * 6371008.8) / 1852;
      },
      make_bcd16: (v) => {
        let out = 0;
        let shift = 0;
        let n = Math.round(v);
        while (n > 0) {
          out |= (n % 10) << shift;
          n = Math.floor(n / 10);
          shift += 4;
        }
        return out;
      },
      make_xpndr_bcd: (v) => window.Avionics.Utils.make_bcd16(v),
    },
  };

  // Magnetic variation: the state provides it as A:MAGVAR at the aircraft;
  // the reference has no world magnetic model.
  window.Facilities = {
    getMagVar: (lat, lon) => {
      bump(log.coherentCalls, 'Facilities.getMagVar');
      return asNumber(readVar('A:MAGVAR', 'degrees'));
    },
  };

  window.Aircraft = { CJ4: 0, A320_NEO: 1, B747_8: 2, AS01B: 3, AS02A: 4 };
  window.RunwayDesignator = { RUNWAY_DESIGNATOR_NONE: 0, RUNWAY_DESIGNATOR_LEFT: 1, RUNWAY_DESIGNATOR_RIGHT: 2, RUNWAY_DESIGNATOR_CENTER: 3, RUNWAY_DESIGNATOR_WATER: 4, RUNWAY_DESIGNATOR_A: 5, RUNWAY_DESIGNATOR_B: 6 };
  window.EBingReference = { SEA: 0, PLANE: 1, AERIAL: 2 };
  window.EBingMode = { PLANE: 0, VFR: 1, CURSOR: 2, NONE: 3 };
  window.EWeatherRadar = { OFF: 0, TOPVIEW: 1, HORIZONTAL: 2, VERTICAL: 3 };

  // Simplane: getters over A-vars, the few the bundles reference.
  const sv = (n, u) => SimVar.GetSimVarValue(n, u);
  window.Simplane = {
    getPressureValue: (u = 'millibar') => sv('KOHLSMAN SETTING MB:1', u === 'inches of mercury' ? 'inches of mercury' : 'millibars'),
    getPressureSelectedUnits: () => (sv('L:XMLVAR_Baro_Selector_HPA_1', 'Bool') ? 'millibar' : 'inches of mercury'),
    // XMLVAR_Baro1_Mode: 0 QFE, 1 QNH, 2 STD (the A320/A380 FLT files set 1).
    getPressureSelectedMode: () => ['QFE', 'QNH'][sv('L:XMLVAR_Baro1_Mode', 'number')] ?? 'STD',
    getAutoPilotSelectedHeadingLockValue: () => sv('AUTOPILOT HEADING LOCK DIR', 'degrees'),
    getAutoPilotMachModeActive: () => !!sv('AUTOPILOT MANAGED SPEED IN MACH', 'bool'),
    getAutoPilotMachHoldValue: () => sv('AUTOPILOT MACH HOLD VAR', 'number'),
    getAutoPilotDisplayedAltitudeLockValue: () => sv('AUTOPILOT ALTITUDE LOCK VAR:3', 'feet'),
    getAutoPilotAirspeedSelected: () => !!sv('AUTOPILOT SPEED SLOT INDEX', 'number'),
    getAutoPilotAirspeedHoldValue: () => sv('AUTOPILOT AIRSPEED HOLD VAR', 'knots'),
    setTransponderToRegion: () => {},
  };

  // ---------------------------------------------------------------------------
  // Include: loads a gauge's html the way the VCockpit does: text/html script
  // templates go into the document, stylesheets are linked, import-script
  // entries run in order.
  // ---------------------------------------------------------------------------
  const loading = new Set();
  function loadScript(src) {
    return new Promise((resolve) => {
      loading.add(src);
      const s = document.createElement('script');
      s.src = src;
      s.async = false;
      s.onload = s.onerror = (e) => {
        loading.delete(src);
        if (e && e.type === 'error') log.errors.push(`script failed to load: ${src}`);
        resolve();
      };
      document.head.appendChild(s);
    });
  }
  function loadCss(href) {
    return new Promise((resolve) => {
      const l = document.createElement('link');
      l.rel = 'stylesheet';
      l.href = href;
      l.onload = l.onerror = () => resolve();
      document.head.appendChild(l);
    });
  }
  const imported = new Set();
  async function addImport(url) {
    const abs = new URL(url, location.href);
    const res = await fetch(abs.pathname);
    if (!res.ok) {
      log.errors.push(`gauge html not found: ${abs.pathname}`);
      return;
    }
    const html = await res.text();
    const doc = new DOMParser().parseFromString(html, 'text/html');
    const nodes = [...doc.querySelectorAll('script, link[rel="stylesheet"]')];
    for (const n of nodes) {
      if (n.tagName === 'LINK') {
        const href = new URL(n.getAttribute('href'), abs).pathname;
        if (!imported.has(href)) {
          imported.add(href);
          await loadCss(href);
        }
      } else if (n.hasAttribute('import-script')) {
        const src = new URL(n.getAttribute('import-script'), abs).pathname;
        if (!imported.has(src)) {
          imported.add(src);
          await loadScript(src);
        }
      } else if (n.getAttribute('type') === 'text/html' && n.id) {
        if (!document.getElementById(n.id)) {
          const t = document.createElement('script');
          t.type = 'text/html';
          t.id = n.id;
          t.innerHTML = n.innerHTML;
          document.body.appendChild(t);
        }
      } else if (n.getAttribute('src')) {
        const src = new URL(n.getAttribute('src'), abs).pathname;
        if (!imported.has(src)) {
          imported.add(src);
          await loadScript(src);
        }
      }
    }
  }
  window.Include = {
    addImport: (url, cb) => addImport(url).then(() => cb && cb()),
    addImports: (url, cb) => addImport(url).then(() => cb && cb()),
    addScript: (url, cb) => loadScript(url).then(() => cb && cb()),
    isLoadingScript: (pattern) => [...loading].some((s) => s.includes(pattern)),
    absolutePath: (cur, rel) => new URL(rel, new URL(cur, location.origin)).pathname,
    absoluteURL: (cur, rel) => new URL(rel, new URL(cur, location.origin)).href,
    setAsyncLoading: () => {},
    onAllResourcesLoaded: () => {},
  };

  // ---------------------------------------------------------------------------
  // TemplateElement / BaseInstrument / VCockpit panel.
  // ---------------------------------------------------------------------------
  class TemplateElement extends HTMLElement {
    connectedCallback() {
      this.instantiateTemplate();
    }
    disconnectedCallback() {}
    instantiateTemplate() {
      if (this._templated) return;
      this._templated = true;
      const id = this.templateID;
      const tpl = id ? document.getElementById(id) : null;
      if (tpl) this.insertAdjacentHTML('beforeend', tpl.innerHTML);
    }
    get templateID() {
      return '';
    }
  }
  window.TemplateElement = TemplateElement;
  window.UIElement = TemplateElement;

  window.ScreenState = { OFF: 0, INIT: 1, WAITING_VALIDATION: 2, ON: 3, REVERSIONARY: 4, 0: 'OFF', 1: 'INIT', 2: 'WAITING_VALIDATION', 3: 'ON', 4: 'REVERSIONARY' };
  window.Quality = { ultra: 0, high: 1, medium: 2, low: 3, hidden: 4, disabled: 5, 0: 'ultra', 1: 'high', 2: 'medium', 3: 'low', 4: 'hidden', 5: 'disabled' };
  window.GameState = { mainmenu: 0, loading: 1, briefing: 2, ingame: 3, 0: 'mainmenu', 1: 'loading', 2: 'briefing', 3: 'ingame' };

  class BaseInstrument extends TemplateElement {
    constructor() {
      super();
      this.urlConfig = { index: null, style: null };
      this._frameCount = 0;
      this._lastTime = 0;
      this._deltaTime = 0;
      this._isConnected = false;
      this._isInitialized = false;
      this._quality = window.Quality.high;
      this._gameState = window.GameState.ingame;
      this.screenState = window.ScreenState.OFF;
      this.isStarted = false;
      this.hasBeenOff = false;
      this.initDuration = 0;
      this.needValidationAfterInit = false;
      this.initAcknowledged = false;
      this.reversionaryMode = false;
      this._pendingCalls = [];
      this._pendingCallUId = 0;
      this._alwaysUpdateList = [];
      this.highlightList = [];
    }
    get initialized() {
      return this._isInitialized;
    }
    get instrumentIdentifier() {
      return this._instrumentId;
    }
    get instrumentIndex() {
      return this.urlConfig.index != null ? this.urlConfig.index : 1;
    }
    get isInteractive() {
      return false;
    }
    get IsGlassCockpit() {
      return false;
    }
    get isPrimary() {
      return this.urlConfig.index == null || this.urlConfig.index == 1;
    }
    get deltaTime() {
      return this._deltaTime;
    }
    get frameCount() {
      return this._frameCount;
    }
    get flightPlanManager() {
      return null;
    }
    get instrumentAlias() {
      return null;
    }
    connectedCallback() {
      super.connectedCallback();
      this.electricity = this.getChildById('Electricity');
      this.highlightSvg = this.getChildById('highlight');
      this.loadURLAttributes();
      this.startTime = Date.now();
      this.createMainLoop();
    }
    disconnectedCallback() {
      this._isConnected = false;
    }
    loadURLAttributes() {
      // MSFS lower-cases the URL before reading its parameters.
      const url = new URL(String(this.getAttribute('Url') || location.href).toLowerCase());
      this.urlConfig.style = url.searchParams.get('style');
      const index = url.searchParams.get('index');
      this.urlConfig.index = index == null ? null : parseInt(index, 10);
      this.urlConfig.wasmModule = url.searchParams.get('wasm_module');
      this.urlConfig.wasmGauge = url.searchParams.get('wasm_gauge');
      let id = this.templateID;
      if (this.urlConfig.index) id += `_${this.urlConfig.index}`;
      this._instrumentId = id;
      if (this.urlConfig.style) this.setAttribute('instrumentstyle', this.urlConfig.style);
    }
    setInstrumentIdentifier(id) {
      if (id) this._instrumentId = id;
    }
    setConfigFile(file) {
      this._xmlConfigFile = file;
    }
    getChildById(sel) {
      if (!sel) return null;
      if (!sel.startsWith('#') && !sel.startsWith('.')) sel = `#${sel}`;
      return this.querySelector(sel);
    }
    getChildrenById(sel) {
      if (!sel) return null;
      if (!sel.startsWith('#') && !sel.startsWith('.')) sel = `#${sel}`;
      return this.querySelectorAll(sel);
    }
    getChildrenByClassName(c) {
      return this.getElementsByClassName(c);
    }
    Init() {
      this._isInitialized = true;
    }
    getQuality() {
      return this._quality;
    }
    getGameState() {
      return this._gameState;
    }
    getTimeSinceStart() {
      return Date.now() - this.startTime;
    }
    getAspectRatio() {
      const r = this.getBoundingClientRect();
      return r.height ? r.width / r.height : 1;
    }
    isComputingAspectRatio() {
      return false;
    }
    isAspectRatioForced() {
      return false;
    }
    onInteractionEvent() {}
    onSoundEnd() {}
    onFlightStart() {}
    reboot() {}
    playInstrumentSound() {
      return false;
    }
    triggerEventToAllInstruments() {}
    triggerEventToInstrument() {}
    startHighlight() {}
    stopHighlight() {}
    clearHighlights() {}
    requestCall(func, timeout = 0) {
      const uid = ++this._pendingCallUId;
      this._pendingCalls.push({ func, timeout, uid });
      return uid;
    }
    removeCall(uid) {
      this._pendingCalls = this._pendingCalls.filter((c) => c.uid !== uid);
    }
    alwaysUpdate(el, on) {
      const i = this._alwaysUpdateList.indexOf(el);
      if (on && i < 0) this._alwaysUpdateList.push(el);
      if (!on && i >= 0) this._alwaysUpdateList.splice(i, 1);
    }
    isElectricityAvailable() {
      return !!SimVar.GetSimVarValue('CIRCUIT AVIONICS ON', 'Bool');
    }
    isBootProcedureComplete() {
      return !this.hasBeenOff || (Date.now() - this.startTime > this.initDuration && (this.initAcknowledged || !this.needValidationAfterInit));
    }
    acknowledgeInit() {
      this.initAcknowledged = true;
    }
    isInReversionaryMode() {
      return this.reversionaryMode;
    }
    wasTurnedOff() {
      return this.hasBeenOff;
    }
    updateElectricity() {
      const setScreen = (state, attr, lum, num) => {
        if (this.screenState === state) return;
        this.screenState = state;
        if (this.electricity) this.electricity.setAttribute('state', attr);
        SimVar.SetSimVarValue(`L:${this.instrumentIdentifier}_ScreenLuminosity`, 'number', lum);
        SimVar.SetSimVarValue(`L:${this.instrumentIdentifier}_State`, 'number', num);
      };
      if (this.isElectricityAvailable()) {
        if (!this.isStarted) {
          this.startTime = Date.now();
          this.isStarted = true;
        }
        if (this.isBootProcedureComplete()) {
          if (this.reversionaryMode) setScreen(window.ScreenState.REVERSIONARY, 'Backup', 1, 3);
          else setScreen(window.ScreenState.ON, 'on', 1, 2);
        } else if (Date.now() - this.startTime > this.initDuration) {
          setScreen(window.ScreenState.WAITING_VALIDATION, 'initWaitingValidation', 0.2, 1);
        } else {
          setScreen(window.ScreenState.INIT, 'init', 0.2, 1);
        }
      } else {
        this.hasBeenOff = true;
        if (this.isStarted) {
          this.isStarted = false;
          this.initAcknowledged = false;
          this._pendingCalls.length = 0;
        }
        setScreen(window.ScreenState.OFF, 'off', 0, 0);
      }
    }
    Update() {
      this.updateElectricity();
    }
    doUpdate() {
      const now = Date.now();
      this._deltaTime = now - this._lastTime;
      this._lastTime = now;
      const calls = this._pendingCalls;
      for (let i = 0; i < calls.length; ) {
        calls[i].timeout -= this._deltaTime;
        if (calls[i].timeout <= 0) {
          const c = calls.splice(i, 1)[0];
          c.func();
        } else i++;
      }
      this.Update();
      this._frameCount++;
    }
    createMainLoop() {
      if (this._isConnected) return;
      this._isConnected = true;
      this._lastTime = Date.now();
      const loop = () => {
        if (!this._isConnected) return;
        try {
          if (BaseInstrument.allInstrumentsLoaded && SimVar.IsReady()) {
            if (!this._isInitialized) this.Init();
            this.doUpdate();
          }
        } catch (e) {
          log.errors.push(`${this.instrumentIdentifier}: ${e && e.stack ? e.stack : e}`);
          console.error(e);
        }
        requestAnimationFrame(loop);
      };
      requestAnimationFrame(loop);
    }
    killMainLoop() {
      this._isConnected = false;
    }
  }
  BaseInstrument.allInstrumentsLoaded = false;
  BaseInstrument.useSvgImages = false;
  window.BaseInstrument = BaseInstrument;
  customElements.define('base-instrument', BaseInstrument);

  class VCockpitPanel extends HTMLElement {
    load(data) {
      this.data = data;
      this.index = -1;
      for (const a of data.daAttributes) document.body.setAttribute(a.name, a.value);
      document.dispatchEvent(new Event('OnVCockpitPanelAttributesChanged'));
      this.loadNext();
    }
    loadNext() {
      this.index++;
      if (this.index >= this.data.daInstruments.length) {
        window.__refPanelLoaded = true;
        // MSFS sends OnAllInstrumentsLoaded once every gauge of the panel exists.
        if (!window.__noAll) Coherent.trigger('OnAllInstrumentsLoaded');
        return;
      }
      const inst = this.data.daInstruments[this.index];
      const base = inst.sUrl.split('?')[0].toLowerCase();
      const prev = this.data.daInstruments.slice(0, this.index).find((i) => i.sUrl.split('?')[0].toLowerCase() === base && i.templateName);
      if (prev) {
        this.createInstrument(prev.templateName);
        return;
      }
      addImport(`/Pages/VCockpit/Instruments/${inst.sUrl}`);
    }
    registerInstrument(name, cls) {
      if (!customElements.get(name)) customElements.define(name, cls);
      this.createInstrument(name);
    }
    createInstrument(name) {
      const inst = this.data.daInstruments[this.index];
      const el = document.createElement(name);
      inst.templateName = name;
      el.setAttribute('Guid', String(inst.iGUId));
      el.setAttribute('Url', new URL(`/Pages/VCockpit/Instruments/${inst.sUrl}`, location.origin).href);
      el.style.position = 'absolute';
      el.style.left = `${inst.vPosAndSize.x}px`;
      el.style.top = `${inst.vPosAndSize.y}px`;
      el.style.width = `${Math.max(10, inst.vPosAndSize.z)}px`;
      el.style.height = `${Math.max(10, inst.vPosAndSize.w)}px`;
      if (el.setConfigFile) el.setConfigFile('');
      this.appendChild(el);
      this.loadNext();
    }
  }
  customElements.define('vcockpit-panel', VCockpitPanel);

  window.registerInstrument = (name, cls) => {
    const panel = document.getElementById('panel');
    // MSFS defers registration by a second to let dependencies finish loading.
    // The reference registers on a microtask instead, so that loading needs no
    // timer while the page clock is paused (see docs, "Determinism").
    if (panel) Promise.resolve().then(() => panel.registerInstrument(name, cls));
  };

  Coherent.on('OnAllInstrumentsLoaded', () => {
    BaseInstrument.allInstrumentsLoaded = true;
  });

  // ---------------------------------------------------------------------------
  // Boot the panel described by the config.
  // ---------------------------------------------------------------------------
  function boot() {
    const screen = CFG.screen;
    if (!screen) return;
    document.title = screen.name;
    const panel = document.getElementById('panel');
    panel.load({
      sName: screen.section || screen.name,
      sConfigFile: '',
      vLogicalSize: { x: screen.width, y: screen.height },
      vDisplaySize: { x: screen.width, y: screen.height },
      daAttributes: [
        { name: 'quality', value: 'high' },
        { name: 'gamestate', value: 'ingame' },
      ],
      daInstruments: screen.gauges.map((g, i) => ({ iGUId: i, sUrl: g.url, vPosAndSize: { x: g.x, y: g.y, z: g.w, w: g.h } })),
    });
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
  else boot();

  window.addEventListener('error', (e) => log.errors.push(`${e.message} @ ${e.filename}:${e.lineno}`));
  window.addEventListener('unhandledrejection', (e) => log.errors.push(`unhandled rejection: ${e.reason && e.reason.stack ? e.reason.stack : e.reason}`));

  // Serialisable snapshot of the log, for reference.mjs.
  window.__refAccessLog = () => {
    const list = (m) => [...m.values()].map((e) => Object.fromEntries(Object.entries(e).map(([k, v]) => [k, v instanceof Set ? [...v] : v])));
    return {
      reads: list(log.reads),
      writes: list(log.writes),
      unitMismatch: list(log.unitMismatch),
      storage: list(log.storage),
      storageWrites: list(log.storageWrites),
      coherentCalls: list(log.coherentCalls),
      coherentOn: list(log.coherentOn),
      coherentTriggers: list(log.coherentTriggers),
      viewListeners: list(log.viewListeners),
      keyEvents: list(log.keyEvents),
      gameVars: list(log.gameVars),
      errors: log.errors.slice(0, 200),
    };
  };
  window.__refLog = log;
})();
