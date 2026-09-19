// The rest of the environment MSFS gives a view (its fs-base-ui scripts):
// view listeners, flow events, stored data, script includes, the position
// types, the game state and quality enums, and Avionics.Utils.
(() => {
  const host = globalThis.__host;

  globalThis.bDebugListeners = false;
  globalThis.bDebugLoading = false;
  globalThis.EDITION_MODE = () => false;
  globalThis.checkAutoload = () => {};
  globalThis.EmptyCallback = { Void: () => {}, Boolean: () => {} };

  // Game state and quality, as MSFS numbers them.
  const GameState = {};
  ['mainmenu', 'loading', 'briefing', 'ingame'].forEach((name, i) => {
    GameState[(GameState[name] = i)] = name;
  });
  globalThis.GameState = GameState;
  const Quality = {};
  ['ultra', 'high', 'medium', 'low', 'hidden', 'disabled'].forEach((name, i) => {
    Quality[(Quality[name] = i)] = name;
  });
  globalThis.Quality = Quality;

  // View listeners. A listener is registered with the simulator by name; it
  // receives the events the simulator sends that listener
  // (EVENT_FROM_VIEW_LISTENER) and global events of the names it listens to.
  class ViewListener {
    constructor(name) {
      this.connected = false;
      this.m_name = String(name).toUpperCase();
      this.m_handlers = null;
      this.CheckCoherentEvent = (listenerName, eventName, ...args) => {
        if (String(listenerName).toUpperCase() !== this.m_name || !this.m_handlers) {
          return;
        }
        for (const h of this.m_handlers.slice()) {
          if (h.name === eventName) {
            if (h.context) {
              h.callback(h.context, ...args);
            } else {
              h.callback(...args);
            }
          }
        }
      };
      this.unregister = () => {
        if (this.m_handlers) {
          for (const h of this.m_handlers) {
            h.globalEventHandler.clear();
          }
          this.m_handlers = null;
        }
        this.eventHandler.clear();
        ViewListener.g_ViewListenersMgr.onUnregister(this.m_name, this);
      };
      this.onEventToAllSubscribers = (eventName, ...args) => {
        Coherent.trigger(eventName, ...args.map((a) => JSON.parse(a)));
      };
      this.eventHandler = Coherent.on('EVENT_FROM_VIEW_LISTENER', this.CheckCoherentEvent);
      this.on('ON_EVENT_TO_ALL_SUBSCRIBERS', this.onEventToAllSubscribers);
    }

    onGlobalEvent(eventName, ...args) {
      if (!this.m_handlers) {
        return;
      }
      for (const h of this.m_handlers.slice()) {
        if (h.name === eventName) {
          if (h.context) {
            h.callback(h.context, ...args);
          } else {
            h.callback(...args);
          }
        }
      }
    }

    on(name, callback, context) {
      if (!this.m_handlers) {
        this.m_handlers = [];
      }
      for (const h of this.m_handlers) {
        if (h.name === name && h.callback === callback && ((!context && !h.context) || context === h.context)) {
          return;
        }
      }
      this.m_handlers.push({
        name,
        callback,
        context,
        globalEventHandler: Coherent.on(name, this.onGlobalEvent.bind(this, name)),
      });
    }

    off(name, callback, context) {
      if (!this.m_handlers) {
        return;
      }
      for (let i = this.m_handlers.length - 1; i >= 0; i--) {
        const h = this.m_handlers[i];
        if (h.name === name && h.callback === callback && h.context === context) {
          h.globalEventHandler.clear();
          this.m_handlers.splice(i, 1);
        }
      }
    }

    call(name, ...args) {
      return Coherent.call(name, ...args).catch((error) => {
        console.error(`C++ API error when calling "${name}" from "${this.m_name}": ${error}`);
        throw error;
      });
    }

    trigger(name, ...args) {
      Coherent.trigger('EVENT_TO_VIEW_LISTENER', this.m_name, name, ...args);
    }

    triggerToAllSubscribers(event, ...args) {
      Coherent.trigger('TRIGGER_EVENT_TO_ALL_SUBSCRIBERS', this.m_name, event, ...args.map((a) => JSON.stringify(a)));
    }
  }

  class ViewListenerMgr {
    constructor() {
      this.m_hash = new Map();
      Coherent.on('VIEW_LISTENER_REGISTERED', (name) => {
        const list = this.m_hash.get(name);
        if (!list) {
          return;
        }
        for (const listener of list) {
          if (listener.m_onConnected || !listener.connected) {
            listener.connected = true;
            const callback = listener.m_onConnected;
            listener.m_onConnected = null;
            if (callback) {
              setTimeout(callback, 0);
            }
          }
        }
      });
    }
    onRegister(name, listener) {
      if (!this.m_hash.has(name)) {
        this.m_hash.set(name, []);
      }
      this.m_hash.get(name).push(listener);
    }
    getListenerByName(name) {
      const list = this.m_hash.get(name);
      return list && list.length > 0 ? list[0] : null;
    }
    onUnregister(name, listener) {
      const list = this.m_hash.get(name);
      if (!list) {
        return;
      }
      const i = list.indexOf(listener);
      if (i >= 0) {
        list.splice(i, 1);
      }
      if (list.length === 0) {
        Coherent.trigger('REMOVE_VIEW_LISTENER', name, location.pathname);
      }
    }
  }
  ViewListener.ViewListener = ViewListener;
  ViewListener.ViewListenerMgr = ViewListenerMgr;
  ViewListener.g_ViewListenersMgr = new ViewListenerMgr();
  globalThis.ViewListener = ViewListener;

  globalThis.RegisterViewListenerT = (name, callback = null, type, requiresSingleton = false) => {
    const key = String(name).toUpperCase();
    const existing = ViewListener.g_ViewListenersMgr.getListenerByName(key);
    if (requiresSingleton && existing) {
      if (existing.connected) {
        if (callback) {
          setTimeout(callback, 0);
        }
      } else {
        const previous = existing.m_onConnected;
        existing.m_onConnected = () => {
          if (callback) {
            callback();
          }
          if (previous) {
            previous();
          }
        };
      }
      return existing;
    }
    const listener = new type(name);
    listener.m_onConnected = callback;
    listener.urlCaller = location.pathname;
    ViewListener.g_ViewListenersMgr.onRegister(key, listener);
    if (existing && existing.connected) {
      listener.connected = true;
      listener.m_onConnected = null;
      if (callback) {
        setTimeout(callback, 0);
      }
    } else if (!existing) {
      Coherent.trigger('ADD_VIEW_LISTENER', key, location.pathname);
    }
    return listener;
  };
  globalThis.RegisterViewListener = (name, callback = null, requiresSingleton = false) =>
    RegisterViewListenerT(name, callback, ViewListener, requiresSingleton);

  class GenericDataListener extends ViewListener {
    onDataReceived(key, callback) {
      this.on(key, (json) => {
        try {
          callback(JSON.parse(json));
        } catch (e) {
          console.warn(e);
        }
      });
    }
    send(key, data) {
      try {
        this.trigger('SEND', key, JSON.stringify(data));
      } catch (e) {
        console.warn(e);
      }
    }
  }
  globalThis.GenericDataListener = GenericDataListener;
  globalThis.RegisterGenericDataListener = (callback) =>
    RegisterViewListenerT('JS_LISTENER_GENERICDATA', callback, GenericDataListener);

  globalThis.LaunchFlowEvent = (eventName, ...args) => {
    Coherent.trigger('LAUNCH_FLOW_EVENT_FROM_VIEW', eventName, ...args);
  };
  globalThis.LaunchFlowEventToGlobalFlow = (eventName, ...args) => {
    Coherent.trigger('LAUNCH_FLOW_EVENT_TO_GLOBAL_FLOW_FROM_VIEW', eventName, ...args);
  };

  // Stored data: strings kept by key across flights. A key never stored
  // reads as the empty string, as in MSFS.
  globalThis.GetStoredData = (key) => host.storedData('get', String(key));
  globalThis.SetStoredData = (key, data) => host.storedData('set', String(key), String(data));
  globalThis.DeleteStoredData = (key) => {
    host.storedData('delete', String(key));
    return null;
  };
  globalThis.SearchStoredData = (key) => JSON.parse(host.storedData('search', String(key)));
  globalThis.GetDataStorage = () => ({
    getData: (key) => GetStoredData(key),
    setData: (key, data) => SetStoredData(key, data),
    searchData: (key) => SearchStoredData(key),
    deleteData: (key) => DeleteStoredData(key),
  });
  globalThis.OnDataStorageReady = () => {};

  // Script includes. The scripts MSFS itself provides (fs-base's /JS/ and
  // the VCockpit core) are part of this runtime already; anything else is
  // read from html_ui.
  const provided = new Set([
    '/js/coherent.js',
    '/js/common.js',
    '/js/types.js',
    '/js/simvar.js',
    '/js/avionics.js',
    '/js/simplane.js',
    '/js/datastorage.js',
    '/js/services/genericdatalistener.js',
    '/pages/vcockpit/instruments/shared/baseinstrument.js',
    '/pages/vcockpit/instruments/shared/utils/xmllogic.js',
  ]);
  const normalise = (path) => {
    const parts = [];
    for (const part of String(path).split('/')) {
      if (part === '..') {
        parts.pop();
      } else if (part !== '.' && part !== '') {
        parts.push(part);
      }
    }
    return '/' + parts.join('/');
  };
  const Include = {
    absolutePath(current, relative) {
      if (String(relative).startsWith('/')) {
        return normalise(relative);
      }
      const base = String(current).split('/').slice(0, -1).join('/');
      return normalise(`${base}/${relative}`);
    },
    absoluteURL(current, relative) {
      return 'coui://html_ui' + Include.absolutePath(current, relative);
    },
    addScript(path) {
      const absolute = Include.absolutePath(location.pathname, String(path).split('?')[0]);
      if (!provided.has(absolute.toLowerCase())) {
        host.runScript(absolute);
      }
    },
    addScripts(paths) {
      for (const path of paths) {
        Include.addScript(path);
      }
    },
    // An HTML import: the page's templates, styles and scripts go into this
    // document (instrument.js).
    addImport(path) {
      __vcockpit.importPage(Include.absolutePath(location.pathname, path));
    },
    isLoadingScript: () => false,
    isImportLoaded: () => true,
    setAsyncLoading: () => {},
  };
  globalThis.Include = Include;

  // fetch: files of the view's package, by coui://html_ui/ path or /VFS/
  // (the package's root, where the build info and aircraft config are
  // fetched from); http URLs go to the plugin (__host.call('fetch')), which
  // answers what it serves (SimBridge's terrain API) with {status, body}
  // and rejects the rest, as fetch fails offline, with a TypeError.
  class Response {
    constructor(body, status, url) {
      this._body = body;
      this.status = status;
      this.ok = status >= 200 && status < 300;
      this.statusText = this.ok ? 'OK' : 'Not Found';
      this.url = url;
      this.headers = { get: () => null, has: () => false };
    }
    text() {
      return Promise.resolve(this._body);
    }
    json() {
      return new Promise((resolve) => resolve(JSON.parse(this._body)));
    }
  }
  globalThis.Response = Response;
  globalThis.fetch = (resource, options = {}) => {
    const url = String(resource && resource.url !== undefined ? resource.url : resource);
    if (/^https?:/i.test(url)) {
      const method = String((options && options.method) || 'GET').toUpperCase();
      const body = options && options.body !== undefined ? String(options.body) : '';
      const reply = host.call('fetch', JSON.stringify([method, url, body]));
      if (reply[0] === '0') {
        const answer = JSON.parse(reply.slice(1));
        if (answer.status !== 404) {
          return Promise.resolve(new Response(answer.body, answer.status, url));
        }
      }
      return Promise.reject(new TypeError(`Failed to fetch ${url}`));
    }
    if (/^[a-z][a-z0-9+.-]*:/i.test(url) && !/^coui:/i.test(url)) {
      return Promise.reject(new TypeError(`Failed to fetch ${url}`));
    }
    const path = url.replace(/^coui:\/\/html_ui/i, '');
    return new Promise((resolve) => {
      try {
        resolve(new Response(host.readFile(Include.absolutePath(location.pathname, path)), 200, url));
      } catch (e) {
        resolve(new Response('', 404, url));
      }
    });
  };

  // Positions.
  const wrapLatLong = (p) => {
    while (p.long > 180 || p.long < -180) {
      p.long = p.long > 180 ? 360 - p.long : -360 - p.long;
    }
    while (p.lat > 90 || p.lat < -90) {
      p.lat = p.lat > 90 ? 180 - p.lat : -180 - p.lat;
    }
  };
  const degreeParts = (value, span) => {
    let v = Math.abs(value);
    const degrees = Math.floor(v);
    v -= degrees;
    const minutes = Math.floor(v * 60);
    v -= minutes / 60;
    const tenths = Math.floor(v * 600);
    return [String(degrees).padStart(span, '0'), String(minutes).padStart(2, '0'), String(tenths)];
  };
  const clampLat = (lat) => Math.min(90, Math.max(-90, lat));
  const wrapLong = (long) => {
    let l = long;
    while (l > 180) l -= 360;
    while (l < -180) l += 360;
    return l;
  };
  const latString = (lat) => {
    const l = clampLat(lat);
    const [d, m, t] = degreeParts(l, 2);
    return `${d}${m}.${t}${l < 0 ? 'S' : 'N'}`;
  };
  const longString = (long) => {
    const l = wrapLong(long);
    const [d, m, t] = degreeParts(l, 3);
    return `${d}${m}.${t}${l < 0 ? 'W' : 'E'}`;
  };
  const degreeString = (p) => {
    const lat = clampLat(p.lat);
    const long = wrapLong(p.long);
    const [ad, am, at] = degreeParts(lat, 2);
    const [od, om, ot] = degreeParts(long, 3);
    return `${lat < 0 ? 'S' : 'N'}${ad}°${am}.${at} ${long < 0 ? 'W' : 'E'}${od}°${om}.${ot}`;
  };

  class LatLong {
    constructor(data, long) {
      this.__Type = 'LatLong';
      if (isFinite(data) && isFinite(long)) {
        this.lat = data;
        this.long = long;
        wrapLatLong(this);
      } else {
        Object.assign(this, data);
      }
    }
    set(lat, long) {
      this.lat = lat;
      this.long = long;
      wrapLatLong(this);
    }
    toStringFloat() {
      return `${this.lat.toFixed(6)}, ${this.long.toFixed(6)}`;
    }
    toString() {
      return `lat ${this.lat.toFixed(2)}, long ${this.long.toFixed(2)}`;
    }
    static fromStringFloat(str) {
      const parts = String(str).toLowerCase().split(',');
      if (parts.length >= 3) {
        return new LatLongAlt(parseFloat(parts[0]), parseFloat(parts[1]), parseFloat(parts[2]));
      }
      if (parts.length >= 2) {
        return new LatLong(parseFloat(parts[0]), parseFloat(parts[1]));
      }
      return null;
    }
    latToDegreeString() {
      return latString(this.lat);
    }
    longToDegreeString() {
      return longString(this.long);
    }
    toDegreeString() {
      return degreeString(this);
    }
    toShortDegreeString() {
      return latString(this.lat) + longString(this.long);
    }
  }

  class LatLongAlt {
    constructor(data, long, alt) {
      this.alt = 0;
      this.__Type = 'LatLongAlt';
      if (isFinite(data) && isFinite(long)) {
        this.lat = data;
        this.long = long;
        if (isFinite(alt)) {
          this.alt = alt;
        }
        wrapLatLong(this);
      } else {
        Object.assign(this, data);
      }
    }
    toLatLong() {
      return new LatLong(this.lat, this.long);
    }
    toStringFloat() {
      return `${this.lat.toFixed(6)}, ${this.long.toFixed(6)}, ${isFinite(this.alt) ? this.alt.toFixed(1) : 'NaN'}`;
    }
    toString() {
      return `lat ${this.lat.toFixed(2)}, long ${this.long.toFixed(2)}, alt ${isFinite(this.alt) ? this.alt.toFixed(2) : 'NaN'}`;
    }
    latToDegreeString() {
      return latString(this.lat);
    }
    longToDegreeString() {
      return longString(this.long);
    }
    toDegreeString() {
      return degreeString(this);
    }
  }

  class PitchBankHeading {
    constructor(data) {
      this.__Type = 'PitchBankHeading';
      Object.assign(this, data);
    }
    toString() {
      return `p ${this.pitchDegree.toFixed(2)}, b ${this.bankDegree.toFixed(2)}, h ${this.headingDegree.toFixed(2)}`;
    }
  }

  class LatLongAltPBH {
    constructor(data) {
      this.__Type = 'LatLongAltPBH';
      this.lla = new LatLongAlt(data.lla);
      this.pbh = new PitchBankHeading(data.pbh);
    }
    toString() {
      return `lla ${this.lla}, pbh ${this.pbh}`;
    }
  }

  class PID_STRUCT {
    constructor(data) {
      Object.assign(this, data);
    }
  }

  class XYZ {
    constructor(data) {
      this.__Type = 'XYZ';
      Object.assign(this, data);
    }
    toString() {
      return `x ${this.x.toFixed(2)}, y ${this.y.toFixed(2)}, z ${this.z.toFixed(2)}`;
    }
  }

  Object.assign(globalThis, { LatLong, LatLongAlt, PitchBankHeading, LatLongAltPBH, PID_STRUCT, XYZ, wrapLatLong });

  // Avionics.Utils, the parts instruments use.
  const DEG2RAD = Math.PI / 180;
  const RAD2DEG = 180 / Math.PI;
  const bcd = (digits, count) => {
    let out = 0;
    for (let i = 0; i < count; i++) {
      out += (Math.floor(digits / Math.pow(10, i)) % 10) << (4 * i);
    }
    return out;
  };
  class Utils {
    static make_bcd16(hz) {
      return bcd(Math.floor(hz / 10000 - 10000), 4);
    }
    static make_adf_bcd32(hz) {
      return bcd(Math.floor(hz / 100), 5) << 12;
    }
    static make_xpndr_bcd16(code) {
      const digits = ('0000' + code).slice(-4);
      return parseInt(digits[0]) * 4096 + parseInt(digits[1]) * 256 + parseInt(digits[2]) * 16 + parseInt(digits[3]);
    }
    static bearingDistanceToCoordinates(bearing, distance, lat, long) {
      const deltaLat = (distance * Math.cos(bearing * DEG2RAD)) / 60;
      const deltaLong = (distance * Math.sin(bearing * DEG2RAD)) / 60 / Math.cos(lat * DEG2RAD);
      return new LatLongAlt(lat + deltaLat, long + deltaLong);
    }
    static computeDistance(from, to) {
      const a =
        0.5 -
        Math.cos((to.lat - from.lat) * DEG2RAD) / 2 +
        (Math.cos(from.lat * DEG2RAD) * Math.cos(to.lat * DEG2RAD) * (1 - Math.cos((to.long - from.long) * DEG2RAD))) / 2;
      return 6880.126 * Math.asin(Math.sqrt(a));
    }
    static computeGreatCircleHeading(from, to) {
      const lat0 = from.lat * DEG2RAD;
      const lat1 = to.lat * DEG2RAD;
      const dlon = (to.long - from.long) * DEG2RAD;
      const cosLat1 = Math.cos(lat1);
      const sinHalf = Math.sin(dlon / 2);
      const x = Math.sin(lat1 - lat0) + sinHalf * sinHalf * 2 * Math.sin(lat0) * cosLat1;
      let heading = Math.atan2(cosLat1 * Math.sin(dlon), x);
      if (heading < 0) {
        heading += 2 * Math.PI;
      }
      return heading * RAD2DEG;
    }
    static computeGreatCircleDistance(from, to) {
      const lat0 = from.lat * DEG2RAD;
      const lat1 = to.lat * DEG2RAD;
      const a1 = Math.sin((lat1 - lat0) / 2);
      const a2 = Math.sin(((to.long - from.long) * DEG2RAD) / 2);
      return Math.asin(Math.sqrt(a1 * a1 + Math.cos(lat0) * Math.cos(lat1) * a2 * a2)) * 6880.126;
    }
    static lerpAngle(from, to, d) {
      if (from * to > 0) {
        return from * (1 - d) + to * d;
      }
      const negative = from < 0 || to < 0;
      let a = (from + 360) % 360;
      let b = (to + 360) % 360;
      if (a - b > 180) {
        a -= 360;
      }
      if (b - a > 180) {
        b -= 360;
      }
      let result = a * (1 - d) + b * d;
      if (negative && result > 180) {
        result -= 360;
      }
      return result;
    }
    static meanAngle(a, b) {
      return Utils.lerpAngle(a, b, 0.5);
    }
    static diffAngle(a, b) {
      let diff = b - a;
      while (diff > 180) diff -= 360;
      while (diff <= -180) diff += 360;
      return diff;
    }
    static clampAngle(a) {
      const angle = a % 360;
      return angle < 0 ? angle + 360 : angle;
    }
    static fmod(a, b) {
      const quotient = Number((a / b).toPrecision(8));
      const floor = Number((Math.floor(quotient) * b).toPrecision(8));
      return Number((a - floor).toPrecision(8));
    }
    static clamp(value, min, max) {
      return Math.max(min, Math.min(max, value));
    }
    static Clamp(value, min, max) {
      return Utils.clamp(value, min, max);
    }
  }
  Utils.DEG2RAD = DEG2RAD;
  Utils.RAD2DEG = RAD2DEG;
  Utils.DEGREE_SYMBOL = '°';
  Utils.METER2FEET = 3.28084;
  Utils.FEET2METER = 1 / Utils.METER2FEET;
  Utils.PSI2BAR = 0.0689476;
  Utils.BAR2PSI = 1 / Utils.PSI2BAR;

  class CurveTool {
    static StringColorRGBInterpolation(c1, c2, dt) {
      const channel = (c, i) => parseInt(c.substr((c[0] === '#' ? 1 : 0) + i, 2), 16);
      const mix = (i) => {
        const v = Math.round(channel(c1, i) * (1 - dt) + channel(c2, i) * dt);
        return ('00' + v.toString(16)).substr(-2, 2);
      };
      return '#' + mix(0) + mix(2) + mix(4);
    }
  }

  globalThis.Avionics = { Utils, CurveTool, SVG: { NS: 'http://www.w3.org/2000/svg' } };

  // Facilities.getMagVar(lat, lon): MagVar.ts and navdata's Mapping.ts call
  // this for waypoints and airports, not just the aircraft's own position
  // (msfs.d.ts). __host.getMagVar reaches X-Plane's own magnetic variation
  // model (XPLMGetMagneticVariation), the same source navdata's own facility
  // magvar uses.
  globalThis.Facilities = {
    getMagVar: (lat, lon) => host.getMagVar(lat, lon),
  };

  // The enums fs-base-ui's JS/SimPlane.js (lines 135-264) declares as
  // globals, with its values; msfs-sdk's facility code reads them.
  const enums = [
    ['EngineType', { ENGINE_TYPE_PISTON: 0, ENGINE_TYPE_JET: 1, ENGINE_TYPE_NONE: 2, ENGINE_TYPE_HELO_TURBINE: 3, ENGINE_TYPE_ROCKET: 4, ENGINE_TYPE_TURBOPROP: 5 }],
    ['PropellerType', { PROPELLER_TYPE_CONSTANT_SPEED: 0, PROPELLER_TYPE_FIXED_PITCH: 1 }],
    ['Aircraft', { CJ4: 0, A320_NEO: 1, B747_8: 2, AS01B: 3, AS02A: 4, AS03D: 5, AS05B: 6 }],
    ['ThrottleMode', { UNKNOWN: 0, REVERSE: 1, IDLE: 2, AUTO: 3, CLIMB: 4, FLEX_MCT: 5, TOGA: 6, HOLD: 7 }],
    ['AutopilotMode', { MANAGED: 0, SELECTED: 1, HOLD: 2 }],
    ['MinimumReferenceMode', { RADIO: 0, BARO: 1 }],
    [
      'FlightState',
      {
        FLIGHT_STATE_BRIEFING: 0, FLIGHT_STATE_INTRO_PLANE: 1, FLIGHT_STATE_INTRO: 2, FLIGHT_STATE_PREFLIGHT_GATE: 3,
        FLIGHT_STATE_PREFLIGHT_PUSHBACK: 4, FLIGHT_STATE_PREFLIGHT_TAXI: 5, FLIGHT_STATE_PREFLIGHT_HOLDSHORT: 6,
        FLIGHT_STATE_FLIGHT_RUNWAY: 7, FLIGHT_STATE_FLIGHT_INITIAL_CLIMB: 8, FLIGHT_STATE_FLIGHT_CLIMB: 9,
        FLIGHT_STATE_FLIGHT_CRUISE: 10, FLIGHT_STATE_FLIGHT_DESCENT: 11, FLIGHT_STATE_JOINPLANE: 12,
        FLIGHT_STATE_LANDING_APPROACH: 13, FLIGHT_STATE_LANDING_FINAL: 14, FLIGHT_STATE_LANDING_TOUCHDOWN: 15,
        FLIGHT_STATE_LANDING_GROUNDROLL: 16, FLIGHT_STATE_LANDING_TAXI: 17, FLIGHT_STATE_LANDING_GATE: 18,
        FLIGHT_STATE_LANDING_REST: 19, FLIGHT_STATE_OUTRO: 20, FLIGHT_STATE_WAITING: 21, FLIGHT_STATE_TELEPORTTOSTATE: 22,
        FLIGHT_STATE_FREEFLIGHT: 23, FLIGHT_STATE_LANDINGCHALLENGE: 24, FLIGHT_STATE_BUSHTRIP: 25,
      },
    ],
    [
      'FlightPhase',
      {
        FLIGHT_PHASE_PREFLIGHT: 0, FLIGHT_PHASE_TAXI: 1, FLIGHT_PHASE_TAKEOFF: 2, FLIGHT_PHASE_CLIMB: 3, FLIGHT_PHASE_CRUISE: 4,
        FLIGHT_PHASE_DESCENT: 5, FLIGHT_PHASE_APPROACH: 6, FLIGHT_PHASE_GOAROUND: 7,
      },
    ],
    [
      'ApproachType',
      {
        APPROACH_TYPE_UNKNOWN: 0, APPROACH_TYPE_GPS: 1, APPROACH_TYPE_VOR: 2, APPROACH_TYPE_NDB: 3, APPROACH_TYPE_ILS: 4,
        APPROACH_TYPE_LOCALIZER: 5, APPROACH_TYPE_SDF: 6, APPROACH_TYPE_LDA: 7, APPROACH_TYPE_VORDME: 8, APPROACH_TYPE_NDBDME: 9,
        APPROACH_TYPE_RNAV: 10, APPROACH_TYPE_LOCALIZER_BACK_COURSE: 11,
      },
    ],
    [
      'RunwayDesignator',
      {
        RUNWAY_DESIGNATOR_NONE: 0, RUNWAY_DESIGNATOR_LEFT: 1, RUNWAY_DESIGNATOR_RIGHT: 2, RUNWAY_DESIGNATOR_CENTER: 3,
        RUNWAY_DESIGNATOR_WATER: 4, RUNWAY_DESIGNATOR_A: 5, RUNWAY_DESIGNATOR_B: 6,
      },
    ],
    ['WorldRegion', { NORTH_AMERICA: 0, AUSTRALIA: 1, HAWAII: 2, OTHER: 3 }],
    ['NAV_AID_STATE', { OFF: 0, ADF: 1, VOR: 2 }],
    ['NAV_AID_MODE', { NONE: 0, MANUAL: 1, REMOTE: 2 }],
  ];
  // Utils, the functions of fs-base-ui's JS/common.js (lines 959-1434)
  // FlyByWire's bundles call. MSFS translates through the page's
  // g_localization; there is none here, so a key reads as common.js reads
  // it without one.
  globalThis.Utils = {
    Clamp: (n, min, max) => (n < min ? min : n > max ? max : n),
    SmoothLinear: (origin, destination, smoothFactor, dTime) => {
      if (origin == undefined || smoothFactor <= 0 || Math.abs(destination - origin) < Number.EPSILON) {
        return destination;
      }
      if (origin > destination) {
        return Math.max(destination, origin - smoothFactor * dTime);
      }
      return Math.min(destination, origin + smoothFactor * dTime);
    },
    generateGUID: () => {
      const S4 = () => (((1 + Math.random()) * 0x10000) | 0).toString(16).substring(1);
      return 'GUID_' + (S4() + S4() + '-' + S4() + '-' + S4() + '-' + S4() + '-' + S4() + S4() + S4());
    },
    Translate: (key) => {
      if (!isNaN(parseFloat(key))) return key;
      if (key == null || key === '') return '';
      const localization = globalThis.top.g_localization;
      return localization ? localization.Translate(key) : null;
    },
  };

  // localStorage, which Coherent gives a page. Kept for the session only:
  // nothing here says where MSFS keeps it between flights.
  const local = new Map();
  globalThis.localStorage = {
    getItem: (key) => (local.has(String(key)) ? local.get(String(key)) : null),
    setItem: (key, value) => local.set(String(key), String(value)),
    removeItem: (key) => local.delete(String(key)),
    clear: () => local.clear(),
    key: (i) => [...local.keys()][i] ?? null,
    get length() {
      return local.size;
    },
  };

  // Simplane, the getters of fs-base-ui's JS/SimPlane.js that FlyByWire's
  // bundles call, reading the variables SimPlane.js reads (its lines in
  // brackets). SimPlane.js caches each read for a few frames; these read
  // every time.
  const sv = (name, unit) => SimVar.GetSimVarValue(name, unit);
  const Simplane = {
    getIndicatedSpeed: () => sv('AIRSPEED INDICATED', 'knots'), // [579]
    getVerticalSpeed: () => sv('VERTICAL SPEED', 'feet per minute'), // [589]
    getGroundSpeed: () => sv('GPS GROUND SPEED', 'knots'), // [594]
    getGreenDotSpeed: () => SimVar.GetGameVarValue('AIRCRAFT GREEN DOT SPEED', 'knots'), // [693]
    getMachToKias: (mach) => SimVar.GetGameVarValue('FROM MACH TO KIAS', 'number', mach), // [721]
    getAutoPilotAirspeedSlotIndex: () => sv('AUTOPILOT SPEED SLOT INDEX', 'number'), // [940]
    getAutoPilotAirspeedManaged: () => Simplane.getAutoPilotAirspeedSlotIndex() == 2, // [945]
    getAutoPilotAirspeedSelected: () => Simplane.getAutoPilotAirspeedSlotIndex() == 1, // [949]
    getAutoPilotAirspeedHoldValue: () => sv('AUTOPILOT AIRSPEED HOLD VAR', 'knots'), // [981]
    getAutoPilotMachModeActive: () => sv('L:XMLVAR_AirSpeedIsInMach', 'bool'), // [996]
    getAutoPilotMachHoldValue: () => sv('AUTOPILOT MACH HOLD VAR', 'number'), // [1073]
    // [1125-1143]
    getAutoPilotSelectedHeadingLockValue: (radians = true) => sv('AUTOPILOT HEADING LOCK DIR:1', radians ? 'radians' : 'degrees'),
    getAutoPilotAltitudeSlotIndex: () => sv('AUTOPILOT ALTITUDE SLOT INDEX', 'number'), // [1182]
    getAutoPilotAltitudeManaged: () => Simplane.getAutoPilotAltitudeSlotIndex() == 2, // [1187]
    getAutoPilotDisplayedAltitudeLockValue: (units = 'feet') => sv('AUTOPILOT ALTITUDE LOCK VAR:3', units), // [1241-1252]
    getAltitude: () => sv('INDICATED ALTITUDE', 'feet'), // [1778]
    getAltitudeAboveGround: () => Math.max(0, sv('PLANE ALT ABOVE GROUND MINUS CG', 'feet')), // [1841]
    getIsGrounded: () => Simplane.getAltitudeAboveGround() < 10, // [1827]
    getAmbientTemperature: () => sv('AMBIENT TEMPERATURE', 'celsius'), // [2083]
    getPressureValue: (units = 'inches of mercury') => sv('KOHLSMAN SETTING HG', units), // [2298]
    getPressureSelectedUnits: () => (sv('L:XMLVAR_Baro_Selector_HPA_1', 'Bool') ? 'millibar' : 'inches of mercury'), // [2308-2318]
    // [2319-2344]
    getPressureSelectedMode: (aircraft) => {
      if (aircraft == Aircraft.A320_NEO) {
        const mode = sv('L:XMLVAR_Baro1_Mode', 'number');
        return mode == 0 ? 'QFE' : mode == 1 ? 'QNH' : 'STD';
      }
      return sv('L:XMLVAR_Baro1_ForcedToSTD', 'bool') ? 'STD' : '';
    },
  };
  globalThis.Simplane = Simplane;

  // The input action names fs-base-ui's JS/common.js (lines 6133-6161)
  // gives InputBar in MSFS 2020; FlyByWire tells MSFS 2024 apart by them.
  globalThis.InputBar = {
    MENU_BUTTON_A: 'MENU_VALID',
    MENU_BUTTON_B: 'MENU_BACK',
    MENU_BUTTON_X: 'MENU_VALI2',
    MENU_BUTTON_Y: 'MENU_CANCEL',
    MENU_BUTTON_START: 'MENU_START',
    MENU_BUTTON_SELECT: 'MENU_SELECT',
    MENU_BUTTON_OPEN: 'MENU_OPEN',
    MENU_BUTTON_RESET: 'MENU_RESET',
    MENU_BUTTON_APPLY: 'MENU_APPLY',
    MENU_BUTTON_PRESET_MANAGER: 'MENU_PRESET_MANAGER',
    MENU_BUTTON_PROFILE_MANAGER: 'MENU_PROFILE_MANAGER',
    MENU_BUTTON_CONTENT_MANAGER: 'MENU_CONTENT_MANAGER',
    MENU_BUTTON_QUIT: 'MENU_QUIT_GAME',
    MENU_BUTTON_CLOSE: 'MENU_CLOSE',
    MENU_BUTTON_BACK: 'MENU_BACK',
    MENU_BUTTON_WM_FILTERS: 'MENU_WM_FILTERS',
    MENU_BUTTON_WM_LEGEND: 'MENU_WM_LEGEND',
    MENU_BUTTON_CUSTOMIZE: 'MENU_CUSTOMIZE',
    MENU_BUTTON_FLY: 'MENU_FLY',
    MENU_BUTTON_FAVORITE: 'MENU_BUTTON_FAVORITE',
    MENU_BUTTON_TAB_LEFT: 'MENU_L1',
    MENU_BUTTON_TAB_RIGHT: 'MENU_R1',
    MENU_BUTTON_SUB_TAB_LEFT: 'MENU_L2',
    MENU_BUTTON_SUB_TAB_RIGHT: 'MENU_R2',
    MENU_BUTTON_RANALOG_X: 'KEY_MENU_SCROLL_AXIS_X',
    MENU_BUTTON_RANALOG_Y: 'KEY_MENU_SCROLL_AXIS_Y',
    MENU_BUTTON_RANALOG_XY: 'MENU_WM_LEGEND',
  };

  for (const [name, values] of enums) {
    const e = {};
    for (const [k, v] of Object.entries(values)) {
      e[(e[k] = v)] = k;
    }
    globalThis[name] = e;
  }
})();
