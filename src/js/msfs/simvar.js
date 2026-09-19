// Simulation variables, as an MSFS view reaches them.
//
// MSFS gives views three native objects, `simvar`, `globalvar` and
// `gamevar`, and builds the `SimVar` namespace on them (and msfs-sdk replaces
// SimVar.GetSimVarValue and SetSimVarValue with its own versions that call
// the natives directly). Both layers are here: the natives over the plugin's
// variables (__host), and SimVar as MSFS defines it.
//
// Names: `L:NAME` (aircraft variables), `A:NAME:index` or no prefix
// (simulator variables), `E:NAME` (environment), `K:EVENT` and `H:EVENT`
// (writes send the event), `Z:NAME` (none here). Units are converted by the
// plugin; `bool` and `boolean` read as 0 or 1.
//
// A registered id (SimVar.GetRegisteredId) is the fast path: the name and
// unit are resolved once, and each read after that is one call with a number.
(() => {
  const host = globalThis.__host;
  const reported = new Set();
  const once = (what) => {
    if (!reported.has(what)) {
      reported.add(what);
      console.warn(`MSFS runtime: ${what} is not supported here`);
    }
  };

  const structUnits = /^(latlonalt|latlonaltpbh|pbh|pid_struct|xyz)$/i;
  const isString = (unit) => /^string$/i.test(String(unit).trim());

  const struct = (name, unit) => {
    const u = String(unit).toLowerCase();
    if (u === 'latlonalt' && /^(A:)?PLANE POSITION$/i.test(name)) {
      return {
        lat: host.getVar('A:PLANE LATITUDE', 'degrees'),
        long: host.getVar('A:PLANE LONGITUDE', 'degrees'),
        alt: host.getVar('A:PLANE ALTITUDE', 'feet'),
      };
    }
    // NAV VOR LATLONALT:1-4, radios.rs's own `destination_point`: the
    // station triangulated from the receiver's bearing and DME, published
    // as fbw/radio/nav<n>/lat,lon (altitude is not knowable from the
    // receiver alone and is left at sea level, which is all the ND's 2D
    // plot uses this struct for).
    const navVor = /^(A:)?NAV VOR LATLONALT:([1-4])$/i.exec(name);
    if (u === 'latlonalt' && navVor) {
      const n = navVor[2];
      return {
        lat: host.getVar(`fbw/radio/nav${n}/lat`, 'degrees'),
        long: host.getVar(`fbw/radio/nav${n}/lon`, 'degrees'),
        alt: 0,
      };
    }
    once(`the ${unit} value of ${name}`);
    return null;
  };

  globalThis.simvar = {
    registerSimVarWatcher: (name, unit) => host.registerVar(String(name), String(unit)),
    getValueReg: (id) => host.getReg(id),
    getValueReg_String: (id) => host.getRegString(id),
    getValue: (name, unit) => (isString(unit) ? host.getString(String(name)) : host.getVar(String(name), String(unit))),
    getValue_String: (name) => host.getString(String(name)),
    getValue_LatLongAlt: (name) => struct(name, 'latlonalt'),
    getValue_LatLongAltPBH: (name) => struct(name, 'latlonaltpbh'),
    getValue_PBH: (name) => struct(name, 'pbh'),
    getValue_PID_STRUCT: (name) => struct(name, 'pid_struct'),
    getValue_XYZ: (name) => struct(name, 'xyz'),
    registerSimVarArrayWatcher: (batch) => {
      once('SimVar array watchers');
      return -1;
    },
  };

  // Global variables are the simulator clock's.
  globalThis.globalvar = {
    registerGlobalVarWatcher: (name, unit) => host.registerVar(`E:${name}`, String(unit)),
    getValueReg: (id) => host.getReg(id),
    getValue: (name, unit) => host.getVar(`E:${name}`, String(unit)),
  };

  // Game variables are computed from the aircraft's state, or come from the
  // plugin's providers (the nav database's cycle).
  const pressureRatio = () => host.getVar('A:AMBIENT PRESSURE', 'inHg') / 29.92126;
  const machToKias = (mach) => {
    const delta = pressureRatio();
    const qc = delta * (Math.pow(1 + 0.2 * mach * mach, 3.5) - 1);
    return 661.4786 * Math.sqrt(5 * (Math.pow(qc + 1, 2 / 7) - 1));
  };
  const kiasToMach = (kias) => {
    const delta = pressureRatio();
    const qc = Math.pow(1 + 0.2 * Math.pow(kias / 661.4786, 2), 3.5) - 1;
    return Math.sqrt(5 * (Math.pow(qc / delta + 1, 2 / 7) - 1));
  };
  const gameNumbers = {
    'FROM MACH TO KIAS': (mach) => machToKias(Number(mach)),
    'FROM KIAS TO MACH': (kias) => kiasToMach(Number(kias)),
  };
  const gameName = (name) => String(name).replace(/_/g, ' ').toUpperCase();
  // The rest reach the plugin as `GAME:NAME` (spaces, upper case), an XYZ
  // as `GAME:NAME:X`, `:Y` and `:Z`.
  globalThis.gamevar = {
    registerGameVarWatcher: () => -1,
    getValue: (name, unit, param1) => {
      const fn = gameNumbers[gameName(name)];
      return fn === undefined ? host.getVar(`GAME:${gameName(name)}`, String(unit)) : fn(param1);
    },
    getValue_String: (name) => host.getString(`GAME:${gameName(name)}`),
    getValue_XYZ: (name) => ({
      __Type: 'XYZ',
      x: host.getVar(`GAME:${gameName(name)}:X`, 'meters'),
      y: host.getVar(`GAME:${gameName(name)}:Y`, 'meters'),
      z: host.getVar(`GAME:${gameName(name)}:Z`, 'meters'),
    }),
  };

  const SimVar = {};
  SimVar.g_bUseWatcher = false;

  class SimVarValue {
    constructor(name = '', unit = 'number', type) {
      this.__type = 'SimVarValue';
      this.name = name;
      this.type = type;
      this.unit = unit;
    }
  }
  SimVar.SimVarValue = SimVarValue;

  SimVar.IsReady = () => true;

  const registered = new Map();
  SimVar.GetRegisteredId = (name, unit, dataSource = '') => {
    const key = `${name}|${unit}|${dataSource}`;
    let id = registered.get(key);
    if (id === undefined) {
      id = simvar.registerSimVarWatcher(name, unit, dataSource);
      registered.set(key, id);
    }
    return id;
  };

  const wrapStruct = (unit, value) => {
    if (value === null) {
      return null;
    }
    switch (String(unit).toLowerCase()) {
      case 'latlonalt':
        return new LatLongAlt(value);
      case 'latlonaltpbh':
        return new LatLongAltPBH(value);
      case 'pbh':
        return new PitchBankHeading(value);
      case 'pid_struct':
        return new PID_STRUCT(value);
      case 'xyz':
        return new XYZ(value);
      default:
        return value;
    }
  };

  SimVar.GetSimVarValue = (name, unit, dataSource = '') => {
    try {
      if (structUnits.test(String(unit))) {
        return wrapStruct(unit, struct(String(name), unit));
      }
      return simvar.getValue(name, unit, dataSource);
    } catch (error) {
      console.warn('ERROR ', error, ' GetSimVarValue ' + name + ' unit : ' + unit);
      return null;
    }
  };
  SimVar.GetSimVarValueFast = (name, unit, dataSource = '') => simvar.getValue(name, unit, dataSource);
  SimVar.GetSimVarValueFastReg = (id) => simvar.getValueReg(id);
  SimVar.GetSimVarValueFastRegString = (id) => simvar.getValueReg_String(id);

  const structSetters = {
    latlonalt: 'setValue_LatLongAlt',
    latlonaltpbh: 'setValue_LatLongAltPBH',
    pbh: 'setValue_PBH',
    pid_struct: 'setValue_PID_STRUCT',
    xyz: 'setValue_XYZ',
  };

  SimVar.SetSimVarValue = (name, unit, value, dataSource = '') => {
    if (value == undefined) {
      console.warn(name + ' : Trying to set a null value');
      return undefined;
    }
    const structCall = structSetters[String(unit).toLowerCase()];
    if (structCall !== undefined) {
      return Coherent.call(structCall, name, value, dataSource);
    }
    switch (String(unit).toLowerCase()) {
      case 'string':
        return Coherent.call('setValue_String', name, value, dataSource);
      case 'bool':
      case 'boolean':
        return Coherent.call('setValue_Bool', name, !!value, dataSource);
      default:
        return Coherent.call('setValue_Number', name, unit, value, dataSource);
    }
  };

  class SimVarBatch {
    constructor(simVarCount, simVarIndex) {
      this.__Type = 'SimVarBatch';
      this.wantedNames = [];
      this.wantedUnits = [];
      this.wantedTypes = [];
      this.requestID = -1;
      this.instrumentID = '';
      this.simVarCount = simVarCount;
      this.simVarIndex = simVarIndex;
    }
    add(name, unit, type = '') {
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
  }
  SimVar.SimVarBatch = SimVarBatch;

  SimVar.GetSimVarArrayValues = (batch, callback, dataSource = '') => {
    Coherent.call('getArrayValues', batch.getCount(), batch.getIndex(), batch.getNames(), batch.getUnits(), dataSource).then(callback);
  };

  SimVar.GetGlobalVarValue = (name, unit) => globalvar.getValue(name, unit);

  SimVar.GetRegisteredGameVarId = (name, unit) => gamevar.registerGameVarWatcher(String(name).replace(/\s/g, '_'), unit);
  SimVar.GetGameVarValue = (name, unit, param1 = 0, param2 = 0) => {
    const n = String(name).replace(/\s/g, '_');
    switch (String(unit).toLowerCase()) {
      case 'string':
        return gamevar.getValue_String(n, param1);
      case 'xyz':
        return gamevar.getValue_XYZ(n, param1);
      default:
        return gamevar.getValue(n, unit, param1, param2);
    }
  };
  SimVar.GetGameVarValueFast = (name, unit, param1 = 0, param2 = 0) => gamevar.getValue(name, unit, param1, param2);
  SimVar.SetGameVarValue = (name, unit, value) => {
    once(`setting game variable ${name}`);
    return Promise.resolve();
  };

  globalThis.SimVar = SimVar;
})();
