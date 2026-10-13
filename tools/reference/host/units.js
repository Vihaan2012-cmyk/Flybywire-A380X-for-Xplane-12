// SimVar unit conversion, as MSFS applies it when a script asks for a variable
// in a different unit than the one it is stored in. The state file gives
// each value with its unit; reads convert to the unit the instrument asks for.
// Units are MSFS's names (SimConnect SDK, "Simulation Variable Units").
(function (global) {
  'use strict';

  // [dimension, factor to base, offset to base]. base = factor * value + offset.
  const U = {};
  const def = (dim, factor, names, offset = 0) => {
    for (const n of names) U[n.toLowerCase()] = { dim, factor, offset };
  };

  // Length, base metre.
  def('length', 1, ['meter', 'meters', 'metre', 'metres', 'm']);
  def('length', 0.3048, ['foot', 'feet', 'ft']);
  def('length', 0.0254, ['inch', 'inches', 'in']);
  def('length', 0.01, ['centimeter', 'centimeters', 'cm']);
  def('length', 0.001, ['millimeter', 'millimeters', 'mm']);
  def('length', 1000, ['kilometer', 'kilometers', 'km']);
  def('length', 1852, ['nautical mile', 'nautical miles', 'nmile', 'nmiles', 'nm']);
  def('length', 1609.344, ['mile', 'miles']);
  def('length', 185.2, ['decinmile', 'decinmiles']);
  def('length', 0.9144, ['yard', 'yards', 'yd']);
  // Speed, base metre per second.
  def('speed', 1, ['meter per second', 'meters per second', 'm/s']);
  def('speed', 1852 / 3600, ['knot', 'knots', 'kt', 'kts', 'kias', 'ktas']);
  def('speed', 1000 / 3600, ['kilometer per hour', 'kilometers per hour', 'km/h', 'kph']);
  def('speed', 1609.344 / 3600, ['mile per hour', 'miles per hour', 'mph']);
  def('speed', 0.3048 / 60, ['foot per minute', 'feet per minute', 'feet/minute', 'ft/min', 'fpm']);
  def('speed', 0.3048, ['foot per second', 'feet per second', 'feet/second', 'ft/s']);
  def('speed', 1 / 60, ['meter per minute', 'meters per minute', 'm/min']);
  // Acceleration, base m/s^2.
  def('accel', 1, ['meter per second squared', 'meters per second squared']);
  def('accel', 0.3048, ['foot per second squared', 'feet per second squared']);
  def('accel', 9.80665, ['g force', 'gforce']);
  // Angle, base radian.
  def('angle', 1, ['radian', 'radians', 'rad']);
  def('angle', Math.PI / 180, ['degree', 'degrees', 'deg', 'degree latitude', 'degrees latitude', 'degree longitude', 'degrees longitude']);
  def('angle', (2 * Math.PI) / 65536, ['degree angl16', 'degrees angl16', 'angl16']);
  def('angle', (2 * Math.PI) / 4294967296, ['degree angl32', 'degrees angl32', 'angl32']);
  def('angle', Math.PI / 200, ['grad', 'grads']);
  // Angular rate, base radian per second.
  def('angrate', 1, ['radian per second', 'radians per second']);
  def('angrate', Math.PI / 180, ['degree per second', 'degrees per second']);
  def('angrate', Math.PI / 180 / 60, ['degree per minute', 'degrees per minute']);
  def('angrate', (2 * Math.PI) / 60, ['rpm', 'rpms', 'revolution per minute', 'revolutions per minute']);
  // Temperature, base kelvin.
  def('temp', 1, ['kelvin']);
  def('temp', 1, ['celsius', 'degree celsius', 'degrees celsius', 'c'], 273.15);
  def('temp', 5 / 9, ['fahrenheit', 'farenheit', 'degree fahrenheit', 'degrees fahrenheit', 'f'], 273.15 - (32 * 5) / 9);
  def('temp', 5 / 9, ['rankine']);
  // Pressure, base pascal.
  def('pressure', 1, ['pascal', 'pascals', 'pa']);
  def('pressure', 100, ['millibar', 'millibars', 'mbar', 'mbars', 'hectopascal', 'hectopascals', 'hpa']);
  def('pressure', 100000, ['bar', 'bars']);
  def('pressure', 1000, ['kilopascal', 'kilopascals', 'kpa']);
  def('pressure', 3386.389, ['inch of mercury', 'inches of mercury', 'inhg']);
  def('pressure', 1333.22, ['centimeter of mercury', 'centimeters of mercury', 'cmhg']);
  def('pressure', 6894.757, ['psi', 'pound per square inch', 'pounds per square inch']);
  def('pressure', 47.880259, ['psf', 'pound per square foot', 'pounds per square foot']);
  def('pressure', 101325, ['atmosphere', 'atmospheres', 'atm']);
  def('pressure', 100 / 16, ['millibar scaler 16', 'millibars scaler 16']);
  // Mass, base kilogram.
  def('mass', 1, ['kilogram', 'kilograms', 'kg', 'kgs']);
  def('mass', 0.45359237, ['pound', 'pounds', 'lb', 'lbs']);
  def('mass', 1000, ['tonne', 'tonnes']);
  def('mass', 14.5939, ['slug', 'slugs']);
  // Mass flow, base kg/s.
  def('massflow', 1, ['kilogram per second', 'kilograms per second']);
  def('massflow', 0.45359237 / 3600, ['pound per hour', 'pounds per hour', 'lbs/hr', 'lbs per hour']);
  def('massflow', 1 / 3600, ['kilogram per hour', 'kilograms per hour', 'kg/hr']);
  // Volume, base cubic metre.
  def('volume', 1, ['cubic meter', 'cubic meters', 'cu m', 'm3']);
  def('volume', 0.001, ['liter', 'liters', 'litre', 'litres']);
  def('volume', 0.003785411784, ['gallon', 'gallons', 'gal']);
  def('volume', 0.000946352946, ['quart', 'quarts']);
  def('volume', 0.0283168, ['cubic foot', 'cubic feet', 'cu ft', 'ft3']);
  // Volume flow, base m3/s.
  def('volflow', 0.003785411784 / 3600, ['gallon per hour', 'gallons per hour', 'gph']);
  // Time, base second.
  def('time', 1, ['second', 'seconds', 'sec', 'secs', 's']);
  def('time', 0.001, ['millisecond', 'milliseconds', 'ms']);
  def('time', 60, ['minute', 'minutes', 'min']);
  def('time', 3600, ['hour', 'hours', 'hr']);
  def('time', 86400, ['day', 'days']);
  def('time', 360, ['hour over 10', 'hours over 10']);
  // Frequency, base hertz.
  def('freq', 1, ['hertz', 'hz']);
  def('freq', 1000, ['kilohertz', 'khz']);
  def('freq', 1000000, ['megahertz', 'mhz']);
  // Ratios.
  def('ratio', 0.01, ['percent', 'percentage']);
  def('ratio', 1, ['percent over 100', 'percentage over 100', 'ratio', 'part', 'parts', 'scalar']);
  def('ratio', 1 / 16384, ['position 16k']);
  def('ratio', 1 / 32767, ['position 32k']);
  def('ratio', 1 / 255, ['position 255']);
  def('ratio', 1 / 128, ['position 128']);
  def('ratio', 1, ['position']);
  // Electrical.
  def('current', 1, ['ampere', 'amperes', 'amp', 'amps', 'a']);
  def('voltage', 1, ['volt', 'volts', 'v']);
  def('power', 1, ['watt', 'watts', 'w']);
  def('power', 745.7, ['horsepower', 'hp']);
  def('power', 1.3558179483314, ['ft lb per second']);
  // Density, torque.
  def('density', 1, ['kilogram per cubic meter', 'kilograms per cubic meter']);
  def('density', 515.3788, ['slug per cubic foot', 'slugs per cubic foot']);
  def('torque', 1, ['newton meter', 'newton meters', 'nm torque']);
  def('torque', 1.3558179483314, ['foot pound', 'foot pounds', 'foot-pound', 'foot-pounds', 'ft-lbs']);

  // Units that carry no dimension: the stored number is returned unchanged.
  const PASSTHROUGH = new Set([
    '', 'number', 'numbers', 'enum', 'enums', 'bool', 'boolean', 'bools', 'mask', 'flags', 'bco16', 'bcd16', 'bcd32',
    'frequency bcd16', 'frequency bcd32', 'frequency adf bcd32', 'string', 'strings', 'index', 'integer',
  ]);

  function lookup(unit) {
    return U[String(unit ?? '').trim().toLowerCase()];
  }

  /**
   * Convert `value` stored in `from` to `to`. Returns { value, ok } where ok is
   * false when the units are unknown or of different dimensions (value is
   * returned unchanged then, and the mock logs it).
   */
  function convert(value, from, to) {
    const f = String(from ?? '').trim().toLowerCase();
    const t = String(to ?? '').trim().toLowerCase();
    if (typeof value !== 'number') return { value, ok: true };
    if (f === t || PASSTHROUGH.has(t) || PASSTHROUGH.has(f)) {
      if ((t === 'bool' || t === 'boolean') && !PASSTHROUGH.has(f)) return { value: value ? 1 : 0, ok: true };
      return { value, ok: true };
    }
    const uf = lookup(f);
    const ut = lookup(t);
    if (!uf || !ut || uf.dim !== ut.dim) return { value, ok: false };
    const base = value * uf.factor + uf.offset;
    return { value: (base - ut.offset) / ut.factor, ok: true };
  }

  global.__refUnits = { convert, known: (u) => PASSTHROUGH.has(String(u ?? '').trim().toLowerCase()) || !!lookup(u) };
  if (typeof module !== 'undefined') module.exports = global.__refUnits;
})(typeof globalThis !== 'undefined' ? globalThis : this);
