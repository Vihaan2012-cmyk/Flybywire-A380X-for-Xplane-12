import {
  Aero,
  CycleFormat,
  Arr,
  Bits,
  TimeFormat,
  DmsFormat,
  Smoother,
  LerpTable,
  Num,
  NumFormat,
  Pool,
  RoundMode,
  Units,
  Uuid,
  V2,
} from '../index';

let failures = 0;
let count = 0;

function eq(actual: unknown, expected: unknown, label: string): void {
  count++;
  if (actual !== expected && !(Number.isNaN(actual) && Number.isNaN(expected))) {
    failures++;
    console.log(`FAIL ${label}: expected ${JSON.stringify(expected)} got ${JSON.stringify(actual)}`);
  }
}

function near(actual: number, expected: number, tolerance: number, label: string): void {
  count++;
  if (!(Math.abs(actual - expected) <= tolerance)) {
    failures++;
    console.log(`FAIL ${label}: expected ~${expected} got ${actual}`);
  }
}

near(Units.METER.convertFrom(10000, Units.FOOT), 3048, 1e-9, 'ft to m');
near(Units.KNOT.convertTo(100, Units.MPS), 51.4444444, 1e-6, 'kt to mps');
near(Units.CELSIUS.convertFrom(32, Units.FAHRENHEIT), 0, 1e-9, 'F to C');
near(Units.FAHRENHEIT.convertFrom(100, Units.CELSIUS), 212, 1e-9, 'C to F');
near(Units.KELVIN.convertFrom(0, Units.CELSIUS), 273.15, 1e-9, 'C to K');
near(Units.DELTA_FAHRENHEIT.convertFrom(10, Units.DELTA_CELSIUS), 18, 1e-9, 'delta');
near(Units.HPA.convertFrom(29.92, Units.IN_HG), 1013.2, 0.1, 'inHg');
near(Units.FPM.convertFrom(1, Units.MPS), 196.85, 0.01, 'mps to fpm');
near(Units.GALLON_JET_A_FUEL.scaleFactor, 6.7 * 0.45359237, 1e-9, 'jet a gallon');
near(Units.KGH.convertFrom(1000, Units.PPH), 453.59237, 1e-6, 'pph');
let threw = false;
try {
  Units.METER.convertFrom(1, Units.KILOGRAM as any);
} catch {
  threw = true;
}
eq(threw, true, 'invalid conversion throws');

const weight = Units.KILOGRAM.createNumber(100);
eq(weight.asUnit(Units.POUND) > 220, true, 'asUnit');
weight.add(1, Units.TONNE);
eq(weight.number, 1100, 'add in place');
const out = Units.POUND.createNumber(0);
weight.subtract(Units.KILOGRAM.createNumber(100), out);
near(out.number, 1000 / 0.45359237, 1e-9, 'subtract to out');
eq(weight.readonly.number, 1100, 'readonly view');
eq(weight.equals(1.1, Units.TONNE), true, 'equals');
eq(weight.compare(1, Units.TONNE), 1, 'compare');
eq(Units.KILOGRAM.createNumber(NaN).equals(Units.POUND.createNumber(NaN)), true, 'nan equals');

eq(Num.round(123.46, 0.1).toFixed(1), '123.5', 'round');
eq(Num.round(1234, 10), 1230, 'round 10');
eq(Num.ceil(1231, 10), 1240, 'ceil');
eq(Num.floor(1239, 10), 1230, 'floor');
eq(Num.normalizeAngleDeg(-10), 350, 'norm deg');
eq(Num.normalizeAngleDeg(370, -180), 10, 'norm deg lb');
eq(Num.diffAngleDeg(350, 10), 20, 'diff dir');
eq(Num.diffAngleDeg(10, 350, false), 20, 'diff nondir');
eq(Num.lerp(5, 0, 10, 0, 100), 50, 'lerp');
eq(Num.lerp(15, 0, 10, 0, 100, false, true), 100, 'lerp clamp');
eq(Num.hardSign(-0), -1, 'hardSign');
eq(Num.driveLinear(0, 10, 2, 3), 6, 'driveLinear');
eq(Num.driveLinear(0, 10, 5, 3), 10, 'driveLinear clamp');

const v = V2.create(3, 4);
eq(V2.abs(v), 5, 'vec abs');
near(V2.theta(V2.setFromPolar(2, 1, V2.create())), 1, 1e-12, 'polar');
eq(
  V2.pointWithinPolygon(
    [V2.create(0, 0), V2.create(4, 0), V2.create(4, 4), V2.create(0, 0)],
    V2.create(3, 1),
  ),
  true,
  'polygon',
);

eq(Bits.isAny(0b101, 0b100), true, 'isAny');
eq(Bits.isAll(0b101, 0b111), false, 'isAll');
eq(Bits.set(0b1010, 0b0101, 0b0011), 0b1001, 'set');

const smoother = new Smoother(1, null);
eq(smoother.next(10, 1), 10, 'smoother first');
near(smoother.next(0, 1), 10 * Math.exp(-1), 1e-12, 'smoother second');

const table = new LerpTable([
  [0, 0],
  [100, 10],
  [200, 20],
]);
eq(table.get(5), 50, 'lerp table');
eq(table.get(-5), 0, 'lerp table low clamp');
eq(table.get(50), 200, 'lerp table high clamp');
const table2 = new LerpTable([
  [0, 0, 0],
  [10, 0, 1],
  [20, 1, 0],
  [40, 1, 1],
]);
eq(table2.get(0.5, 0.5), 17.5, 'lerp table 2d');

eq(Arr.range(3, 1, 2).join(), '1,3,5', 'range');
eq(Arr.flat([[1], [2, [3]]]).length, 3, 'flat');
eq(Arr.binarySearch([1, 3, 5], 4, (a, b) => a - b), -3, 'binary search');

let created = 0;
const heap = new Pool<{ id: number }>(
  () => ({ id: created++ }),
  () => undefined,
  undefined,
  undefined,
  0,
  2,
);
const first = heap.allocate();
heap.allocate();
let exhausted = false;
try {
  heap.allocate();
} catch {
  exhausted = true;
}
eq(exhausted, true, 'heap limit');
heap.free(first);
eq(heap.allocate(), first, 'heap reuse');
eq(/^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/.test(Uuid.GenerateUuid()), true, 'uuid');

near(Aero.isaPressure(0), 1013.25, 1e-6, 'isa p0');
near(Aero.isaPressure(11000), 226.32, 0.01, 'isa p11');
near(Aero.isaTemperature(11000), -56.5, 1e-9, 'isa t11');
near(Aero.isaPressure(Units.METER.convertFrom(27000, Units.FOOT)), 344.3, 0.2, 'isa fl270');
near(Aero.isaAltitude(226.32), 11000, 1, 'isa alt');
near(Aero.casToMach(Units.MPS.convertFrom(250, Units.KNOT), 1013.25), 0.3777, 0.001, 'cas mach sea');
near(Aero.machToCas(Aero.casToMach(150, 400), 400), 150, 1e-6, 'cas roundtrip');

const f1 = NumFormat.create({ precision: 0.1, nanString: '---.-' });
eq(f1(123.456), '123.5', 'fmt 0.1');
eq(f1(5), '5.0', 'fmt zero pad decimals');
eq(f1(NaN), '---.-', 'fmt nan');
eq(NumFormat.create({ precision: 1 })(1234.5), '1235', 'fmt int');
eq(NumFormat.create({ precision: 1, pad: 3 })(7), '007', 'fmt pad');
eq(NumFormat.create({ precision: 1, pad: 3, maxDigits: 4 })(7.4), '007', 'fmt pad maxdigits');
eq(NumFormat.create({ precision: 0.01, maxDigits: 4 })(12.345), '12.35', 'fmt maxdigits');
eq(NumFormat.create({ precision: 0.01, maxDigits: 4 })(123.456), '123.5', 'fmt maxdigits shrink');
eq(NumFormat.create({ precision: 0.01, maxDigits: 4 })(1234.56), '1235', 'fmt maxdigits int');
eq(NumFormat.create({ precision: 0.01, forceDecimalZeroes: false })(1.5), '1.5', 'fmt no zeroes');
eq(NumFormat.create({ precision: 1, showCommas: true })(1234567), '1,234,567', 'fmt commas');
eq(NumFormat.create({ precision: 1, forceSign: true })(5), '+5', 'fmt force sign');
eq(NumFormat.create({ precision: 1, useMinusSign: true })(-5), '−5', 'fmt minus sign');
eq(NumFormat.create({ precision: 1, hideSign: true })(-5), '5', 'fmt hide sign');
eq(NumFormat.create({ precision: 10, round: RoundMode.Down })(1239), '1230', 'fmt round down');
eq(NumFormat.create({ precision: 10, round: RoundMode.Up })(1231), '1240', 'fmt round up');
eq(NumFormat.create({})(1.25), '1.25', 'fmt infinite precision');
eq(NumFormat.create({})(Infinity), 'Infinity', 'fmt inf');
eq(NumFormat.create({ pad: 0, precision: 0.1 })(0.5), '.5', 'fmt pad 0');
const hyst = NumFormat.create({ precision: 1, hysteresis: 0.25 });
eq(hyst(10), '10', 'hyst first');
eq(hyst(10.7), '10', 'hyst within');
eq(hyst(10.8), '11', 'hyst beyond');

const time = Date.UTC(2020, 7, 18, 4, 0, 0);
eq(TimeFormat.create('{dd}-{MM}-{YY}')(0), '01-01-70', 'dt date');
eq(TimeFormat.create('{YYYY}-{MM}-{dd}T{HH}:{mm}:{ss}')(time), '2020-08-18T04:00:00', 'dt iso');
eq(TimeFormat.create('{hh}:{mm}:{ss} {am}')(5145000), '01:25:45 am', 'dt 12h');
eq(TimeFormat.create('{hh}:{mm}:{ss} {am}')(0), '12:00:00 am', 'dt midnight');
eq(TimeFormat.create('{hh}:{mm}:{ss} {am}')(57600000), '04:00:00 pm', 'dt pm');
eq(TimeFormat.create('{H}:{mm.m(m)}')(5145000), '1:25.75', 'dt decimals');
eq(TimeFormat.create('{H}:{mm.m(m)}')(0), '0:00.0', 'dt decimals zero');
eq(TimeFormat.create('{dd}{MON}{YY}')(time), '18AUG20', 'dt MON');
eq(TimeFormat.create('{HH}:{mm}', { nanString: '--:--' })(NaN), '--:--', 'dt nan');
eq(TimeFormat.create('{DAY} {day} {dy.} {DY}')(time), 'TUESDAY tuesday tue. TUE', 'dt day names');

const cycle2309 = {
  ident: '2309',
  cycle: 9,
  cycleString: '09',
  effectiveTimestamp: Date.UTC(2023, 8, 7),
  expirationTimestamp: Date.UTC(2023, 9, 5),
};
eq(
  CycleFormat.create('{YYCC} {eff({dd}-{MON})}/{exp({dd}-{MON})}')(cycle2309),
  '2309 07-SEP/05-OCT',
  'airac exp',
);
eq(
  CycleFormat.create('{YYCC} {eff({dd}-{MON})}/{expMinus({dd}-{MON})}')(cycle2309),
  '2309 07-SEP/04-OCT',
  'airac expMinus',
);
eq(
  CycleFormat.create('{eff({dd}{MON}{YY})}-{exp({dd}{MON}{YY})}({YYCC}) {CC}')(cycle2309),
  '07SEP23-05OCT23(2309) 09',
  'airac combined',
);
let badAirac = false;
try {
  CycleFormat.create('{nope}');
} catch {
  badAirac = true;
}
eq(badAirac, true, 'airac invalid throws');

const dms = DmsFormat.create('{d}°{mm}\'{ss}"', Units.DEGREE, 0);
eq(dms(10), '10°00\'00"', 'dms zero');
eq(dms(10.51), '10°30\'36"', 'dms 10.51');
eq(DmsFormat.create('{d}°{mm}\'{ss.s(s)}"', Units.DEGREE, 0)(10.09169), '10°05\'30.08"', 'dms decimals');
eq(DmsFormat.create('{d}°{mm}\'{ss.s(s)}"', Units.DEGREE, 0)(10), '10°00\'00.0"', 'dms forced');
eq(DmsFormat.create('{d}°{mm.mm}\'', Units.DEGREE, 0)(10.09169), '10°05.50\'', 'dms min decimals');
eq(DmsFormat.create('{-}{d}°{mm}\'', Units.DEGREE, 0)(-10), '-10°00\'', 'dms sign');
eq(DmsFormat.create('{+-}{d}°{mm}\'', Units.DEGREE, 0)(10), '+10°00\'', 'dms plus');
eq(DmsFormat.create('{-[–]}{d}°{mm}\'', Units.DEGREE, 0)(-10), '–10°00\'', 'dms real minus');
eq(DmsFormat.create('{dd}°{mm}.{s}{+[N]-[S]}', Units.DEGREE, 0.1)(40.5), '40°30.0N', 'dms lat');
eq(DmsFormat.create('{ddd}°{mm}.{s}{+[E]-[W]}', Units.DEGREE, 0.1)(-73.8), '073°48.0W', 'dms lon');
eq(DmsFormat.create('{d}°{mm}', Units.DEGREE, 0)(0.3), '0°18', 'dms float safe');
eq(DmsFormat.create('{d}', Units.DEGREE, 0, 'x')(NaN), 'x', 'dms nan');

console.log(`${count - failures}/${count} passed`);
if (failures > 0) {
  process.exit(1);
}
