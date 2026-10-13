import {
  AeroMath,
  AiracCycleFormatter,
  ArrayUtils,
  BitFlags,
  DateTimeFormatter,
  DmsFormatter2,
  ExpSmoother,
  LerpLookupTable,
  MathUtils,
  NumberFormatter,
  ResourceHeap,
  Rounding,
  UnitType,
  UUID,
  Vec2Math,
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

near(UnitType.METER.convertFrom(10000, UnitType.FOOT), 3048, 1e-9, 'ft to m');
near(UnitType.KNOT.convertTo(100, UnitType.MPS), 51.4444444, 1e-6, 'kt to mps');
near(UnitType.CELSIUS.convertFrom(32, UnitType.FAHRENHEIT), 0, 1e-9, 'F to C');
near(UnitType.FAHRENHEIT.convertFrom(100, UnitType.CELSIUS), 212, 1e-9, 'C to F');
near(UnitType.KELVIN.convertFrom(0, UnitType.CELSIUS), 273.15, 1e-9, 'C to K');
near(UnitType.DELTA_FAHRENHEIT.convertFrom(10, UnitType.DELTA_CELSIUS), 18, 1e-9, 'delta');
near(UnitType.HPA.convertFrom(29.92, UnitType.IN_HG), 1013.2, 0.1, 'inHg');
near(UnitType.FPM.convertFrom(1, UnitType.MPS), 196.85, 0.01, 'mps to fpm');
near(UnitType.GALLON_JET_A_FUEL.scaleFactor, 6.7 * 0.45359237, 1e-9, 'jet a gallon');
near(UnitType.KGH.convertFrom(1000, UnitType.PPH), 453.59237, 1e-6, 'pph');
let threw = false;
try {
  UnitType.METER.convertFrom(1, UnitType.KILOGRAM as any);
} catch {
  threw = true;
}
eq(threw, true, 'invalid conversion throws');

const weight = UnitType.KILOGRAM.createNumber(100);
eq(weight.asUnit(UnitType.POUND) > 220, true, 'asUnit');
weight.add(1, UnitType.TONNE);
eq(weight.number, 1100, 'add in place');
const out = UnitType.POUND.createNumber(0);
weight.subtract(UnitType.KILOGRAM.createNumber(100), out);
near(out.number, 1000 / 0.45359237, 1e-9, 'subtract to out');
eq(weight.readonly.number, 1100, 'readonly view');
eq(weight.equals(1.1, UnitType.TONNE), true, 'equals');
eq(weight.compare(1, UnitType.TONNE), 1, 'compare');
eq(UnitType.KILOGRAM.createNumber(NaN).equals(UnitType.POUND.createNumber(NaN)), true, 'nan equals');

eq(MathUtils.round(123.46, 0.1).toFixed(1), '123.5', 'round');
eq(MathUtils.round(1234, 10), 1230, 'round 10');
eq(MathUtils.ceil(1231, 10), 1240, 'ceil');
eq(MathUtils.floor(1239, 10), 1230, 'floor');
eq(MathUtils.normalizeAngleDeg(-10), 350, 'norm deg');
eq(MathUtils.normalizeAngleDeg(370, -180), 10, 'norm deg lb');
eq(MathUtils.diffAngleDeg(350, 10), 20, 'diff dir');
eq(MathUtils.diffAngleDeg(10, 350, false), 20, 'diff nondir');
eq(MathUtils.lerp(5, 0, 10, 0, 100), 50, 'lerp');
eq(MathUtils.lerp(15, 0, 10, 0, 100, false, true), 100, 'lerp clamp');
eq(MathUtils.hardSign(-0), -1, 'hardSign');
eq(MathUtils.driveLinear(0, 10, 2, 3), 6, 'driveLinear');
eq(MathUtils.driveLinear(0, 10, 5, 3), 10, 'driveLinear clamp');

const v = Vec2Math.create(3, 4);
eq(Vec2Math.abs(v), 5, 'vec abs');
near(Vec2Math.theta(Vec2Math.setFromPolar(2, 1, Vec2Math.create())), 1, 1e-12, 'polar');
eq(
  Vec2Math.pointWithinPolygon(
    [Vec2Math.create(0, 0), Vec2Math.create(4, 0), Vec2Math.create(4, 4), Vec2Math.create(0, 0)],
    Vec2Math.create(3, 1),
  ),
  true,
  'polygon',
);

eq(BitFlags.isAny(0b101, 0b100), true, 'isAny');
eq(BitFlags.isAll(0b101, 0b111), false, 'isAll');
eq(BitFlags.set(0b1010, 0b0101, 0b0011), 0b1001, 'set');

const smoother = new ExpSmoother(1, null);
eq(smoother.next(10, 1), 10, 'smoother first');
near(smoother.next(0, 1), 10 * Math.exp(-1), 1e-12, 'smoother second');

const table = new LerpLookupTable([
  [0, 0],
  [100, 10],
  [200, 20],
]);
eq(table.get(5), 50, 'lerp table');
eq(table.get(-5), 0, 'lerp table low clamp');
eq(table.get(50), 200, 'lerp table high clamp');
const table2 = new LerpLookupTable([
  [0, 0, 0],
  [10, 0, 1],
  [20, 1, 0],
  [40, 1, 1],
]);
eq(table2.get(0.5, 0.5), 17.5, 'lerp table 2d');

eq(ArrayUtils.range(3, 1, 2).join(), '1,3,5', 'range');
eq(ArrayUtils.flat([[1], [2, [3]]]).length, 3, 'flat');
eq(ArrayUtils.binarySearch([1, 3, 5], 4, (a, b) => a - b), -3, 'binary search');

let created = 0;
const heap = new ResourceHeap<{ id: number }>(
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
eq(/^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/.test(UUID.GenerateUuid()), true, 'uuid');

near(AeroMath.isaPressure(0), 1013.25, 1e-6, 'isa p0');
near(AeroMath.isaPressure(11000), 226.32, 0.01, 'isa p11');
near(AeroMath.isaTemperature(11000), -56.5, 1e-9, 'isa t11');
near(AeroMath.isaPressure(UnitType.METER.convertFrom(27000, UnitType.FOOT)), 344.3, 0.2, 'isa fl270');
near(AeroMath.isaAltitude(226.32), 11000, 1, 'isa alt');
near(AeroMath.casToMach(UnitType.MPS.convertFrom(250, UnitType.KNOT), 1013.25), 0.3777, 0.001, 'cas mach sea');
near(AeroMath.machToCas(AeroMath.casToMach(150, 400), 400), 150, 1e-6, 'cas roundtrip');

const f1 = NumberFormatter.create({ precision: 0.1, nanString: '---.-' });
eq(f1(123.456), '123.5', 'fmt 0.1');
eq(f1(5), '5.0', 'fmt zero pad decimals');
eq(f1(NaN), '---.-', 'fmt nan');
eq(NumberFormatter.create({ precision: 1 })(1234.5), '1235', 'fmt int');
eq(NumberFormatter.create({ precision: 1, pad: 3 })(7), '007', 'fmt pad');
eq(NumberFormatter.create({ precision: 1, pad: 3, maxDigits: 4 })(7.4), '007', 'fmt pad maxdigits');
eq(NumberFormatter.create({ precision: 0.01, maxDigits: 4 })(12.345), '12.35', 'fmt maxdigits');
eq(NumberFormatter.create({ precision: 0.01, maxDigits: 4 })(123.456), '123.5', 'fmt maxdigits shrink');
eq(NumberFormatter.create({ precision: 0.01, maxDigits: 4 })(1234.56), '1235', 'fmt maxdigits int');
eq(NumberFormatter.create({ precision: 0.01, forceDecimalZeroes: false })(1.5), '1.5', 'fmt no zeroes');
eq(NumberFormatter.create({ precision: 1, showCommas: true })(1234567), '1,234,567', 'fmt commas');
eq(NumberFormatter.create({ precision: 1, forceSign: true })(5), '+5', 'fmt force sign');
eq(NumberFormatter.create({ precision: 1, useMinusSign: true })(-5), '−5', 'fmt minus sign');
eq(NumberFormatter.create({ precision: 1, hideSign: true })(-5), '5', 'fmt hide sign');
eq(NumberFormatter.create({ precision: 10, round: Rounding.Down })(1239), '1230', 'fmt round down');
eq(NumberFormatter.create({ precision: 10, round: Rounding.Up })(1231), '1240', 'fmt round up');
eq(NumberFormatter.create({})(1.25), '1.25', 'fmt infinite precision');
eq(NumberFormatter.create({})(Infinity), 'Infinity', 'fmt inf');
eq(NumberFormatter.create({ pad: 0, precision: 0.1 })(0.5), '.5', 'fmt pad 0');
const hyst = NumberFormatter.create({ precision: 1, hysteresis: 0.25 });
eq(hyst(10), '10', 'hyst first');
eq(hyst(10.7), '10', 'hyst within');
eq(hyst(10.8), '11', 'hyst beyond');

const time = Date.UTC(2020, 7, 18, 4, 0, 0);
eq(DateTimeFormatter.create('{dd}-{MM}-{YY}')(0), '01-01-70', 'dt date');
eq(DateTimeFormatter.create('{YYYY}-{MM}-{dd}T{HH}:{mm}:{ss}')(time), '2020-08-18T04:00:00', 'dt iso');
eq(DateTimeFormatter.create('{hh}:{mm}:{ss} {am}')(5145000), '01:25:45 am', 'dt 12h');
eq(DateTimeFormatter.create('{hh}:{mm}:{ss} {am}')(0), '12:00:00 am', 'dt midnight');
eq(DateTimeFormatter.create('{hh}:{mm}:{ss} {am}')(57600000), '04:00:00 pm', 'dt pm');
eq(DateTimeFormatter.create('{H}:{mm.m(m)}')(5145000), '1:25.75', 'dt decimals');
eq(DateTimeFormatter.create('{H}:{mm.m(m)}')(0), '0:00.0', 'dt decimals zero');
eq(DateTimeFormatter.create('{dd}{MON}{YY}')(time), '18AUG20', 'dt MON');
eq(DateTimeFormatter.create('{HH}:{mm}', { nanString: '--:--' })(NaN), '--:--', 'dt nan');
eq(DateTimeFormatter.create('{DAY} {day} {dy.} {DY}')(time), 'TUESDAY tuesday tue. TUE', 'dt day names');

const cycle2309 = {
  ident: '2309',
  cycle: 9,
  cycleString: '09',
  effectiveTimestamp: Date.UTC(2023, 8, 7),
  expirationTimestamp: Date.UTC(2023, 9, 5),
};
eq(
  AiracCycleFormatter.create('{YYCC} {eff({dd}-{MON})}/{exp({dd}-{MON})}')(cycle2309),
  '2309 07-SEP/05-OCT',
  'airac exp',
);
eq(
  AiracCycleFormatter.create('{YYCC} {eff({dd}-{MON})}/{expMinus({dd}-{MON})}')(cycle2309),
  '2309 07-SEP/04-OCT',
  'airac expMinus',
);
eq(
  AiracCycleFormatter.create('{eff({dd}{MON}{YY})}-{exp({dd}{MON}{YY})}({YYCC}) {CC}')(cycle2309),
  '07SEP23-05OCT23(2309) 09',
  'airac combined',
);
let badAirac = false;
try {
  AiracCycleFormatter.create('{nope}');
} catch {
  badAirac = true;
}
eq(badAirac, true, 'airac invalid throws');

const dms = DmsFormatter2.create('{d}°{mm}\'{ss}"', UnitType.DEGREE, 0);
eq(dms(10), '10°00\'00"', 'dms zero');
eq(dms(10.51), '10°30\'36"', 'dms 10.51');
eq(DmsFormatter2.create('{d}°{mm}\'{ss.s(s)}"', UnitType.DEGREE, 0)(10.09169), '10°05\'30.08"', 'dms decimals');
eq(DmsFormatter2.create('{d}°{mm}\'{ss.s(s)}"', UnitType.DEGREE, 0)(10), '10°00\'00.0"', 'dms forced');
eq(DmsFormatter2.create('{d}°{mm.mm}\'', UnitType.DEGREE, 0)(10.09169), '10°05.50\'', 'dms min decimals');
eq(DmsFormatter2.create('{-}{d}°{mm}\'', UnitType.DEGREE, 0)(-10), '-10°00\'', 'dms sign');
eq(DmsFormatter2.create('{+-}{d}°{mm}\'', UnitType.DEGREE, 0)(10), '+10°00\'', 'dms plus');
eq(DmsFormatter2.create('{-[–]}{d}°{mm}\'', UnitType.DEGREE, 0)(-10), '–10°00\'', 'dms real minus');
eq(DmsFormatter2.create('{dd}°{mm}.{s}{+[N]-[S]}', UnitType.DEGREE, 0.1)(40.5), '40°30.0N', 'dms lat');
eq(DmsFormatter2.create('{ddd}°{mm}.{s}{+[E]-[W]}', UnitType.DEGREE, 0.1)(-73.8), '073°48.0W', 'dms lon');
eq(DmsFormatter2.create('{d}°{mm}', UnitType.DEGREE, 0)(0.3), '0°18', 'dms float safe');
eq(DmsFormatter2.create('{d}', UnitType.DEGREE, 0, 'x')(NaN), 'x', 'dms nan');

console.log(`${count - failures}/${count} passed`);
if (failures > 0) {
  process.exit(1);
}
