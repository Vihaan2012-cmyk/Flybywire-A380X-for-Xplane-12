export type ReadonlyFloat64ArrayX = Readonly<Omit<Float64Array, 'set' | 'copyWithin' | 'sort'>>;

export interface LatLonInterfaceX {
  lat: number;
  lon: number;
}

export interface LatLongInterfaceX {
  lat: number;
  long: number;
}

export interface LatLonAltInterfaceX {
  lat: number;
  lon: number;
  alt: number;
}

export enum RoundMode {
  Down = -1,
  Nearest = 0,
  Up = 1,
}

export class Num {
  public static readonly TWO_PI = Math.PI * 2;
  public static readonly HALF_PI = Math.PI / 2;
  public static readonly THIRD_PI = Math.PI / 3;
  public static readonly QUARTER_PI = Math.PI / 4;
  public static readonly SIXTH_PI = Math.PI / 6;
  public static readonly SQRT3 = Math.sqrt(3);
  public static readonly SQRT1_3 = Math.sqrt(1 / 3);

  public static clamp(value: number, min: number, max: number): number {
    return Math.max(min, Math.min(max, value));
  }

  public static round(value: number, precision = 1): number {
    return Math.round(value / precision) * precision;
  }

  public static ceil(value: number, precision = 1): number {
    return Math.ceil(value / precision) * precision;
  }

  public static floor(value: number, precision = 1): number {
    return Math.floor(value / precision) * precision;
  }

  public static normalizeAngle(radians: number, lowerBound = 0): number {
    const twoPi = Num.TWO_PI;
    return ((((radians - lowerBound) % twoPi) + twoPi) % twoPi) + lowerBound;
  }

  public static normalizeAngleDeg(degrees: number, lowerBound = 0): number {
    return ((((degrees - lowerBound) % 360) + 360) % 360) + lowerBound;
  }

  public static angularDistance(startRadians: number, endRadians: number, direction: number): number {
    const forward = Num.diffAngle(startRadians, endRadians, true);
    if (direction > 0) {
      return forward;
    }
    if (direction < 0) {
      return forward === 0 ? 0 : forward - Num.TWO_PI;
    }
    return forward <= Math.PI ? forward : forward - Num.TWO_PI;
  }

  public static angularDistanceDeg(startDegrees: number, endDegrees: number, direction: number): number {
    const forward = Num.diffAngleDeg(startDegrees, endDegrees, true);
    if (direction > 0) {
      return forward;
    }
    if (direction < 0) {
      return forward === 0 ? 0 : forward - 360;
    }
    return forward <= 180 ? forward : forward - 360;
  }

  public static diffAngle(start: number, end: number, directional = true): number {
    const diff = Num.normalizeAngle(end - start);
    return directional ? diff : Math.min(diff, Num.TWO_PI - diff);
  }

  public static diffAngleDeg(start: number, end: number, directional = true): number {
    const diff = Num.normalizeAngleDeg(end - start);
    return directional ? diff : Math.min(diff, 360 - diff);
  }

  public static lerp(
    x: number,
    x0: number,
    x1: number,
    y0: number,
    y1: number,
    clampStart = false,
    clampEnd = false,
  ): number {
    const fraction = (x - x0) / (x1 - x0);
    let result = y0 + fraction * (y1 - y0);
    if (clampStart || clampEnd) {
      const rising = y1 >= y0;
      if (clampStart && (rising ? result < y0 : result > y0)) {
        result = y0;
      }
      if (clampEnd && (rising ? result > y1 : result < y1)) {
        result = y1;
      }
    }
    return result;
  }

  public static lerpVector(
    out: Float64Array,
    x: number,
    x0: number,
    x1: number,
    y0: ArrayLike<number>,
    y1: ArrayLike<number>,
    clampStart = false,
    clampEnd = false,
  ): Float64Array {
    const length = Math.min(out.length, y0.length, y1.length);
    for (let i = 0; i < length; i++) {
      out[i] = Num.lerp(x, x0, x1, y0[i], y1[i], clampStart, clampEnd);
    }
    return out;
  }

  public static hardSign(n: number): 1 | -1 | typeof NaN {
    if (Number.isNaN(n)) {
      return NaN;
    }
    return n < 0 || Object.is(n, -0) ? -1 : 1;
  }

  public static driveLinear(
    initialValue: number,
    targetValue: number,
    rate: number,
    dt: number,
    clampStart = true,
    clampEnd = true,
  ): number {
    if (initialValue === targetValue) {
      return targetValue;
    }
    const direction = targetValue > initialValue ? 1 : -1;
    let result = initialValue + direction * rate * dt;
    if (clampEnd && direction * (result - targetValue) > 0) {
      result = targetValue;
    }
    if (clampStart && direction * (result - initialValue) < 0) {
      result = initialValue;
    }
    return result;
  }

  public static driveExp(initialValue: number, targetValue: number, timeConstant: number, dt: number): number {
    return targetValue + (initialValue - targetValue) * Math.exp(-dt / timeConstant);
  }
}

export class V2 {
  public static create(): Float64Array;
  public static create(x: number, y: number): Float64Array;
  public static create(x?: number, y?: number): Float64Array {
    const vec = new Float64Array(2);
    if (x !== undefined && y !== undefined) {
      vec[0] = x;
      vec[1] = y;
    }
    return vec;
  }

  public static theta(vec: ReadonlyFloat64ArrayX): number {
    return Math.atan2(vec[1], vec[0]);
  }

  public static set(x: number, y: number, vec: Float64Array): Float64Array {
    vec[0] = x;
    vec[1] = y;
    return vec;
  }

  public static setFromPolar(r: number, theta: number, vec: Float64Array): Float64Array {
    vec[0] = r * Math.cos(theta);
    vec[1] = r * Math.sin(theta);
    return vec;
  }

  public static add(v1: ReadonlyFloat64ArrayX, v2: ReadonlyFloat64ArrayX, out: Float64Array): Float64Array {
    out[0] = v1[0] + v2[0];
    out[1] = v1[1] + v2[1];
    return out;
  }

  public static sub(v1: ReadonlyFloat64ArrayX, v2: ReadonlyFloat64ArrayX, out: Float64Array): Float64Array {
    out[0] = v1[0] - v2[0];
    out[1] = v1[1] - v2[1];
    return out;
  }

  public static dot(v1: ReadonlyFloat64ArrayX, v2: ReadonlyFloat64ArrayX): number {
    return v1[0] * v2[0] + v1[1] * v2[1];
  }

  public static det(v1: ReadonlyFloat64ArrayX, v2: ReadonlyFloat64ArrayX): number {
    return v1[0] * v2[1] - v1[1] * v2[0];
  }

  public static multScalar(v1: ReadonlyFloat64ArrayX, scalar: number, out: Float64Array): Float64Array {
    out[0] = v1[0] * scalar;
    out[1] = v1[1] * scalar;
    return out;
  }

  public static abs(v1: ReadonlyFloat64ArrayX): number {
    return Math.hypot(v1[0], v1[1]);
  }

  public static normalize(v1: ReadonlyFloat64ArrayX, out: Float64Array): Float64Array {
    const magnitude = V2.abs(v1);
    out[0] = v1[0] / magnitude;
    out[1] = v1[1] / magnitude;
    return out;
  }

  public static normal(v1: ReadonlyFloat64ArrayX, out: Float64Array, counterClockwise = true): Float64Array {
    const x = v1[0];
    const y = v1[1];
    const magnitude = V2.abs(v1);
    if (counterClockwise) {
      out[0] = -y / magnitude;
      out[1] = x / magnitude;
    } else {
      out[0] = y / magnitude;
      out[1] = -x / magnitude;
    }
    return out;
  }

  public static distance(vec1: ReadonlyFloat64ArrayX, vec2: ReadonlyFloat64ArrayX): number {
    return Math.hypot(vec2[0] - vec1[0], vec2[1] - vec1[1]);
  }

  public static angle(vec1: ReadonlyFloat64ArrayX, vec2: ReadonlyFloat64ArrayX): number {
    const cosine = V2.dot(vec1, vec2) / (V2.abs(vec1) * V2.abs(vec2));
    return Math.acos(Num.clamp(cosine, -1, 1));
  }

  public static unitAngle(vec1: ReadonlyFloat64ArrayX, vec2: ReadonlyFloat64ArrayX): number {
    return Math.acos(Num.clamp(V2.dot(vec1, vec2), -1, 1));
  }

  public static equals(vec1: ReadonlyFloat64ArrayX, vec2: ReadonlyFloat64ArrayX): boolean {
    return vec1[0] === vec2[0] && vec1[1] === vec2[1];
  }

  public static isFinite(vec: ReadonlyFloat64ArrayX): boolean {
    return Number.isFinite(vec[0]) && Number.isFinite(vec[1]);
  }

  public static copy(from: ReadonlyFloat64ArrayX, to: Float64Array): Float64Array {
    to[0] = from[0];
    to[1] = from[1];
    return to;
  }

  public static pointWithinPolygon(polygon: ReadonlyFloat64ArrayX[], point: Float64Array): boolean | undefined {
    if (polygon.length < 2) {
      return undefined;
    }
    const first = polygon[0];
    const last = polygon[polygon.length - 1];
    if (first[0] !== last[0] || first[1] !== last[1]) {
      throw new Error('First and last points in a polygon must be the same.');
    }
    const px = point[0];
    const py = point[1];
    let inside = false;
    for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
      const xi = polygon[i][0];
      const yi = polygon[i][1];
      const xj = polygon[j][0];
      const yj = polygon[j][1];
      const cross = (px - xi) * (yj - yi) - (py - yi) * (xj - xi);
      if (
        cross === 0 &&
        px >= Math.min(xi, xj) &&
        px <= Math.max(xi, xj) &&
        py >= Math.min(yi, yj) &&
        py <= Math.max(yi, yj)
      ) {
        return true;
      }
      if (yi > py !== yj > py && px < ((xj - xi) * (py - yi)) / (yj - yi) + xi) {
        inside = !inside;
      }
    }
    return inside;
  }
}

export class V3 {
  public static create(): Float64Array;
  public static create(x: number, y: number, z: number): Float64Array;
  public static create(x?: number, y?: number, z?: number): Float64Array {
    const vec = new Float64Array(3);
    if (x !== undefined && y !== undefined && z !== undefined) {
      vec[0] = x;
      vec[1] = y;
      vec[2] = z;
    }
    return vec;
  }

  public static theta(vec: ReadonlyFloat64ArrayX): number {
    return Math.acos(vec[2] / V3.abs(vec));
  }

  public static phi(vec: ReadonlyFloat64ArrayX): number {
    return Math.atan2(vec[1], vec[0]);
  }

  public static set(x: number, y: number, z: number, vec: Float64Array): Float64Array {
    vec[0] = x;
    vec[1] = y;
    vec[2] = z;
    return vec;
  }

  public static setFromSpherical(r: number, theta: number, phi: number, vec: Float64Array): Float64Array {
    const planar = r * Math.sin(theta);
    vec[0] = planar * Math.cos(phi);
    vec[1] = planar * Math.sin(phi);
    vec[2] = r * Math.cos(theta);
    return vec;
  }

  public static add(v1: ReadonlyFloat64ArrayX, v2: ReadonlyFloat64ArrayX, out: Float64Array): Float64Array {
    out[0] = v1[0] + v2[0];
    out[1] = v1[1] + v2[1];
    out[2] = v1[2] + v2[2];
    return out;
  }

  public static sub(v1: ReadonlyFloat64ArrayX, v2: ReadonlyFloat64ArrayX, out: Float64Array): Float64Array {
    out[0] = v1[0] - v2[0];
    out[1] = v1[1] - v2[1];
    out[2] = v1[2] - v2[2];
    return out;
  }

  public static dot(v1: ReadonlyFloat64ArrayX, v2: ReadonlyFloat64ArrayX): number {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
  }

  public static cross(v1: ReadonlyFloat64ArrayX, v2: ReadonlyFloat64ArrayX, out: Float64Array): Float64Array {
    const x = v1[1] * v2[2] - v1[2] * v2[1];
    const y = v1[2] * v2[0] - v1[0] * v2[2];
    const z = v1[0] * v2[1] - v1[1] * v2[0];
    out[0] = x;
    out[1] = y;
    out[2] = z;
    return out;
  }

  public static multScalar(v1: ReadonlyFloat64ArrayX, scalar: number, out: Float64Array): Float64Array {
    out[0] = v1[0] * scalar;
    out[1] = v1[1] * scalar;
    out[2] = v1[2] * scalar;
    return out;
  }

  public static abs(v1: ReadonlyFloat64ArrayX): number {
    return Math.hypot(v1[0], v1[1], v1[2]);
  }

  public static setMagnitude(v1: ReadonlyFloat64ArrayX, magnitude: number, out: Float64Array): Float64Array {
    return V3.multScalar(v1, magnitude / V3.abs(v1), out);
  }

  public static normalize(v1: ReadonlyFloat64ArrayX, out: Float64Array): Float64Array {
    return V3.multScalar(v1, 1 / V3.abs(v1), out);
  }

  public static distance(vec1: ReadonlyFloat64ArrayX, vec2: ReadonlyFloat64ArrayX): number {
    return Math.hypot(vec2[0] - vec1[0], vec2[1] - vec1[1], vec2[2] - vec1[2]);
  }

  public static angle(vec1: ReadonlyFloat64ArrayX, vec2: ReadonlyFloat64ArrayX): number {
    const cosine = V3.dot(vec1, vec2) / (V3.abs(vec1) * V3.abs(vec2));
    return Math.acos(Num.clamp(cosine, -1, 1));
  }

  public static unitAngle(vec1: ReadonlyFloat64ArrayX, vec2: ReadonlyFloat64ArrayX): number {
    return Math.acos(Num.clamp(V3.dot(vec1, vec2), -1, 1));
  }

  public static equals(vec1: ReadonlyFloat64ArrayX, vec2: ReadonlyFloat64ArrayX): boolean {
    return vec1[0] === vec2[0] && vec1[1] === vec2[1] && vec1[2] === vec2[2];
  }

  public static isFinite(vec: ReadonlyFloat64ArrayX): boolean {
    return Number.isFinite(vec[0]) && Number.isFinite(vec[1]) && Number.isFinite(vec[2]);
  }

  public static copy(from: ReadonlyFloat64ArrayX, to: Float64Array): Float64Array {
    to[0] = from[0];
    to[1] = from[1];
    to[2] = from[2];
    return to;
  }
}

export class Bits {
  public static createFlag(index: number): number {
    if (index < 0 || index > 32) {
      throw new Error(`Invalid bit flag index: ${index}.`);
    }
    return 1 << index;
  }

  public static not(flags: number, mask?: number): number {
    return mask === undefined ? ~flags : flags ^ mask;
  }

  public static union(...flags: number[]): number {
    let result = 0;
    for (const flag of flags) {
      result |= flag;
    }
    return result;
  }

  public static intersection(...flags: number[]): number {
    let result = ~0;
    for (const flag of flags) {
      result &= flag;
    }
    return result;
  }

  public static set(flags: number, valuesToSet: number, mask: number): number {
    return (flags & ~mask) | (valuesToSet & mask);
  }

  public static isAny(flags: number, conditions: number): boolean {
    return (flags & conditions) !== 0;
  }

  public static isAll(flags: number, conditions: number): boolean {
    return (flags & conditions) >>> 0 === conditions >>> 0;
  }

  public static forEach(
    flags: number,
    callback: (value: boolean, index: number, flags: number) => void,
    valueFilter?: boolean,
    startIndex = 0,
    endIndex = 32,
  ): void {
    for (let i = startIndex; i < endIndex; i++) {
      const value = (flags & (1 << i)) !== 0;
      if (valueFilter === undefined || valueFilter === value) {
        callback(value, i, flags);
      }
    }
  }
}

export class Smoother {
  private lastValue: number | null;

  public constructor(
    public readonly tau: number,
    initial: number | null = null,
    public readonly dtThreshold = Infinity,
  ) {
    this.lastValue = initial;
  }

  public last(): number | null {
    return this.lastValue;
  }

  public next(raw: number, dt: number): number {
    let next: number;
    if (this.lastValue === null || dt > this.dtThreshold) {
      next = raw;
    } else {
      next = Smoother.smooth(raw, this.lastValue, this.calculateFactor(dt));
    }
    this.lastValue = next;
    return next;
  }

  private calculateFactor(dt: number): number {
    if (this.tau <= 0) {
      return 1;
    }
    return 1 - Math.exp(-dt / this.tau);
  }

  public reset(): null;
  public reset<T extends number | null>(value: T): T;
  public reset(value?: number | null): number | null;
  public reset(value: number | null = null): number | null {
    return (this.lastValue = value);
  }

  private static smooth(value: number, last: number, factor: number): number {
    return value * factor + last * (1 - factor);
  }
}

type FlattenArray<A extends ReadonlyArray<any>> = A extends ReadonlyArray<infer T>
  ? Array<T extends ReadonlyArray<infer T1> ? T1 : T>
  : never;

type ArrayElement<A extends ReadonlyArray<any>> = A extends ReadonlyArray<infer T> ? T : never;

export class Arr {
  private static readonly STRICT_EQUALS = (a: unknown, b: unknown): boolean => a === b;

  public static create<T>(length: number, init: (index: number) => T): T[] {
    const array = new Array<T>(length);
    for (let i = 0; i < length; i++) {
      array[i] = init(i);
    }
    return array;
  }

  public static range(length: number, start = 0, increment = 1): number[] {
    return Arr.create(length, (index) => start + index * increment);
  }

  public static fillRange(array: number[], length: number, startIndex = 0, start = startIndex, increment = 1): number[] {
    for (let i = 0; i < length; i++) {
      array[startIndex + i] = start + i * increment;
    }
    return array;
  }

  public static at<T>(array: ArrayLike<T>, index: number): T {
    const resolved = index < 0 ? array.length + index : index;
    if (resolved < 0 || resolved >= array.length) {
      throw new RangeError(`Index ${index} is out of bounds for array of length ${array.length}.`);
    }
    return array[resolved];
  }

  public static peekAt<T>(array: ArrayLike<T>, index: number): T | undefined {
    const resolved = index < 0 ? array.length + index : index;
    return resolved < 0 || resolved >= array.length ? undefined : array[resolved];
  }

  public static first<T>(array: ArrayLike<T>): T {
    if (array.length === 0) {
      throw new RangeError('Cannot get the first element of an empty array.');
    }
    return array[0];
  }

  public static peekFirst<T>(array: ArrayLike<T>): T | undefined {
    return array.length === 0 ? undefined : array[0];
  }

  public static last<T>(array: ArrayLike<T>): T {
    if (array.length === 0) {
      throw new RangeError('Cannot get the last element of an empty array.');
    }
    return array[array.length - 1];
  }

  public static peekLast<T>(array: ArrayLike<T>): T | undefined {
    return array.length === 0 ? undefined : array[array.length - 1];
  }

  public static includes<T>(array: readonly T[], searchElement: any, fromIndex?: number): searchElement is T {
    return array.includes(searchElement, fromIndex);
  }

  public static equals<T1, T2>(
    a: ArrayLike<T1>,
    b: ArrayLike<T2>,
    equalsFunc: (a: T1, b: T2) => boolean = Arr.STRICT_EQUALS as (a: T1, b: T2) => boolean,
  ): boolean {
    if (a.length !== b.length) {
      return false;
    }
    for (let i = 0; i < a.length; i++) {
      if (!equalsFunc(a[i], b[i])) {
        return false;
      }
    }
    return true;
  }

  public static flatMap<O, A extends ReadonlyArray<any>>(
    array: A,
    map: (value: ArrayElement<A>, index: number, array: A) => O,
  ): FlattenArray<O[]> {
    const result: unknown[] = [];
    for (let i = 0; i < array.length; i++) {
      const mapped = map(array[i], i, array);
      if (Array.isArray(mapped)) {
        for (const item of mapped) {
          result.push(item);
        }
      } else {
        result.push(mapped);
      }
    }
    return result as FlattenArray<O[]>;
  }

  public static flat<A extends ReadonlyArray<any>>(array: A): FlattenArray<A>;
  public static flat<A extends ReadonlyArray<any>>(array: A, depth: number | undefined): any;
  public static flat<T = unknown>(array: readonly unknown[], depth?: number): T[];
  public static flat(array: readonly unknown[], depth = 1): unknown[] {
    const out: unknown[] = [];
    Arr.flatHelper(array, depth, 0, out);
    return out;
  }

  private static flatHelper(array: readonly unknown[], maxDepth: number, depth: number, out: unknown[]): void {
    for (const element of array) {
      if (Array.isArray(element) && depth < maxDepth) {
        Arr.flatHelper(element, maxDepth, depth + 1, out);
      } else {
        out.push(element);
      }
    }
  }

  public static shallowCopy<T>(source: readonly T[], target: T[] = []): T[] {
    target.length = source.length;
    for (let i = 0; i < source.length; i++) {
      target[i] = source[i];
    }
    return target;
  }

  public static binarySearch<T>(
    array: ArrayLike<T>,
    element: T,
    comparator: (a: T, b: T) => number,
    first = true,
  ): number {
    let low = 0;
    let high = array.length - 1;
    let found = -1;
    while (low <= high) {
      const mid = (low + high) >> 1;
      const order = comparator(array[mid], element);
      if (order < 0) {
        low = mid + 1;
      } else if (order > 0) {
        high = mid - 1;
      } else {
        found = mid;
        if (first) {
          high = mid - 1;
        } else {
          low = mid + 1;
        }
      }
    }
    return found >= 0 ? found : -(low + 1);
  }

  public static getMaxStringLength(array: string[]): number {
    let max = 0;
    for (const item of array) {
      max = Math.max(max, item.length);
    }
    return max;
  }
}

interface LerpNode {
  key: number;
  value: number;
  children: LerpNode[];
}

export class LerpTable {
  private readonly _dimensionCount: number;
  private readonly table: LerpNode[] = [];

  public get dimensionCount(): number {
    return this._dimensionCount;
  }

  public constructor(dimensionCount: number);
  public constructor(breakpoints: readonly (readonly number[])[]);
  public constructor(arg: number | readonly (readonly number[])[]) {
    if (typeof arg === 'number') {
      this._dimensionCount = Math.max(0, arg);
    } else {
      let shortest = Infinity;
      for (const breakpoint of arg) {
        shortest = Math.min(shortest, breakpoint.length);
      }
      this._dimensionCount = arg.length === 0 ? 0 : Math.max(0, shortest - 1);
      if (this._dimensionCount > 0) {
        for (const breakpoint of arg) {
          this.insertBreakpoint(breakpoint);
        }
      }
    }
  }

  public insertBreakpoint(breakpoint: readonly number[]): this {
    if (this._dimensionCount < 1) {
      throw new Error('Cannot insert a breakpoint into a table with zero dimensions.');
    }
    if (breakpoint.length - 1 < this._dimensionCount) {
      throw new Error('Breakpoint has fewer dimensions than the table.');
    }
    this.insertBreakpointHelper(breakpoint, 1, this.table);
    return this;
  }

  private insertBreakpointHelper(breakpoint: readonly number[], dimension: number, array: LerpNode[]): void {
    const key = breakpoint[dimension];
    let low = 0;
    let high = array.length;
    while (low < high) {
      const mid = (low + high) >> 1;
      if (array[mid].key < key) {
        low = mid + 1;
      } else {
        high = mid;
      }
    }
    let node = array[low];
    if (!node || node.key !== key) {
      node = { key, value: NaN, children: [] };
      array.splice(low, 0, node);
    }
    if (dimension === this._dimensionCount) {
      node.value = breakpoint[0];
    } else {
      this.insertBreakpointHelper(breakpoint, dimension + 1, node.children);
    }
  }

  public get(...key: number[]): number {
    if (this._dimensionCount < 1) {
      throw new Error('Cannot look up a value in a table with zero dimensions.');
    }
    if (key.length < this._dimensionCount) {
      throw new Error('Lookup key has fewer dimensions than the table.');
    }
    const value = this.lookupHelper(key, 0, this.table);
    if (value === undefined) {
      throw new Error('Could not retrieve a value from the table.');
    }
    return value;
  }

  private lookupHelper(key: number[], dimension: number, nodes: LerpNode[]): number | undefined {
    if (nodes.length === 0) {
      return undefined;
    }
    const target = key[dimension];
    let upperIndex = 0;
    while (upperIndex < nodes.length && nodes[upperIndex].key < target) {
      upperIndex++;
    }
    let lower: LerpNode;
    let upper: LerpNode;
    if (upperIndex === nodes.length) {
      lower = upper = nodes[nodes.length - 1];
    } else if (nodes[upperIndex].key === target || upperIndex === 0) {
      lower = upper = nodes[upperIndex];
    } else {
      lower = nodes[upperIndex - 1];
      upper = nodes[upperIndex];
    }
    const last = dimension === this._dimensionCount - 1;
    const lowerValue = last ? lower.value : this.lookupHelper(key, dimension + 1, lower.children);
    if (lower === upper) {
      return lowerValue;
    }
    const upperValue = last ? upper.value : this.lookupHelper(key, dimension + 1, upper.children);
    if (lowerValue === undefined || upperValue === undefined) {
      return undefined;
    }
    return Num.lerp(target, lower.key, upper.key, lowerValue, upperValue);
  }
}

export class Pool<T> {
  private readonly cache: T[] = [];
  private numAllocated = 0;

  public constructor(
    private readonly factory: () => T,
    private readonly destructor: (resource: T) => void,
    private readonly onAllocated?: (resource: T) => void,
    private readonly onFreed?: (resource: T) => void,
    initialSize = 0,
    public readonly maxSize = Number.MAX_SAFE_INTEGER,
    private readonly autoShrinkThreshold = Number.MAX_SAFE_INTEGER,
  ) {
    for (let i = 0; i < Math.min(initialSize, maxSize); i++) {
      this.cache.push(this.factory());
    }
  }

  public allocate(): T {
    let resource = this.cache.pop();
    if (resource === undefined) {
      if (this.numAllocated >= this.maxSize) {
        throw new Error('Resource heap has reached its allocation limit.');
      }
      resource = this.factory();
    }
    this.numAllocated++;
    this.onAllocated?.(resource);
    return resource;
  }

  public free(resource: T): void {
    this.onFreed?.(resource);
    this.numAllocated = Math.max(0, this.numAllocated - 1);
    if (this.numAllocated + this.cache.length >= this.autoShrinkThreshold) {
      this.destructor(resource);
    } else {
      this.cache.push(resource);
    }
  }
}

export class Uuid {
  public static GenerateUuid(): string {
    const bytes = new Array<number>(16);
    for (let i = 0; i < 16; i++) {
      bytes[i] = Math.floor(Math.random() * 256);
    }
    bytes[6] = (bytes[6] & 0x0f) | 0x40;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    const hex = bytes.map((byte) => byte.toString(16).padStart(2, '0'));
    return `${hex.slice(0, 4).join('')}-${hex.slice(4, 6).join('')}-${hex.slice(6, 8).join('')}-${hex
      .slice(8, 10)
      .join('')}-${hex.slice(10).join('')}`;
  }
}
