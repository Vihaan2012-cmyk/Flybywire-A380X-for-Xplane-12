// Canvas 2D into the display stream. A canvas keeps what was drawn on it,
// as ops in its own pixel space, until it is cleared whole or resized; its
// element's chunk replays them scaled to the element's box. Paths are
// transformed as they are built, as the canvas specification says, so the
// ops need no transform of their own.
(() => {
  const D = globalThis.__dom;
  const { css, geom, OP, warnOnce } = D;
  const TAU = Math.PI * 2;

  // Past this many numbers a canvas that is never cleared stops recording.
  const MAX_OPS = 4000000;

  // The signed sweep of an arc (HTML 4.12.5.1.10, as src/display/path.rs).
  const arcSweep = (start, end, ccw) => {
    const rem = (a) => ((a % TAU) + TAU) % TAU;
    if (!ccw) return end - start >= TAU ? TAU : rem(end - start);
    return start - end >= TAU ? -TAU : -rem(start - end);
  };

  // Writes path ops in canvas space from local coordinates under a matrix.
  class Sink {
    constructor(out) {
      this.out = out;
      this.m = geom.IDENTITY;
      this.started = false;
    }
    pt(x, y) {
      const m = this.m;
      return [m[0] * x + m[2] * y + m[4], m[1] * x + m[3] * y + m[5]];
    }
    move(x, y) {
      const [a, b] = this.pt(x, y);
      this.out.push(OP.MOVE_TO, a, b);
      this.started = true;
    }
    line(x, y) {
      const [a, b] = this.pt(x, y);
      this.out.push(this.started ? OP.LINE_TO : OP.MOVE_TO, a, b);
      this.started = true;
    }
    quad(cx, cy, x, y) {
      if (!this.started) this.move(cx, cy);
      const [a, b] = this.pt(cx, cy);
      const [c, d] = this.pt(x, y);
      this.out.push(OP.QUAD_TO, a, b, c, d);
    }
    cubic(c1x, c1y, c2x, c2y, x, y) {
      if (!this.started) this.move(c1x, c1y);
      const [a, b] = this.pt(c1x, c1y);
      const [c, d] = this.pt(c2x, c2y);
      const [e, f] = this.pt(x, y);
      this.out.push(OP.CUBIC_TO, a, b, c, d, e, f);
    }
    close() {
      this.out.push(OP.CLOSE_PATH);
    }
    rect(x, y, w, h) {
      this.move(x, y);
      this.line(x + w, y);
      this.line(x + w, y + h);
      this.line(x, y + h);
      this.close();
      this.move(x, y);
    }
    ellipse(cx, cy, rx, ry, rotation, start, end, ccw) {
      const m = this.m;
      const det = m[0] * m[3] - m[1] * m[2];
      const s = Math.sqrt(Math.abs(det));
      const similar = s > 0 && Math.abs(Math.abs(m[0]) - Math.abs(m[3])) < 1e-9 * (1 + s) && Math.abs(Math.abs(m[1]) - Math.abs(m[2])) < 1e-9 * (1 + s) && Math.abs(m[0] * m[2] + m[1] * m[3]) < 1e-9 * (1 + s * s);
      const [c0, c1] = this.pt(cx, cy);
      if (similar) {
        const r = Math.atan2(m[1], m[0]);
        const sweep = arcSweep(start, end, ccw);
        if (det > 0) this.out.push(OP.ELLIPSE, c0, c1, rx * s, ry * s, rotation + r, start, start + sweep, sweep < 0 ? 1 : 0);
        else this.out.push(OP.ELLIPSE, c0, c1, rx * s, ry * s, r - rotation, -start, -start - sweep, sweep < 0 ? 0 : 1);
        this.started = true;
        return;
      }
      // Any other matrix: the arc as cubic Béziers, transformed point by point.
      const sweep = arcSweep(start, end, ccw);
      const n = Math.max(1, Math.ceil(Math.abs(sweep) / (Math.PI / 2) - 1e-9));
      const cos = Math.cos(rotation);
      const sin = Math.sin(rotation);
      const at = (ux, uy) => [cx + cos * rx * ux - sin * ry * uy, cy + sin * rx * ux + cos * ry * uy];
      const p0 = at(Math.cos(start), Math.sin(start));
      this.line(p0[0], p0[1]);
      for (let i = 0; i < n; i++) {
        const t0 = start + (sweep * i) / n;
        const t1 = start + (sweep * (i + 1)) / n;
        const k = (4 / 3) * Math.tan((t1 - t0) / 4);
        const a = at(Math.cos(t0) - k * Math.sin(t0), Math.sin(t0) + k * Math.cos(t0));
        const b = at(Math.cos(t1) + k * Math.sin(t1), Math.sin(t1) - k * Math.cos(t1));
        const e = at(Math.cos(t1), Math.sin(t1));
        this.cubic(a[0], a[1], b[0], b[1], e[0], e[1]);
      }
    }
    // Replay local-space ops (a Path2D's) through the matrix.
    replay(ops) {
      for (let i = 0; i < ops.length; ) {
        const o = ops[i];
        switch (o) {
          case OP.MOVE_TO:
            this.move(ops[i + 1], ops[i + 2]);
            i += 3;
            break;
          case OP.LINE_TO:
            this.line(ops[i + 1], ops[i + 2]);
            i += 3;
            break;
          case OP.QUAD_TO:
            this.quad(ops[i + 1], ops[i + 2], ops[i + 3], ops[i + 4]);
            i += 5;
            break;
          case OP.CUBIC_TO:
            this.cubic(ops[i + 1], ops[i + 2], ops[i + 3], ops[i + 4], ops[i + 5], ops[i + 6]);
            i += 7;
            break;
          case OP.ELLIPSE:
            this.ellipse(ops[i + 1], ops[i + 2], ops[i + 3], ops[i + 4], ops[i + 5], ops[i + 6], ops[i + 7], ops[i + 8]);
            i += 9;
            break;
          case OP.RECT:
            this.rect(ops[i + 1], ops[i + 2], ops[i + 3], ops[i + 4]);
            i += 5;
            break;
          case OP.CLOSE_PATH:
            this.close();
            i += 1;
            break;
          default:
            return;
        }
      }
    }
  }

  // The path-building methods shared by the context and Path2D, over
  // `_sink()` and `_last` (the current point in local coordinates).
  const pathMethods = {
    moveTo(x, y) {
      if (![x, y].every(Number.isFinite)) return;
      this._sink().move(x, y);
      this._last = [x, y];
      this._first = [x, y];
    },
    lineTo(x, y) {
      if (![x, y].every(Number.isFinite)) return;
      this._sink().line(x, y);
      if (!this._first) this._first = [x, y];
      this._last = [x, y];
    },
    quadraticCurveTo(cx, cy, x, y) {
      if (![cx, cy, x, y].every(Number.isFinite)) return;
      if (!this._last) this.moveTo(cx, cy);
      this._sink().quad(cx, cy, x, y);
      this._last = [x, y];
    },
    bezierCurveTo(c1x, c1y, c2x, c2y, x, y) {
      if (![c1x, c1y, c2x, c2y, x, y].every(Number.isFinite)) return;
      if (!this._last) this.moveTo(c1x, c1y);
      this._sink().cubic(c1x, c1y, c2x, c2y, x, y);
      this._last = [x, y];
    },
    arc(cx, cy, r, start, end, ccw = false) {
      this.ellipse(cx, cy, r, r, 0, start, end, ccw);
    },
    ellipse(cx, cy, rx, ry, rotation, start, end, ccw = false) {
      if (![cx, cy, rx, ry, rotation, start, end].every(Number.isFinite)) return;
      if (rx < 0 || ry < 0) throw D.indexSizeError('The radius provided is negative.');
      this._sink().ellipse(cx, cy, rx, ry, rotation, start, end, !!ccw);
      const t = start + arcSweep(start, end, !!ccw);
      const c = Math.cos(rotation);
      const s = Math.sin(rotation);
      const ex = rx * Math.cos(t);
      const ey = ry * Math.sin(t);
      if (!this._first) this._first = [cx + c * rx * Math.cos(start) - s * ry * Math.sin(start), cy + s * rx * Math.cos(start) + c * ry * Math.sin(start)];
      this._last = [cx + c * ex - s * ey, cy + s * ex + c * ey];
    },
    // HTML 4.12.5.1.10 arcTo.
    arcTo(x1, y1, x2, y2, r) {
      if (![x1, y1, x2, y2, r].every(Number.isFinite)) return;
      if (r < 0) throw D.indexSizeError('The radius provided is negative.');
      if (!this._last) this.moveTo(x1, y1);
      const [x0, y0] = this._last;
      const v1x = x0 - x1;
      const v1y = y0 - y1;
      const v2x = x2 - x1;
      const v2y = y2 - y1;
      const cross = v1x * v2y - v1y * v2x;
      if ((x0 === x1 && y0 === y1) || (x1 === x2 && y1 === y2) || r === 0 || Math.abs(cross) < 1e-12) {
        this.lineTo(x1, y1);
        return;
      }
      const l1 = Math.hypot(v1x, v1y);
      const l2 = Math.hypot(v2x, v2y);
      const angle = Math.acos(Math.max(-1, Math.min(1, (v1x * v2x + v1y * v2y) / (l1 * l2))));
      const d = r / Math.tan(angle / 2);
      const t1x = x1 + (v1x / l1) * d;
      const t1y = y1 + (v1y / l1) * d;
      const t2x = x1 + (v2x / l2) * d;
      const t2y = y1 + (v2y / l2) * d;
      const ccw = cross > 0;
      const nx = ccw ? v1y / l1 : -v1y / l1;
      const ny = ccw ? -v1x / l1 : v1x / l1;
      const cx = t1x - nx * r;
      const cy = t1y - ny * r;
      this.lineTo(t1x, t1y);
      const a0 = Math.atan2(t1y - cy, t1x - cx);
      const a1 = Math.atan2(t2y - cy, t2x - cx);
      this._sink().ellipse(cx, cy, r, r, 0, a0, a1, !ccw);
      this._last = [t2x, t2y];
    },
    rect(x, y, w, h) {
      if (![x, y, w, h].every(Number.isFinite)) return;
      this._sink().rect(x, y, w, h);
      this._last = [x, y];
      this._first = [x, y];
    },
    roundRect(x, y, w, h, radii = 0) {
      const r = Array.isArray(radii) ? Number(radii[0]) || 0 : typeof radii === 'object' ? Number(radii.x) || 0 : Number(radii) || 0;
      if (r <= 0) return this.rect(x, y, w, h);
      const sink = this._sink();
      const ops = geom.rectOps(x, y, w, h, Math.min(r, Math.abs(w) / 2), Math.min(r, Math.abs(h) / 2)).ops;
      sink.replay(ops);
      this._last = [x, y];
      this._first = [x, y];
    },
    closePath() {
      this._sink().close();
      if (this._first) this._last = this._first.slice();
    },
  };

  D.indexSizeError = (message) => {
    const e = new Error(message);
    e.name = 'IndexSizeError';
    return e;
  };

  class Path2D {
    constructor(init) {
      this._ops = [];
      this._last = null;
      this._first = null;
      this._local = new Sink(this._ops);
      if (typeof init === 'string') {
        const b = geom.parsePathData(init);
        for (const v of b.a) this._ops.push(v);
        this._last = b.x1 >= b.x0 ? [0, 0] : null;
        this._local.started = this._ops.length > 0;
      } else if (init instanceof Path2D) {
        for (const v of init._ops) this._ops.push(v);
        this._local.started = init._local.started;
        this._last = init._last && init._last.slice();
      }
    }
    _sink() {
      return this._local;
    }
    addPath(path, transform) {
      if (!(path instanceof Path2D)) throw new TypeError("Failed to execute 'addPath' on 'Path2D': parameter 1 is not of type 'Path2D'.");
      const sink = new Sink(this._ops);
      if (transform) sink.m = [transform.a ?? 1, transform.b ?? 0, transform.c ?? 0, transform.d ?? 1, transform.e ?? 0, transform.f ?? 0];
      sink.replay(path._ops);
      this._local.started = true;
    }
  }
  Object.assign(Path2D.prototype, pathMethods);

  class CanvasGradient {
    constructor(x0, y0, x1, y1) {
      this._line = [x0, y0, x1, y1];
      this._stops = [];
    }
    addColorStop(offset, color) {
      const o = Number(offset);
      if (!(o >= 0 && o <= 1)) throw D.indexSizeError('The provided offset is outside the range [0, 1].');
      const c = css.parseColor(String(color));
      if (!Array.isArray(c)) {
        const e = new Error(`The value provided ('${color}') could not be parsed as a color.`);
        e.name = 'SyntaxError';
        throw e;
      }
      this._stops.push([o, ...c]);
      this._stops.sort((a, b) => a[0] - b[0]);
    }
  }

  // font shorthand -> {family, size, weight, italic}, cached.
  const fonts = new Map();
  function parseFont(text) {
    let f = fonts.get(text);
    if (f) return f;
    const decls = css.expand('font', text);
    if (!decls.length) return null;
    const get = (name) => decls.find((d) => d[0] === name)?.[1];
    const sizeLen = css.parseLength(get('font-size') ?? '');
    const size = sizeLen ? css.toPx(sizeLen, 10, 10, null) : { 'xx-small': 9, 'x-small': 10, small: 13, medium: 16, large: 18, 'x-large': 24, 'xx-large': 32 }[get('font-size')];
    if (!(size > 0)) return null;
    const weightText = get('font-weight');
    const weight = weightText === 'bold' || weightText === 'bolder' ? 700 : weightText === 'lighter' ? 100 : parseFloat(weightText) || 400;
    const family = css.splitCommas(get('font-family') ?? 'sans-serif').map((s) => s.trim().replace(/^(['"])(.*)\1$/, '$2')).join(', ');
    f = { family, size, weight, italic: /italic|oblique/.test(get('font-style') ?? '') ? 1 : 0 };
    if (fonts.size > 256) fonts.clear();
    fonts.set(text, f);
    return f;
  }

  const defaults = () => ({
    m: geom.IDENTITY,
    fillStyle: '#000000',
    fill: [0, 0, 0, 1],
    strokeStyle: '#000000',
    stroke: [0, 0, 0, 1],
    lineWidth: 1,
    lineCap: 'butt',
    lineJoin: 'miter',
    miterLimit: 10,
    dash: [],
    dashOffset: 0,
    globalAlpha: 1,
    font: '10px sans-serif',
    parsedFont: { family: 'sans-serif', size: 10, weight: 400, italic: 0 },
    textAlign: 'start',
    textBaseline: 'alphabetic',
    direction: 'ltr',
    composite: 'source-over',
    clips: [],
  });

  class CanvasRenderingContext2D {
    constructor(canvas) {
      this.canvas = canvas;
      this._reset();
    }
    _reset() {
      this._state = defaults();
      this._stack = [];
      this._ops = [];
      this._path = [];
      this._pathSink = new Sink(this._path);
      this._last = null;
      this._first = null;
      this._changed();
    }
    _changed() {
      const el = this.canvas;
      if (el && el._connected && el.ownerDocument && el.ownerDocument._markPaint) el.ownerDocument._markPaint(el);
    }
    _sink() {
      this._pathSink.m = this._state.m;
      return this._pathSink;
    }
    _record(...values) {
      if (this._ops.length > MAX_OPS) {
        warnOnce('canvas-ops', 'a canvas drew over 4M numbers without being cleared: further drawing is dropped');
        return;
      }
      for (const v of values) this._ops.push(v);
      this._changed();
    }
    _append(list) {
      if (this._ops.length + list.length > MAX_OPS) return this._record();
      const ops = this._ops;
      for (let i = 0; i < list.length; i++) ops.push(list[i]);
    }

    // State.
    save() {
      const s = this._state;
      this._stack.push(s);
      this._state = { ...s, dash: s.dash.slice(), clips: [] };
      this._record(OP.SAVE);
    }
    restore() {
      if (!this._stack.length) return;
      this._state = this._stack.pop();
      this._record(OP.RESTORE);
    }
    reset() {
      this.canvas && this._reset();
    }

    // Transforms.
    _setMatrix(m) {
      this._state.m = m;
    }
    transform(a, b, c, d, e, f) {
      if (![a, b, c, d, e, f].every(Number.isFinite)) return;
      this._setMatrix(geom.multiply(this._state.m, [a, b, c, d, e, f]));
    }
    setTransform(a, b, c, d, e, f) {
      if (typeof a === 'object' && a !== null) {
        const t = a;
        this._setMatrix([t.a ?? t.m11 ?? 1, t.b ?? t.m12 ?? 0, t.c ?? t.m21 ?? 0, t.d ?? t.m22 ?? 1, t.e ?? t.m41 ?? 0, t.f ?? t.m42 ?? 0]);
        return;
      }
      if (a === undefined) {
        this._setMatrix(geom.IDENTITY);
        return;
      }
      if (![a, b, c, d, e, f].every(Number.isFinite)) return;
      this._setMatrix([a, b, c, d, e, f]);
    }
    resetTransform() {
      this._setMatrix(geom.IDENTITY);
    }
    getTransform() {
      const [a, b, c, d, e, f] = this._state.m;
      return { a, b, c, d, e, f, m11: a, m12: b, m21: c, m22: d, m41: e, m42: f, is2D: true, isIdentity: geom.isIdentity(this._state.m) };
    }
    translate(x, y) {
      this.transform(1, 0, 0, 1, x, y);
    }
    scale(x, y) {
      this.transform(x, 0, 0, y, 0, 0);
    }
    rotate(angle) {
      const c = Math.cos(angle);
      const s = Math.sin(angle);
      this.transform(c, s, -s, c, 0, 0);
    }

    // Styles.
    get fillStyle() {
      return this._state.fillStyle;
    }
    set fillStyle(v) {
      this._style('fill', v);
    }
    get strokeStyle() {
      return this._state.strokeStyle;
    }
    set strokeStyle(v) {
      this._style('stroke', v);
    }
    _style(which, v) {
      if (v instanceof CanvasGradient) {
        this._state[which] = v;
        this._state[`${which}Style`] = v;
        return;
      }
      if (typeof v === 'object' && v !== null) {
        warnOnce('canvas-pattern', 'canvas patterns are not supported (ignored)');
        return;
      }
      const c = css.parseColor(String(v));
      if (!Array.isArray(c)) return;
      this._state[which] = c;
      this._state[`${which}Style`] = String(v);
    }
    get lineWidth() {
      return this._state.lineWidth;
    }
    set lineWidth(v) {
      const n = Number(v);
      if (n > 0 && Number.isFinite(n)) this._state.lineWidth = n;
    }
    get lineCap() {
      return this._state.lineCap;
    }
    set lineCap(v) {
      if (v === 'butt' || v === 'round' || v === 'square') this._state.lineCap = v;
    }
    get lineJoin() {
      return this._state.lineJoin;
    }
    set lineJoin(v) {
      if (v === 'miter' || v === 'round' || v === 'bevel') this._state.lineJoin = v;
    }
    get miterLimit() {
      return this._state.miterLimit;
    }
    set miterLimit(v) {
      const n = Number(v);
      if (n > 0 && Number.isFinite(n)) this._state.miterLimit = n;
    }
    setLineDash(list) {
      const d = Array.from(list, Number);
      if (d.some((x) => !(x >= 0) || !Number.isFinite(x))) return;
      this._state.dash = d.length % 2 ? [...d, ...d] : d;
    }
    getLineDash() {
      return this._state.dash.slice();
    }
    get lineDashOffset() {
      return this._state.dashOffset;
    }
    set lineDashOffset(v) {
      const n = Number(v);
      if (Number.isFinite(n)) this._state.dashOffset = n;
    }
    get globalAlpha() {
      return this._state.globalAlpha;
    }
    set globalAlpha(v) {
      const n = Number(v);
      if (n >= 0 && n <= 1) this._state.globalAlpha = n;
    }
    get globalCompositeOperation() {
      return this._state.composite;
    }
    set globalCompositeOperation(v) {
      if (v !== 'source-over') warnOnce(`composite:${v}`, `globalCompositeOperation ${v} is drawn as source-over`);
      this._state.composite = String(v);
    }
    get font() {
      return this._state.font;
    }
    set font(v) {
      const f = parseFont(String(v));
      if (!f) return;
      this._state.font = String(v);
      this._state.parsedFont = f;
    }
    get textAlign() {
      return this._state.textAlign;
    }
    set textAlign(v) {
      if (['start', 'end', 'left', 'right', 'center'].includes(v)) this._state.textAlign = v;
    }
    get textBaseline() {
      return this._state.textBaseline;
    }
    set textBaseline(v) {
      if (['top', 'hanging', 'middle', 'alphabetic', 'ideographic', 'bottom'].includes(v)) this._state.textBaseline = v;
    }
    get direction() {
      return this._state.direction;
    }
    set direction(v) {
      if (v === 'ltr' || v === 'rtl' || v === 'inherit') this._state.direction = v;
    }
    get shadowBlur() {
      return 0;
    }
    set shadowBlur(v) {
      if (Number(v) > 0) warnOnce('canvas-shadow', 'canvas shadows are not drawn');
    }
    get shadowColor() {
      return 'rgba(0, 0, 0, 0)';
    }
    set shadowColor(_v) {}
    get shadowOffsetX() {
      return 0;
    }
    set shadowOffsetX(_v) {}
    get shadowOffsetY() {
      return 0;
    }
    set shadowOffsetY(_v) {}
    get imageSmoothingEnabled() {
      return true;
    }
    set imageSmoothingEnabled(_v) {}

    // Paths.
    beginPath() {
      this._path.length = 0;
      this._pathSink.started = false;
      this._last = null;
      this._first = null;
    }

    _pathOps(path) {
      if (path instanceof Path2D) {
        const out = [];
        const sink = new Sink(out);
        sink.m = this._state.m;
        sink.replay(path._ops);
        return out;
      }
      return this._path;
    }
    _scale() {
      const m = this._state.m;
      const det = Math.abs(m[0] * m[3] - m[1] * m[2]);
      if (Math.abs(Math.hypot(m[0], m[1]) - Math.hypot(m[2], m[3])) > 1e-6 * (1 + Math.sqrt(det))) {
        warnOnce('canvas-nonuniform-stroke', 'canvas strokes under a non-uniform scale use its mean scale');
      }
      return Math.sqrt(det);
    }
    _fillPaint(paint, rule) {
      const a = this._state.globalAlpha;
      if (paint instanceof CanvasGradient) {
        const stops = paint._stops;
        if (!stops.length) return;
        this._record(OP.SAVE, OP.TRANSFORM, ...this._state.m, OP.LINEAR_GRADIENT_FILL, ...paint._line, stops.length);
        for (const s of stops) this._record(s[0], s[1], s[2], s[3], s[4] * a);
        this._record(rule, OP.RESTORE);
        return;
      }
      if (paint[3] * a <= 0) return;
      this._record(OP.FILL, paint[0], paint[1], paint[2], paint[3] * a, rule);
    }
    fill(a, b) {
      const path = a instanceof Path2D ? a : null;
      const rule = (path ? b : a) === 'evenodd' ? 1 : 0;
      const ops = this._pathOps(path);
      if (!ops.length) return;
      this._append([OP.BEGIN_PATH]);
      this._append(ops);
      this._fillPaint(this._state.fill, rule);
    }
    stroke(path) {
      const ops = this._pathOps(path instanceof Path2D ? path : null);
      if (!ops.length) return;
      this._append([OP.BEGIN_PATH]);
      this._append(ops);
      this._strokeOp();
    }
    _strokeOp() {
      const s = this._state;
      let c = s.stroke;
      if (c instanceof CanvasGradient) {
        warnOnce('canvas-stroke-gradient', 'canvas gradient strokes are drawn in the first stop colour');
        if (!c._stops.length) return;
        c = c._stops[0].slice(1);
      }
      const k = this._scale();
      const a = c[3] * s.globalAlpha;
      if (a <= 0) return;
      this._record(OP.STROKE, c[0], c[1], c[2], a, s.lineWidth * k, { butt: 0, round: 1, square: 2 }[s.lineCap], { miter: 0, round: 1, bevel: 2 }[s.lineJoin], s.miterLimit, s.dash.length, ...s.dash.map((d) => d * k), s.dashOffset * k);
    }
    clip(a, b) {
      const path = a instanceof Path2D ? a : null;
      const rule = (path ? b : a) === 'evenodd' ? 1 : 0;
      const ops = this._pathOps(path);
      const clip = [OP.BEGIN_PATH];
      for (let i = 0; i < ops.length; i++) clip.push(ops[i]);
      clip.push(OP.CLIP_PATH, rule);
      this._state.clips = [...this._state.clips, clip];
      this._append(clip);
      this._changed();
    }
    isPointInPath() {
      warnOnce('isPointInPath', 'isPointInPath() is not supported (false)');
      return false;
    }
    isPointInStroke() {
      warnOnce('isPointInStroke', 'isPointInStroke() is not supported (false)');
      return false;
    }

    // Rectangles.
    _rectOps(x, y, w, h) {
      const out = [];
      const sink = new Sink(out);
      sink.m = this._state.m;
      sink.rect(x, y, w, h);
      return out;
    }
    fillRect(x, y, w, h) {
      if (![x, y, w, h].every(Number.isFinite) || !w || !h) return;
      this._record(OP.BEGIN_PATH, ...this._rectOps(x, y, w, h));
      this._fillPaint(this._state.fill, 0);
    }
    strokeRect(x, y, w, h) {
      if (![x, y, w, h].every(Number.isFinite)) return;
      this._record(OP.BEGIN_PATH, ...this._rectOps(x, y, w, h));
      this._strokeOp();
    }
    clearRect(x, y, w, h) {
      if (![x, y, w, h].every(Number.isFinite)) return;
      const box = geom.transformBox(this._state.m, [Math.min(x, x + w), Math.min(y, y + h), Math.max(x, x + w), Math.max(y, y + h)]);
      const cw = this.canvas ? this.canvas.width : 0;
      const ch = this.canvas ? this.canvas.height : 0;
      const axisAligned = this._state.m[1] === 0 && this._state.m[2] === 0;
      if (axisAligned && box[0] <= 0 && box[1] <= 0 && box[2] >= cw && box[3] >= ch && this._state.clips.length === 0) {
        // Everything drawn so far is gone; open saves and their clips remain.
        this._ops = [];
        for (const level of this._stack) {
          this._ops.push(...level.clips.flat());
          this._ops.push(OP.SAVE);
        }
        this._changed();
        return;
      }
      warnOnce('clearRect-partial', 'clearRect() of part of a canvas cannot erase what is under it (ignored)');
    }

    // Text.
    _text(text, x, y, maxWidth, stroke) {
      const s = this._state;
      const f = s.parsedFont;
      const str = String(text).replace(/[\t\n\f\r]/g, ' ');
      if (![x, y].every(Number.isFinite)) return;
      const rtl = s.direction === 'rtl';
      const align = { left: 0, right: 2, center: 1, start: rtl ? 2 : 0, end: rtl ? 0 : 2 }[s.textAlign];
      const baseline = { alphabetic: 0, middle: 1, top: 2, bottom: 3, hanging: 4, ideographic: 3 }[s.textBaseline];
      const a = s.globalAlpha;
      let fill = [0, 0, 0, 0];
      let strokeWidth = 0;
      let strokeColor = [0, 0, 0, 0];
      if (stroke) {
        let c = s.stroke instanceof CanvasGradient ? (s.stroke._stops[0] ?? [0, 0, 0, 0, 0]).slice(1) : s.stroke;
        strokeColor = [c[0], c[1], c[2], c[3] * a];
        strokeWidth = s.lineWidth;
        if (strokeColor[3] <= 0) return;
      } else {
        const c = s.fill instanceof CanvasGradient ? (s.fill._stops[0] ?? [0, 0, 0, 0, 0]).slice(1) : s.fill;
        fill = [c[0], c[1], c[2], c[3] * a];
        if (fill[3] <= 0) return;
      }
      let m = s.m;
      if (maxWidth !== undefined && Number.isFinite(maxWidth)) {
        const width = D.text.measure(this._screen(), f.family, f.size, str);
        if (maxWidth <= 0) return;
        if (width > maxWidth) m = geom.multiply(m, [maxWidth / width, 0, 0, 1, x - (x * maxWidth) / width, 0]);
      }
      this._record(OP.SAVE, ...(geom.isIdentity(m) ? [] : [OP.TRANSFORM, ...m]), OP.TEXT, str, f.family, f.size, f.weight, f.italic, x, y, align, baseline, ...fill, strokeWidth, ...strokeColor, OP.RESTORE);
    }
    _screen() {
      const doc = this.canvas && this.canvas.ownerDocument;
      return doc && doc.__screen ? doc.__screen : '';
    }
    fillText(text, x, y, maxWidth) {
      this._text(text, x, y, maxWidth, false);
    }
    strokeText(text, x, y, maxWidth) {
      this._text(text, x, y, maxWidth, true);
    }
    measureText(text) {
      const f = this._state.parsedFont;
      const str = String(text);
      const width = D.text.measure(this._screen(), f.family, f.size, str);
      const [ascent, descent] = D.text.metrics(this._screen(), f.family, f.size);
      return {
        width,
        actualBoundingBoxLeft: 0,
        actualBoundingBoxRight: width,
        actualBoundingBoxAscent: ascent,
        actualBoundingBoxDescent: descent,
        fontBoundingBoxAscent: ascent,
        fontBoundingBoxDescent: descent,
      };
    }

    // Images.
    drawImage(image, ...args) {
      let url = null;
      let nw = 0;
      let nh = 0;
      if (image && image.localName === 'img') {
        url = image.getAttribute('src');
        [nw, nh] = D.imageSize(url ?? '');
      } else {
        warnOnce('drawImage-source', 'drawImage() of anything but an <img> is not supported');
        return;
      }
      if (!url) return;
      let sx = 0;
      let sy = 0;
      let sw = nw;
      let sh = nh;
      let dx;
      let dy;
      let dw = nw;
      let dh = nh;
      if (args.length === 2) [dx, dy] = args;
      else if (args.length === 4) [dx, dy, dw, dh] = args;
      else if (args.length === 8) [sx, sy, sw, sh, dx, dy, dw, dh] = args;
      else throw new TypeError("Failed to execute 'drawImage': Valid arities are: [3, 5, 9].");
      if (!(nw > 0 && nh > 0)) {
        warnOnce(`drawImage-size:${url}`, `the size of image ${url} is unknown (__host.imageSize): not drawn`);
        return;
      }
      const a = this._state.globalAlpha;
      this._record(OP.SAVE, ...(geom.isIdentity(this._state.m) ? [] : [OP.TRANSFORM, ...this._state.m]), ...(a < 1 ? [OP.GLOBAL_ALPHA, a] : []), OP.IMAGE, url, sx, sy, sw, sh, dx, dy, dw, dh, OP.RESTORE);
    }

    createLinearGradient(x0, y0, x1, y1) {
      return new CanvasGradient(x0, y0, x1, y1);
    }
    createRadialGradient(x0, y0, r0, x1, y1) {
      warnOnce('createRadialGradient', 'radial canvas gradients are drawn as linear ones');
      return new CanvasGradient(x0, y0, x1, y1);
    }
    createPattern() {
      warnOnce('createPattern', 'canvas patterns are not supported');
      return null;
    }
    getImageData() {
      throw new Error('getImageData() is not supported: this canvas draws to a vector stream');
    }
    putImageData() {
      warnOnce('putImageData', 'putImageData() is not supported (ignored)');
    }

    // Replay into an element chunk: the canvas's pixels scaled to its box.
    _paint(w, doc, x, y, cw, ch) {
      const width = this.canvas.width;
      const height = this.canvas.height;
      if (!width || !height || cw <= 0 || ch <= 0 || !this._ops.length) return;
      w.op(OP.SAVE);
      w.nums(OP.CLIP_RECT, [x, y, cw, ch]);
      const scale = [cw / width, 0, 0, ch / height, x, y];
      if (!geom.isIdentity(scale)) w.transform(scale);
      const ops = this._ops;
      w.room(ops.length + this._stack.length + 2);
      const buf = w.buf;
      let n = w.n;
      for (let i = 0; i < ops.length; i++) {
        const v = ops[i];
        buf[n++] = typeof v === 'string' ? doc._str(v) : v;
      }
      const depth = countDepth(ops);
      w.n = n;
      for (let i = 0; i < depth; i++) w.op(OP.RESTORE);
      w.op(OP.RESTORE);
    }
  }
  Object.assign(CanvasRenderingContext2D.prototype, pathMethods);

  // Open SAVEs at the end of a recording, counted op by op.
  const ARGS = { 1: 0, 2: 0, 3: 6, 4: 6, 5: 1, 6: 4, 10: 0, 11: 2, 12: 2, 13: 4, 14: 6, 15: 6, 16: 8, 17: 4, 18: 0, 20: 5, 22: 1, 40: 9 };
  function countDepth(ops) {
    let depth = 0;
    for (let i = 0; i < ops.length; ) {
      const o = ops[i];
      if (o === OP.SAVE) depth++;
      else if (o === OP.RESTORE) depth = Math.max(0, depth - 1);
      if (o === OP.STROKE) i += 1 + 9 + ops[i + 9] + 1;
      else if (o === OP.TEXT) i += 1 + 18;
      else if (o === OP.LINEAR_GRADIENT_FILL) i += 1 + 5 + ops[i + 5] * 5 + 1;
      else if (ARGS[o] !== undefined) i += 1 + ARGS[o];
      else break;
    }
    return depth;
  }

  Object.assign(D, { CanvasRenderingContext2D, Path2D, CanvasGradient, arcSweep });
})();
