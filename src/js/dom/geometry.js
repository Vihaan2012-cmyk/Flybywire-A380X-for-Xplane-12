// Geometry: SVG path data and shapes to display-stream path ops, affine
// matrices and bounding boxes.
//
// Parsed paths are kept as Float64Arrays already in stream format
// (MOVE_TO x y, LINE_TO x y, ...), cached by their `d` string, so drawing
// a path is a copy.
(() => {
  const D = globalThis.__dom;

  const OP = {
    SAVE: 1, RESTORE: 2, TRANSFORM: 3, SET_TRANSFORM: 4, GLOBAL_ALPHA: 5, CLIP_RECT: 6,
    BEGIN_PATH: 10, MOVE_TO: 11, LINE_TO: 12, QUAD_TO: 13, CUBIC_TO: 14, ARC: 15, ELLIPSE: 16, RECT: 17, CLOSE_PATH: 18,
    FILL: 20, STROKE: 21, CLIP_PATH: 22, TEXT: 30, IMAGE: 40, LINEAR_GRADIENT_FILL: 50,
  };
  D.OP = OP;

  // ---------------------------------------------------------------- matrices

  const IDENTITY = Object.freeze([1, 0, 0, 1, 0, 0]);
  const multiply = (m, n) => [
    m[0] * n[0] + m[2] * n[1],
    m[1] * n[0] + m[3] * n[1],
    m[0] * n[2] + m[2] * n[3],
    m[1] * n[2] + m[3] * n[3],
    m[0] * n[4] + m[2] * n[5] + m[4],
    m[1] * n[4] + m[3] * n[5] + m[5],
  ];
  const invert = (m) => {
    const det = m[0] * m[3] - m[1] * m[2];
    if (!det) return null;
    return [m[3] / det, -m[1] / det, -m[2] / det, m[0] / det, (m[2] * m[5] - m[3] * m[4]) / det, (m[1] * m[4] - m[0] * m[5]) / det];
  };
  const apply = (m, x, y) => [m[0] * x + m[2] * y + m[4], m[1] * x + m[3] * y + m[5]];
  const isIdentity = (m) => m[0] === 1 && m[1] === 0 && m[2] === 0 && m[3] === 1 && m[4] === 0 && m[5] === 0;
  // The bounding box [x0, y0, x1, y1] of a box under a matrix.
  const transformBox = (m, b) => {
    const pts = [apply(m, b[0], b[1]), apply(m, b[2], b[1]), apply(m, b[0], b[3]), apply(m, b[2], b[3])];
    return [Math.min(...pts.map((p) => p[0])), Math.min(...pts.map((p) => p[1])), Math.max(...pts.map((p) => p[0])), Math.max(...pts.map((p) => p[1]))];
  };

  // ---------------------------------------------------------------- path data

  class Builder {
    constructor() {
      this.a = [];
      this.x0 = Infinity;
      this.y0 = Infinity;
      this.x1 = -Infinity;
      this.y1 = -Infinity;
    }
    pt(x, y) {
      if (x < this.x0) this.x0 = x;
      if (x > this.x1) this.x1 = x;
      if (y < this.y0) this.y0 = y;
      if (y > this.y1) this.y1 = y;
    }
    move(x, y) {
      this.a.push(OP.MOVE_TO, x, y);
      this.pt(x, y);
    }
    line(x, y) {
      this.a.push(OP.LINE_TO, x, y);
      this.pt(x, y);
    }
    quad(cx, cy, x, y) {
      this.a.push(OP.QUAD_TO, cx, cy, x, y);
      this.pt(cx, cy);
      this.pt(x, y);
    }
    cubic(c1x, c1y, c2x, c2y, x, y) {
      this.a.push(OP.CUBIC_TO, c1x, c1y, c2x, c2y, x, y);
      this.pt(c1x, c1y);
      this.pt(c2x, c2y);
      this.pt(x, y);
    }
    ellipse(cx, cy, rx, ry, rotation, start, end, ccw) {
      this.a.push(OP.ELLIPSE, cx, cy, rx, ry, rotation, start, end, ccw ? 1 : 0);
      const r = Math.max(rx, ry);
      this.pt(cx - r, cy - r);
      this.pt(cx + r, cy + r);
    }
    close() {
      this.a.push(OP.CLOSE_PATH);
    }
    done() {
      const bbox = this.x0 <= this.x1 ? [this.x0, this.y0, this.x1, this.y1] : [0, 0, 0, 0];
      return { ops: Float64Array.from(this.a), bbox };
    }
  }

  // An SVG elliptical arc (endpoint form) as an ELLIPSE op (centre form),
  // per SVG 1.1 appendix F.6.5.
  function arcTo(b, x1, y1, rx, ry, rotationDeg, largeArc, sweep, x2, y2) {
    if (x1 === x2 && y1 === y2) return;
    rx = Math.abs(rx);
    ry = Math.abs(ry);
    if (rx === 0 || ry === 0) {
      b.line(x2, y2);
      return;
    }
    const phi = (rotationDeg * Math.PI) / 180;
    const cos = Math.cos(phi);
    const sin = Math.sin(phi);
    const dx = (x1 - x2) / 2;
    const dy = (y1 - y2) / 2;
    const x1p = cos * dx + sin * dy;
    const y1p = -sin * dx + cos * dy;
    const lambda = (x1p * x1p) / (rx * rx) + (y1p * y1p) / (ry * ry);
    if (lambda > 1) {
      const s = Math.sqrt(lambda);
      rx *= s;
      ry *= s;
    }
    const num = rx * rx * ry * ry - rx * rx * y1p * y1p - ry * ry * x1p * x1p;
    const den = rx * rx * y1p * y1p + ry * ry * x1p * x1p;
    const coef = (largeArc !== sweep ? 1 : -1) * Math.sqrt(Math.max(0, num / den));
    const cxp = (coef * rx * y1p) / ry;
    const cyp = (-coef * ry * x1p) / rx;
    const cx = cos * cxp - sin * cyp + (x1 + x2) / 2;
    const cy = sin * cxp + cos * cyp + (y1 + y2) / 2;
    const vecAngle = (ux, uy, vx, vy) => {
      const a = Math.atan2(uy, ux);
      const c = Math.atan2(vy, vx);
      return c - a;
    };
    const theta1 = vecAngle(1, 0, (x1p - cxp) / rx, (y1p - cyp) / ry);
    let dtheta = vecAngle((x1p - cxp) / rx, (y1p - cyp) / ry, (-x1p - cxp) / rx, (-y1p - cyp) / ry);
    while (dtheta > Math.PI * 2) dtheta -= Math.PI * 2;
    while (dtheta < -Math.PI * 2) dtheta += Math.PI * 2;
    if (!sweep && dtheta > 0) dtheta -= Math.PI * 2;
    else if (sweep && dtheta < 0) dtheta += Math.PI * 2;
    b.ellipse(cx, cy, rx, ry, phi, theta1, theta1 + dtheta, !sweep);
  }

  const ARGS = { M: 2, L: 2, H: 1, V: 1, C: 6, S: 4, Q: 4, T: 2, A: 7, Z: 0 };

  // Parse path data into stream ops. Invalid data draws up to the error,
  // as browsers do.
  function parsePathData(d, builder) {
    const b = builder ?? new Builder();
    const s = d;
    const n = s.length;
    let i = 0;
    const sep = () => {
      while (i < n) {
        const c = s.charCodeAt(i);
        if (c === 32 || c === 44 || c === 9 || c === 10 || c === 13 || c === 12) i++;
        else break;
      }
    };
    const num = () => {
      sep();
      const start = i;
      if (s[i] === '+' || s[i] === '-') i++;
      let digits = 0;
      while (i < n && s.charCodeAt(i) >= 48 && s.charCodeAt(i) <= 57) {
        i++;
        digits++;
      }
      if (s[i] === '.') {
        i++;
        while (i < n && s.charCodeAt(i) >= 48 && s.charCodeAt(i) <= 57) {
          i++;
          digits++;
        }
      }
      if (!digits) {
        i = start;
        return NaN;
      }
      if (s[i] === 'e' || s[i] === 'E') {
        const save = i;
        i++;
        if (s[i] === '+' || s[i] === '-') i++;
        let exp = 0;
        while (i < n && s.charCodeAt(i) >= 48 && s.charCodeAt(i) <= 57) {
          i++;
          exp++;
        }
        if (!exp) i = save;
      }
      return parseFloat(s.slice(start, i));
    };
    const flag = () => {
      sep();
      const c = s[i];
      if (c === '0' || c === '1') {
        i++;
        return c === '1';
      }
      return null;
    };

    let cmd = null;
    let x = 0;
    let y = 0;
    let sx = 0;
    let sy = 0;
    let lastC = null; // reflected control point for S
    let lastQ = null; // for T
    let first = true;
    for (;;) {
      sep();
      if (i >= n) break;
      const c = s[i];
      if (/[a-zA-Z]/.test(c)) {
        if (ARGS[c.toUpperCase()] === undefined) break;
        cmd = c;
        i++;
        if (first && cmd.toUpperCase() !== 'M') break;
      } else if (cmd === null) {
        break;
      } else if (cmd.toUpperCase() === 'Z') {
        break;
      }
      first = false;
      const rel = cmd === cmd.toLowerCase();
      const up = cmd.toUpperCase();
      let ok = true;
      switch (up) {
        case 'M': {
          const a = num();
          const bb = num();
          if (Number.isNaN(a) || Number.isNaN(bb)) {
            ok = false;
            break;
          }
          x = rel ? x + a : a;
          y = rel ? y + bb : bb;
          sx = x;
          sy = y;
          b.move(x, y);
          cmd = rel ? 'l' : 'L';
          lastC = lastQ = null;
          break;
        }
        case 'L': {
          const a = num();
          const bb = num();
          if (Number.isNaN(a) || Number.isNaN(bb)) {
            ok = false;
            break;
          }
          x = rel ? x + a : a;
          y = rel ? y + bb : bb;
          b.line(x, y);
          lastC = lastQ = null;
          break;
        }
        case 'H': {
          const a = num();
          if (Number.isNaN(a)) {
            ok = false;
            break;
          }
          x = rel ? x + a : a;
          b.line(x, y);
          lastC = lastQ = null;
          break;
        }
        case 'V': {
          const a = num();
          if (Number.isNaN(a)) {
            ok = false;
            break;
          }
          y = rel ? y + a : a;
          b.line(x, y);
          lastC = lastQ = null;
          break;
        }
        case 'C': {
          const v = [num(), num(), num(), num(), num(), num()];
          if (v.some(Number.isNaN)) {
            ok = false;
            break;
          }
          const ox = rel ? x : 0;
          const oy = rel ? y : 0;
          b.cubic(v[0] + ox, v[1] + oy, v[2] + ox, v[3] + oy, v[4] + ox, v[5] + oy);
          lastC = [v[2] + ox, v[3] + oy];
          x = v[4] + ox;
          y = v[5] + oy;
          lastQ = null;
          break;
        }
        case 'S': {
          const v = [num(), num(), num(), num()];
          if (v.some(Number.isNaN)) {
            ok = false;
            break;
          }
          const ox = rel ? x : 0;
          const oy = rel ? y : 0;
          const c1x = lastC ? 2 * x - lastC[0] : x;
          const c1y = lastC ? 2 * y - lastC[1] : y;
          b.cubic(c1x, c1y, v[0] + ox, v[1] + oy, v[2] + ox, v[3] + oy);
          lastC = [v[0] + ox, v[1] + oy];
          x = v[2] + ox;
          y = v[3] + oy;
          lastQ = null;
          break;
        }
        case 'Q': {
          const v = [num(), num(), num(), num()];
          if (v.some(Number.isNaN)) {
            ok = false;
            break;
          }
          const ox = rel ? x : 0;
          const oy = rel ? y : 0;
          b.quad(v[0] + ox, v[1] + oy, v[2] + ox, v[3] + oy);
          lastQ = [v[0] + ox, v[1] + oy];
          x = v[2] + ox;
          y = v[3] + oy;
          lastC = null;
          break;
        }
        case 'T': {
          const v = [num(), num()];
          if (v.some(Number.isNaN)) {
            ok = false;
            break;
          }
          const cx = lastQ ? 2 * x - lastQ[0] : x;
          const cy = lastQ ? 2 * y - lastQ[1] : y;
          const nx = rel ? x + v[0] : v[0];
          const ny = rel ? y + v[1] : v[1];
          b.quad(cx, cy, nx, ny);
          lastQ = [cx, cy];
          x = nx;
          y = ny;
          lastC = null;
          break;
        }
        case 'A': {
          const rx = num();
          const ry = num();
          const rot = num();
          const large = flag();
          const sweep = flag();
          const ex = num();
          const ey = num();
          if ([rx, ry, rot, ex, ey].some(Number.isNaN) || large === null || sweep === null) {
            ok = false;
            break;
          }
          const nx = rel ? x + ex : ex;
          const ny = rel ? y + ey : ey;
          arcTo(b, x, y, rx, ry, rot, large, sweep, nx, ny);
          x = nx;
          y = ny;
          lastC = lastQ = null;
          break;
        }
        case 'Z':
          b.close();
          x = sx;
          y = sy;
          lastC = lastQ = null;
          break;
        default:
          ok = false;
      }
      if (!ok) break;
    }
    return b;
  }

  const pathCache = new Map();

  // Cached {ops, bbox} for a `d` string.
  function pathOps(d) {
    let p = pathCache.get(d);
    if (!p) {
      p = parsePathData(d).done();
      if (pathCache.size > 4096) pathCache.clear();
      pathCache.set(d, p);
    }
    return p;
  }

  // "x1,y1 x2,y2 ..." for polyline/polygon.
  function pointsOps(text, close) {
    const nums = (text.match(/[+-]?(?:\d*\.\d+|\d+\.?)(?:[eE][+-]?\d+)?/g) || []).map(parseFloat);
    const b = new Builder();
    for (let i = 0; i + 1 < nums.length; i += 2) {
      if (i === 0) b.move(nums[0], nums[1]);
      else b.line(nums[i], nums[i + 1]);
    }
    if (close && nums.length >= 4) b.close();
    return b.done();
  }

  function rectOps(x, y, w, h, rx, ry) {
    const b = new Builder();
    if (w <= 0 || h <= 0) return b.done();
    if (rx > 0 || ry > 0) {
      if (!(rx > 0)) rx = ry;
      if (!(ry > 0)) ry = rx;
      rx = Math.min(rx, w / 2);
      ry = Math.min(ry, h / 2);
      const half = Math.PI / 2;
      b.move(x + rx, y);
      b.line(x + w - rx, y);
      b.ellipse(x + w - rx, y + ry, rx, ry, 0, -half, 0, false);
      b.line(x + w, y + h - ry);
      b.ellipse(x + w - rx, y + h - ry, rx, ry, 0, 0, half, false);
      b.line(x + rx, y + h);
      b.ellipse(x + rx, y + h - ry, rx, ry, 0, half, Math.PI, false);
      b.line(x, y + ry);
      b.ellipse(x + rx, y + ry, rx, ry, 0, Math.PI, Math.PI * 1.5, false);
      b.close();
    } else {
      b.a.push(OP.RECT, x, y, w, h);
      b.pt(x, y);
      b.pt(x + w, y + h);
    }
    return b.done();
  }

  function ellipseOps(cx, cy, rx, ry) {
    const b = new Builder();
    if (rx <= 0 || ry <= 0) return b.done();
    b.move(cx + rx, cy);
    b.ellipse(cx, cy, rx, ry, 0, 0, Math.PI * 2, false);
    b.close();
    return b.done();
  }

  function lineOps(x1, y1, x2, y2) {
    const b = new Builder();
    b.move(x1, y1);
    b.line(x2, y2);
    return b.done();
  }

  D.geom = { IDENTITY, multiply, invert, apply, isIdentity, transformBox, Builder, parsePathData, pathOps, pointsOps, rectOps, ellipseOps, lineOps, arcTo };
})();
