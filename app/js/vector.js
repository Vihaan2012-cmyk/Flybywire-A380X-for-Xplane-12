(() => {
  const D = globalThis.__dom;
  const { OP, geom } = D;
  const SVG = 'http://www.w3.org/2000/svg';
  const SKIP = new Set(['defs', 'clipPath', 'mask', 'linearGradient', 'radialGradient', 'symbol', 'marker', 'pattern', 'style', 'script', 'title', 'desc', 'metadata', 'filter', 'foreignObject']);
  const SHAPES = new Set(['path', 'rect', 'circle', 'ellipse', 'line', 'polyline', 'polygon']);
  const HTML_SKIP = new Set(['SCRIPT', 'STYLE', 'LINK', 'META', 'HEAD', 'TITLE', 'TEMPLATE', 'NOSCRIPT', 'CANVAS', 'IFRAME']);
  const CAPS = { butt: 0, round: 1, square: 2 };
  const JOINS = { miter: 0, 'miter-clip': 0, round: 1, bevel: 2, arcs: 0 };
  const BASELINES = { auto: -1, baseline: 0, alphabetic: 0, middle: 1, central: 1, hanging: 4, 'text-before-edge': 2, 'before-edge': 2, 'text-top': 2, 'text-after-edge': 3, 'after-edge': 3, 'text-bottom': 3, ideographic: 3, mathematical: 0 };

  class Writer {
    constructor() {
      this.buf = new Float64Array(1 << 16);
      this.n = 0;
      this.strings = [];
      this.index = new Map();
    }
    room(k) {
      if (this.n + k <= this.buf.length) return;
      let size = this.buf.length * 2;
      while (size < this.n + k) size *= 2;
      const b = new Float64Array(size);
      b.set(this.buf.subarray(0, this.n));
      this.buf = b;
    }
    op(o) {
      this.room(1);
      this.buf[this.n++] = o;
    }
    nums(o, list) {
      this.room(1 + list.length);
      this.buf[this.n++] = o;
      for (let i = 0; i < list.length; i++) this.buf[this.n++] = list[i];
    }
    copy(a) {
      this.room(a.length);
      this.buf.set(a, this.n);
      this.n += a.length;
    }
    str(s) {
      let i = this.index.get(s);
      if (i === undefined) {
        i = this.strings.length;
        this.strings.push(s);
        this.index.set(s, i);
      }
      return i;
    }
  }

  const colours = new Map();
  function colour(text) {
    let c = colours.get(text);
    if (c !== undefined) return c;
    c = null;
    const m = /^rgba?\(\s*([\d.]+)[\s,]+([\d.]+)[\s,]+([\d.]+)(?:[\s,/]+([\d.]+%?))?\s*\)$/.exec(text);
    if (m) {
      let a = m[4] === undefined ? 1 : m[4].endsWith('%') ? parseFloat(m[4]) / 100 : parseFloat(m[4]);
      c = [m[1] / 255, m[2] / 255, m[3] / 255, a];
    } else if (text === 'transparent') c = [0, 0, 0, 0];
    if (colours.size > 512) colours.clear();
    colours.set(text, c);
    return c;
  }

  const urlOf = (paint) => {
    const m = /url\(\s*["']?#([^"')]+)["']?\s*\)\s*(.*)$/.exec(paint);
    return m ? { id: m[1], fallback: m[2] } : null;
  };

  const matrixOf = (m) => [m.a, m.b, m.c, m.d, m.e, m.f];
  const IDENTITY = [1, 0, 0, 1, 0, 0];

  function ctm(el) {
    const m = el.getScreenCTM();
    return m ? matrixOf(m) : IDENTITY;
  }

  function strokeArgs(cs, rgba, alpha, m) {
    let width = parseFloat(cs.strokeWidth);
    if (!(width > 0)) return null;
    if (cs.vectorEffect === 'non-scaling-stroke') {
      const s = Math.sqrt(Math.abs(m[0] * m[3] - m[1] * m[2])) || 1;
      width /= s;
    }
    let dashes = [];
    if (cs.strokeDasharray && cs.strokeDasharray !== 'none') {
      dashes = cs.strokeDasharray.split(/[\s,]+/).filter(Boolean).map(parseFloat);
      if (dashes.length % 2) dashes = dashes.concat(dashes);
      if (dashes.some((d) => !(d >= 0)) || dashes.reduce((a, b) => a + b, 0) <= 0) dashes = [];
    }
    const out = [rgba[0], rgba[1], rgba[2], rgba[3] * alpha, width, CAPS[cs.strokeLinecap] ?? 0, JOINS[cs.strokeLinejoin] ?? 0, parseFloat(cs.strokeMiterlimit) || 4, dashes.length];
    for (const d of dashes) out.push(d);
    out.push(parseFloat(cs.strokeDashoffset) || 0);
    return out;
  }

  function shapeOps(el, cs) {
    switch (el.localName) {
      case 'path': {
        let d = el.getAttribute('d');
        if (!d && cs.d && cs.d !== 'none') {
          const m = /path\(\s*["'](.*)["']\s*\)/.exec(cs.d);
          d = m ? m[1] : '';
        }
        return d ? geom.pathOps(d) : null;
      }
      case 'rect': {
        const rx = el.hasAttribute('rx') ? el.rx.baseVal.value : -1;
        const ry = el.hasAttribute('ry') ? el.ry.baseVal.value : -1;
        return geom.rectOps(el.x.baseVal.value, el.y.baseVal.value, el.width.baseVal.value, el.height.baseVal.value, rx, ry);
      }
      case 'circle': {
        const r = el.r.baseVal.value;
        return geom.ellipseOps(el.cx.baseVal.value, el.cy.baseVal.value, r, r);
      }
      case 'ellipse':
        return geom.ellipseOps(el.cx.baseVal.value, el.cy.baseVal.value, el.rx.baseVal.value, el.ry.baseVal.value);
      case 'line':
        return geom.lineOps(el.x1.baseVal.value, el.y1.baseVal.value, el.x2.baseVal.value, el.y2.baseVal.value);
      case 'polyline':
        return geom.pointsOps(el.getAttribute('points') ?? '', false);
      case 'polygon':
        return geom.pointsOps(el.getAttribute('points') ?? '', true);
      default:
        return null;
    }
  }

  function stopsOf(grad, depth = 0) {
    const stops = [];
    for (const s of grad.children) {
      if (s.localName !== 'stop') continue;
      const cs = getComputedStyle(s);
      const c = colour(cs.stopColor) ?? [0, 0, 0, 1];
      const prev = stops.length ? stops[stops.length - 1][0] : 0;
      const off = Math.min(1, Math.max(0, s.offset.baseVal, prev));
      stops.push([off, c[0], c[1], c[2], c[3] * (parseFloat(cs.stopOpacity) || 0)]);
    }
    if (!stops.length && depth < 8) {
      const href = grad.getAttribute('href') ?? grad.getAttribute('xlink:href');
      const other = href && href.startsWith('#') ? document.getElementById(href.slice(1)) : null;
      if (other) return stopsOf(other, depth + 1);
    }
    return stops;
  }

  function gradientFill(w, el, url, alpha, rule) {
    const g = document.getElementById(url.id);
    if (!g || (g.localName !== 'linearGradient' && g.localName !== 'radialGradient')) {
      const c = colour(url.fallback);
      if (c) w.nums(OP.FILL, [c[0], c[1], c[2], c[3] * alpha, rule]);
      return;
    }
    const stops = stopsOf(g);
    if (!stops.length) return;
    if (stops.length === 1 || g.localName === 'radialGradient') {
      const s = stops[stops.length - 1];
      w.nums(OP.FILL, [s[1], s[2], s[3], s[4] * alpha, rule]);
      return;
    }
    const bboxUnits = g.gradientUnits.baseVal !== SVGUnitTypes.SVG_UNIT_TYPE_USERSPACEONUSE;
    let m = null;
    if (bboxUnits) {
      const b = el.getBBox();
      if (!(b.width > 0) || !(b.height > 0)) return;
      m = [b.width, 0, 0, b.height, b.x, b.y];
    }
    const gt = g.gradientTransform.baseVal.consolidate();
    if (gt) m = geom.multiply(m ?? IDENTITY, matrixOf(gt.matrix));
    const len = (l) => (bboxUnits && l.unitType === SVGLength.SVG_LENGTHTYPE_PERCENTAGE ? l.valueInSpecifiedUnits / 100 : l.value);
    if (m) {
      w.op(OP.SAVE);
      w.nums(OP.TRANSFORM, m);
    }
    const nums = [len(g.x1.baseVal), len(g.y1.baseVal), len(g.x2.baseVal), len(g.y2.baseVal), stops.length];
    for (const s of stops) nums.push(s[0], s[1], s[2], s[3], s[4] * alpha);
    nums.push(rule);
    w.nums(OP.LINEAR_GRADIENT_FILL, nums);
    if (m) w.op(OP.RESTORE);
  }

  function paintShape(w, el, cs, alpha) {
    const g = shapeOps(el, cs);
    if (!g || !g.ops.length) return;
    const m = ctm(el);
    w.nums(OP.SET_TRANSFORM, m);
    w.op(OP.BEGIN_PATH);
    w.copy(g.ops);
    const rule = cs.fillRule === 'evenodd' ? 1 : 0;
    const fill = () => {
      if (el.localName === 'line' || cs.fill === 'none') return;
      const fo = parseFloat(cs.fillOpacity);
      const url = cs.fill.startsWith('url(') ? urlOf(cs.fill) : null;
      if (url) return gradientFill(w, el, url, alpha * fo, rule);
      const c = colour(cs.fill);
      if (c && c[3] * fo * alpha > 0) w.nums(OP.FILL, [c[0], c[1], c[2], c[3] * fo * alpha, rule]);
    };
    const stroke = () => {
      if (cs.stroke === 'none') return;
      let c;
      if (cs.stroke.startsWith('url(')) {
        const url = urlOf(cs.stroke);
        const gr = url && document.getElementById(url.id);
        const stops = gr ? stopsOf(gr) : [];
        c = stops.length ? stops[0].slice(1) : url && colour(url.fallback);
      } else c = colour(cs.stroke);
      if (!c) return;
      const a = parseFloat(cs.strokeOpacity) * alpha;
      if (c[3] * a <= 0) return;
      const args = strokeArgs(cs, c, a, m);
      if (args) w.nums(OP.STROKE, args);
    };
    if (cs.paintOrder.startsWith('stroke')) {
      stroke();
      fill();
    } else {
      fill();
      stroke();
    }
  }

  function clipWith(w, el, id) {
    const clip = document.getElementById(id);
    if (!clip || clip.localName !== 'clipPath') return false;
    const shapes = [];
    for (const c of clip.children) {
      if (!SHAPES.has(c.localName)) continue;
      const cs = getComputedStyle(c);
      if (cs.display === 'none' || cs.visibility !== 'visible') continue;
      shapes.push([c, cs]);
    }
    w.op(OP.SAVE);
    if (!shapes.length) {
      w.nums(OP.CLIP_RECT, [0, 0, 0, 0]);
      return true;
    }
    let base = ctm(el);
    if (clip.clipPathUnits.baseVal === SVGUnitTypes.SVG_UNIT_TYPE_OBJECTBOUNDINGBOX) {
      const b = el.getBBox();
      base = geom.multiply(base, [b.width, 0, 0, b.height, b.x, b.y]);
    }
    const ct = clip.transform.baseVal.consolidate();
    if (ct) base = geom.multiply(base, matrixOf(ct.matrix));
    w.op(OP.BEGIN_PATH);
    for (const [s] of shapes) {
      const g = shapeOps(s, getComputedStyle(s));
      if (!g) continue;
      const st = s.transform.baseVal.consolidate();
      w.nums(OP.SET_TRANSFORM, st ? geom.multiply(base, matrixOf(st.matrix)) : base);
      w.copy(g.ops);
    }
    w.nums(OP.CLIP_PATH, [shapes[0][1].clipRule === 'evenodd' ? 1 : 0]);
    return true;
  }

  const fontFamily = (cs) => cs.fontFamily.replace(/["']/g, '');
  const weightOf = (cs) => Math.min(900, Math.max(100, Math.round((parseInt(cs.fontWeight, 10) || 400) / 100) * 100));
  const italicOf = (cs) => (cs.fontStyle === 'italic' || cs.fontStyle.startsWith('oblique') ? 1 : 0);

  function textColours(cs, alpha) {
    let fc = [0, 0, 0, 0];
    if (cs.fill !== 'none') {
      if (cs.fill.startsWith('url(')) {
        const url = urlOf(cs.fill);
        const g = url && document.getElementById(url.id);
        const stops = g ? stopsOf(g) : [];
        fc = stops.length ? stops[0].slice(1) : (url && colour(url.fallback)) || fc;
      } else fc = colour(cs.fill) ?? fc;
    }
    const fa = fc[3] * parseFloat(cs.fillOpacity) * alpha;
    let sw = 0;
    let sc = [0, 0, 0, 0];
    if (cs.stroke !== 'none' && !cs.stroke.startsWith('url(')) {
      const c = colour(cs.stroke);
      if (c) {
        sw = parseFloat(cs.strokeWidth) || 0;
        sc = [c[0], c[1], c[2], c[3] * parseFloat(cs.strokeOpacity) * alpha];
      }
    }
    return { fill: [fc[0], fc[1], fc[2], fa], sw, sc };
  }

  function emitText(w, cs, text, x, y, baseline, col) {
    w.nums(OP.TEXT, [w.str(text), w.str(fontFamily(cs)), parseFloat(cs.fontSize), weightOf(cs), italicOf(cs), x, y, 0, baseline, col.fill[0], col.fill[1], col.fill[2], col.fill[3], col.sw, col.sc[0], col.sc[1], col.sc[2], col.sc[3]]);
  }

  function textRun(w, el, cs, alpha) {
    if (cs.visibility !== 'visible') return;
    const n = el.getNumberOfChars();
    if (!n) return;
    const col = textColours(cs, alpha);
    if (col.fill[3] <= 0 && (col.sw <= 0 || col.sc[3] <= 0)) return;
    let text = '';
    for (const c of el.childNodes) if (c.nodeType === 3) text += c.data;
    text = text.replace(/[\n\r\t]/g, ' ').replace(/ {2,}/g, ' ');
    if (text.length !== n) text = text.trim();
    if (!text.length) return;
    const spaced = cs.letterSpacing !== 'normal' && parseFloat(cs.letterSpacing) !== 0;
    if (spaced || text.length !== n) {
      const chars = [...text];
      for (let i = 0; i < Math.min(n, chars.length); i++) {
        if (chars[i] === ' ') continue;
        const p = el.getStartPositionOfChar(i);
        emitText(w, cs, chars[i], p.x, p.y, 0, col);
      }
      return;
    }
    const p = el.getStartPositionOfChar(0);
    emitText(w, cs, text, p.x, p.y, 0, col);
  }

  function paintText(w, el, cs, alpha) {
    w.nums(OP.SET_TRANSFORM, ctm(el));
    const visit = (node, ncs, a) => {
      let hasChildElements = false;
      for (const c of node.children) {
        if (c.localName === 'tspan' || c.localName === 'a' || c.localName === 'textPath') {
          hasChildElements = true;
          break;
        }
      }
      if (!hasChildElements) return textRun(w, node, ncs, a);
      let offset = 0;
      for (const c of node.childNodes) {
        if (c.nodeType === 1 && c.namespaceURI === SVG) {
          const ccs = getComputedStyle(c);
          if (ccs.display === 'none') continue;
          visit(c, ccs, a * (parseFloat(ccs.opacity) || 1));
          offset += c.getNumberOfChars ? c.getNumberOfChars() : 0;
        } else if (c.nodeType === 3) {
          const t = c.data.replace(/[\n\r\t]/g, ' ').replace(/ {2,}/g, ' ');
          if (t.trim() && ncs.visibility === 'visible' && offset < node.getNumberOfChars()) {
            const col = textColours(ncs, a);
            const p = node.getStartPositionOfChar(offset + (t.length - t.trimStart().length));
            emitText(w, ncs, t.trim(), p.x, p.y, 0, col);
          }
          offset += t.trim() ? t.length : 0;
        }
      }
    };
    visit(el, cs, alpha);
  }

  function svgNode(w, el, alpha) {
    const name = el.localName;
    if (SKIP.has(name)) return;
    const cs = getComputedStyle(el);
    if (cs.display === 'none') return;
    const a = alpha * (parseFloat(cs.opacity) ?? 1);
    if (!(a > 0)) return;
    let saves = 0;
    if (cs.clipPath && cs.clipPath !== 'none') {
      const url = urlOf(cs.clipPath);
      if (url && clipWith(w, el, url.id)) saves++;
    }
    if (name === 'svg' && cs.overflow !== 'visible' && cs.overflow !== 'auto') {
      w.op(OP.SAVE);
      if (el.ownerSVGElement && el.parentNode && el.parentNode.getScreenCTM) {
        w.nums(OP.SET_TRANSFORM, ctm(el.parentNode));
        w.nums(OP.CLIP_RECT, [el.x.baseVal.value, el.y.baseVal.value, el.width.baseVal.value, el.height.baseVal.value]);
      } else {
        const r = el.getBoundingClientRect();
        w.nums(OP.SET_TRANSFORM, IDENTITY);
        w.nums(OP.CLIP_RECT, [r.left, r.top, r.width, r.height]);
      }
      saves++;
    }
    if (SHAPES.has(name)) {
      if (cs.visibility === 'visible') paintShape(w, el, cs, a);
    } else if (name === 'text') {
      paintText(w, el, cs, a);
    } else if (name === 'image') {
      paintImage(w, el, cs, a);
    } else if (name === 'use') {
      paintUse(w, el, a);
    } else {
      for (const c of el.children) svgNode(w, c, a);
    }
    for (let i = 0; i < saves; i++) w.op(OP.RESTORE);
  }

  const imageSizes = new Map();
  function imageSize(href) {
    let s = imageSizes.get(href);
    if (!s) {
      s = { w: 0, h: 0 };
      imageSizes.set(href, s);
      const img = new Image();
      img.onload = () => {
        s.w = img.naturalWidth;
        s.h = img.naturalHeight;
      };
      img.src = href;
    }
    return s;
  }

  function paintImage(w, el, cs, alpha) {
    if (cs.visibility !== 'visible') return;
    const href = el.href.baseVal;
    if (!href) return;
    const s = imageSize(href);
    const x = el.x.baseVal.value;
    const y = el.y.baseVal.value;
    const width = el.width.baseVal.value || s.w;
    const height = el.height.baseVal.value || s.h;
    if (!(s.w > 0 && s.h > 0 && width > 0 && height > 0)) return;
    const k = Math.min(width / s.w, height / s.h);
    const dw = s.w * k;
    const dh = s.h * k;
    w.op(OP.SAVE);
    w.nums(OP.SET_TRANSFORM, ctm(el));
    if (alpha < 1) w.nums(OP.GLOBAL_ALPHA, [alpha]);
    w.nums(OP.CLIP_RECT, [x, y, width, height]);
    w.nums(OP.IMAGE, [w.str(href), 0, 0, s.w, s.h, x + (width - dw) / 2, y + (height - dh) / 2, dw, dh]);
    w.op(OP.RESTORE);
  }

  function paintUse(w, el, alpha) {
    const href = el.href.baseVal;
    const ref = href && href.startsWith('#') ? document.getElementById(href.slice(1)) : null;
    if (!ref || !ref.getScreenCTM) return;
    const um = geom.multiply(ctm(el), [1, 0, 0, 1, el.x.baseVal.value, el.y.baseVal.value]);
    const rm = ref.getScreenCTM();
    if (!rm) return;
    const fix = geom.multiply(um, geom.invert(matrixOf(rm)));
    const inner = new Writer();
    svgNode(inner, ref, alpha);
    w.op(OP.SAVE);
    w.nums(OP.SET_TRANSFORM, fix);
    rebase(w, inner, fix);
    w.op(OP.RESTORE);
  }

  function rebase(w, inner, fix) {
    const b = inner.buf;
    let i = 0;
    while (i < inner.n) {
      const op = b[i];
      if (op === OP.SET_TRANSFORM) {
        w.nums(OP.SET_TRANSFORM, geom.multiply(fix, [b[i + 1], b[i + 2], b[i + 3], b[i + 4], b[i + 5], b[i + 6]]));
        i += 7;
        continue;
      }
      const len = opLength(b, i);
      if (op === OP.TEXT || op === OP.IMAGE) {
        const s = inner.strings;
        const copy = Array.from(b.subarray(i + 1, i + len));
        copy[0] = w.str(s[copy[0]]);
        if (op === OP.TEXT) copy[1] = w.str(s[copy[1]]);
        w.nums(op, copy);
      } else w.copy(b.subarray(i, i + len));
      i += len;
    }
  }

  function opLength(b, i) {
    switch (b[i]) {
      case OP.SAVE: case OP.RESTORE: case OP.BEGIN_PATH: case OP.CLOSE_PATH: return 1;
      case OP.TRANSFORM: case OP.SET_TRANSFORM: return 7;
      case OP.GLOBAL_ALPHA: case OP.CLIP_PATH: return 2;
      case OP.CLIP_RECT: case OP.RECT: case OP.QUAD_TO: return 5;
      case OP.MOVE_TO: case OP.LINE_TO: return 3;
      case OP.CUBIC_TO: return 7;
      case OP.ARC: return 7;
      case OP.ELLIPSE: return 9;
      case OP.FILL: return 6;
      case OP.STROKE: return 11 + b[i + 9];
      case OP.TEXT: return 19;
      case OP.IMAGE: return 10;
      case OP.LINEAR_GRADIENT_FILL: return 7 + 5 * b[i + 5];
      default: return 1;
    }
  }

  const fontMetrics = new Map();
  let measureCtx = null;
  function ascentOf(cs) {
    const key = `${cs.fontStyle} ${cs.fontWeight} ${cs.fontSize} ${cs.fontFamily}`;
    let m = fontMetrics.get(key);
    if (!m) {
      measureCtx = measureCtx || document.createElement('canvas').getContext('2d');
      measureCtx.font = key;
      const t = measureCtx.measureText('Hg');
      m = [t.fontBoundingBoxAscent, t.fontBoundingBoxDescent];
      fontMetrics.set(key, m);
    }
    return m;
  }

  function htmlText(w, node, cs, alpha) {
    if (cs.visibility !== 'visible') return;
    const raw = node.data;
    if (!raw.trim()) return;
    const c = colour(cs.color);
    if (!c || c[3] * alpha <= 0) return;
    const range = document.createRange();
    range.selectNodeContents(node);
    const rects = range.getClientRects();
    if (!rects.length) return;
    const col = { fill: [c[0], c[1], c[2], c[3] * alpha], sw: 0, sc: [0, 0, 0, 0] };
    const [asc] = ascentOf(cs);
    const text = raw.replace(/\s+/g, ' ').trim();
    if (rects.length === 1) {
      const r = rects[0];
      emitText(w, cs, text, r.left, r.top + asc + (r.height - asc - ascentOf(cs)[1]) / 2, 0, col);
      return;
    }
    const chars = [...raw];
    let start = 0;
    for (let i = 0; i <= chars.length; i++) {
      if (i < chars.length) {
        range.setStart(node, i);
        range.setEnd(node, i + 1);
      }
      const r = i < chars.length ? range.getBoundingClientRect() : null;
      if (i === chars.length || (i > start && r && r.width > 0 && Math.abs(r.top - lineTop) > 1)) {
        const seg = chars.slice(start, i).join('').replace(/\s+/g, ' ').trim();
        if (seg) emitText(w, cs, seg, lineLeft, lineTop + asc + (lineH - asc - ascentOf(cs)[1]) / 2, 0, col);
        start = i;
      }
      if (r && r.width > 0 && i === start) {
        lineTop = r.top;
        lineLeft = r.left;
        lineH = r.height;
      }
    }
  }
  let lineTop = 0;
  let lineLeft = 0;
  let lineH = 0;

  function htmlBox(w, el, cs, alpha) {
    const r = el.getBoundingClientRect();
    if (!(r.width > 0 && r.height > 0)) return;
    const bg = colour(cs.backgroundColor);
    const radius = parseFloat(cs.borderTopLeftRadius) || 0;
    const box = (x, y, wd, ht) => {
      w.op(OP.BEGIN_PATH);
      if (radius > 0) w.copy(geom.rectOps(x, y, wd, ht, radius, radius).ops);
      else w.nums(OP.RECT, [x, y, wd, ht]);
    };
    if (bg && bg[3] * alpha > 0) {
      w.nums(OP.SET_TRANSFORM, IDENTITY);
      box(r.left, r.top, r.width, r.height);
      w.nums(OP.FILL, [bg[0], bg[1], bg[2], bg[3] * alpha, 0]);
    }
    const bw = parseFloat(cs.borderTopWidth) || 0;
    if (bw > 0 && cs.borderTopStyle !== 'none' && cs.borderTopStyle !== 'hidden') {
      const bc = colour(cs.borderTopColor);
      if (bc && bc[3] * alpha > 0) {
        w.nums(OP.SET_TRANSFORM, IDENTITY);
        box(r.left + bw / 2, r.top + bw / 2, r.width - bw, r.height - bw);
        w.nums(OP.STROKE, [bc[0], bc[1], bc[2], bc[3] * alpha, bw, 0, 0, 4, cs.borderTopStyle === 'dashed' ? 2 : 0, ...(cs.borderTopStyle === 'dashed' ? [bw * 3, bw * 3] : []), 0]);
      }
    }
  }

  function htmlNode(w, el, alpha) {
    if (HTML_SKIP.has(el.tagName)) return;
    const cs = getComputedStyle(el);
    if (cs.display === 'none') return;
    const a = alpha * (parseFloat(cs.opacity) ?? 1);
    if (!(a > 0)) return;
    let saved = false;
    if (cs.overflowX !== 'visible' || cs.overflowY !== 'visible') {
      const r = el.getBoundingClientRect();
      w.op(OP.SAVE);
      w.nums(OP.SET_TRANSFORM, IDENTITY);
      w.nums(OP.CLIP_RECT, [r.left, r.top, r.width, r.height]);
      saved = true;
    }
    if (cs.visibility === 'visible') htmlBox(w, el, cs, a);
    if (el.tagName === 'IMG') {
      if (cs.visibility === 'visible' && el.complete && el.naturalWidth > 0) {
        const r = el.getBoundingClientRect();
        w.op(OP.SAVE);
        w.nums(OP.SET_TRANSFORM, IDENTITY);
        if (a < 1) w.nums(OP.GLOBAL_ALPHA, [a]);
        w.nums(OP.IMAGE, [w.str(el.getAttribute('src')), 0, 0, el.naturalWidth, el.naturalHeight, r.left, r.top, r.width, r.height]);
        w.op(OP.RESTORE);
      }
      if (saved) w.op(OP.RESTORE);
      return;
    }
    for (const c of el.childNodes) {
      if (c.nodeType === 3) {
        w.nums(OP.SET_TRANSFORM, IDENTITY);
        htmlText(w, c, cs, a);
      } else if (c.nodeType === 1) {
        if (c.namespaceURI === SVG) svgNode(w, c, a);
        else htmlNode(w, c, a);
      }
    }
    if (saved) w.op(OP.RESTORE);
  }

  function capture(root) {
    const w = new Writer();
    const start = performance.now();
    htmlNode(w, root || document.body, 1);
    return { ops: w.buf.subarray(0, w.n), strings: w.strings, ms: performance.now() - start, width: innerWidth, height: innerHeight };
  }

  globalThis.__xphfbwVector = { capture, opLength };
})();
