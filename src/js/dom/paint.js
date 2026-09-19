// Painting: the tree to display-stream ops (docs/display-stream.md), SVG
// and CSS boxes alike, with a cached chunk per element; and the geometry
// that follows from it (getBBox, getComputedTextLength,
// getBoundingClientRect, hit testing).
//
// A chunk only ever draws relative to the transform it starts under
// (TRANSFORM multiplies), so an element's chunk stays valid when anything
// outside its subtree changes, and a frame copies clean chunks whole.
(() => {
  const D = globalThis.__dom;
  const { css, geom, OP, warnOnce } = D;
  const HTML_NS = D.NS.HTML;
  const SVG_NS = D.NS.SVG;
  const XLINK_NS = D.NS.XLINK;

  // Past this many strings the table starts again and every chunk is rebuilt.
  const MAX_STRINGS = 4096;

  // ---------------------------------------------------------------- the writer

  class Writer {
    constructor() {
      this.buf = new Float64Array(1 << 15);
      this.n = 0;
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
    transform(m) {
      this.nums(OP.TRANSFORM, m);
    }
    copy(chunk) {
      this.room(chunk.length);
      this.buf.set(chunk, this.n);
      this.n += chunk.length;
    }
  }
  D.Writer = Writer;

  const isOuterSvg = (el) => el.namespaceURI === SVG_NS && el.localName === 'svg' && !(el.parentNode && el.parentNode.nodeType === 1 && el.parentNode.namespaceURI === SVG_NS);

  // ---------------------------------------------------------------- lengths

  // An SVG length attribute in user units; percentages of `base`.
  function attrLen(el, name, base, fontSize, fallback = 0) {
    const v = el.getAttribute(name);
    if (v === null || v === '') return fallback;
    const len = css.parseLength(v);
    if (!len) return fallback;
    return css.toPx(len, base, fontSize, el.ownerDocument);
  }
  const diag = (vp) => Math.sqrt((vp.w * vp.w + vp.h * vp.h) / 2);

  function viewBoxMatrix(vb, w, h, par) {
    const sx = w / vb[2];
    const sy = h / vb[3];
    const p = (par || 'xMidYMid meet').trim().split(/\s+/);
    if (p[0] === 'none') return [sx, 0, 0, sy, -vb[0] * sx, -vb[1] * sy];
    const s = p[1] === 'slice' ? Math.max(sx, sy) : Math.min(sx, sy);
    const ax = p[0].slice(0, 4);
    const ay = p[0].slice(4);
    let tx = -vb[0] * s;
    let ty = -vb[1] * s;
    if (ax === 'xMid') tx += (w - vb[2] * s) / 2;
    else if (ax === 'xMax') tx += w - vb[2] * s;
    if (ay === 'YMid') ty += (h - vb[3] * s) / 2;
    else if (ay === 'YMax') ty += h - vb[3] * s;
    return [s, 0, 0, s, tx, ty];
  }

  // transform-origin as [x, y] px within a w x h box.
  function originOf(cs, w, h, svg) {
    const raw = cs['transform-origin'];
    const t = raw === 'initial' || raw === '' ? (svg ? ['0', '0'] : ['50%', '50%']) : css.splitTokens(raw);
    const word = { left: '0%', center: '50%', right: '100%', top: '0%', bottom: '100%' };
    let xs = t[0] ?? '50%';
    let ys = t[1] ?? '50%';
    if (xs === 'top' || xs === 'bottom') [xs, ys] = [ys, xs];
    const one = (s, base) => {
      const len = css.parseLength(word[s] ?? s);
      return len ? css.toPx(len, base, cs['font-size'], null) : 0;
    };
    return [one(xs, w), one(ys, h)];
  }

  const around = (m, ox, oy) => (ox || oy ? geom.multiply(geom.multiply([1, 0, 0, 1, ox, oy], m), [1, 0, 0, 1, -ox, -oy]) : m);

  // The matrix an SVG element's transform (CSS property or attribute) gives,
  // cached on the element by the attribute's text.
  function svgTransform(el, cs, vp) {
    if (cs.transform !== 'none') {
      const m = css.transformMatrix(cs.transform, vp.w, vp.h, cs['font-size'], el.ownerDocument);
      const [ox, oy] = originOf(cs, vp.w, vp.h, true);
      return around(m, ox, oy);
    }
    const attr = el._attrs.get('transform');
    if (attr === undefined || attr === '') return null;
    if (el._tfText === attr) return el._tf;
    const list = css.parseTransform(attr);
    const m = list === 'none' || !list.length ? null : css.transformMatrix(list, 0, 0, cs['font-size'], el.ownerDocument);
    el._tfText = attr;
    el._tf = m;
    return m;
  }

  // ---------------------------------------------------------------- colours and paint

  const colorOf = (c, cs) => (c === css.CURRENT ? cs.color : c);


  function strokeOp(w, cs, rgba, alpha, vp) {
    const width = css.toPx(cs['stroke-width'], diag(vp), cs['font-size'], null);
    if (!(width > 0)) return;
    const cap = { butt: 0, round: 1, square: 2 }[cs['stroke-linecap']] ?? 0;
    const join = { miter: 0, 'miter-clip': 0, round: 1, bevel: 2, arcs: 0 }[cs['stroke-linejoin']] ?? 0;
    let dashes = [];
    const da = cs['stroke-dasharray'];
    if (da !== 'none') {
      dashes = da.map((l) => css.toPx(l, diag(vp), cs['font-size'], null));
      if (dashes.some((d) => d < 0) || dashes.reduce((a, b) => a + b, 0) <= 0) dashes = [];
    }
    const offset = css.toPx(cs['stroke-dashoffset'], diag(vp), cs['font-size'], null);
    w.room(12 + dashes.length);
    w.nums(OP.STROKE, [rgba[0], rgba[1], rgba[2], rgba[3] * alpha, width, cap, join, cs['stroke-miterlimit'], dashes.length, ...dashes, offset]);
  }

  // ---------------------------------------------------------------- references

  // Elements by id, cached until the tree or an id changes (document.js
  // drops the cache).
  function byId(doc, id) {
    if (!doc._idCache) {
      const map = new Map();
      D.walkElements(doc, (el) => {
        const v = el._attrs.get('id');
        if (v !== undefined && !map.has(v)) map.set(v, el);
        return false;
      });
      doc._idCache = map;
    }
    return doc._idCache.get(id) ?? null;
  }

  function use(doc, user, id) {
    let set = doc._refUsers.get(id);
    if (!set) doc._refUsers.set(id, (set = new Set()));
    set.add(user);
  }

  const hrefOf = (el) => el.getAttribute('href') ?? el.getAttributeNS(XLINK_NS, 'href') ?? el.getAttribute('xlink:href');

  function gradientStops(doc, grad, user, depth = 0) {
    const stops = [];
    for (let c = grad._first; c; c = c._next) {
      if (c.nodeType !== 1 || c.localName !== 'stop' || !c._cs) continue;
      const raw = c.getAttribute('offset') ?? '0';
      let offset = raw.trim().endsWith('%') ? parseFloat(raw) / 100 : parseFloat(raw);
      if (!Number.isFinite(offset)) offset = 0;
      offset = Math.min(1, Math.max(0, offset, stops.length ? stops[stops.length - 1][0] : 0));
      const color = colorOf(c._cs['stop-color'], c._cs);
      stops.push([offset, color[0], color[1], color[2], color[3] * c._cs['stop-opacity']]);
    }
    if (!stops.length && depth < 8) {
      const href = hrefOf(grad);
      if (href && href.startsWith('#')) {
        const other = byId(doc, href.slice(1));
        if (other) {
          use(doc, user, href.slice(1));
          return gradientStops(doc, other, user, depth + 1);
        }
      }
    }
    return stops;
  }

  // Fill the current path with a paint server; false when it is not one.
  function fillWithServer(w, doc, el, paint, cs, alpha, rule, bbox, vp) {
    use(doc, el, paint.url);
    const server = byId(doc, paint.url);
    if (!server || server.namespaceURI !== SVG_NS || (server.localName !== 'linearGradient' && server.localName !== 'radialGradient')) {
      if (paint.fallback && Array.isArray(paint.fallback)) {
        const c = paint.fallback;
        w.nums(OP.FILL, [c[0], c[1], c[2], c[3] * alpha, rule]);
      } else warnOnce(`paint:${paint.url}`, `paint server #${paint.url} is not a gradient (not drawn)`);
      return;
    }
    const stops = gradientStops(doc, server, el);
    if (!stops.length) return;
    if (server.localName === 'radialGradient') warnOnce('radialGradient', 'radialGradient is drawn in its last stop colour');
    if (stops.length === 1 || server.localName === 'radialGradient') {
      const s = stops[stops.length - 1];
      w.nums(OP.FILL, [s[1], s[2], s[3], s[4] * alpha, rule]);
      return;
    }
    const userSpace = server.getAttribute('gradientUnits') === 'userSpaceOnUse';
    let m = null;
    let base = vp;
    if (!userSpace) {
      const bw = bbox[2] - bbox[0];
      const bh = bbox[3] - bbox[1];
      if (!(bw > 0) || !(bh > 0)) return;
      m = [bw, 0, 0, bh, bbox[0], bbox[1]];
      base = { w: 1, h: 1 };
    }
    const g = server.getAttribute('gradientTransform');
    if (g) {
      const list = css.parseTransform(g);
      if (list !== 'none' && list.length) m = geom.multiply(m ?? geom.IDENTITY, css.transformMatrix(list, 0, 0, 16, doc));
    }
    const len = (name, dflt, b) => {
      const v = server.getAttribute(name);
      const l = css.parseLength(v ?? dflt) ?? css.parseLength(dflt);
      return css.toPx(l, b, 16, doc);
    };
    const x1 = len('x1', '0%', base.w);
    const y1 = len('y1', '0%', base.h);
    const x2 = len('x2', '100%', base.w);
    const y2 = len('y2', '0%', base.h);
    if (m) {
      w.op(OP.SAVE);
      w.transform(m);
    }
    const nums = [x1, y1, x2, y2, stops.length];
    for (const s of stops) nums.push(s[0], s[1], s[2], s[3], s[4] * alpha);
    nums.push(rule);
    w.nums(OP.LINEAR_GRADIENT_FILL, nums);
    if (m) w.op(OP.RESTORE);
  }

  // Intersect the clip with a clipPath's shapes.
  function clipWith(w, doc, el, id, bboxOf, vp) {
    use(doc, el, id);
    const clip = byId(doc, id);
    if (!clip || clip.localName !== 'clipPath') {
      warnOnce(`clip:${id}`, `clip-path url(#${id}) does not name a clipPath (not clipped)`);
      return;
    }
    const shapes = [];
    for (let c = clip._first; c; c = c._next) {
      if (c.nodeType !== 1 || c.namespaceURI !== SVG_NS || !c._cs || c._cs.display === 'none' || c._cs.visibility !== 'visible') continue;
      if (!SHAPES.has(c.localName)) {
        warnOnce(`clip-child:${c.localName}`, `<${c.localName}> in a clipPath is not supported (ignored)`);
        continue;
      }
      shapes.push(c);
    }
    const clipCs = clip._cs ?? css.INITIAL;
    const own = clip._cs ? svgTransform(clip, clipCs, vp) : null;
    let bboxM = null;
    if (clip.getAttribute('clipPathUnits') === 'objectBoundingBox') {
      const b = bboxOf();
      bboxM = [b[2] - b[0], 0, 0, b[3] - b[1], b[0], b[1]];
    }
    if (shapes.length === 1 && !own && !bboxM && shapes[0].localName === 'rect' && !svgTransform(shapes[0], shapes[0]._cs, vp) && !shapes[0].getAttribute('rx') && !shapes[0].getAttribute('ry')) {
      const s = shapes[0];
      const fs = s._cs['font-size'];
      w.nums(OP.CLIP_RECT, [attrLen(s, 'x', vp.w, fs), attrLen(s, 'y', vp.h, fs), attrLen(s, 'width', vp.w, fs), attrLen(s, 'height', vp.h, fs)]);
      return;
    }
    if (!shapes.length) {
      w.nums(OP.CLIP_RECT, [0, 0, 0, 0]);
      return;
    }
    w.op(OP.BEGIN_PATH);
    const wrap = own || bboxM;
    if (wrap) {
      w.op(OP.SAVE);
      if (own) w.transform(own);
      if (bboxM) w.transform(bboxM);
    }
    for (const s of shapes) {
      const m = svgTransform(s, s._cs, vp);
      if (m) {
        w.op(OP.SAVE);
        w.transform(m);
      }
      w.copy(shapeGeometry(s, vp).ops);
      if (m) w.op(OP.RESTORE);
    }
    if (wrap) w.op(OP.RESTORE);
    w.nums(OP.CLIP_PATH, [shapes[0]._cs['clip-rule'] === 'evenodd' ? 1 : 0]);
  }

  // ---------------------------------------------------------------- SVG shapes

  const SHAPES = new Set(['path', 'rect', 'circle', 'ellipse', 'line', 'polyline', 'polygon']);

  function shapeGeometry(el, vp) {
    const fs = el._cs ? el._cs['font-size'] : 16;
    switch (el.localName) {
      case 'path':
        return geom.pathOps(el.getAttribute('d') ?? '');
      case 'rect': {
        const rxA = el.getAttribute('rx');
        const ryA = el.getAttribute('ry');
        const rx = rxA === null || rxA === 'auto' ? -1 : attrLen(el, 'rx', vp.w, fs);
        const ry = ryA === null || ryA === 'auto' ? -1 : attrLen(el, 'ry', vp.h, fs);
        return geom.rectOps(attrLen(el, 'x', vp.w, fs), attrLen(el, 'y', vp.h, fs), attrLen(el, 'width', vp.w, fs), attrLen(el, 'height', vp.h, fs), rx, ry);
      }
      case 'circle': {
        const r = attrLen(el, 'r', diag(vp), fs);
        return geom.ellipseOps(attrLen(el, 'cx', vp.w, fs), attrLen(el, 'cy', vp.h, fs), r, r);
      }
      case 'ellipse':
        return geom.ellipseOps(attrLen(el, 'cx', vp.w, fs), attrLen(el, 'cy', vp.h, fs), attrLen(el, 'rx', vp.w, fs), attrLen(el, 'ry', vp.h, fs));
      case 'line':
        return geom.lineOps(attrLen(el, 'x1', vp.w, fs), attrLen(el, 'y1', vp.h, fs), attrLen(el, 'x2', vp.w, fs), attrLen(el, 'y2', vp.h, fs));
      case 'polyline':
        return geom.pointsOps(el.getAttribute('points') ?? '', false);
      case 'polygon':
        return geom.pointsOps(el.getAttribute('points') ?? '', true);
      default:
        return { ops: new Float64Array(0), bbox: [0, 0, 0, 0] };
    }
  }

  function paintShape(el, w, doc, cs, vp) {
    const g = shapeGeometry(el, vp);
    if (!g.ops.length || cs.visibility !== 'visible') return;
    const fill = cs.fill;
    const stroke = cs.stroke;
    const strokeFirst = cs['paint-order'].startsWith('stroke');
    const rule = cs['fill-rule'] === 'evenodd' ? 1 : 0;
    const doFill = () => {
      if (fill === 'none' || el.localName === 'line') return;
      if (fill.url !== undefined) return fillWithServer(w, doc, el, fill, cs, cs['fill-opacity'], rule, g.bbox, vp);
      const c = colorOf(fill, cs);
      if (c[3] * cs['fill-opacity'] > 0) w.nums(OP.FILL, [c[0], c[1], c[2], c[3] * cs['fill-opacity'], rule]);
    };
    const doStroke = () => {
      if (stroke === 'none') return;
      let c;
      if (stroke.url !== undefined) {
        use(doc, el, stroke.url);
        const server = byId(doc, stroke.url);
        const stops = server ? gradientStops(doc, server, el) : [];
        if (stops.length) {
          warnOnce('stroke-gradient', 'gradient strokes are drawn in the gradient\'s first stop colour');
          c = stops[0].slice(1);
        } else if (Array.isArray(stroke.fallback)) c = stroke.fallback;
        else return;
      } else c = colorOf(stroke, cs);
      if (c[3] * cs['stroke-opacity'] > 0) strokeOp(w, cs, c, cs['stroke-opacity'], vp);
    };
    w.op(OP.BEGIN_PATH);
    w.copy(g.ops);
    if (strokeFirst) {
      doStroke();
      doFill();
    } else {
      doFill();
      doStroke();
    }
  }

  // ---------------------------------------------------------------- SVG text

  const BASELINES = {
    auto: -1, baseline: 0, alphabetic: 0, middle: 1, central: 1, hanging: 4, 'text-before-edge': 2, 'before-edge': 2,
    'text-top': 2, 'text-after-edge': 3, 'after-edge': 3, 'text-bottom': 3, ideographic: 3, mathematical: 0,
  };
  const baselineOf = (cs) => {
    const a = BASELINES[cs['alignment-baseline']] ?? -1;
    if (a >= 0) return a;
    const d = BASELINES[cs['dominant-baseline']] ?? -1;
    return d >= 0 ? d : 0;
  };

  const firstNumber = (el, name, base, fs, warnKey) => {
    const v = el._attrs.get(name);
    if (v === undefined) return null;
    const cache = el._numCache || (el._numCache = new Map());
    const hit = cache.get(name);
    if (hit && hit[0] === v && hit[1] === base && hit[2] === fs) return hit[3];
    const parts = v.trim().split(/[\s,]+/).filter(Boolean);
    let out = null;
    if (parts.length) {
      if (parts.length > 1) warnOnce(`${warnKey}-list`, `<${el.localName} ${name}="${v}">: only the first value of a position list is used`);
      const len = css.parseLength(parts[0]);
      out = len ? css.toPx(len, base, fs, el.ownerDocument) : null;
    }
    cache.set(name, [v, base, fs, out]);
    return out;
  };

  // Lay out a <text>: chunks of runs with their positions (anchors applied).
  function layoutText(textEl, doc, vp) {
    const segs = [];
    const walk = (el, inheritedPreserve) => {
      const cs = el._cs;
      if (!cs || cs.display === 'none') return;
      const fs = cs['font-size'];
      segs.push({
        pos: true, el, cs,
        x: firstNumber(el, 'x', vp.w, fs, 'x'), y: firstNumber(el, 'y', vp.h, fs, 'y'),
        dx: firstNumber(el, 'dx', vp.w, fs, 'dx') ?? 0, dy: firstNumber(el, 'dy', vp.h, fs, 'dy') ?? 0,
      });
      const space = el.getAttribute('xml:space');
      const preserve = space === 'preserve' || (space === null && inheritedPreserve);
      for (let c = el._first; c; c = c._next) {
        if (c.nodeType === 3) segs.push({ text: c._data, cs, el, preserve });
        else if (c.nodeType === 1 && c.namespaceURI === SVG_NS && (c.localName === 'tspan' || c.localName === 'a' || c.localName === 'textPath')) walk(c, preserve);
      }
    };
    walk(textEl, false);
    // White space (SVG 1.1 10.15, as browsers apply it).
    let prevSpace = true;
    let lastText = null;
    for (const s of segs) {
      if (s.text === undefined) continue;
      let t;
      if (s.preserve) t = s.text.replace(/[\n\r\t]/g, ' ');
      else {
        t = /[\n\r\t]| {2}/.test(s.text) ? s.text.replace(/[\n\r]/g, '').replace(/\t/g, ' ').replace(/ +/g, ' ') : s.text;
        if (prevSpace && t.startsWith(' ')) t = t.slice(1);
      }
      s.text = t;
      if (t.length) {
        prevSpace = t.endsWith(' ');
        lastText = s;
      }
    }
    if (lastText && !lastText.preserve) lastText.text = lastText.text.replace(/ +$/, '');
    const chunks = [];
    let chunk = null;
    let x = 0;
    let y = 0;
    for (const s of segs) {
      if (s.pos) {
        if (s.x !== null || s.y !== null || !chunk) {
          if (s.x !== null) x = s.x;
          if (s.y !== null) y = s.y;
          chunk = { start: x, anchor: s.cs['text-anchor'], runs: [] };
          chunks.push(chunk);
        }
        x += s.dx;
        y += s.dy;
        continue;
      }
      if (!s.text) continue;
      const wdt = D.text.runWidth(doc, s.cs, s.text);
      chunk.runs.push({ text: s.text, x, y, w: wdt, cs: s.cs, el: s.el });
      x += wdt;
      chunk.end = x;
    }
    for (const c of chunks) {
      if (!c.runs.length) continue;
      const width = c.end - c.start;
      const shift = c.anchor === 'middle' ? -width / 2 : c.anchor === 'end' ? -width : 0;
      if (shift) for (const r of c.runs) r.x += shift;
    }
    return chunks;
  }

  function paintText(el, w, doc, vp) {
    const chunks = layoutText(el, doc, vp);
    for (const c of chunks) {
      for (const r of c.runs) {
        const cs = r.cs;
        if (cs.visibility !== 'visible') continue;
        const fill = cs.fill;
        let fc = [0, 0, 0, 0];
        if (fill !== 'none') {
          if (fill.url !== undefined) {
            use(doc, el, fill.url);
            const server = byId(doc, fill.url);
            const stops = server ? gradientStops(doc, server, el) : [];
            if (stops.length) fc = stops[0].slice(1);
            warnOnce('text-gradient', 'text filled with a gradient is drawn in its first stop colour');
          } else fc = colorOf(fill, cs);
        }
        let sw = 0;
        let sc = [0, 0, 0, 0];
        if (cs.stroke !== 'none' && cs.stroke.url === undefined) {
          sc = colorOf(cs.stroke, cs);
          sw = css.toPx(cs['stroke-width'], diag(vp), cs['font-size'], null);
          sc = [sc[0], sc[1], sc[2], sc[3] * cs['stroke-opacity']];
        }
        const fa = fc[3] * cs['fill-opacity'];
        if (fa <= 0 && (sw <= 0 || sc[3] <= 0)) continue;
        if (cs['paint-order'].startsWith('stroke') === false && sw > 0) warnOnce('text-paint-order', 'stroked SVG text is drawn stroke under fill');
        emitText(w, doc, cs, r.text, r.x, r.y, 0, baselineOf(cs), [fc[0], fc[1], fc[2], fa], sw, sc);
      }
    }
  }

  // TEXT ops for a run; letter-spacing places each character.
  function emitText(w, doc, cs, text, x, y, align, baseline, rgba, strokeWidth, stroke) {
    const family = doc._str(D.text.familyOf(cs));
    const size = cs['font-size'];
    const weight = Math.min(900, Math.max(100, Math.round(cs['font-weight'] / 100) * 100));
    const italic = cs['font-style'] === 'italic' || cs['font-style'] === 'oblique' ? 1 : 0;
    const one = (t, tx) => w.nums(OP.TEXT, [doc._str(t), family, size, weight, italic, tx, y, align, baseline, rgba[0], rgba[1], rgba[2], rgba[3], strokeWidth, stroke[0], stroke[1], stroke[2], stroke[3]]);
    const ls = cs['letter-spacing'];
    if (!ls) {
      one(text, x);
      return;
    }
    let px = x;
    for (const ch of text) {
      one(ch, px);
      px += D.text.measure(doc.__screen, D.text.familyOf(cs), size, ch) + ls;
    }
  }

  // ---------------------------------------------------------------- SVG elements

  function paintSvg(el, w, doc, vp) {
    const cs = el._cs;
    const name = el.localName;
    const isShape = SHAPES.has(name);
    const isGroup = name === 'g' || name === 'a' || name === 'switch' || name === 'svg';
    if (!isShape && !isGroup && name !== 'text' && name !== 'image') {
      if (name === 'use' || name === 'foreignObject') warnOnce(`svg:${name}`, `<${name}> is not drawn`);
      return;
    }
    if (cs.opacity <= 0) return;
    let saved = false;
    const save = () => {
      if (!saved) {
        w.op(OP.SAVE);
        saved = true;
      }
    };
    let childVp = vp;
    if (name === 'svg') {
      const fs = cs['font-size'];
      const x = attrLen(el, 'x', vp.w, fs);
      const y = attrLen(el, 'y', vp.h, fs);
      const width = attrLen(el, 'width', vp.w, fs, vp.w);
      const height = attrLen(el, 'height', vp.h, fs, vp.h);
      if (width <= 0 || height <= 0) return;
      save();
      if (cs['overflow-x'] !== 'visible' && cs['overflow-x'] !== 'auto') w.nums(OP.CLIP_RECT, [x, y, width, height]);
      if (x || y) w.transform([1, 0, 0, 1, x, y]);
      const vb = D.viewBoxOf(el);
      if (vb) {
        const m = viewBoxMatrix(vb, width, height, el.getAttribute('preserveAspectRatio'));
        if (!geom.isIdentity(m)) w.transform(m);
        childVp = { w: vb[2], h: vb[3] };
      } else childVp = { w: width, h: height };
    } else {
      const m = svgTransform(el, cs, vp);
      if (m) {
        save();
        w.transform(m);
      }
    }
    if (cs.opacity < 1) {
      save();
      w.nums(OP.GLOBAL_ALPHA, [cs.opacity]);
    }
    const clip = cs['clip-path'];
    if (clip !== 'none' && clip.url !== undefined) {
      const id = clip.url.startsWith('#') ? clip.url.slice(1) : clip.url;
      save();
      clipWith(w, doc, el, id, () => localBBox(el, doc, vp), vp);
    }
    if (isShape) paintShape(el, w, doc, cs, vp);
    else if (name === 'text') paintText(el, w, doc, vp);
    else if (name === 'image') paintSvgImage(el, w, doc, cs, vp);
    else for (let c = el._first; c; c = c._next) if (c.nodeType === 1 && c.namespaceURI === SVG_NS) emit(c, w, doc, childVp);
    if (saved) w.op(OP.RESTORE);
  }

  function paintSvgImage(el, w, doc, cs, vp) {
    if (cs.visibility !== 'visible') return;
    const href = hrefOf(el);
    if (!href) return;
    const fs = cs['font-size'];
    const [nw, nh] = D.imageSize(href);
    let width = attrLen(el, 'width', vp.w, fs, -1);
    let height = attrLen(el, 'height', vp.h, fs, -1);
    if (width < 0) width = nw;
    if (height < 0) height = nh;
    if (!(nw > 0 && nh > 0)) {
      warnOnce(`image-size:${href}`, `the size of image ${href} is unknown (__host.imageSize): not drawn`);
      return;
    }
    const x = attrLen(el, 'x', vp.w, fs);
    const y = attrLen(el, 'y', vp.h, fs);
    const m = viewBoxMatrix([0, 0, nw, nh], width, height, el.getAttribute('preserveAspectRatio'));
    w.op(OP.SAVE);
    w.nums(OP.CLIP_RECT, [x, y, width, height]);
    w.nums(OP.IMAGE, [doc._str(href), 0, 0, nw, nh, x + m[4], y + m[5], nw * m[0], nh * m[3]]);
    w.op(OP.RESTORE);
  }

  // ---------------------------------------------------------------- CSS boxes

  const radiusOf = (cs, prop, w, h) => {
    const v = cs[prop];
    if (typeof v === 'string') return 0;
    return css.toPx(v, Math.min(w, h), cs['font-size'], null);
  };

  function boxPath(w, x, y, width, height, cs) {
    const r = radiusOf(cs, 'border-top-left-radius', width, height);
    const uniform = r === radiusOf(cs, 'border-top-right-radius', width, height) && r === radiusOf(cs, 'border-bottom-right-radius', width, height) && r === radiusOf(cs, 'border-bottom-left-radius', width, height);
    if (!uniform) warnOnce('border-radius-mixed', 'different border radii per corner are drawn with the top-left radius');
    w.op(OP.BEGIN_PATH);
    w.copy(geom.rectOps(x, y, width, height, r, r).ops);
  }

  function paintBackground(el, w, doc, cs, box) {
    const bg = cs['background-color'];
    if (bg[3] > 0) {
      boxPath(w, 0, 0, box.w, box.h, cs);
      w.nums(OP.FILL, [bg[0], bg[1], bg[2], bg[3], 0]);
    }
    const image = cs['background-image'];
    if (image !== 'none' && image.url) {
      const url = image.url;
      const [nw, nh] = D.imageSize(url);
      if (!(nw > 0 && nh > 0)) {
        warnOnce(`bg-size:${url}`, `the size of background image ${url.slice(0, 60)} is unknown (__host.imageSize): not drawn`);
      } else {
        const size = cs['background-size'];
        let dw = nw;
        let dh = nh;
        if (size === 'cover' || size === 'contain') {
          const s = size === 'cover' ? Math.max(box.w / nw, box.h / nh) : Math.min(box.w / nw, box.h / nh);
          dw = nw * s;
          dh = nh * s;
        } else if (size !== 'auto') {
          const t = css.splitTokens(size);
          const a = css.parseLength(t[0]);
          const b = t[1] && t[1] !== 'auto' ? css.parseLength(t[1]) : null;
          if (a) dw = css.toPx(a, box.w, cs['font-size'], doc);
          dh = b ? css.toPx(b, box.h, cs['font-size'], doc) : (dw * nh) / nw;
        }
        // background-position defaults to 0% 0%; cover and contain are
        // centred by FlyByWire's CSS only through that default, so none is applied.
        w.op(OP.SAVE);
        w.nums(OP.CLIP_RECT, [0, 0, box.w, box.h]);
        w.nums(OP.IMAGE, [doc._str(url), 0, 0, nw, nh, 0, 0, dw, dh]);
        w.op(OP.RESTORE);
        if (cs['background-repeat'] !== 'initial' && !/no-repeat/.test(cs['background-repeat']) && (dw < box.w || dh < box.h)) {
          warnOnce('background-repeat', 'repeated background images are drawn once');
        }
      }
    }
  }

  function paintBorders(w, cs, box) {
    const sides = [
      ['top', box.bt, [0, 0, box.w, box.bt]],
      ['right', box.br, [box.w - box.br, 0, box.br, box.h]],
      ['bottom', box.bb, [0, box.h - box.bb, box.w, box.bb]],
      ['left', box.bl, [0, 0, box.bl, box.h]],
    ];
    const r = radiusOf(cs, 'border-top-left-radius', box.w, box.h);
    const color0 = colorOf(cs['border-top-color'], cs);
    const uniform = box.bt === box.br && box.bt === box.bb && box.bt === box.bl && sides.every(([s]) => cs[`border-${s}-style`] === cs['border-top-style']) && sides.every(([s]) => String(colorOf(cs[`border-${s}-color`], cs)) === String(color0));
    if (r > 0 && uniform && box.bt > 0) {
      const t = box.bt;
      w.op(OP.BEGIN_PATH);
      w.copy(geom.rectOps(t / 2, t / 2, box.w - t, box.h - t, Math.max(0, r - t / 2), Math.max(0, r - t / 2)).ops);
      w.nums(OP.STROKE, [color0[0], color0[1], color0[2], color0[3], t, 0, 0, 4, 0, 0]);
      return;
    }
    for (const [side, width, rect] of sides) {
      if (!(width > 0)) continue;
      const style = cs[`border-${side}-style`];
      if (style !== 'solid') warnOnce(`border-style:${style}`, `border-style ${style} is drawn solid`);
      const c = colorOf(cs[`border-${side}-color`], cs);
      if (c[3] <= 0) continue;
      w.op(OP.BEGIN_PATH);
      w.nums(OP.RECT, rect);
      w.nums(OP.FILL, [c[0], c[1], c[2], c[3], 0]);
    }
  }

  function paintOutline(w, cs, box) {
    const style = cs['outline-style'];
    const width = cs['outline-width'];
    if (style === 'none' || !(width > 0)) return;
    const c = colorOf(cs['outline-color'], cs);
    const off = css.toPx(cs['outline-offset'], 0, cs['font-size'], null);
    const o = off + width / 2;
    w.op(OP.BEGIN_PATH);
    w.nums(OP.RECT, [-o, -o, box.w + 2 * o, box.h + 2 * o]);
    w.nums(OP.STROKE, [c[0], c[1], c[2], c[3], width, 0, 0, 4, 0, 0]);
  }

  function paintFragments(el, w, doc) {
    const frags = el._frags;
    // Inline backgrounds under the text.
    for (const f of frags) {
      for (let o = f.owner; o && o !== el && o.nodeType === 1; o = o.parentNode) {
        const bg = o._cs && o._cs['background-color'];
        if (bg && bg[3] > 0 && o._cs.visibility === 'visible') {
          w.op(OP.BEGIN_PATH);
          w.nums(OP.RECT, [f.x, f.y - f.a, f.w, f.a + f.d]);
          w.nums(OP.FILL, [bg[0], bg[1], bg[2], bg[3], 0]);
          break;
        }
      }
    }
    for (const f of frags) {
      const cs = f.cs;
      if (cs.visibility !== 'visible') continue;
      const c = cs.color;
      if (c[3] <= 0) continue;
      emitText(w, doc, cs, f.text, f.x, f.y, 0, 0, c, 0, [0, 0, 0, 0]);
      for (let o = f.owner; o && o.nodeType === 1; o = o.parentNode) {
        const ocs = o._cs;
        if (ocs && ocs['text-decoration-line'] === 'underline') {
          // Without the font's underline metrics: one fifteenth of the size,
          // below the baseline by the same.
          const t = Math.max(1, cs['font-size'] / 15);
          w.op(OP.BEGIN_PATH);
          w.nums(OP.RECT, [f.x, f.y + t, f.w, t]);
          w.nums(OP.FILL, [c[0], c[1], c[2], c[3], 0]);
          warnOnce('underline', 'underlines are placed without the font\'s underline metrics');
          break;
        }
        if (o === el) break;
      }
    }
  }

  // Children in painting order: negative z-index, in flow, positioned, positive.
  function paintOrder(el) {
    const kids = [];
    let sort = false;
    for (let c = el._first; c; c = c._next) {
      if (c.nodeType !== 1) continue;
      const cs = c._cs;
      let layer = 0;
      let z = 0;
      if (cs && cs.position !== 'static') {
        z = cs['z-index'] === 'auto' ? 0 : cs['z-index'];
        layer = z < 0 ? -1 : z > 0 ? 2 : 1;
        sort = true;
      }
      kids.push({ c, layer, z, i: kids.length });
    }
    if (sort) kids.sort((a, b) => a.layer - b.layer || a.z - b.z || a.i - b.i);
    return kids.map((k) => k.c);
  }

  // The matrix from a box's own coordinates to its parent's.
  function boxSelfMatrix(el) {
    const box = el._box;
    const cs = el._cs;
    let m = box.x || box.y ? [1, 0, 0, 1, box.x, box.y] : null;
    if (cs.transform !== 'none') {
      const t = css.transformMatrix(cs.transform, box.w, box.h, cs['font-size'], el.ownerDocument);
      const [ox, oy] = originOf(cs, box.w, box.h, false);
      m = geom.multiply(m ?? geom.IDENTITY, around(t, ox, oy));
    }
    return m;
  }

  // From a box's children's coordinates to its own.
  function boxChildMatrix(el) {
    const box = el._box;
    if (isOuterSvg(el)) {
      const cw = box.w - box.bl - box.br - box.pl - box.pr;
      const ch = box.h - box.bt - box.bb - box.pt - box.pb;
      let m = [1, 0, 0, 1, box.bl + box.pl, box.bt + box.pt];
      const vb = D.viewBoxOf(el);
      if (vb && cw > 0 && ch > 0) m = geom.multiply(m, viewBoxMatrix(vb, cw, ch, el.getAttribute('preserveAspectRatio')));
      return m;
    }
    const st = el._scrollTop ?? 0;
    const sl = el._scrollLeft ?? 0;
    return st || sl ? [1, 0, 0, 1, -sl, -st] : null;
  }

  const clips = (cs) => cs['overflow-x'] !== 'visible' || cs['overflow-y'] !== 'visible';

  function paintBox(el, w, doc) {
    const box = el._box;
    const cs = el._cs;
    if (!box || el._boxPass !== doc._layoutPass || cs.opacity <= 0) return;
    let saved = false;
    const save = () => {
      if (!saved) {
        w.op(OP.SAVE);
        saved = true;
      }
    };
    const self = boxSelfMatrix(el);
    if (self) {
      save();
      w.transform(self);
    }
    if (cs.opacity < 1) {
      save();
      w.nums(OP.GLOBAL_ALPHA, [cs.opacity]);
    }
    const visible = cs.visibility === 'visible';
    if (visible) {
      paintBackground(el, w, doc, cs, box);
      paintBorders(w, cs, box);
    }
    if (clips(cs)) {
      save();
      w.nums(OP.CLIP_RECT, [box.bl, box.bt, box.w - box.bl - box.br, box.h - box.bt - box.bb]);
    }
    let childM = boxChildMatrix(el);
    if (childM && geom.isIdentity(childM)) childM = null;
    if (el._frags && el._frags.length) paintFragments(el, w, doc);
    if (childM) {
      save();
      w.transform(childM);
    }
    if (isOuterSvg(el)) {
      const cw = box.w - box.bl - box.br - box.pl - box.pr;
      const ch = box.h - box.bt - box.bb - box.pt - box.pb;
      const vb = D.viewBoxOf(el);
      const vp = vb ? { w: vb[2], h: vb[3] } : { w: cw, h: ch };
      for (let c = el._first; c; c = c._next) if (c.nodeType === 1 && c.namespaceURI === SVG_NS) emit(c, w, doc, vp);
    } else {
      if (el.localName === 'canvas' && visible && el._context) {
        el._context._paint(w, doc, box.bl + box.pl, box.bt + box.pt, box.w - box.bl - box.br - box.pl - box.pr, box.h - box.bt - box.bb - box.pt - box.pb);
      } else if (el.localName === 'img' && visible) {
        const src = el.getAttribute('src');
        const [nw, nh] = D.imageSize(src ?? '');
        if (src && nw > 0 && nh > 0) {
          w.nums(OP.IMAGE, [doc._str(src), 0, 0, nw, nh, box.bl + box.pl, box.bt + box.pt, box.w - box.bl - box.br - box.pl - box.pr, box.h - box.bt - box.bb - box.pt - box.pb]);
        } else if (src) warnOnce(`img-size:${src}`, `the size of image ${src} is unknown (__host.imageSize): not drawn`);
      }
      for (const c of paintOrder(el)) if (c.namespaceURI === HTML_NS || isOuterSvg(c)) emit(c, w, doc, null);
    }
    if (visible) paintOutline(w, cs, box);
    if (saved) w.op(OP.RESTORE);
  }

  // ---------------------------------------------------------------- chunks

  // An element rebuilt this many frames running keeps no chunk: copying out
  // a chunk that is thrown away next frame costs more than it saves.
  const HOT_FRAMES = 2;

  function emit(el, w, doc, vp) {
    const chunk = el._chunk;
    if (chunk) {
      w.copy(chunk);
      return;
    }
    const start = w.n;
    const cs = el._cs;
    if (cs && cs.display !== 'none') {
      if (el.namespaceURI === SVG_NS && !isOuterSvg(el)) paintSvg(el, w, doc, vp);
      else paintBox(el, w, doc);
    }
    const frame = doc._paintFrame;
    el._hot = el._rebuilt === frame - 1 ? (el._hot | 0) + 1 : 0;
    el._rebuilt = frame;
    el._chunk = el._hot >= HOT_FRAMES ? null : w.buf.slice(start, w.n);
  }

  function paintDocument(doc) {
    if (!doc._writer) doc._writer = new Writer();
    const w = doc._writer;
    w.n = 0;
    doc._paintFrame = (doc._paintFrame | 0) + 1;
    if (doc._strings.length > MAX_STRINGS) {
      doc._strings = [];
      doc._stringIndex = new Map();
      doc._markPaintDeep(doc.documentElement);
    }
    const root = doc.documentElement;
    if (root) emit(root, w, doc, null);
    return w.buf.slice(0, w.n);
  }

  // ---------------------------------------------------------------- geometry

  // The nearest SVG viewport size for `el`'s lengths.
  function viewportOf(el) {
    for (let p = el.parentNode; p && p.nodeType === 1; p = p.parentNode) {
      if (p.namespaceURI === SVG_NS && p.localName === 'svg') {
        const vb = D.viewBoxOf(p);
        if (vb) return { w: vb[2], h: vb[3] };
        if (isOuterSvg(p) && p._box) return { w: p._box.w - p._box.bl - p._box.br - p._box.pl - p._box.pr, h: p._box.h - p._box.bt - p._box.bb - p._box.pt - p._box.pb };
        const outer = viewportOf(p);
        return { w: attrLen(p, 'width', outer.w, 16, outer.w), h: attrLen(p, 'height', outer.h, 16, outer.h) };
      }
    }
    return { w: 0, h: 0 };
  }

  const union = (a, b) => (a ? [Math.min(a[0], b[0]), Math.min(a[1], b[1]), Math.max(a[2], b[2]), Math.max(a[3], b[3])] : b);

  // Bounding box [x0, y0, x1, y1] of an SVG element in its own user space,
  // or null when it has no geometry.
  function localBBox(el, doc, vp) {
    const cs = el._cs;
    if (!cs || cs.display === 'none') return null;
    const name = el.localName;
    if (SHAPES.has(name)) {
      const g = shapeGeometry(el, vp);
      return g.ops.length ? g.bbox : null;
    }
    if (name === 'text' || name === 'tspan') {
      let textEl = el;
      while (textEl && textEl.localName !== 'text') textEl = textEl.parentNode;
      if (!textEl || textEl.nodeType !== 1) return null;
      let box = null;
      for (const c of layoutText(textEl, doc, vp)) {
        for (const r of c.runs) {
          if (name === 'tspan' && !el.contains(r.el)) continue;
          const [a, d] = D.text.metrics(doc.__screen, D.text.familyOf(r.cs), r.cs['font-size']);
          const b = baselineOf(r.cs);
          const top = b === 0 ? r.y - a : b === 1 ? r.y - (a + d) / 2 : b === 2 || b === 4 ? r.y : r.y - a - d;
          box = union(box, [r.x, top, r.x + r.w, top + a + d]);
        }
      }
      return box;
    }
    if (name === 'image') {
      const fs = cs['font-size'];
      const x = attrLen(el, 'x', vp.w, fs);
      const y = attrLen(el, 'y', vp.h, fs);
      return [x, y, x + attrLen(el, 'width', vp.w, fs), y + attrLen(el, 'height', vp.h, fs)];
    }
    if (name === 'g' || name === 'svg' || name === 'a' || name === 'switch') {
      let box = null;
      const childVp = name === 'svg' ? svgChildViewport(el, vp) : vp;
      for (let c = el._first; c; c = c._next) {
        if (c.nodeType !== 1 || c.namespaceURI !== SVG_NS) continue;
        const b = localBBox(c, doc, childVp);
        if (!b) continue;
        const m = c._cs ? svgTransform(c, c._cs, childVp) : null;
        box = union(box, m ? geom.transformBox(m, b) : b);
      }
      return box;
    }
    return null;
  }

  function svgChildViewport(el, vp) {
    const vb = D.viewBoxOf(el);
    if (vb) return { w: vb[2], h: vb[3] };
    return { w: attrLen(el, 'width', vp.w, 16, vp.w), h: attrLen(el, 'height', vp.h, 16, vp.h) };
  }

  // From an element's own coordinates to the screen.
  function screenMatrix(el) {
    const chain = [];
    for (let n = el; n && n.nodeType === 1; n = n.parentNode) chain.push(n);
    let m = geom.IDENTITY;
    for (let i = chain.length - 1; i >= 0; i--) {
      const n = chain[i];
      const self = matrixOf(n);
      if (self) m = geom.multiply(m, self);
      if (i > 0) {
        const child = childMatrixOf(n);
        if (child) m = geom.multiply(m, child);
      }
    }
    return m;
  }
  function matrixOf(n) {
    if (n.namespaceURI === SVG_NS && !isOuterSvg(n)) {
      if (!n._cs) return null;
      if (n.localName === 'svg') {
        const vp = viewportOf(n);
        const x = attrLen(n, 'x', vp.w, 16);
        const y = attrLen(n, 'y', vp.h, 16);
        return x || y ? [1, 0, 0, 1, x, y] : null;
      }
      return svgTransform(n, n._cs, viewportOf(n));
    }
    return n._box && n._cs ? boxSelfMatrix(n) : null;
  }
  function childMatrixOf(n) {
    if (n.namespaceURI === SVG_NS && !isOuterSvg(n)) {
      if (n.localName !== 'svg') return null;
      const vb = D.viewBoxOf(n);
      if (!vb) return null;
      const vp = viewportOf(n);
      return viewBoxMatrix(vb, attrLen(n, 'width', vp.w, 16, vp.w), attrLen(n, 'height', vp.h, 16, vp.h), n.getAttribute('preserveAspectRatio'));
    }
    return n._box ? boxChildMatrix(n) : null;
  }

  function clientRect(doc, el) {
    const m = screenMatrix(el);
    let b = null;
    if (el.namespaceURI === SVG_NS && !isOuterSvg(el)) b = localBBox(el, doc, viewportOf(el));
    else if (el._box && el._boxPass === doc._layoutPass) b = [0, 0, el._box.w, el._box.h];
    else if (el._inlineRect && el._inlineRect.pass === doc._layoutPass) {
      const r = el._inlineRect;
      b = [r.x0, r.y0, r.x1, r.y1];
    }
    if (!b) return new D.DOMRect(0, 0, 0, 0);
    const t = geom.transformBox(m, b);
    return new D.DOMRect(t[0], t[1], t[2] - t[0], t[3] - t[1]);
  }

  function bbox(doc, el) {
    const b = localBBox(el, doc, viewportOf(el));
    return b ? new D.DOMRect(b[0], b[1], b[2] - b[0], b[3] - b[1]) : new D.DOMRect(0, 0, 0, 0);
  }

  function textLength(doc, el) {
    let textEl = el;
    while (textEl && textEl.nodeType === 1 && textEl.localName !== 'text') textEl = textEl.parentNode;
    if (!textEl || textEl.nodeType !== 1 || !textEl._cs) return 0;
    let sum = 0;
    for (const c of layoutText(textEl, doc, viewportOf(textEl))) for (const r of c.runs) if (el === textEl || el.contains(r.el)) sum += r.w;
    return sum;
  }

  // ---------------------------------------------------------------- hit testing

  const receives = (cs) => cs.visibility === 'visible' && cs['pointer-events'] !== 'none';

  // The topmost element under (x, y) in screen coordinates, or null.
  function hitTest(doc, x, y) {
    doc._flush();
    let hit = null;
    const inside = (m, b) => {
      const inv = geom.invert(m);
      if (!inv) return false;
      const [lx, ly] = geom.apply(inv, x, y);
      return lx >= b[0] && lx <= b[2] && ly >= b[1] && ly <= b[3];
    };
    const visitSvg = (el, m, vp) => {
      const cs = el._cs;
      if (!cs || cs.display === 'none') return;
      const t = svgTransform(el, cs, vp);
      let me = t ? geom.multiply(m, t) : m;
      const name = el.localName;
      if (SHAPES.has(name) || name === 'text' || name === 'image') {
        if (receives(cs)) {
          const b = localBBox(el, doc, vp);
          if (b && inside(me, b)) {
            hit = el;
            if (name === 'text') {
              // A tspan under the point is the target.
              for (let c = el._first; c; c = c._next) {
                if (c.nodeType === 1 && c.localName === 'tspan' && c._cs && receives(c._cs)) {
                  const cb = localBBox(c, doc, vp);
                  if (cb && inside(me, cb)) hit = c;
                }
              }
            }
          }
        }
        return;
      }
      let childVp = vp;
      if (name === 'svg') {
        const fs = cs['font-size'];
        const sx = attrLen(el, 'x', vp.w, fs);
        const sy = attrLen(el, 'y', vp.h, fs);
        const width = attrLen(el, 'width', vp.w, fs, vp.w);
        const height = attrLen(el, 'height', vp.h, fs, vp.h);
        me = geom.multiply(m, [1, 0, 0, 1, sx, sy]);
        const vb = D.viewBoxOf(el);
        if (vb) {
          me = geom.multiply(me, viewBoxMatrix(vb, width, height, el.getAttribute('preserveAspectRatio')));
          childVp = { w: vb[2], h: vb[3] };
        } else childVp = { w: width, h: height };
      } else if (name !== 'g' && name !== 'a' && name !== 'switch') return;
      for (let c = el._first; c; c = c._next) if (c.nodeType === 1 && c.namespaceURI === SVG_NS) visitSvg(c, me, childVp);
    };
    const visitBox = (el, m) => {
      const cs = el._cs;
      const box = el._box;
      if (!cs || cs.display === 'none') return;
      if (!box || el._boxPass !== doc._layoutPass) {
        // An inline element: its text's union, in its parent's coordinates.
        const r = el._inlineRect;
        if (r && r.pass === doc._layoutPass && receives(cs) && inside(m, [r.x0, r.y0, r.x1, r.y1])) hit = el;
        for (const c of paintOrder(el)) if (c.namespaceURI === HTML_NS || isOuterSvg(c)) visitBox(c, m);
        return;
      }
      const self = boxSelfMatrix(el);
      const me = self ? geom.multiply(m, self) : m;
      const within = inside(me, [0, 0, box.w, box.h]);
      if (within && receives(cs)) hit = el;
      if (clips(cs) && !within) return;
      if (el._frags) {
        for (const f of el._frags) {
          const owner = f.owner;
          if (owner && owner !== el && owner._cs && receives(owner._cs) && inside(me, [f.x, f.y - f.a, f.x + f.w, f.y + f.d])) hit = owner;
        }
      }
      const cm = boxChildMatrix(el);
      const mc = cm ? geom.multiply(me, cm) : me;
      if (isOuterSvg(el)) {
        const vb = D.viewBoxOf(el);
        const vp = vb ? { w: vb[2], h: vb[3] } : { w: box.w, h: box.h };
        for (let c = el._first; c; c = c._next) if (c.nodeType === 1 && c.namespaceURI === SVG_NS) visitSvg(c, mc, vp);
        return;
      }
      for (const c of paintOrder(el)) if (c.namespaceURI === HTML_NS || isOuterSvg(c)) visitBox(c, mc);
    };
    const root = doc.documentElement;
    if (root) visitBox(root, geom.IDENTITY);
    return hit;
  }

  D.paint = { document: paintDocument, clientRect, bbox, textLength, hitTest, layoutText, viewBoxMatrix, emitText, strokeOp };
})();
