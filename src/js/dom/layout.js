// Text measurement (through __host) and HTML layout: block flow, inline
// text with wrapping, absolute/relative/fixed positioning, flexbox, grid,
// replaced elements (outer <svg>, <canvas>, <img>) and ::before/::after
// text. Only what the instruments' CSS uses (docs/dom-survey.md).
//
// Every CSS box gets `_box`: its border box (x, y relative to the parent
// element's border box, w, h), its edges, and for blocks that lay out
// inline content, `_frags` (text runs in its own coordinates). Inline
// elements have no box of their own: they sit at 0,0 in their parent's
// coordinates and record the union of their text in `_inlineRect`.
(() => {
  const D = globalThis.__dom;
  const { css, warnOnce } = D;
  const SVG_NS = D.NS.SVG;

  // ---------------------------------------------------------------- text

  const familyNames = new WeakMap();
  const familyOf = (cs) => {
    const list = cs['font-family'];
    if (typeof list === 'string') return list;
    let s = familyNames.get(list);
    if (s === undefined) {
      s = list.join(', ');
      familyNames.set(list, s);
    }
    return s;
  };

  const widths = new Map();
  const metricsCache = new Map();
  const hostFn = (name) => {
    const host = globalThis.__host;
    return host && typeof host[name] === 'function' ? host[name] : null;
  };

  // Width of `text` in CSS px (before letter-spacing) on `screen`.
  function measure(screen, family, size, text) {
    if (text === '' || !(size > 0)) return 0;
    const key = `${screen}\u0000${family}\u0000${size}\u0000${text}`;
    let w = widths.get(key);
    if (w === undefined) {
      const fn = hostFn('measureText');
      if (fn) w = Number(fn(family, size, text, screen)) || 0;
      else {
        warnOnce('measureText', 'there is no __host.measureText: text measures 0');
        w = 0;
      }
      if (widths.size > 50000) widths.clear();
      widths.set(key, w);
    }
    return w;
  }

  // [ascent, descent] in CSS px, both positive.
  function metrics(screen, family, size) {
    const key = `${screen}\u0000${family}\u0000${size}`;
    let m = metricsCache.get(key);
    if (m === undefined) {
      const fn = hostFn('fontMetrics');
      m = [0, 0];
      if (fn) {
        const r = fn(family, size, screen);
        if (r && r.length >= 2) m = [Number(r[0]) || 0, Number(r[1]) || 0];
      } else {
        warnOnce('fontMetrics', 'there is no __host.fontMetrics: text has no height');
      }
      if (metricsCache.size > 4096) metricsCache.clear();
      metricsCache.set(key, m);
    }
    return m;
  }

  const transformText = (cs, text) => {
    switch (cs['text-transform']) {
      case 'uppercase':
        return text.toUpperCase();
      case 'lowercase':
        return text.toLowerCase();
      case 'capitalize':
        return text.replace(/(^|\s)(\S)/g, (_, a, b) => a + b.toUpperCase());
      default:
        return text;
    }
  };

  // A run's width with letter-spacing, which follows every character.
  const runWidth = (doc, cs, text) => measure(doc.__screen, familyOf(cs), cs['font-size'], text) + cs['letter-spacing'] * [...text].length;

  // The used line-height in px.
  const lineHeight = (cs, a, d, doc) => {
    const lh = cs['line-height'];
    if (lh === 'normal') return a + d;
    if (lh.mult !== undefined) return lh.mult * cs['font-size'];
    return css.toPx(lh, cs['font-size'], cs['font-size'], doc);
  };

  D.text = { familyOf, measure, metrics, runWidth, lineHeight, transformText };

  // ---------------------------------------------------------------- helpers

  let doc = null;
  let pass = 0;

  // A length value in px, or null for auto/none/normal and percentages of
  // an indefinite base.
  const px = (v, base, cs) => {
    if (v === undefined || v === null || typeof v === 'string') return null;
    if (base === null && css.isPercent(v)) return null;
    return css.toPx(v, base ?? 0, cs['font-size'], doc);
  };

  const borderWidth = (cs, side) => {
    const style = cs[`border-${side}-style`];
    return style === 'none' || style === 'hidden' ? 0 : cs[`border-${side}-width`];
  };

  function edges(cs, cbW) {
    return {
      ml: px(cs['margin-left'], cbW, cs),
      mr: px(cs['margin-right'], cbW, cs),
      mt: px(cs['margin-top'], cbW, cs),
      mb: px(cs['margin-bottom'], cbW, cs),
      pl: px(cs['padding-left'], cbW, cs) ?? 0,
      pr: px(cs['padding-right'], cbW, cs) ?? 0,
      pt: px(cs['padding-top'], cbW, cs) ?? 0,
      pb: px(cs['padding-bottom'], cbW, cs) ?? 0,
      bl: borderWidth(cs, 'left'),
      br: borderWidth(cs, 'right'),
      bt: borderWidth(cs, 'top'),
      bb: borderWidth(cs, 'bottom'),
    };
  }

  // A width/height property as a border-box size, or null.
  const boxSize = (cs, prop, base, hEdges) => {
    const v = px(cs[prop], base, cs);
    if (v === null) return null;
    return cs['box-sizing'] === 'border-box' ? Math.max(v, hEdges) : v + hEdges;
  };

  const clampSize = (cs, size, minProp, maxProp, base, edgesSum) => {
    const max = boxSize(cs, maxProp, base, edgesSum);
    if (max !== null && size > max) size = max;
    const min = boxSize(cs, minProp, base, edgesSum);
    if (min !== null && size < min) size = min;
    return Math.max(size, edgesSum);
  };

  const displayOf = (el) => (el._cs ? el._cs.display : 'none');
  const isAbs = (cs) => cs.position === 'absolute' || cs.position === 'fixed';
  const isReplaced = (el) => (el.namespaceURI === SVG_NS && el.localName === 'svg') || el.localName === 'canvas' || el.localName === 'img';
  const INLINE_LEVEL = new Set(['inline', 'inline-block', 'inline-flex', 'inline-grid', 'inline-table']);
  const isInlineLevel = (el) => INLINE_LEVEL.has(el._cs.display);
  const isAtomic = (el) => el._cs.display !== 'inline' || isReplaced(el) || el.localName === 'input' || el.localName === 'button';
  const isContainingBlock = (cs) => cs.position !== 'static' || cs.transform !== 'none';
  const isWhitespace = (node) => node.nodeType === 3 && !/[^ \t\n\r\f]/.test(node._data);

  // A pseudo-element's text (content: "..." / attr(x)).
  function pseudoText(el, pcs) {
    const content = pcs && pcs.content;
    if (!content || typeof content === 'string') return null;
    let out = '';
    for (const part of content) out += part.text !== undefined ? part.text : el.getAttribute(part.attr) ?? '';
    return out;
  }

  // Children in flow order, with ::before/::after as {pseudo} items.
  function flowChildren(el) {
    const out = [];
    const before = pseudoText(el, el._csBefore);
    if (before !== null && el._csBefore.display !== 'none') out.push({ pseudo: el._csBefore, text: before, owner: el });
    for (let c = el._first; c; c = c._next) if (c.nodeType === 1 || c.nodeType === 3) out.push(c);
    const after = pseudoText(el, el._csAfter);
    if (after !== null && el._csAfter.display !== 'none') out.push({ pseudo: el._csAfter, text: after, owner: el });
    return out;
  }
  const isElementItem = (n) => n.nodeType === 1;

  // ---------------------------------------------------------------- replaced

  function svgAttrLength(el, name, base) {
    const v = el.getAttribute(name);
    if (v === null) return null;
    const len = css.parseLength(v);
    if (!len) return null;
    if (css.isPercent(len)) return base === null ? null : css.toPx(len, base, 16, doc);
    return css.toPx(len, 0, 16, doc);
  }

  function viewBoxOf(el) {
    const v = el.getAttribute('viewBox');
    if (!v) return null;
    const n = v.trim().split(/[\s,]+/).map(Number);
    if (n.length !== 4 || n.some((x) => !Number.isFinite(x)) || n[2] <= 0 || n[3] <= 0) return null;
    return n;
  }
  D.viewBoxOf = viewBoxOf;

  // Intrinsic width, height and ratio of a replaced element.
  function intrinsicReplaced(el, cbW, cbH) {
    if (el.localName === 'canvas') return { w: el.width, h: el.height, ratio: el.height ? el.width / el.height : null };
    if (el.localName === 'img') {
      const [w, h] = D.imageSize(el.getAttribute('src') ?? '');
      const aw = svgAttrLength(el, 'width', cbW);
      const ah = svgAttrLength(el, 'height', cbH);
      return { w: aw ?? (w || null), h: ah ?? (h || null), ratio: w && h ? w / h : null };
    }
    const vb = viewBoxOf(el);
    return { w: svgAttrLength(el, 'width', cbW), h: svgAttrLength(el, 'height', cbH), ratio: vb ? vb[2] / vb[3] : null };
  }

  // Content size of a replaced element (CSS 2.2 10.3.2 and 10.6.2).
  function replacedSize(el, cs, cbW, cbH, specW, specH, fill) {
    const i = intrinsicReplaced(el, cbW, cbH);
    let w = specW;
    let h = specH;
    if (w === null && h === null) {
      if (i.w !== null && i.h !== null) {
        w = i.w;
        h = i.h;
      } else if (i.w !== null) {
        w = i.w;
        h = i.ratio ? w / i.ratio : 150;
      } else if (i.h !== null) {
        h = i.h;
        w = i.ratio ? h * i.ratio : 300;
      } else if (i.ratio) {
        w = fill ?? 300;
        h = w / i.ratio;
      } else {
        w = 300;
        h = 150;
      }
    } else if (w === null) {
      w = i.w !== null && i.ratio === null ? i.w : i.ratio ? h * i.ratio : i.w ?? 300;
    } else if (h === null) {
      h = i.h !== null && i.ratio === null ? i.h : i.ratio ? w / i.ratio : i.h ?? 150;
    }
    return { w, h };
  }

  // ---------------------------------------------------------------- intrinsic widths

  // {min, max} content contributions (border box, without margins).
  function intrinsic(el) {
    if (el._intrPass === pass) return el._intr;
    const cs = el._cs;
    const e = edges(cs, 0);
    const hE = e.pl + e.pr + e.bl + e.br;
    let r;
    const spec = css.isPercent(cs.width) ? null : boxSize(cs, 'width', 0, hE);
    if (spec !== null) r = { min: spec, max: spec };
    else if (isReplaced(el)) {
      const s = replacedSize(el, cs, null, null, null, css.isPercent(cs.height) ? null : px(cs.height, null, cs), null);
      r = { min: s.w + hE, max: s.w + hE };
    } else {
      const d = cs.display;
      let min = 0;
      let max = 0;
      if (d === 'flex' || d === 'inline-flex') {
        const row = cs['flex-direction'].startsWith('row');
        const gap = px(cs['column-gap'], 0, cs) ?? 0;
        const items = flexItems(el);
        items.forEach((item, i) => {
          const c = itemIntrinsic(item);
          if (row) {
            max += c.max + (i ? gap : 0);
            if (cs['flex-wrap'] === 'nowrap') min += c.min + (i ? gap : 0);
            else min = Math.max(min, c.min);
          } else {
            max = Math.max(max, c.max);
            min = Math.max(min, c.min);
          }
        });
      } else if (d === 'grid' || d === 'inline-grid') {
        const g = gridIntrinsic(el, cs);
        min = g.min;
        max = g.max;
      } else {
        let group = [];
        const flush = () => {
          if (!group.length) return;
          const w = inlineIntrinsic(el, group);
          min = Math.max(min, w.min);
          max = Math.max(max, w.max);
          group = [];
        };
        for (const child of flowChildren(el)) {
          if (!isElementItem(child)) group.push(child);
          else {
            const ccs = child._cs;
            if (!ccs || ccs.display === 'none' || isAbs(ccs)) continue;
            if (isInlineLevel(child)) group.push(child);
            else {
              flush();
              const c = intrinsic(child);
              const ce = edges(ccs, 0);
              const m = (ce.ml ?? 0) + (ce.mr ?? 0);
              min = Math.max(min, c.min + m);
              max = Math.max(max, c.max + m);
            }
          }
        }
        flush();
      }
      r = { min: min + hE, max: Math.max(min, max) + hE };
      const maxW = css.isPercent(cs['max-width']) ? null : boxSize(cs, 'max-width', 0, hE);
      if (maxW !== null) {
        r.min = Math.min(r.min, maxW);
        r.max = Math.min(r.max, maxW);
      }
      const minW = css.isPercent(cs['min-width']) ? null : boxSize(cs, 'min-width', 0, hE);
      if (minW !== null) {
        r.min = Math.max(r.min, minW);
        r.max = Math.max(r.max, minW);
      }
    }
    el._intr = r;
    el._intrPass = pass;
    return r;
  }

  const itemIntrinsic = (item) => {
    if (item.anon) return inlineIntrinsic(item.parent, item.anon);
    const c = intrinsic(item.el);
    const e = edges(item.el._cs, 0);
    const m = (e.ml ?? 0) + (e.mr ?? 0);
    return { min: c.min + m, max: c.max + m };
  };

  // Widths of inline content: the longest unbreakable piece, and the
  // longest line with only forced breaks.
  function inlineIntrinsic(container, nodes) {
    const items = collectItems(container, nodes);
    let min = 0;
    let max = 0;
    let line = 0;
    let word = 0;
    for (const it of items) {
      if (it.kind === 'text') {
        line += it.w;
        if (it.space && it.wrap) {
          min = Math.max(min, word);
          word = 0;
        } else word += it.w;
      } else if (it.kind === 'atomic') {
        const c = intrinsic(it.el);
        const e = edges(it.el._cs, 0);
        const w = c.max + (e.ml ?? 0) + (e.mr ?? 0);
        line += w;
        min = Math.max(min, word, c.min + (e.ml ?? 0) + (e.mr ?? 0));
        word = 0;
      } else if (it.kind === 'edge') {
        line += it.w;
        word += it.w;
      } else if (it.kind === 'break') {
        max = Math.max(max, line);
        min = Math.max(min, word);
        line = word = 0;
      }
    }
    return { min: Math.max(min, word), max: Math.max(max, line) };
  }

  // ---------------------------------------------------------------- inline items

  // Flatten inline content into measured pieces:
  //   {kind: 'text', text, w, cs, node, owner, space, wrap, collapsible}
  //   {kind: 'atomic', el} {kind: 'break'} {kind: 'edge', w, el, open}
  //   {kind: 'abs', el}
  function collectItems(container, nodes) {
    const items = [];
    let lastSpace = true;
    const text = (raw, cs, node, owner) => {
      const ws = cs['white-space'];
      const collapse = ws === 'normal' || ws === 'nowrap' || ws === 'pre-line';
      const keepBreaks = ws !== 'normal' && ws !== 'nowrap';
      const wrap = ws !== 'nowrap' && ws !== 'pre';
      let t = transformText(cs, raw);
      if (collapse) {
        t = keepBreaks ? t.replace(/[ \t\r\f]*\n[ \t\r\f]*/g, '\n').replace(/[ \t\r\f]+/g, ' ') : t.replace(/[ \t\n\r\f]+/g, ' ');
      } else {
        t = t.replace(/\r\n?/g, '\n').replace(/\t/g, '        ');
      }
      const re = /\n| +|[^ \n]+/g;
      let m;
      while ((m = re.exec(t))) {
        const s = m[0];
        if (s === '\n') {
          if (keepBreaks) {
            items.push({ kind: 'break' });
            lastSpace = true;
          }
          continue;
        }
        const space = s[0] === ' ';
        if (space && collapse && lastSpace) continue;
        const piece = space && collapse ? ' ' : s;
        items.push({ kind: 'text', text: piece, w: runWidth(doc, cs, piece), cs, node, owner, space, wrap, collapsible: space && collapse });
        lastSpace = space;
      }
    };
    const walk = (node) => {
      if (node.pseudo) {
        text(node.text, node.pseudo, null, node.owner);
        return;
      }
      if (node.nodeType === 3) {
        const parent = node.parentNode;
        if (parent && parent._cs) text(node._data, parent._cs, node, parent);
        return;
      }
      if (node.nodeType !== 1) return;
      const cs = node._cs;
      if (!cs || cs.display === 'none') return;
      if (isAbs(cs)) {
        items.push({ kind: 'abs', el: node });
        return;
      }
      if (node.localName === 'br') {
        items.push({ kind: 'break' });
        lastSpace = true;
        return;
      }
      if (isAtomic(node)) {
        items.push({ kind: 'atomic', el: node });
        lastSpace = false;
        return;
      }
      node._ifc = container;
      const e = edges(cs, 0);
      items.push({ kind: 'edge', w: (e.ml ?? 0) + e.bl + e.pl, el: node, open: true });
      for (const c of flowChildren(node)) walk(c);
      items.push({ kind: 'edge', w: (e.mr ?? 0) + e.br + e.pr, el: node, open: false });
    };
    for (const n of nodes) walk(n);
    return items;
  }

  // Lay out inline content in lines `width` wide, starting at (ox, oy) in
  // the container's coordinates. Returns {height, baseline}.
  function layoutInline(container, nodes, width, ox, oy, absList) {
    const items = collectItems(container, nodes);
    if (!items.some((it) => it.kind === 'atomic' || it.kind === 'break' || (it.kind === 'text' && !it.collapsible) || it.kind === 'abs')) {
      return { height: 0, baseline: null };
    }
    const ccs = container._cs;
    const align = ccs['text-align'];
    const strutM = metrics(doc.__screen, familyOf(ccs), ccs['font-size']);
    const strutLh = lineHeight(ccs, strutM[0], strutM[1], doc);
    const strutAbove = strutM[0] + (strutLh - strutM[0] - strutM[1]) / 2;
    const strutBelow = strutLh - strutAbove;
    const frags = container._frags;
    let y = 0;
    let firstBaseline = null;
    let line = [];
    let x = 0;
    const lineWidthLimit = width === null ? Infinity : width;
    const finish = (forced) => {
      // Collapsible spaces at the end of a line hang and take no room.
      while (line.length && line[line.length - 1].kind === 'text' && line[line.length - 1].collapsible) {
        x -= line[line.length - 1].w;
        line.pop();
      }
      while (line.length && line[0].kind === 'text' && line[0].collapsible) {
        x -= line[0].w;
        const lead = line.shift().w;
        for (const p of line) p.x -= lead;
      }
      if (!line.length && !forced) return;
      let above = strutAbove;
      let below = strutBelow;
      for (const p of line) {
        if (p.kind === 'text') {
          const [a, d] = metrics(doc.__screen, familyOf(p.cs), p.cs['font-size']);
          const lh = lineHeight(p.cs, a, d, doc);
          const half = (lh - a - d) / 2;
          p.a = a;
          p.d = d;
          above = Math.max(above, a + half);
          below = Math.max(below, d + half);
        } else if (p.kind === 'atomic') {
          const b = p.box;
          if (p.el._cs['vertical-align'] === 'middle') {
            const mid = (strutM[0] - strutM[1]) / 2;
            above = Math.max(above, (b.h + b.mt + b.mb) / 2 + mid);
            below = Math.max(below, (b.h + b.mt + b.mb) / 2 - mid);
          } else {
            above = Math.max(above, b.h + b.mt + b.mb);
          }
        }
      }
      const used = x;
      let offset = 0;
      if (width !== null) {
        const free = width - used;
        if (align === 'center' || align === '-webkit-center') offset = free / 2;
        else if (align === 'right' || align === 'end' || align === '-webkit-right') offset = free;
      }
      const baseline = oy + y + above;
      if (firstBaseline === null) firstBaseline = baseline;
      let run = null;
      for (const p of line) {
        if (p.kind === 'text') {
          if (run && run.node === p.node && run.cs === p.cs && Math.abs(run.x + run.w - (ox + offset + p.x)) < 1e-9) {
            run.text += p.text;
            run.w += p.w;
          } else {
            run = { text: p.text, x: ox + offset + p.x, y: baseline, w: p.w, a: p.a, d: p.d, cs: p.cs, node: p.node, owner: p.owner };
            frags.push(run);
          }
          for (let o = p.owner; o && o !== container && o.nodeType === 1; o = o.parentNode) {
            const r = o._inlineRect;
            const x0 = ox + offset + p.x;
            const y0 = baseline - p.a;
            if (!r || r.pass !== pass) o._inlineRect = { pass, x0, y0, x1: x0 + p.w, y1: baseline + p.d };
            else {
              r.x0 = Math.min(r.x0, x0);
              r.y0 = Math.min(r.y0, y0);
              r.x1 = Math.max(r.x1, x0 + p.w);
              r.y1 = Math.max(r.y1, baseline + p.d);
            }
          }
        } else {
          run = null;
          if (p.kind === 'atomic') {
            const b = p.box;
            const top = p.el._cs['vertical-align'] === 'middle' ? baseline - (strutM[0] - strutM[1]) / 2 - (b.h + b.mt + b.mb) / 2 : baseline - (b.h + b.mt + b.mb);
            place(p.el, ox + offset + p.x + b.ml, top + b.mt);
          } else if (p.kind === 'abs') {
            absList.push({ el: p.el, sx: ox + offset + p.x, sy: oy + y });
          }
        }
      }
      y += above + below;
      line = [];
      x = 0;
    };
    let canBreak = false;
    for (const it of items) {
      if (it.kind === 'break') {
        finish(true);
        canBreak = false;
        continue;
      }
      if (it.kind === 'abs') {
        line.push({ kind: 'abs', el: it.el, x });
        continue;
      }
      let w;
      let piece;
      if (it.kind === 'atomic') {
        const box = layoutElement(it.el, { cbW: width, cbH: null, shrink: true, availW: width === null ? Infinity : width, absList });
        w = box.w + box.ml + box.mr;
        piece = { kind: 'atomic', el: it.el, box, x };
      } else if (it.kind === 'edge') {
        x += it.w;
        continue;
      } else {
        if (it.collapsible && line.length === 0) continue;
        w = it.w;
        piece = { kind: 'text', text: it.text, w, cs: it.cs, node: it.node, owner: it.owner, collapsible: it.collapsible, x };
      }
      // A line may break after a wrapping space and around atomic boxes.
      const breakable = canBreak || (it.kind === 'atomic' && line.length > 0);
      if (!(it.kind === 'text' && it.collapsible) && breakable && line.length && x + w > lineWidthLimit + 1e-6) {
        finish(false);
        piece.x = 0;
      }
      line.push(piece);
      x += w;
      canBreak = it.kind === 'atomic' || (it.kind === 'text' && it.space && it.wrap);
    }
    finish(false);
    return { height: y, baseline: firstBaseline };
  }

  // ---------------------------------------------------------------- boxes

  function place(el, x, y) {
    const box = el._box;
    const cs = el._cs;
    if (cs.position === 'relative') {
      const cbW = box.cbW;
      const cbH = box.cbH;
      const left = px(cs.left, cbW, cs);
      const right = px(cs.right, cbW, cs);
      const top = px(cs.top, cbH, cs);
      const bottom = px(cs.bottom, cbH, cs);
      x += left ?? (right !== null ? -right : 0);
      y += top ?? (bottom !== null ? -bottom : 0);
    }
    box.x = x;
    box.y = y;
  }

  // Lay out `el` and its content. Options:
  //   cbW, cbH   containing block size (cbH null when indefinite)
  //   shrink     width is shrink-to-fit within availW (else fills cbW)
  //   forcedW/H  border-box size imposed by a flex or grid container
  function layoutElement(el, o) {
    const cs = el._cs;
    const e = edges(cs, o.cbW);
    const hE = e.pl + e.pr + e.bl + e.br;
    const vE = e.pt + e.pb + e.bt + e.bb;
    const replaced = isReplaced(el);
    const specW = boxSize(cs, 'width', o.cbW, hE);
    let specH = o.forcedH ?? boxSize(cs, 'height', o.cbH, vE);
    let w;
    let rsize = null;
    if (o.forcedW !== undefined && o.forcedW !== null) w = o.forcedW;
    else if (specW !== null) w = specW;
    else if (replaced) {
      rsize = replacedSize(el, cs, o.cbW, o.cbH, null, specH === null ? null : specH - vE, o.cbW === null ? null : o.cbW - (e.ml ?? 0) - (e.mr ?? 0) - hE);
      w = rsize.w + hE;
    } else if (o.shrink) {
      const c = intrinsic(el);
      const avail = (o.availW ?? Infinity) - (e.ml ?? 0) - (e.mr ?? 0);
      w = Math.min(Math.max(c.min, avail), c.max);
    } else {
      w = (o.cbW ?? 0) - (e.ml ?? 0) - (e.mr ?? 0);
    }
    if (o.forcedW === undefined || o.forcedW === null) w = clampSize(cs, w, 'min-width', 'max-width', o.cbW, hE);
    // Auto horizontal margins of a block with a width centre it.
    if (!o.shrink && o.forcedW == null && o.cbW !== null && specW !== null) {
      const free = o.cbW - w - (e.ml ?? 0) - (e.mr ?? 0);
      if (e.ml === null && e.mr === null) e.ml = e.mr = free / 2;
      else if (e.ml === null) e.ml = o.cbW - w - e.mr;
      else if (e.mr === null) e.mr = o.cbW - w - e.ml;
    }
    const box = el._box && el._box.pass === pass ? el._box : { pass, x: 0, y: 0 };
    Object.assign(box, e, { ml: e.ml ?? 0, mr: e.mr ?? 0, mt: e.mt ?? 0, mb: e.mb ?? 0, w, cbW: o.cbW, cbH: o.cbH, baseline: null });
    el._box = box;
    el._frags = [];
    el._boxPass = pass;
    const contentW = Math.max(0, w - hE);
    const contentHDef = specH !== null ? Math.max(0, specH - vE) : null;
    const containing = isContainingBlock(cs) || el === doc.documentElement;
    const absList = containing ? [] : o.absList;
    let contentH;
    const d = cs.display;
    if (replaced) {
      if (!rsize) rsize = replacedSize(el, cs, o.cbW, o.cbH, contentW, contentHDef, contentW);
      contentH = contentHDef ?? rsize.h;
      if (el.localName === 'svg') for (let c = el._first; c; c = c._next) if (c.nodeType === 1) clearBoxes(c);
    } else if (d === 'flex' || d === 'inline-flex') {
      contentH = layoutFlex(el, box, contentW, contentHDef, absList);
    } else if (d === 'grid' || d === 'inline-grid') {
      contentH = layoutGrid(el, box, contentW, contentHDef, absList);
    } else {
      if (d !== 'block' && d !== 'inline-block' && d !== 'list-item' && d !== 'inline' && d !== 'flow-root') {
        warnOnce(`display:${d}`, `display: ${d} is laid out as a block`);
      }
      contentH = layoutBlock(el, box, contentW, absList);
    }
    let h = specH ?? contentH + vE;
    if (o.forcedH == null) h = clampSize(cs, h, 'min-height', 'max-height', o.cbH, vE);
    box.h = h;
    box.contentH = contentH;
    if (containing) layoutAbsolute(el, box, absList, el === doc.documentElement);
    return box;
  }

  function clearBoxes(el) {
    el._boxPass = -1;
  }

  function layoutBlock(el, box, contentW, absList) {
    const ox = box.bl + box.pl;
    const oy = box.bt + box.pt;
    let y = 0;
    let prevMb = null;
    let group = [];
    const flush = () => {
      if (!group.length) return;
      const r = layoutInline(el, group, contentW, ox, oy + y, absList);
      if (r.height > 0) {
        if (box.baseline === null && r.baseline !== null) box.baseline = r.baseline;
        if (prevMb !== null) y += prevMb;
        y += r.height;
        prevMb = null;
      }
      group = [];
    };
    for (const child of flowChildren(el)) {
      if (!isElementItem(child)) {
        group.push(child);
        continue;
      }
      const cs = child._cs;
      if (!cs || cs.display === 'none') {
        if (child.nodeType === 1) clearBoxes(child);
        continue;
      }
      if (isAbs(cs)) {
        if (group.length) group.push(child);
        else absList.push({ el: child, sx: ox, sy: oy + y + (prevMb ?? 0) });
        continue;
      }
      if (isInlineLevel(child)) {
        group.push(child);
        continue;
      }
      flush();
      const cb = layoutElement(child, { cbW: contentW, cbH: null, absList });
      const gap = prevMb === null ? cb.mt : collapse(prevMb, cb.mt);
      y += gap;
      place(child, ox + cb.ml, oy + y);
      if (box.baseline === null && cb.baseline !== null) box.baseline = cb.y + cb.baseline;
      y += cb.h;
      prevMb = cb.mb;
    }
    flush();
    if (prevMb !== null) y += prevMb;
    return y;
  }

  // Two adjoining vertical margins (CSS 2.2 8.3.1).
  const collapse = (a, b) => Math.max(a, b, 0) + Math.min(a, b, 0);

  // Absolutely positioned boxes whose containing block is `host`.
  function layoutAbsolute(host, hostBox, list, isRoot) {
    if (!list.length) return;
    const cbX = isRoot ? 0 : hostBox.bl;
    const cbY = isRoot ? 0 : hostBox.bt;
    const cbW = isRoot ? doc.__width : hostBox.w - hostBox.bl - hostBox.br;
    const cbH = isRoot ? doc.__height : hostBox.h - hostBox.bt - hostBox.bb;
    for (const { el, sx, sy } of list) {
      const cs = el._cs;
      const parent = el.parentNode;
      // The static position is in the parent's coordinates; the offsets of
      // the parent chain up to the host bring it into the host's.
      let offX = 0;
      let offY = 0;
      for (let p = parent; p && p !== host && p.nodeType === 1; p = p.parentNode) {
        if (p._box && p._boxPass === pass) {
          offX += p._box.x;
          offY += p._box.y;
        }
      }
      const e = edges(cs, cbW);
      const hE = e.pl + e.pr + e.bl + e.br;
      const vE = e.pt + e.pb + e.bt + e.bb;
      const left = px(cs.left, cbW, cs);
      const right = px(cs.right, cbW, cs);
      const top = px(cs.top, cbH, cs);
      const bottom = px(cs.bottom, cbH, cs);
      let forcedW = null;
      let forcedH = null;
      if (boxSize(cs, 'width', cbW, hE) === null && !isReplaced(el) && left !== null && right !== null) {
        forcedW = clampSize(cs, cbW - left - right - (e.ml ?? 0) - (e.mr ?? 0), 'min-width', 'max-width', cbW, hE);
      }
      if (boxSize(cs, 'height', cbH, vE) === null && !isReplaced(el) && top !== null && bottom !== null) {
        forcedH = clampSize(cs, cbH - top - bottom - (e.mt ?? 0) - (e.mb ?? 0), 'min-height', 'max-height', cbH, vE);
      }
      const avail = cbW - (left ?? 0) - (right ?? 0);
      const box = layoutElement(el, { cbW, cbH, shrink: forcedW === null, availW: avail, forcedW, forcedH });
      let x;
      if (left !== null) x = cbX + left + box.ml;
      else if (right !== null) x = cbX + cbW - right - box.mr - box.w;
      else x = sx + offX + box.ml;
      let y;
      if (top !== null) y = cbY + top + box.mt;
      else if (bottom !== null) y = cbY + cbH - bottom - box.mb - box.h;
      else y = sy + offY + box.mt;
      box.x = x - offX;
      box.y = y - offY;
    }
  }

  // ---------------------------------------------------------------- flex

  // Flex items: elements in flow, and runs of text as anonymous items.
  function flexItems(el) {
    const items = [];
    let anon = null;
    for (const child of flowChildren(el)) {
      if (isElementItem(child)) {
        const cs = child._cs;
        if (!cs || cs.display === 'none') {
          clearBoxes(child);
          continue;
        }
        if (isAbs(cs)) {
          items.push({ abs: child });
          continue;
        }
        anon = null;
        items.push({ el: child, cs, order: cs.order });
      } else {
        if (child.nodeType === 3 && isWhitespace(child) && !anon) continue;
        if (!anon) items.push((anon = { anon: [], parent: el, cs: el._cs, order: 0 }));
        anon.anon.push(child);
      }
    }
    return items.filter((i) => !i.abs).sort((a, b) => a.order - b.order);
  }

  const alignOf = (item, containerCs) => {
    if (item.anon) return containerCs['align-items'];
    const self = item.cs['align-self'];
    return self === 'auto' ? containerCs['align-items'] : self;
  };

  function layoutFlex(el, box, contentW, contentH, absList) {
    const cs = el._cs;
    const dir = cs['flex-direction'];
    const row = dir.startsWith('row');
    const reverse = dir.endsWith('reverse');
    const wrap = cs['flex-wrap'] !== 'nowrap';
    const colGap = px(cs['column-gap'], contentW, cs) ?? 0;
    const rowGap = px(cs['row-gap'], contentH, cs) ?? 0;
    const mainGap = row ? colGap : rowGap;
    const crossGap = row ? rowGap : colGap;
    const mainSize = row ? contentW : contentH;
    const ox = box.bl + box.pl;
    const oy = box.bt + box.pt;
    for (const child of flowChildren(el)) {
      if (isElementItem(child) && child._cs && child._cs.display !== 'none' && isAbs(child._cs)) absList.push({ el: child, sx: ox, sy: oy });
    }
    const items = flexItems(el);
    // Base and hypothetical main sizes.
    for (const it of items) {
      if (it.anon) {
        const c = inlineIntrinsic(el, it.anon);
        it.grow = 0;
        it.shrink = 1;
        it.mMain0 = it.mMain1 = it.mCross0 = it.mCross1 = 0;
        if (row) {
          it.base = c.max;
          it.minMain = c.min;
        } else {
          it.base = anonHeight(el, it, contentW);
          it.minMain = it.base;
        }
        it.maxMain = Infinity;
        it.hyp = Math.max(it.base, it.minMain);
        continue;
      }
      const ics = it.cs;
      const e = edges(ics, contentW);
      const hE = e.pl + e.pr + e.bl + e.br;
      const vE = e.pt + e.pb + e.bt + e.bb;
      it.e = e;
      it.grow = ics['flex-grow'];
      it.shrink = ics['flex-shrink'];
      it.mMain0 = (row ? e.ml : e.mt) ?? 0;
      it.mMain1 = (row ? e.mr : e.mb) ?? 0;
      it.mCross0 = (row ? e.mt : e.ml) ?? 0;
      it.mCross1 = (row ? e.mb : e.mr) ?? 0;
      it.autoMain = (row ? e.ml : e.mt) === null || (row ? e.mr : e.mb) === null;
      const mainEdges = row ? hE : vE;
      const basis = ics['flex-basis'];
      let base = null;
      if (typeof basis !== 'string') {
        const v = px(basis, mainSize, ics);
        if (v !== null) base = ics['box-sizing'] === 'border-box' ? Math.max(v, mainEdges) : v + mainEdges;
      }
      if (base === null) base = boxSize(ics, row ? 'width' : 'height', mainSize, mainEdges);
      if (base === null) {
        if (row) base = intrinsic(it.el).max;
        else base = layoutElement(it.el, { cbW: contentW, cbH: contentH, shrink: boxSize(ics, 'width', contentW, hE) === null && alignOf(it, cs) !== 'stretch' && alignOf(it, cs) !== 'normal', availW: contentW, forcedW: crossStretchWidth(it, cs, contentW, row), absList }).h;
      }
      it.base = base;
      const minProp = row ? 'min-width' : 'min-height';
      let minMain = boxSize(ics, minProp, mainSize, mainEdges);
      if (minMain === null) {
        // The automatic minimum size of a flex item (CSS Flexbox 4.5).
        const scrolls = (ics['overflow-x'] !== 'visible' && ics['overflow-x'] !== 'clip') || (ics['overflow-y'] !== 'visible' && ics['overflow-y'] !== 'clip');
        if (scrolls) minMain = mainEdges;
        else if (row) {
          const spec = boxSize(ics, 'width', mainSize, mainEdges);
          minMain = Math.min(intrinsic(it.el).min, spec ?? Infinity);
        } else {
          const spec = boxSize(ics, 'height', mainSize, mainEdges);
          minMain = Math.min(it.base, spec ?? Infinity);
        }
      }
      const maxMain = boxSize(ics, row ? 'max-width' : 'max-height', mainSize, mainEdges);
      it.minMain = minMain;
      it.maxMain = maxMain ?? Infinity;
      it.hyp = Math.min(Math.max(base, minMain), it.maxMain);
    }
    // Lines.
    const lines = [];
    let cur = [];
    let used = 0;
    for (const it of items) {
      const outer = it.hyp + it.mMain0 + it.mMain1;
      if (wrap && mainSize !== null && cur.length && used + mainGap + outer > mainSize + 1e-6) {
        lines.push(cur);
        cur = [];
        used = 0;
      }
      used += (cur.length ? mainGap : 0) + outer;
      cur.push(it);
    }
    if (cur.length) lines.push(cur);
    // Flexible lengths (CSS Flexbox 9.7).
    for (const line of lines) {
      const gaps = mainGap * Math.max(0, line.length - 1);
      for (const it of line) it.target = it.hyp;
      if (mainSize === null) continue;
      const sumHyp = line.reduce((s, it) => s + it.hyp + it.mMain0 + it.mMain1, 0) + gaps;
      const growing = sumHyp < mainSize;
      const frozen = new Set(line.filter((it) => (growing ? it.grow === 0 || it.base > it.hyp : it.shrink === 0 || it.base < it.hyp)));
      for (let round = 0; round < line.length + 1; round++) {
        const unfrozen = line.filter((it) => !frozen.has(it));
        if (!unfrozen.length) break;
        const taken = line.reduce((s, it) => s + (frozen.has(it) ? it.target : it.base) + it.mMain0 + it.mMain1, 0) + gaps;
        let free = mainSize - taken;
        if (growing) {
          const sum = unfrozen.reduce((s, it) => s + it.grow, 0);
          if (sum < 1) free *= sum;
          for (const it of unfrozen) it.target = it.base + (sum > 0 ? (free * it.grow) / sum : 0);
        } else {
          const sum = unfrozen.reduce((s, it) => s + it.shrink * it.base, 0);
          for (const it of unfrozen) it.target = it.base + (sum > 0 ? (free * it.shrink * it.base) / sum : 0);
        }
        let violation = 0;
        for (const it of unfrozen) {
          const clamped = Math.min(Math.max(it.target, it.minMain), it.maxMain);
          violation += clamped - it.target;
          it.clamped = clamped;
        }
        for (const it of unfrozen) {
          if ((violation > 1e-9 && it.clamped > it.target) || (violation < -1e-9 && it.clamped < it.target) || Math.abs(violation) <= 1e-9) {
            it.target = it.clamped;
            frozen.add(it);
          }
        }
        if (Math.abs(violation) <= 1e-9) break;
      }
    }
    // Cross sizes.
    const crossDef = row ? contentH : contentW;
    const lineCross = [];
    for (const line of lines) {
      let maxCross = 0;
      let maxAbove = 0;
      let maxBelow = 0;
      for (const it of line) {
        measureCross(el, it, row, contentW, contentH, absList);
        const outer = it.cross + it.mCross0 + it.mCross1;
        if (row && alignOf(it, cs) === 'baseline') {
          const b = it.baselineOff ?? it.cross;
          maxAbove = Math.max(maxAbove, b + it.mCross0);
          maxBelow = Math.max(maxBelow, outer - b - it.mCross0);
        }
        maxCross = Math.max(maxCross, outer);
      }
      lineCross.push(Math.max(maxCross, maxAbove + maxBelow));
      line.baselineMax = maxAbove;
    }
    if (lines.length === 1 && crossDef !== null) lineCross[0] = crossDef;
    // align-content (normal behaves as stretch).
    let crossStart = 0;
    let crossBetween = crossGap;
    if (lines.length > 1 && crossDef !== null) {
      const free = crossDef - lineCross.reduce((a, b) => a + b, 0) - crossGap * (lines.length - 1);
      const ac = cs['align-content'];
      if (free > 0) {
        if (ac === 'normal' || ac === 'stretch') for (let i = 0; i < lineCross.length; i++) lineCross[i] += free / lines.length;
        else if (ac === 'center') crossStart = free / 2;
        else if (ac === 'flex-end' || ac === 'end') crossStart = free;
        else if (ac === 'space-between' && lines.length > 1) crossBetween += free / (lines.length - 1);
        else if (ac === 'space-around') {
          crossStart = free / lines.length / 2;
          crossBetween += free / lines.length;
        } else if (ac === 'space-evenly') {
          crossStart = free / (lines.length + 1);
          crossBetween += free / (lines.length + 1);
        }
      }
    }
    // Place.
    let crossPos = crossStart;
    let mainUsedMax = 0;
    lines.forEach((line, li) => {
      const lc = lineCross[li];
      const gaps = mainGap * Math.max(0, line.length - 1);
      const usedMain = line.reduce((s, it) => s + it.target + it.mMain0 + it.mMain1, 0) + gaps;
      mainUsedMax = Math.max(mainUsedMax, usedMain);
      let free = mainSize === null ? 0 : mainSize - usedMain;
      let start = 0;
      let between = mainGap;
      const autoCount = line.reduce((s, it) => s + (it.e ? ((row ? it.e.ml : it.e.mt) === null) + ((row ? it.e.mr : it.e.mb) === null) : 0), 0);
      if (free > 0 && autoCount > 0) {
        const each = free / autoCount;
        for (const it of line) {
          if (!it.e) continue;
          if ((row ? it.e.ml : it.e.mt) === null) it.mMain0 += each;
          if ((row ? it.e.mr : it.e.mb) === null) it.mMain1 += each;
        }
        free = 0;
      }
      const jc = cs['justify-content'];
      const n = line.length;
      if (free > 0 || jc === 'center' || jc === 'flex-end' || jc === 'end') {
        if (jc === 'center') start = free / 2;
        else if (jc === 'flex-end' || jc === 'end' || (jc === 'right' && row)) start = free;
        else if (jc === 'space-between') between += n > 1 ? free / (n - 1) : 0;
        else if (jc === 'space-around') {
          start = free / n / 2;
          between += free / n;
        } else if (jc === 'space-evenly') {
          start = free / (n + 1);
          between += free / (n + 1);
        }
        if (free < 0 && jc !== 'center' && jc !== 'flex-end' && jc !== 'end') start = 0;
      }
      let mainPos = start;
      for (const it of line) {
        mainPos += it.mMain0;
        const align = alignOf(it, cs);
        let crossOff = 0;
        const outer = it.cross + it.mCross0 + it.mCross1;
        if (align === 'stretch' || align === 'normal') {
          if (it.stretchable) {
            it.cross = Math.max(0, lc - it.mCross0 - it.mCross1);
            if (!it.anon) {
              const minC = boxSize(it.cs, row ? 'min-height' : 'min-width', row ? contentH : contentW, row ? it.e.pt + it.e.pb + it.e.bt + it.e.bb : it.e.pl + it.e.pr + it.e.bl + it.e.br);
              const maxC = boxSize(it.cs, row ? 'max-height' : 'max-width', row ? contentH : contentW, row ? it.e.pt + it.e.pb + it.e.bt + it.e.bb : it.e.pl + it.e.pr + it.e.bl + it.e.br);
              if (maxC !== null) it.cross = Math.min(it.cross, maxC);
              if (minC !== null) it.cross = Math.max(it.cross, minC);
            }
            relayItem(el, it, row, contentW, contentH, absList);
          }
        } else if (align === 'center') crossOff = (lc - outer) / 2;
        else if (align === 'flex-end' || align === 'end' || align === 'self-end') crossOff = lc - outer;
        else if (align === 'baseline' && row) crossOff = line.baselineMax - (it.baselineOff ?? it.cross) - it.mCross0;
        const mainAt = reverse ? (mainSize ?? usedMain) - mainPos - it.target : mainPos;
        const cx = crossPos + crossOff + it.mCross0;
        if (it.anon) {
          const fx = row ? ox + mainAt : ox + cx;
          const fy = row ? oy + cx : oy + mainAt;
          layoutInline(el, it.anon, row ? it.target : it.cross, fx, fy, absList);
        } else if (row) place(it.el, ox + mainAt, oy + cx);
        else place(it.el, ox + cx, oy + mainAt);
        if (box.baseline === null && li === 0 && !it.anon && it.el._box.baseline !== null) box.baseline = it.el._box.y + it.el._box.baseline;
        mainPos += it.target + it.mMain1 + between;
      }
      crossPos += lc + crossBetween;
    });
    const crossUsed = lineCross.reduce((a, b) => a + b, 0) + crossGap * Math.max(0, lines.length - 1);
    return row ? crossUsed : mainSize ?? mainUsedMax;
  }

  // The width a column-direction item stretches to, or null.
  function crossStretchWidth(it, containerCs, contentW, row) {
    if (row || it.anon) return null;
    const a = alignOf(it, containerCs);
    const e = it.e ?? edges(it.cs, contentW);
    const hE = e.pl + e.pr + e.bl + e.br;
    if ((a === 'stretch' || a === 'normal') && boxSize(it.cs, 'width', contentW, hE) === null && e.ml !== null && e.mr !== null) {
      return Math.max(hE, contentW - e.ml - e.mr);
    }
    return null;
  }

  function anonHeight(el, it, width) {
    const saved = el._frags;
    el._frags = [];
    const r = layoutInline(el, it.anon, width, 0, 0, []);
    el._frags = saved;
    return r.height;
  }

  function measureCross(el, it, row, contentW, contentH, absList) {
    const cs = el._cs;
    const a = alignOf(it, cs);
    if (it.anon) {
      it.cross = row ? anonHeight(el, it, it.target) : inlineIntrinsic(el, it.anon).max;
      it.stretchable = true;
      return;
    }
    const e = it.e;
    const hE = e.pl + e.pr + e.bl + e.br;
    const vE = e.pt + e.pb + e.bt + e.bb;
    if (row) {
      const box = layoutElement(it.el, { cbW: contentW, cbH: contentH, forcedW: it.target, absList });
      it.cross = box.h;
      it.baselineOff = box.baseline;
      it.stretchable = (a === 'stretch' || a === 'normal') && boxSize(it.cs, 'height', contentH, vE) === null && e.mt !== null && e.mb !== null;
    } else {
      const stretchW = crossStretchWidth(it, cs, contentW, row);
      const box = layoutElement(it.el, { cbW: contentW, cbH: contentH, forcedH: it.target, forcedW: stretchW, shrink: stretchW === null && boxSize(it.cs, 'width', contentW, hE) === null, availW: contentW, absList });
      it.cross = box.w;
      it.stretchable = false;
    }
  }

  function relayItem(el, it, row, contentW, contentH, absList) {
    if (it.anon) return;
    if (row) layoutElement(it.el, { cbW: contentW, cbH: contentH, forcedW: it.target, forcedH: it.cross, absList });
  }

  // ---------------------------------------------------------------- grid

  const trackList = (v) => (v === 'none' || !v ? [] : v);

  function gridItems(el) {
    const items = [];
    for (const child of flowChildren(el)) {
      if (isElementItem(child)) {
        const cs = child._cs;
        if (!cs || cs.display === 'none') {
          clearBoxes(child);
          continue;
        }
        if (isAbs(cs)) continue;
        items.push({ el: child, cs, order: cs.order });
      } else if (!(child.nodeType === 3 && isWhitespace(child))) {
        items.push({ anon: [child], parent: el, cs: el._cs, order: 0 });
      }
    }
    return items.sort((a, b) => a.order - b.order);
  }

  // Place items on the grid (CSS Grid 8.5, row flow, sparse).
  function placeGrid(el, cs, items) {
    const cols = trackList(cs['grid-template-columns']);
    let nCols = Math.max(1, cols.length);
    const lineIndex = (v, n) => (v.line > 0 ? v.line - 1 : n + 1 + v.line);
    const spanOf = (start, end) => {
      if (start && start.span) return start.span;
      if (end && end.span) return end.span;
      if (start && start.line !== undefined && end && end.line !== undefined) return Math.max(1, lineIndex(end, nCols) - lineIndex(start, nCols));
      return 1;
    };
    for (const it of items) {
      const c = it.anon ? {} : it.cs;
      const cStart = c['grid-column-start'];
      const cEnd = c['grid-column-end'];
      const rStart = c['grid-row-start'];
      const rEnd = c['grid-row-end'];
      it.cspan = spanOf(cStart && cStart !== 'auto' ? cStart : null, cEnd && cEnd !== 'auto' ? cEnd : null);
      it.rspan = spanOf(rStart && rStart !== 'auto' ? rStart : null, rEnd && rEnd !== 'auto' ? rEnd : null);
      it.col = cStart && cStart.line !== undefined ? lineIndex(cStart, nCols) : cEnd && cEnd.line !== undefined ? lineIndex(cEnd, nCols) - it.cspan : null;
      it.row = rStart && rStart.line !== undefined ? rStart.line - 1 : rEnd && rEnd.line !== undefined ? rEnd.line - 1 - it.rspan : null;
      if (it.col !== null) nCols = Math.max(nCols, it.col + it.cspan);
      else nCols = Math.max(nCols, it.cspan);
    }
    const taken = new Set();
    const free = (r, c, rs, cs2) => {
      if (c + cs2 > nCols) return false;
      for (let i = 0; i < rs; i++) for (let j = 0; j < cs2; j++) if (taken.has(`${r + i},${c + j}`)) return false;
      return true;
    };
    const take = (it) => {
      for (let i = 0; i < it.rspan; i++) for (let j = 0; j < it.cspan; j++) taken.add(`${it.row + i},${it.col + j}`);
    };
    for (const it of items) if (it.row !== null && it.col !== null) take(it);
    for (const it of items) {
      if (it.row !== null && it.col === null) {
        let c = 0;
        while (!free(it.row, c, it.rspan, it.cspan) && c < nCols) c++;
        it.col = Math.min(c, Math.max(0, nCols - it.cspan));
        take(it);
      }
    }
    let r = 0;
    let c = 0;
    for (const it of items) {
      if (it.row !== null) continue;
      if (it.col !== null) {
        if (it.col < c) r++;
        c = it.col;
        while (!free(r, c, it.rspan, it.cspan)) r++;
      } else {
        for (;;) {
          if (c + it.cspan > nCols) {
            c = 0;
            r++;
          }
          if (free(r, c, it.rspan, it.cspan)) break;
          c++;
        }
      }
      it.row = r;
      it.col = c;
      take(it);
      c += it.cspan;
    }
    const nRows = items.reduce((m, it) => Math.max(m, it.row + it.rspan), trackList(cs['grid-template-rows']).length);
    return { nCols, nRows };
  }

  const trackAt = (list, auto, i) => (i < list.length ? list[i] : auto.length ? auto[(i - list.length) % auto.length] : { auto: true });

  function sizeTracks(tracks, avail, gap, contributions) {
    const n = tracks.length;
    const sizes = new Array(n).fill(0);
    let fixed = 0;
    let frSum = 0;
    const autos = [];
    tracks.forEach((t, i) => {
      if (t.len) {
        const v = css.isPercent(t.len) ? (avail === null ? null : css.toPx(t.len, avail, 16, doc)) : css.toPx(t.len, 0, 16, doc);
        if (v !== null) {
          sizes[i] = v;
          fixed += v;
          return;
        }
      }
      if (t.fr !== undefined && avail !== null) {
        frSum += t.fr;
        return;
      }
      sizes[i] = contributions[i].max;
      fixed += sizes[i];
      autos.push(i);
    });
    const gaps = gap * Math.max(0, n - 1);
    if (avail !== null) {
      let free = avail - fixed - gaps;
      if (frSum > 0) {
        const unit = Math.max(0, free) / Math.max(1, frSum);
        tracks.forEach((t, i) => {
          if (t.fr !== undefined) sizes[i] = Math.max(unit * t.fr, t.min ? css.toPx(t.min, avail, 16, doc) : contributions[i].min);
        });
      } else if (free > 0 && autos.length) {
        for (const i of autos) sizes[i] += free / autos.length;
      } else if (free < 0 && autos.length) {
        // Auto tracks give back down to their minimum contribution.
        for (const i of autos) {
          const give = Math.min(sizes[i] - contributions[i].min, -free);
          if (give > 0) {
            sizes[i] -= give;
            free += give;
          }
        }
      }
    }
    return sizes;
  }

  function gridIntrinsic(el, cs) {
    const items = gridItems(el);
    const { nCols } = placeGrid(el, cs, items);
    const cols = trackList(cs['grid-template-columns']);
    const autoCols = trackList(cs['grid-auto-columns']);
    const contrib = Array.from({ length: nCols }, () => ({ min: 0, max: 0 }));
    for (const it of items) {
      if (it.cspan !== 1) continue;
      const c = itemIntrinsic(it);
      contrib[it.col].min = Math.max(contrib[it.col].min, c.min);
      contrib[it.col].max = Math.max(contrib[it.col].max, c.max);
    }
    const gap = px(cs['column-gap'], 0, cs) ?? 0;
    let min = 0;
    let max = 0;
    for (let i = 0; i < nCols; i++) {
      const t = trackAt(cols, autoCols, i);
      const v = t.len && !css.isPercent(t.len) ? css.toPx(t.len, 0, 16, doc) : null;
      min += v ?? contrib[i].min;
      max += v ?? contrib[i].max;
    }
    return { min: min + gap * (nCols - 1), max: max + gap * (nCols - 1) };
  }

  function layoutGrid(el, box, contentW, contentH, absList) {
    const cs = el._cs;
    const ox = box.bl + box.pl;
    const oy = box.bt + box.pt;
    for (const child of flowChildren(el)) {
      if (isElementItem(child) && child._cs && child._cs.display !== 'none' && isAbs(child._cs)) absList.push({ el: child, sx: ox, sy: oy });
    }
    const items = gridItems(el);
    const { nCols, nRows } = placeGrid(el, cs, items);
    const colGap = px(cs['column-gap'], contentW, cs) ?? 0;
    const rowGap = px(cs['row-gap'], contentH, cs) ?? 0;
    const cols = Array.from({ length: nCols }, (_, i) => trackAt(trackList(cs['grid-template-columns']), trackList(cs['grid-auto-columns']), i));
    const rows = Array.from({ length: nRows }, (_, i) => trackAt(trackList(cs['grid-template-rows']), trackList(cs['grid-auto-rows']), i));
    const colContrib = cols.map(() => ({ min: 0, max: 0 }));
    for (const it of items) {
      if (it.cspan !== 1) continue;
      const c = itemIntrinsic(it);
      colContrib[it.col].min = Math.max(colContrib[it.col].min, c.min);
      colContrib[it.col].max = Math.max(colContrib[it.col].max, c.max);
    }
    const colSizes = sizeTracks(cols, contentW, colGap, colContrib);
    const colStart = [];
    let acc = 0;
    for (let i = 0; i < nCols; i++) {
      colStart.push(acc);
      acc += colSizes[i] + colGap;
    }
    const areaW = (it) => colSizes.slice(it.col, it.col + it.cspan).reduce((a, b) => a + b, 0) + colGap * (it.cspan - 1);
    // Item heights at their column widths.
    const rowContrib = rows.map(() => ({ min: 0, max: 0 }));
    for (const it of items) {
      const w = areaW(it);
      if (it.anon) {
        it.h = anonHeight(el, it, w);
      } else {
        const e = edges(it.cs, contentW);
        it.e = e;
        const hE = e.pl + e.pr + e.bl + e.br;
        const js = it.cs['justify-self'];
        const stretch = (js === 'auto' || js === 'normal' || js === 'stretch') && boxSize(it.cs, 'width', w, hE) === null && !isReplaced(it.el);
        it.forcedW = stretch ? Math.max(hE, w - (e.ml ?? 0) - (e.mr ?? 0)) : null;
        const b = layoutElement(it.el, { cbW: w, cbH: null, forcedW: it.forcedW, shrink: !stretch, availW: w, absList });
        it.h = b.h + b.mt + b.mb;
        it.w = b.w + b.ml + b.mr;
      }
      if (it.rspan === 1) {
        rowContrib[it.row].min = Math.max(rowContrib[it.row].min, it.h);
        rowContrib[it.row].max = Math.max(rowContrib[it.row].max, it.h);
      }
    }
    const rowSizes = sizeTracks(rows, contentH, rowGap, rowContrib);
    for (const it of items) {
      if (it.rspan === 1) continue;
      const have = rowSizes.slice(it.row, it.row + it.rspan).reduce((a, b) => a + b, 0) + rowGap * (it.rspan - 1);
      if (it.h > have) rowSizes[it.row + it.rspan - 1] += it.h - have;
    }
    const rowStart = [];
    acc = 0;
    for (let i = 0; i < nRows; i++) {
      rowStart.push(acc);
      acc += rowSizes[i] + rowGap;
    }
    const totalH = rowSizes.reduce((a, b) => a + b, 0) + rowGap * Math.max(0, nRows - 1);
    for (const it of items) {
      const x = colStart[it.col];
      const y = rowStart[it.row];
      const w = areaW(it);
      const h = rowSizes.slice(it.row, it.row + it.rspan).reduce((a, b) => a + b, 0) + rowGap * (it.rspan - 1);
      if (it.anon) {
        layoutInline(el, it.anon, w, ox + x, oy + y, absList);
        continue;
      }
      const e = it.e;
      const vE = e.pt + e.pb + e.bt + e.bb;
      const as = it.cs['align-self'] === 'auto' ? cs['align-items'] : it.cs['align-self'];
      const js = it.cs['justify-self'] === 'auto' ? cs['justify-items'] : it.cs['justify-self'];
      const stretchH = (as === 'normal' || as === 'stretch') && boxSize(it.cs, 'height', h, vE) === null && !isReplaced(it.el);
      const b = layoutElement(it.el, { cbW: w, cbH: h, forcedW: it.forcedW, forcedH: stretchH ? Math.max(vE, h - (e.mt ?? 0) - (e.mb ?? 0)) : null, shrink: it.forcedW === null, availW: w, absList });
      let dx = b.ml;
      let dy = b.mt;
      const outerW = b.w + b.ml + b.mr;
      const outerH = b.h + b.mt + b.mb;
      if (js === 'center') dx += (w - outerW) / 2;
      else if (js === 'end' || js === 'flex-end' || js === 'right') dx += w - outerW;
      if (as === 'center') dy += (h - outerH) / 2;
      else if (as === 'end' || as === 'flex-end') dy += h - outerH;
      place(it.el, ox + x + dx, oy + y + dy);
    }
    return totalH;
  }

  // ---------------------------------------------------------------- the pass

  function signature(el) {
    const b = el._box;
    if (!b || el._boxPass !== pass) return '';
    let s = `${b.x},${b.y},${b.w},${b.h}`;
    if (el._frags && el._frags.length) for (const f of el._frags) s += `|${f.text}@${f.x},${f.y}`;
    return s;
  }

  function run(document) {
    doc = document;
    const root = doc.documentElement;
    if (!root || !root._cs) return;
    const before = new Map();
    const collect = (el) => {
      for (let c = el._first; c; c = c._next) {
        if (c.nodeType !== 1 || c.namespaceURI === SVG_NS) {
          if (c.nodeType === 1 && c.localName === 'svg') before.set(c, signature(c));
          continue;
        }
        before.set(c, signature(c));
        collect(c);
      }
    };
    before.set(root, signature(root));
    collect(root);
    pass = ++doc._layoutPass;
    // After the pass number moves, signature() of the old boxes reads ''.
    if (root._cs.display !== 'none') {
      const box = layoutElement(root, { cbW: doc.__width, cbH: doc.__height });
      box.x = box.ml;
      box.y = box.mt;
    }
    for (const [el, old] of before) {
      if (signature(el) !== old) {
        if (el.localName === 'svg' && el.namespaceURI === SVG_NS) doc._markPaintDeep(el);
        else doc._markPaint(el);
      }
    }
    // Elements that got a box for the first time.
    const fresh = (el) => {
      for (let c = el._first; c; c = c._next) {
        if (c.nodeType !== 1) continue;
        if (!before.has(c) && c._boxPass === pass) doc._markPaint(c);
        if (c.namespaceURI !== SVG_NS) fresh(c);
      }
    };
    fresh(root);
  }

  D.layout = { run, edges, viewBoxOf, svgAttrLength: (el, name, base) => ((doc = el.ownerDocument), svgAttrLength(el, name, base)) };
})();
