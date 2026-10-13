// CSS: value parsing, the property table, shorthands, stylesheets, the
// cascade (specificity, !important, presentation attributes, inline style,
// inheritance, custom properties) and @keyframes evaluation.
//
// Computed styles are plain objects keyed by property name. Values are
// parsed once per distinct string and shared, so comparing two computed
// styles is a matter of identity.
(() => {
  const D = globalThis.__dom;
  const warnOnce = D.warnOnce;

  // ---------------------------------------------------------------- colours

  const NAMED = {};
  (
    'aliceblue f0f8ff antiquewhite faebd7 aqua 00ffff aquamarine 7fffd4 azure f0ffff beige f5f5dc bisque ffe4c4 ' +
    'black 000000 blanchedalmond ffebcd blue 0000ff blueviolet 8a2be2 brown a52a2a burlywood deb887 cadetblue 5f9ea0 ' +
    'chartreuse 7fff00 chocolate d2691e coral ff7f50 cornflowerblue 6495ed cornsilk fff8dc crimson dc143c cyan 00ffff ' +
    'darkblue 00008b darkcyan 008b8b darkgoldenrod b8860b darkgray a9a9a9 darkgreen 006400 darkgrey a9a9a9 ' +
    'darkkhaki bdb76b darkmagenta 8b008b darkolivegreen 556b2f darkorange ff8c00 darkorchid 9932cc darkred 8b0000 ' +
    'darksalmon e9967a darkseagreen 8fbc8f darkslateblue 483d8b darkslategray 2f4f4f darkslategrey 2f4f4f ' +
    'darkturquoise 00ced1 darkviolet 9400d3 deeppink ff1493 deepskyblue 00bfff dimgray 696969 dimgrey 696969 ' +
    'dodgerblue 1e90ff firebrick b22222 floralwhite fffaf0 forestgreen 228b22 fuchsia ff00ff gainsboro dcdcdc ' +
    'ghostwhite f8f8ff gold ffd700 goldenrod daa520 gray 808080 green 008000 greenyellow adff2f grey 808080 ' +
    'honeydew f0fff0 hotpink ff69b4 indianred cd5c5c indigo 4b0082 ivory fffff0 khaki f0e68c lavender e6e6fa ' +
    'lavenderblush fff0f5 lawngreen 7cfc00 lemonchiffon fffacd lightblue add8e6 lightcoral f08080 lightcyan e0ffff ' +
    'lightgoldenrodyellow fafad2 lightgray d3d3d3 lightgreen 90ee90 lightgrey d3d3d3 lightpink ffb6c1 ' +
    'lightsalmon ffa07a lightseagreen 20b2aa lightskyblue 87cefa lightslategray 778899 lightslategrey 778899 ' +
    'lightsteelblue b0c4de lightyellow ffffe0 lime 00ff00 limegreen 32cd32 linen faf0e6 magenta ff00ff maroon 800000 ' +
    'mediumaquamarine 66cdaa mediumblue 0000cd mediumorchid ba55d3 mediumpurple 9370db mediumseagreen 3cb371 ' +
    'mediumslateblue 7b68ee mediumspringgreen 00fa9a mediumturquoise 48d1cc mediumvioletred c71585 ' +
    'midnightblue 191970 mintcream f5fffa mistyrose ffe4e1 moccasin ffe4b5 navajowhite ffdead navy 000080 ' +
    'oldlace fdf5e6 olive 808000 olivedrab 6b8e23 orange ffa500 orangered ff4500 orchid da70d6 palegoldenrod eee8aa ' +
    'palegreen 98fb98 paleturquoise afeeee palevioletred db7093 papayawhip ffefd5 peachpuff ffdab9 peru cd853f ' +
    'pink ffc0cb plum dda0dd powderblue b0e0e6 purple 800080 rebeccapurple 663399 red ff0000 rosybrown bc8f8f ' +
    'royalblue 4169e1 saddlebrown 8b4513 salmon fa8072 sandybrown f4a460 seagreen 2e8b57 seashell fff5ee ' +
    'sienna a0522d silver c0c0c0 skyblue 87ceeb slateblue 6a5acd slategray 708090 slategrey 708090 snow fffafa ' +
    'springgreen 00ff7f steelblue 4682b4 tan d2b48c teal 008080 thistle d8bfd8 tomato ff6347 turquoise 40e0d0 ' +
    'violet ee82ee wheat f5deb3 white ffffff whitesmoke f5f5f5 yellow ffff00 yellowgreen 9acd32'
  )
    .split(' ')
    .forEach((w, i, all) => {
      if (i % 2 === 0) NAMED[w] = all[i + 1];
    });

  const CURRENT = 'currentcolor';
  const TRANSPARENT = Object.freeze([0, 0, 0, 0]);

  const clamp01 = (x) => (x < 0 ? 0 : x > 1 ? 1 : x);

  const hexColor = (h) => {
    if (h.length === 3 || h.length === 4) h = [...h].map((c) => c + c).join('');
    if (h.length !== 6 && h.length !== 8) return null;
    const n = [0, 2, 4, 6].map((i) => (i < h.length ? parseInt(h.slice(i, i + 2), 16) : 255));
    if (n.some((x) => Number.isNaN(x))) return null;
    return [n[0] / 255, n[1] / 255, n[2] / 255, n[3] / 255];
  };

  const channel = (t, scale) => (t.endsWith('%') ? parseFloat(t) / 100 : parseFloat(t) / scale);

  const hslToRgb = (h, s, l) => {
    h = (((h % 360) + 360) % 360) / 360;
    const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
    const p = 2 * l - q;
    const hue = (t) => {
      if (t < 0) t += 1;
      if (t > 1) t -= 1;
      if (t < 1 / 6) return p + (q - p) * 6 * t;
      if (t < 1 / 2) return q;
      if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6;
      return p;
    };
    return [hue(h + 1 / 3), hue(h), hue(h - 1 / 3)];
  };

  const colorCache = new Map();

  // A colour string to [r, g, b, a] in 0..1, CURRENT for currentColor, or
  // null when it is not a colour.
  function parseColor(text) {
    const key = text;
    if (colorCache.has(key)) return colorCache.get(key);
    let out = null;
    const t = text.trim().toLowerCase();
    if (t === 'transparent') out = TRANSPARENT;
    else if (t === 'currentcolor') out = CURRENT;
    else if (t.startsWith('#')) out = hexColor(t.slice(1));
    else if (NAMED[t]) out = hexColor(NAMED[t]);
    else {
      const m = /^(rgba?|hsla?)\((.*)\)$/.exec(t);
      if (m) {
        const parts = m[2].split(/[\s,/]+/).filter(Boolean);
        if (m[1].startsWith('rgb') && parts.length >= 3) {
          out = [channel(parts[0], 255), channel(parts[1], 255), channel(parts[2], 255), parts.length > 3 ? channel(parts[3], 1) : 1];
        } else if (m[1].startsWith('hsl') && parts.length >= 3) {
          const h = parseFloat(parts[0]) * (parts[0].endsWith('rad') ? 180 / Math.PI : parts[0].endsWith('turn') ? 360 : 1);
          const rgb = hslToRgb(h, clamp01(parseFloat(parts[1]) / 100), clamp01(parseFloat(parts[2]) / 100));
          out = [...rgb, parts.length > 3 ? channel(parts[3], 1) : 1];
        }
        if (out && out.some((x) => Number.isNaN(x))) out = null;
        if (out && out !== CURRENT) out = out.map(clamp01);
      }
    }
    if (Array.isArray(out)) out = Object.freeze(out);
    if (colorCache.size > 4096) colorCache.clear();
    colorCache.set(key, out);
    return out;
  }

  // ---------------------------------------------------------------- tokens

  // Split on whitespace (and optionally commas) outside parentheses/quotes.
  function splitTokens(text, commas = false) {
    const out = [];
    let depth = 0;
    let quote = null;
    let cur = '';
    for (const ch of text) {
      if (quote) {
        cur += ch;
        if (ch === quote) quote = null;
        continue;
      }
      if (ch === '"' || ch === "'") {
        quote = ch;
        cur += ch;
      } else if (ch === '(') {
        depth++;
        cur += ch;
      } else if (ch === ')') {
        depth--;
        cur += ch;
      } else if (depth === 0 && (/\s/.test(ch) || (commas && ch === ','))) {
        if (cur) out.push(cur);
        cur = '';
        if (commas && ch === ',') out.push(',');
      } else {
        cur += ch;
      }
    }
    if (cur) out.push(cur);
    return out;
  }

  // Split a comma-separated list at top level.
  function splitCommas(text) {
    const out = [];
    let depth = 0;
    let quote = null;
    let start = 0;
    for (let i = 0; i < text.length; i++) {
      const ch = text[i];
      if (quote) {
        if (ch === quote) quote = null;
      } else if (ch === '"' || ch === "'") quote = ch;
      else if (ch === '(') depth++;
      else if (ch === ')') depth--;
      else if (ch === ',' && depth === 0) {
        out.push(text.slice(start, i).trim());
        start = i + 1;
      }
    }
    out.push(text.slice(start).trim());
    return out;
  }

  // ---------------------------------------------------------------- lengths

  const NUM = /^([+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?)([a-z%]*)$/i;
  const LENGTH_UNITS = new Set(['', 'px', 'mm', 'cm', 'in', 'pt', 'pc', 'q', 'em', 'rem', 'ex', 'ch', '%', 'vw', 'vh', 'vmin', 'vmax']);
  const lengthCache = new Map();

  // "12px" -> {n: 12, u: 'px'}; calc() -> {calc: {unit: coefficient}}.
  function parseLength(text) {
    if (lengthCache.has(text)) return lengthCache.get(text);
    let out = null;
    const t = text.trim();
    const m = NUM.exec(t);
    if (m) {
      const u = m[2].toLowerCase();
      if (LENGTH_UNITS.has(u)) out = Object.freeze({ n: parseFloat(m[1]), u: u === '' ? 'px' : u });
    } else if (/^calc\(/i.test(t)) {
      out = parseCalc(t.slice(5, -1));
    }
    if (lengthCache.size > 8192) lengthCache.clear();
    lengthCache.set(text, out);
    return out;
  }

  // calc() as a linear combination of units: sums, differences, and
  // products/quotients by plain numbers.
  function parseCalc(text) {
    const tokens = text.match(/[+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?[a-z%]*|[-+*/()]/gi) || [];
    let i = 0;
    const expr = () => {
      let acc = term();
      while (tokens[i] === '+' || tokens[i] === '-') {
        const op = tokens[i++];
        const rhs = term();
        if (!acc || !rhs) return null;
        acc = combine(acc, rhs, op === '+' ? 1 : -1);
      }
      return acc;
    };
    const term = () => {
      let acc = factor();
      while (tokens[i] === '*' || tokens[i] === '/') {
        const op = tokens[i++];
        const rhs = factor();
        if (!acc || !rhs) return null;
        if (op === '*') {
          if (isPlain(rhs)) acc = scale(acc, rhs[''] ?? 0);
          else if (isPlain(acc)) acc = scale(rhs, acc[''] ?? 0);
          else return null;
        } else {
          if (!isPlain(rhs) || !rhs['']) return null;
          acc = scale(acc, 1 / rhs['']);
        }
      }
      return acc;
    };
    const factor = () => {
      const t = tokens[i++];
      if (t === '(') {
        const v = expr();
        i++;
        return v;
      }
      const m = t && NUM.exec(t);
      if (!m) return null;
      return { [m[2].toLowerCase()]: parseFloat(m[1]) };
    };
    const isPlain = (v) => Object.keys(v).every((k) => k === '');
    const scale = (v, k) => Object.fromEntries(Object.entries(v).map(([u, n]) => [u, n * k]));
    const combine = (a, b, sign) => {
      const out = { ...a };
      for (const [u, n] of Object.entries(b)) out[u === '' ? 'px' : u] = (out[u === '' ? 'px' : u] ?? 0) + sign * n;
      return out;
    };
    const v = expr();
    if (!v) return null;
    if (v[''] !== undefined) {
      v.px = (v.px ?? 0) + v[''];
      delete v[''];
    }
    return Object.freeze({ calc: Object.freeze(v) });
  }

  const UNIT_PX = { px: 1, mm: 96 / 25.4, cm: 96 / 2.54, in: 96, pt: 96 / 72, pc: 16, q: 96 / 101.6 };

  // A parsed length in px. `base` is what 100% is; `fontSize` what 1em is.
  function toPx(len, base, fontSize, doc) {
    if (len === null || typeof len !== 'object') return 0;
    if (len.calc) {
      let sum = 0;
      for (const u in len.calc) sum += unitPx(len.calc[u], u, base, fontSize, doc);
      return sum;
    }
    return unitPx(len.n, len.u, base, fontSize, doc);
  }

  function unitPx(n, u, base, fontSize, doc) {
    const k = UNIT_PX[u];
    if (k !== undefined) return n * k;
    switch (u) {
      case '%':
        return (n * (base || 0)) / 100;
      case 'em':
        return n * fontSize;
      case 'rem':
        return n * (doc ? doc._rootFontSize : 16);
      case 'ex':
      case 'ch':
        return n * fontSize * 0.5;
      case 'vw':
        return (n * (doc ? doc.__width : 0)) / 100;
      case 'vh':
        return (n * (doc ? doc.__height : 0)) / 100;
      case 'vmin':
        return (n * (doc ? Math.min(doc.__width, doc.__height) : 0)) / 100;
      case 'vmax':
        return (n * (doc ? Math.max(doc.__width, doc.__height) : 0)) / 100;
      default:
        return n;
    }
  }

  const isPercent = (len) => !!len && typeof len === 'object' && (len.u === '%' || (len.calc && len.calc['%'] !== undefined));

  // ---------------------------------------------------------------- transforms

  const transformCache = new Map();

  // A CSS or SVG transform list to [{f, a: [{n, u}]}], or 'none'. Lenient:
  // a missing closing parenthesis at the end is accepted (FBW's altitude
  // tape writes one), and the valid functions before an error are kept.
  function parseTransform(text) {
    if (transformCache.has(text)) return transformCache.get(text);
    const t = text.trim();
    let out;
    if (t === '' || t === 'none') out = 'none';
    else {
      out = [];
      const re = /([a-zA-Z0-9]+)\s*\(([^)]*)(\)|$)/g;
      let m;
      let last = 0;
      while ((m = re.exec(t))) {
        if (t.slice(last, m.index).replace(/[\s,]/g, '') !== '') break;
        const args = m[2]
          .split(/[\s,]+/)
          .filter(Boolean)
          .map((a) => {
            const n = NUM.exec(a);
            return n ? Object.freeze({ n: parseFloat(n[1]), u: n[2].toLowerCase() }) : null;
          });
        if (args.some((a) => a === null)) break;
        out.push(Object.freeze({ f: m[1], a: Object.freeze(args) }));
        last = re.lastIndex;
        if (m[3] === '') break;
      }
      out = Object.freeze(out);
    }
    if (transformCache.size > 8192) transformCache.clear();
    transformCache.set(text, out);
    return out;
  }

  const angle = (v) => {
    if (!v) return 0;
    switch (v.u) {
      case 'rad':
        return v.n;
      case 'grad':
        return (v.n * Math.PI) / 200;
      case 'turn':
        return v.n * 2 * Math.PI;
      default:
        return (v.n * Math.PI) / 180;
    }
  };

  const mul = (m, n) => [
    m[0] * n[0] + m[2] * n[1],
    m[1] * n[0] + m[3] * n[1],
    m[0] * n[2] + m[2] * n[3],
    m[1] * n[2] + m[3] * n[3],
    m[0] * n[4] + m[2] * n[5] + m[4],
    m[1] * n[4] + m[3] * n[5] + m[5],
  ];

  // A parsed transform list to a matrix [a b c d e f]. Percentages in
  // translations are of `w`/`h`.
  function transformMatrix(list, w, h, fontSize, doc) {
    let m = [1, 0, 0, 1, 0, 0];
    if (list === 'none') return m;
    const len = (v, base) => (v ? (v.u === '' ? v.n : toPx(v, base, fontSize, doc)) : 0);
    for (const { f, a } of list) {
      let n = null;
      switch (f) {
        case 'matrix':
          if (a.length === 6) n = a.map((v) => v.n);
          break;
        case 'translate':
          n = [1, 0, 0, 1, len(a[0], w), a.length > 1 ? len(a[1], h) : 0];
          break;
        case 'translateX':
          n = [1, 0, 0, 1, len(a[0], w), 0];
          break;
        case 'translateY':
          n = [1, 0, 0, 1, 0, len(a[0], h)];
          break;
        case 'translate3d':
          n = [1, 0, 0, 1, len(a[0], w), len(a[1], h)];
          break;
        case 'scale':
        case 'scale3d':
          n = [a[0].n, 0, 0, a.length > 1 ? a[1].n : a[0].n, 0, 0];
          break;
        case 'scaleX':
          n = [a[0].n, 0, 0, 1, 0, 0];
          break;
        case 'scaleY':
          n = [1, 0, 0, a[0].n, 0, 0];
          break;
        case 'rotate':
        case 'rotateZ': {
          const r = angle(a[0]);
          const c = Math.cos(r);
          const s = Math.sin(r);
          n = [c, s, -s, c, 0, 0];
          if (a.length === 3) {
            // SVG rotate(angle cx cy).
            const cx = a[1].n;
            const cy = a[2].n;
            n = mul(mul([1, 0, 0, 1, cx, cy], n), [1, 0, 0, 1, -cx, -cy]);
          }
          break;
        }
        case 'rotateX':
        case 'rotateY':
          // A 3D rotation about an axis in the screen plane: only its
          // projection (a scale along one axis) is drawn.
          if (a[0] && a[0].n !== 0) {
            warnOnce(`transform:${f}`, `transform ${f}() is drawn as a flat projection`);
            const c = Math.cos(angle(a[0]));
            n = f === 'rotateX' ? [1, 0, 0, c, 0, 0] : [c, 0, 0, 1, 0, 0];
          }
          break;
        case 'skewX':
          n = [1, 0, Math.tan(angle(a[0])), 1, 0, 0];
          break;
        case 'skewY':
          n = [1, Math.tan(angle(a[0])), 0, 1, 0, 0];
          break;
        case 'skew':
          n = [1, Math.tan(angle(a[1])), Math.tan(angle(a[0])), 1, 0, 0];
          break;
        default:
          warnOnce(`transform:${f}`, `transform function ${f}() is not supported (ignored)`);
      }
      if (n) m = mul(m, n);
    }
    return m;
  }

  // ---------------------------------------------------------------- property table

  // kind: how a value is computed. inh: inherited. layout: changing it can
  // move boxes (HTML layout); every property can change paint.
  const P = {};
  const def = (names, kind, init, opts = {}) => {
    for (const name of names.split(' ')) P[name] = { kind, init, inh: !!opts.inh, layout: !!opts.layout };
  };
  const L = { layout: true };
  const IL = { inh: true, layout: true };
  const I = { inh: true };

  def('color', 'color', 'black', I);
  def('font-size', 'fontSize', 'medium', IL);
  def('font-family', 'fontFamily', 'sans-serif', IL);
  def('font-weight', 'fontWeight', 'normal', IL);
  def('font-style', 'keyword', 'normal', IL);
  def('letter-spacing word-spacing', 'spacing', 'normal', IL);
  def('line-height', 'lineHeight', 'normal', IL);
  def('text-align', 'keyword', 'start', IL);
  def('text-transform', 'keyword', 'none', IL);
  def('white-space', 'keyword', 'normal', IL);
  def('visibility', 'keyword', 'visible', I);
  def('pointer-events', 'keyword', 'auto', I);
  def('fill', 'paint', 'black', I);
  def('stroke', 'paint', 'none', I);
  def('fill-opacity stroke-opacity', 'opacity', '1', I);
  def('fill-rule clip-rule', 'keyword', 'nonzero', I);
  def('stroke-width', 'length', '1', I);
  def('stroke-linecap', 'keyword', 'butt', I);
  def('stroke-linejoin', 'keyword', 'miter', I);
  def('stroke-miterlimit', 'number', '4', I);
  def('stroke-dasharray', 'dash', 'none', I);
  def('stroke-dashoffset', 'length', '0', I);
  def('paint-order', 'keyword', 'normal', I);
  def('text-anchor', 'keyword', 'start', I);
  def('dominant-baseline', 'keyword', 'auto', I);
  def('text-decoration-line', 'keyword', 'none', L);
  def('text-decoration-color', 'color', 'currentcolor');
  def('alignment-baseline', 'keyword', 'auto');
  def('display', 'keyword', 'inline', L);
  def('position', 'keyword', 'static', L);
  def('top right bottom left', 'length', 'auto', L);
  def('width height min-width min-height', 'length', 'auto', L);
  def('max-width max-height', 'length', 'none', L);
  def('margin-top margin-right margin-bottom margin-left', 'length', '0', L);
  def('padding-top padding-right padding-bottom padding-left', 'length', '0', L);
  def('border-top-width border-right-width border-bottom-width border-left-width', 'borderWidth', 'medium', L);
  def('border-top-style border-right-style border-bottom-style border-left-style', 'keyword', 'none', L);
  def('border-top-color border-right-color border-bottom-color border-left-color', 'color', 'currentcolor');
  def('border-top-left-radius border-top-right-radius border-bottom-right-radius border-bottom-left-radius', 'length', '0');
  def('outline-width', 'borderWidth', 'medium');
  def('outline-style', 'keyword', 'none');
  def('outline-color', 'color', 'currentcolor');
  def('outline-offset', 'length', '0');
  def('box-sizing', 'keyword', 'content-box', L);
  def('background-color', 'color', 'transparent');
  def('background-image', 'url', 'none');
  def('background-size', 'raw', 'auto');
  def('background-repeat background-position', 'raw', 'initial');
  def('opacity', 'opacity', '1');
  def('overflow-x overflow-y', 'keyword', 'visible', L);
  def('z-index', 'zIndex', 'auto');
  def('transform', 'transform', 'none');
  def('transform-origin', 'raw', 'initial');
  def('vertical-align', 'raw', 'baseline', L);
  def('flex-direction', 'keyword', 'row', L);
  def('flex-wrap', 'keyword', 'nowrap', L);
  def('justify-content align-content justify-items', 'keyword', 'normal', L);
  def('align-items', 'keyword', 'normal', L);
  def('align-self justify-self', 'keyword', 'auto', L);
  def('flex-grow', 'number', '0', L);
  def('flex-shrink', 'number', '1', L);
  def('flex-basis', 'length', 'auto', L);
  def('order', 'number', '0', L);
  def('row-gap column-gap', 'length', 'normal', L);
  def('grid-template-columns grid-template-rows', 'tracks', 'none', L);
  def('grid-auto-rows grid-auto-columns', 'tracks', 'auto', L);
  def('grid-auto-flow', 'keyword', 'row', L);
  def('grid-column-start grid-column-end grid-row-start grid-row-end', 'gridLine', 'auto', L);
  def('stop-color', 'color', 'black');
  def('stop-opacity', 'opacity', '1');
  def('clip-path', 'url', 'none');
  def('content', 'content', 'normal', L);
  def('animation-name', 'list', 'none');
  def('animation-duration animation-delay', 'timeList', '0s');
  def('animation-timing-function', 'list', 'ease');
  def('animation-iteration-count', 'list', '1');
  def('animation-direction', 'list', 'normal');
  def('animation-fill-mode', 'list', 'none');
  def('animation-play-state', 'list', 'running');
  def('transition-property', 'raw', 'all');
  def('transition-duration transition-delay', 'timeList', '0s');
  def('transition-timing-function', 'raw', 'ease');
  def('box-shadow filter', 'raw', 'none');
  def('float', 'keyword', 'none', L);
  def('text-overflow', 'keyword', 'clip');

  // Accepted and deliberately without visual effect here.
  const NO_EFFECT = new Set([
    'cursor', 'user-select', 'appearance', 'resize', 'tab-size', 'list-style', 'list-style-type', 'list-style-position',
    'text-size-adjust', 'will-change', 'touch-action', 'text-rendering', 'shape-rendering', 'image-rendering',
    'table-layout', 'border-collapse', 'border-spacing', 'scroll-behavior', 'caret-color', 'outline-style',
    'isolation', 'backface-visibility', 'contain', 'color-interpolation', 'color-interpolation-filters',
    'font-display', 'unicode-range', 'src', 'speak', 'overflow-anchor', 'scrollbar-width', 'text-decoration-style',
    'text-decoration-thickness', 'enable-background', 'color-rendering', 'font-smoothing',
  ]);

  // SVG presentation attributes (attribute name = property name).
  const PRESENTATION = new Set([
    'fill', 'fill-opacity', 'fill-rule', 'stroke', 'stroke-width', 'stroke-opacity', 'stroke-linecap', 'stroke-linejoin',
    'stroke-miterlimit', 'stroke-dasharray', 'stroke-dashoffset', 'opacity', 'visibility', 'display', 'font-family',
    'font-size', 'font-weight', 'font-style', 'letter-spacing', 'word-spacing', 'text-anchor', 'dominant-baseline',
    'alignment-baseline', 'clip-path', 'clip-rule', 'color', 'stop-color', 'stop-opacity', 'paint-order', 'overflow',
    'pointer-events', 'text-decoration', 'white-space',
  ]);

  const INHERITED = Object.keys(P).filter((k) => P[k].inh);

  // ---------------------------------------------------------------- shorthands

  const box4 = (v) => {
    const t = splitTokens(v);
    const top = t[0];
    const right = t[1] ?? top;
    const bottom = t[2] ?? top;
    const left = t[3] ?? right;
    return [top, right, bottom, left];
  };
  const SIDES = ['top', 'right', 'bottom', 'left'];
  const BORDER_STYLES = new Set(['none', 'hidden', 'dotted', 'dashed', 'solid', 'double', 'groove', 'ridge', 'inset', 'outset']);
  const isLengthish = (t) => parseLength(t) !== null || ['thin', 'medium', 'thick'].includes(t);

  const borderParts = (v) => {
    let width = 'medium';
    let style = 'none';
    let color = 'currentcolor';
    for (const t of splitTokens(v)) {
      if (BORDER_STYLES.has(t)) style = t;
      else if (isLengthish(t)) width = t;
      else color = t;
    }
    return [width, style, color];
  };

  const TIMING_WORDS = new Set(['ease', 'linear', 'ease-in', 'ease-out', 'ease-in-out', 'step-start', 'step-end']);

  // Expand shorthands to longhands: [[property, value], ...].
  function expand(prop, v) {
    switch (prop) {
      case 'margin':
      case 'padding':
        return box4(v).map((x, i) => [`${prop}-${SIDES[i]}`, x]);
      case 'inset':
        return box4(v).map((x, i) => [SIDES[i], x]);
      case 'border-width':
      case 'border-style':
      case 'border-color': {
        const part = prop.slice(7);
        return box4(v).map((x, i) => [`border-${SIDES[i]}-${part}`, x]);
      }
      case 'border': {
        const [w, s, c] = borderParts(v);
        return SIDES.flatMap((side) => [
          [`border-${side}-width`, w],
          [`border-${side}-style`, s],
          [`border-${side}-color`, c],
        ]);
      }
      case 'border-top':
      case 'border-right':
      case 'border-bottom':
      case 'border-left': {
        const [w, s, c] = borderParts(v);
        return [
          [`${prop}-width`, w],
          [`${prop}-style`, s],
          [`${prop}-color`, c],
        ];
      }
      case 'border-radius': {
        const [tl, tr, br, bl] = box4(v.split('/')[0]);
        return [
          ['border-top-left-radius', tl],
          ['border-top-right-radius', tr],
          ['border-bottom-right-radius', br],
          ['border-bottom-left-radius', bl],
        ];
      }
      case 'outline': {
        const [w, s, c] = borderParts(v);
        return [
          ['outline-width', w],
          ['outline-style', s],
          ['outline-color', c],
        ];
      }
      case 'background': {
        let color = 'transparent';
        let image = 'none';
        for (const t of splitTokens(v.split('/')[0])) {
          if (/^url\(/i.test(t) || /gradient\(/i.test(t)) image = t;
          else if (t === 'none') image = 'none';
          else if (parseColor(t) !== null) color = t;
          else if (!/^(no-repeat|repeat|repeat-x|repeat-y|center|top|bottom|left|right|scroll|fixed|local|border-box|padding-box|content-box)$/.test(t) && parseLength(t) === null) {
            warnOnce(`background:${t}`, `background value '${t}' is not supported (ignored)`);
          }
        }
        return [
          ['background-color', color],
          ['background-image', image],
        ];
      }
      case 'flex': {
        const t = splitTokens(v);
        if (t.length === 1 && t[0] === 'none') return [['flex-grow', '0'], ['flex-shrink', '0'], ['flex-basis', 'auto']];
        if (t.length === 1 && t[0] === 'auto') return [['flex-grow', '1'], ['flex-shrink', '1'], ['flex-basis', 'auto']];
        if (t.length === 1 && t[0] === 'initial') return [['flex-grow', '0'], ['flex-shrink', '1'], ['flex-basis', 'auto']];
        let grow = '1';
        let shrink = '1';
        let basis = '0%';
        const nums = t.filter((x) => /^[+-]?(\d+\.?\d*|\.\d+)$/.test(x));
        const others = t.filter((x) => !/^[+-]?(\d+\.?\d*|\.\d+)$/.test(x));
        if (nums[0] !== undefined) grow = nums[0];
        if (nums[1] !== undefined) shrink = nums[1];
        if (others[0] !== undefined) basis = others[0];
        else if (nums.length === 3) basis = nums[2];
        return [['flex-grow', grow], ['flex-shrink', shrink], ['flex-basis', basis]];
      }
      case 'flex-flow': {
        const out = [];
        for (const t of splitTokens(v)) out.push([/wrap/.test(t) ? 'flex-wrap' : 'flex-direction', t]);
        return out;
      }
      case 'gap':
      case 'grid-gap': {
        const t = splitTokens(v);
        return [['row-gap', t[0]], ['column-gap', t[1] ?? t[0]]];
      }
      case 'grid-column-gap':
        return [['column-gap', v]];
      case 'grid-row-gap':
        return [['row-gap', v]];
      case 'grid-column':
      case 'grid-row': {
        const [a, b] = v.split('/').map((x) => x.trim());
        return [[`${prop}-start`, a], [`${prop}-end`, b ?? (a.startsWith('span') ? 'auto' : 'auto')]];
      }
      case 'place-content':
      case 'place-items':
      case 'place-self': {
        const t = splitTokens(v);
        const kind = prop.slice(6);
        return [[`align-${kind}`, t[0]], [`justify-${kind}`, t[1] ?? t[0]]];
      }
      case 'overflow': {
        const t = splitTokens(v);
        return [['overflow-x', t[0]], ['overflow-y', t[1] ?? t[0]]];
      }
      case 'text-decoration': {
        const out = [];
        for (const t of splitTokens(v)) {
          if (/^(none|underline|overline|line-through)$/.test(t)) out.push(['text-decoration-line', t]);
          else if (parseColor(t) !== null) out.push(['text-decoration-color', t]);
        }
        return out;
      }
      case 'font': {
        const m = /^(.*?)([\d.]+[a-z%]*|smaller|larger|small|medium|large|x-large|x-small|xx-large|xx-small)(\s*\/\s*([^\s]+))?\s+(.+)$/i.exec(v.trim());
        if (!m) {
          warnOnce(`font:${v}`, `font shorthand '${v}' is not understood (ignored)`);
          return [];
        }
        const out = [['font-size', m[2]], ['font-family', m[5]], ['font-style', 'normal'], ['font-weight', 'normal'], ['line-height', m[4] ?? 'normal']];
        for (const t of splitTokens(m[1])) {
          if (t === 'italic' || t === 'oblique') out[2][1] = t;
          else if (/^(bold|bolder|lighter|\d00)$/.test(t)) out[3][1] = t;
        }
        return out;
      }
      case 'animation': {
        const cols = { name: [], duration: [], timing: [], delay: [], count: [], direction: [], fill: [], play: [] };
        for (const one of splitCommas(v)) {
          let name = 'none';
          let duration = '0s';
          let delay = '0s';
          let timing = 'ease';
          let count = '1';
          let direction = 'normal';
          let fill = 'none';
          let play = 'running';
          let times = 0;
          for (const t of splitTokens(one)) {
            if (/^[+-]?(\d+\.?\d*|\.\d+)m?s$/.test(t)) {
              if (times++ === 0) duration = t;
              else delay = t;
            } else if (TIMING_WORDS.has(t) || /^(steps|cubic-bezier)\(/.test(t)) timing = t;
            else if (t === 'infinite' || /^(\d+\.?\d*|\.\d+)$/.test(t)) count = t;
            else if (/^(normal|reverse|alternate|alternate-reverse)$/.test(t)) direction = t;
            else if (/^(forwards|backwards|both)$/.test(t)) fill = t;
            else if (/^(running|paused)$/.test(t)) play = t;
            else name = t;
          }
          cols.name.push(name);
          cols.duration.push(duration);
          cols.timing.push(timing);
          cols.delay.push(delay);
          cols.count.push(count);
          cols.direction.push(direction);
          cols.fill.push(fill);
          cols.play.push(play);
        }
        return [
          ['animation-name', cols.name.join(', ')],
          ['animation-duration', cols.duration.join(', ')],
          ['animation-timing-function', cols.timing.join(', ')],
          ['animation-delay', cols.delay.join(', ')],
          ['animation-iteration-count', cols.count.join(', ')],
          ['animation-direction', cols.direction.join(', ')],
          ['animation-fill-mode', cols.fill.join(', ')],
          ['animation-play-state', cols.play.join(', ')],
        ];
      }
      case 'transition': {
        const props = [];
        const durations = [];
        for (const one of splitCommas(v)) {
          let times = 0;
          let property = 'all';
          let duration = '0s';
          for (const t of splitTokens(one)) {
            if (/^[+-]?(\d+\.?\d*|\.\d+)m?s$/.test(t)) {
              if (times++ === 0) duration = t;
            } else if (!(TIMING_WORDS.has(t) || /^(steps|cubic-bezier)\(/.test(t))) property = t;
          }
          props.push(property);
          durations.push(duration);
        }
        return [['transition-property', props.join(', ')], ['transition-duration', durations.join(', ')]];
      }
      default:
        return [[prop, v]];
    }
  }

  // ---------------------------------------------------------------- declarations and sheets

  const stripComments = (text) => text.replace(/\/\*[\s\S]*?\*\//g, '');

  // "a: b; c: d !important" -> [[prop, value, important], ...] with
  // shorthands expanded.
  function parseDeclarations(text) {
    const out = [];
    let depth = 0;
    let quote = null;
    let start = 0;
    const flush = (end) => {
      const decl = text.slice(start, end);
      const colon = decl.indexOf(':');
      if (colon > 0) {
        const prop = decl.slice(0, colon).trim();
        let value = decl.slice(colon + 1).trim();
        let important = false;
        const bang = /!\s*important\s*$/i.exec(value);
        if (bang) {
          important = true;
          value = value.slice(0, bang.index).trim();
        }
        if (prop && value !== '') {
          const name = prop.startsWith('--') ? prop : prop.toLowerCase();
          for (const [p, v] of name.startsWith('--') ? [[name, value]] : expand(name, value)) out.push([p, v, important]);
        }
      }
      start = end + 1;
    };
    for (let i = 0; i < text.length; i++) {
      const ch = text[i];
      if (quote) {
        if (ch === quote) quote = null;
      } else if (ch === '"' || ch === "'") quote = ch;
      else if (ch === '(') depth++;
      else if (ch === ')') depth--;
      else if (ch === ';' && depth === 0) flush(i);
    }
    flush(text.length);
    return out;
  }

  // Find the index just past the block that opens at text[open] === '{'.
  const blockEnd = (text, open) => {
    let depth = 0;
    let quote = null;
    for (let i = open; i < text.length; i++) {
      const ch = text[i];
      if (quote) {
        if (ch === quote) quote = null;
      } else if (ch === '"' || ch === "'") quote = ch;
      else if (ch === '{') depth++;
      else if (ch === '}' && --depth === 0) return i + 1;
    }
    return text.length;
  };

  const unquote = (s) => s.trim().replace(/^(['"])(.*)\1$/, '$2');

  // A stylesheet: {rules: [{selectors, decls}], fontFaces, keyframes}.
  function parseStyleSheet(text) {
    const sheet = { rules: [], fontFaces: [], keyframes: new Map() };
    const src = stripComments(text);
    parseBlockInto(src, sheet);
    return sheet;
  }

  function parseBlockInto(src, sheet) {
    let i = 0;
    while (i < src.length) {
      const open = src.indexOf('{', i);
      const semi = src.indexOf(';', i);
      if (open < 0) break;
      const prelude = src.slice(i, open).trim();
      if (prelude.startsWith('@') && semi >= 0 && semi < open) {
        // A statement at-rule (@import, @charset).
        const statement = src.slice(i, semi).trim();
        if (!/^@charset/i.test(statement)) warnOnce(`at:${statement}`, `${statement} is not supported (ignored)`);
        i = semi + 1;
        continue;
      }
      const end = blockEnd(src, open);
      const body = src.slice(open + 1, end - 1);
      i = end;
      if (prelude.startsWith('@')) {
        const at = /^@([-\w]+)\s*(.*)$/s.exec(prelude);
        const name = at ? at[1].toLowerCase() : '';
        if (name === 'font-face') {
          const face = { family: null, src: null, weight: 'normal', style: 'normal' };
          for (const [p, v] of parseDeclarations(body)) {
            if (p === 'font-family') face.family = unquote(v);
            else if (p === 'src') {
              const m = /url\(\s*(['"]?)([^'")]+)\1\s*\)/i.exec(v);
              if (m) face.src = m[2];
            } else if (p === 'font-weight') face.weight = v;
            else if (p === 'font-style') face.style = v;
          }
          if (face.family && face.src) sheet.fontFaces.push(face);
        } else if (name === 'keyframes' || name === '-webkit-keyframes') {
          sheet.keyframes.set(unquote(at[2]), parseKeyframes(body));
        } else {
          warnOnce(`at:@${name}`, `@${name} rules are not supported (their contents are ignored)`);
        }
        continue;
      }
      let selectors;
      try {
        selectors = D.selectors.parseList(prelude);
      } catch {
        warnOnce(`selector:${prelude}`, `selector '${prelude}' is not supported (rule ignored)`);
        continue;
      }
      sheet.rules.push({ selectors, decls: parseDeclarations(body) });
    }
  }

  function parseKeyframes(body) {
    const frames = [];
    let i = 0;
    while (i < body.length) {
      const open = body.indexOf('{', i);
      if (open < 0) break;
      const prelude = body.slice(i, open).trim();
      const end = blockEnd(body, open);
      const decls = parseDeclarations(body.slice(open + 1, end - 1));
      const offsets = prelude
        .split(',')
        .map((s) => s.trim().toLowerCase())
        .map((s) => (s === 'from' ? 0 : s === 'to' ? 1 : parseFloat(s) / 100))
        .filter((x) => !Number.isNaN(x));
      frames.push({ offsets, decls });
      i = end;
    }
    return { frames, tracks: null };
  }

  // ---------------------------------------------------------------- computed values

  const valueCache = new Map();
  const cached = (kind, text, fn) => {
    const key = `${kind}\u0000${text}`;
    let v = valueCache.get(key);
    if (v === undefined) {
      v = fn(text);
      if (valueCache.size > 16384) valueCache.clear();
      valueCache.set(key, v === undefined ? null : v);
    }
    return v;
  };

  const FONT_SIZES = { 'xx-small': 9, 'x-small': 10, small: 13, medium: 16, large: 18, 'x-large': 24, 'xx-large': 32 };
  const INVALID = Symbol('invalid');

  const parseTimeValue = (t) => {
    const m = /^([+-]?(?:\d+\.?\d*|\.\d+))(m?s)$/.exec(t.trim());
    if (!m) return 0;
    return m[2] === 's' ? parseFloat(m[1]) * 1000 : parseFloat(m[1]);
  };

  const parseTracks = (text) => {
    const t = text.trim();
    if (t === 'none') return 'none';
    const out = [];
    const tokens = splitTokens(t);
    const one = (tok) => {
      if (tok === 'auto' || tok === 'min-content' || tok === 'max-content') return { auto: true };
      if (/fr$/.test(tok)) return { fr: parseFloat(tok) };
      const mm = /^minmax\((.*)\)$/.exec(tok);
      if (mm) {
        const [a, b] = splitCommas(mm[1]);
        const hi = one(b);
        const lo = one(a);
        return hi.fr !== undefined ? { fr: hi.fr, min: lo.len } : hi.len ? { len: hi.len, min: lo.len } : { auto: true, min: lo.len };
      }
      const len = parseLength(tok);
      if (len) return { len };
      warnOnce(`grid-track:${tok}`, `grid track '${tok}' is not supported (auto used)`);
      return { auto: true };
    };
    for (const tok of tokens) {
      const rep = /^repeat\((.*)\)$/.exec(tok);
      if (rep) {
        const [count, ...rest] = splitCommas(rep[1]);
        const n = parseInt(count, 10);
        if (!Number.isFinite(n)) {
          warnOnce(`grid-repeat:${count}`, `grid repeat(${count}, ...) is not supported`);
          continue;
        }
        const inner = splitTokens(rest.join(','));
        for (let k = 0; k < n; k++) for (const x of inner) out.push(one(x));
      } else if (tok.startsWith('[')) {
        warnOnce('grid-line-names', 'named grid lines are not supported (ignored)');
      } else {
        out.push(one(tok));
      }
    }
    return Object.freeze(out);
  };

  // Compute one property's value from its specified string. `cs` is the
  // style being built (color and font-size are already final in it).
  function computeValue(prop, text, cs, parentCs, doc) {
    const spec = P[prop];
    switch (spec.kind) {
      case 'color': {
        const c = parseColor(text);
        if (c === null) return INVALID;
        return c === CURRENT ? (prop === 'color' ? (parentCs ? parentCs.color : [0, 0, 0, 1]) : CURRENT) : c;
      }
      case 'paint':
        return cached('paint', text, (t) => {
          const s = t.trim();
          if (s === 'none') return 'none';
          const url = /^url\(\s*['"]?#([^'")]+)['"]?\s*\)\s*(.*)$/i.exec(s);
          if (url) return Object.freeze({ url: url[1], fallback: url[2] ? parseColor(url[2]) : null });
          const c = parseColor(s);
          return c === null ? INVALID : c;
        });
      case 'fontSize': {
        const t = text.trim();
        const parentSize = parentCs ? parentCs['font-size'] : 16;
        if (FONT_SIZES[t] !== undefined) return FONT_SIZES[t];
        if (t === 'smaller') return parentSize / 1.2;
        if (t === 'larger') return parentSize * 1.2;
        const len = parseLength(t);
        if (!len) return INVALID;
        return toPx(len, parentSize, parentSize, doc);
      }
      case 'fontFamily':
        return cached('family', text, (t) => Object.freeze(splitCommas(t).map(unquote)));
      case 'fontWeight': {
        const t = text.trim();
        if (t === 'normal') return 400;
        if (t === 'bold') return 700;
        const parentWeight = parentCs ? parentCs['font-weight'] : 400;
        if (t === 'bolder') return parentWeight < 600 ? 700 : 900;
        if (t === 'lighter') return parentWeight > 500 ? 400 : 100;
        const n = parseFloat(t);
        return Number.isFinite(n) ? n : INVALID;
      }
      case 'spacing': {
        const t = text.trim();
        if (t === 'normal') return 0;
        const len = parseLength(t);
        return len ? toPx(len, 0, cs['font-size'], doc) : INVALID;
      }
      case 'lineHeight':
        return cached('lh', text, (t) => {
          const s = t.trim();
          if (s === 'normal') return 'normal';
          if (/^[+-]?(\d+\.?\d*|\.\d+)$/.test(s)) return Object.freeze({ mult: parseFloat(s) });
          return parseLength(s) ?? INVALID;
        });
      case 'length': {
        const t = text.trim();
        if (/^[a-z-]+$/.test(t)) return t;
        return parseLength(t) ?? INVALID;
      }
      case 'borderWidth': {
        const t = text.trim();
        if (t === 'thin') return 1;
        if (t === 'medium') return 3;
        if (t === 'thick') return 5;
        const len = parseLength(t);
        return len ? toPx(len, 0, cs['font-size'], doc) : INVALID;
      }
      case 'number': {
        const n = parseFloat(text);
        return Number.isFinite(n) ? n : INVALID;
      }
      case 'opacity': {
        const t = text.trim();
        const n = t.endsWith('%') ? parseFloat(t) / 100 : parseFloat(t);
        return Number.isFinite(n) ? clamp01(n) : INVALID;
      }
      case 'zIndex': {
        const t = text.trim();
        if (t === 'auto') return 'auto';
        const n = parseInt(t, 10);
        return Number.isFinite(n) ? n : INVALID;
      }
      case 'keyword':
        return text.trim().toLowerCase();
      case 'raw':
        return text.trim();
      case 'dash':
        return cached('dash', text, (t) => {
          const s = t.trim();
          if (s === 'none' || s === '') return 'none';
          const parts = s.split(/[\s,]+/).filter(Boolean).map(parseLength);
          if (parts.some((p) => p === null)) return INVALID;
          return Object.freeze(parts.length % 2 ? [...parts, ...parts] : parts);
        });
      case 'transform':
        return parseTransform(text);
      case 'url':
        return cached('url', text, (t) => {
          const s = t.trim();
          if (s === 'none') return 'none';
          const m = /^url\(\s*(['"]?)(.*?)\1\s*\)$/is.exec(s);
          if (m) return Object.freeze({ url: m[2] });
          if (/gradient\(/i.test(s)) {
            warnOnce('css-gradient', `CSS gradients are not supported: ${s.slice(0, 40)}`);
            return 'none';
          }
          warnOnce(`url-value:${s}`, `'${s}' is not a supported url value`);
          return 'none';
        });
      case 'tracks':
        return cached('tracks', text, parseTracks);
      case 'gridLine':
        return cached('gridline', text, (t) => {
          const s = t.trim();
          if (s === 'auto') return 'auto';
          const span = /^span\s+(\d+)$/.exec(s);
          if (span) return Object.freeze({ span: parseInt(span[1], 10) });
          const n = parseInt(s, 10);
          if (Number.isFinite(n)) return Object.freeze({ line: n });
          warnOnce(`grid-line:${s}`, `grid line '${s}' is not supported`);
          return 'auto';
        });
      case 'content':
        return cached('content', text, (t) => {
          const s = t.trim();
          if (s === 'none' || s === 'normal') return s;
          const parts = [];
          for (const tok of splitTokens(s)) {
            if (/^["']/.test(tok)) parts.push({ text: unquote(tok) });
            else {
              const attr = /^attr\(\s*([-\w]+)\s*\)$/.exec(tok);
              if (attr) parts.push({ attr: attr[1] });
              else warnOnce(`content:${tok}`, `content value '${tok}' is not supported`);
            }
          }
          return Object.freeze(parts);
        });
      case 'list':
        return cached('list', text, (t) => Object.freeze(splitCommas(t)));
      case 'timeList':
        return cached('time', text, (t) => Object.freeze(splitCommas(t).map(parseTimeValue)));
      default:
        return text;
    }
  }

  // Initial values, computed once.
  const INITIAL = Object.create(null);
  {
    const seed = { 'font-size': 16, color: Object.freeze([0, 0, 0, 1]) };
    INITIAL.color = seed.color;
    INITIAL['font-size'] = 16;
    for (const prop of Object.keys(P)) {
      if (prop === 'color' || prop === 'font-size') continue;
      const v = computeValue(prop, P[prop].init, seed, null, null);
      INITIAL[prop] = v === INVALID ? P[prop].init : v;
    }
  }

  // ---------------------------------------------------------------- cascade

  // Replace var(--name, fallback) using custom properties in `cs`.
  function substituteVars(text, cs) {
    let guard = 0;
    while (text.includes('var(') && guard++ < 16) {
      const i = text.indexOf('var(');
      let depth = 0;
      let j = i + 3;
      for (; j < text.length; j++) {
        if (text[j] === '(') depth++;
        else if (text[j] === ')' && --depth === 0) break;
      }
      const inner = text.slice(i + 4, j);
      const comma = inner.indexOf(',');
      const name = (comma < 0 ? inner : inner.slice(0, comma)).trim();
      const fallback = comma < 0 ? '' : inner.slice(comma + 1).trim();
      const value = cs[name] !== undefined ? cs[name] : fallback;
      text = text.slice(0, i) + value + text.slice(j + 1);
    }
    return text;
  }

  const SVG_NS = 'http://www.w3.org/2000/svg';

  // Build the computed style of `el` (or of its ::before/::after when
  // `pseudo` is given) from the matched declarations, presentation
  // attributes, inline style, animations and the parent's style.
  //
  // `matched` is the list of declaration arrays in cascade order (lowest
  // first). `animated` maps property -> value string from running
  // animations, or null.
  function computeStyle(el, parentCs, doc, matched, pseudo, animated) {
    const cs = Object.create(INITIAL);
    if (parentCs) {
      for (let i = 0; i < INHERITED.length; i++) cs[INHERITED[i]] = parentCs[INHERITED[i]];
      for (const k in parentCs) if (k.charCodeAt(0) === 45 && k.charCodeAt(1) === 45) cs[k] = parentCs[k];
    }
    const specified = Object.create(null);
    const important = Object.create(null);
    if (!pseudo && el.namespaceURI === SVG_NS && el._attrs.size) {
      for (const [name, value] of el._attrs) {
        if (!PRESENTATION.has(name)) continue;
        if (name === 'text-decoration') specified['text-decoration-line'] = value;
        else if (name === 'overflow') specified['overflow-x'] = specified['overflow-y'] = value;
        else specified[name] = value;
      }
    }
    for (let r = 0; r < matched.length; r++) {
      const decls = matched[r];
      for (let d = 0; d < decls.length; d++) {
        const decl = decls[d];
        if (decl[2]) continue;
        specified[decl[0]] = decl[1];
      }
    }
    const inline = !pseudo && el._style ? el._style._decls : null;
    if (inline) for (const [p, d] of inline) if (!d.important) specified[p] = d.value;
    if (animated) for (const p in animated) specified[p] = animated[p];
    for (let r = 0; r < matched.length; r++) {
      const decls = matched[r];
      for (let d = 0; d < decls.length; d++) {
        const decl = decls[d];
        if (!decl[2]) continue;
        specified[decl[0]] = decl[1];
        important[decl[0]] = true;
      }
    }
    if (inline) {
      for (const [p, d] of inline) {
        if (!d.important) continue;
        specified[p] = d.value;
        important[p] = true;
      }
    }

    // Custom properties first, then the properties others depend on.
    for (const p in specified) if (p.charCodeAt(0) === 45 && p.charCodeAt(1) === 45) cs[p] = specified[p];
    const apply = (p) => {
      let text = specified[p];
      if (text === undefined) return;
      if (text.includes('var(')) text = substituteVars(text, cs);
      const keyword = text.trim().toLowerCase();
      if (keyword === 'inherit') {
        cs[p] = parentCs ? parentCs[p] : INITIAL[p];
        return;
      }
      if (keyword === 'initial') {
        cs[p] = INITIAL[p];
        return;
      }
      if (keyword === 'unset' || keyword === 'revert') {
        cs[p] = P[p].inh && parentCs ? parentCs[p] : INITIAL[p];
        return;
      }
      const v = computeValue(p, text, cs, parentCs, doc);
      if (v === INVALID) {
        warnOnce(`invalid:${p}:${text}`, `invalid value '${text}' for ${p} on <${el.localName}> (ignored)`);
        return;
      }
      cs[p] = v;
    };
    apply('color');
    apply('font-size');
    for (const p in specified) {
      if (p === 'color' || p === 'font-size' || (p.charCodeAt(0) === 45 && p.charCodeAt(1) === 45)) continue;
      if (P[p] === undefined) {
        if (!NO_EFFECT.has(p) && p.charCodeAt(0) !== 45) {
          warnOnce(`prop:${p}`, `CSS property ${p} is not supported (first seen on <${el.localName}${el.id ? `#${el.id}` : ''}>)`);
        }
        continue;
      }
      apply(p);
    }
    if (specified['transition-duration'] !== undefined && cs['transition-duration'].some((d) => d > 0)) {
      warnOnce(`transition:${el.localName}`, `CSS transitions are applied immediately (on <${el.localName}${el.className ? ` class="${el.getAttribute('class')}"` : ''}>)`);
    }
    if (specified['box-shadow'] !== undefined && cs['box-shadow'] !== 'none') warnOnce('box-shadow', 'box-shadow is not drawn');
    if (specified.filter !== undefined && cs.filter !== 'none') warnOnce('filter', 'CSS filter is not drawn');
    if (specified.float !== undefined && cs.float !== 'none') warnOnce('float', 'float is laid out as a block');
    return cs;
  }

  // Whether two computed styles differ, and how: {paint, layout, inherited}.
  function diffStyles(a, b) {
    if (!a) return { paint: true, layout: true, inherited: true };
    let paint = false;
    let layout = false;
    let inherited = false;
    const note = (k) => {
      paint = true;
      const spec = P[k];
      if (spec) {
        if (spec.layout) layout = true;
        if (spec.inh) inherited = true;
      } else if (k.charCodeAt(0) === 45) {
        inherited = true;
      }
    };
    for (const k of Object.keys(b)) if (a[k] !== b[k] && !sameValue(a[k], b[k])) note(k);
    for (const k of Object.keys(a)) if (!Object.prototype.hasOwnProperty.call(b, k) && a[k] !== b[k] && !sameValue(a[k], b[k])) note(k);
    return paint ? { paint, layout, inherited } : null;
  }

  // Colours and small arrays built at compute time are compared by value.
  const sameValue = (x, y) => {
    if (Array.isArray(x) && Array.isArray(y) && x.length === y.length) {
      for (let i = 0; i < x.length; i++) if (x[i] !== y[i]) return false;
      return true;
    }
    return false;
  };

  // ---------------------------------------------------------------- animations

  const cubicBezier = (x1, y1, x2, y2) => (t) => {
    // Solve x(u) = t for u by Newton's method with bisection fallback.
    const bx = (u) => 3 * (1 - u) * (1 - u) * u * x1 + 3 * (1 - u) * u * u * x2 + u * u * u;
    const by = (u) => 3 * (1 - u) * (1 - u) * u * y1 + 3 * (1 - u) * u * u * y2 + u * u * u;
    let lo = 0;
    let hi = 1;
    let u = t;
    for (let i = 0; i < 20; i++) {
      const x = bx(u) - t;
      if (Math.abs(x) < 1e-5) break;
      if (x > 0) hi = u;
      else lo = u;
      u = (lo + hi) / 2;
    }
    return by(u);
  };

  const timingCache = new Map();
  function timingFunction(text) {
    const t = (text || 'ease').trim();
    if (timingCache.has(t)) return timingCache.get(t);
    let fn;
    const steps = /^steps\(\s*(\d+)\s*(?:,\s*([-a-z]+))?\s*\)$/.exec(t);
    const bezier = /^cubic-bezier\(([^)]*)\)$/.exec(t);
    if (t === 'linear') fn = (x) => x;
    else if (t === 'ease') fn = cubicBezier(0.25, 0.1, 0.25, 1);
    else if (t === 'ease-in') fn = cubicBezier(0.42, 0, 1, 1);
    else if (t === 'ease-out') fn = cubicBezier(0, 0, 0.58, 1);
    else if (t === 'ease-in-out') fn = cubicBezier(0.42, 0, 0.58, 1);
    else if (t === 'step-start') fn = (x) => (x > 0 ? 1 : 0);
    else if (t === 'step-end') fn = (x) => (x >= 1 ? 1 : 0);
    else if (steps) {
      const n = parseInt(steps[1], 10);
      const start = steps[2] === 'start' || steps[2] === 'jump-start';
      fn = (x) => (x >= 1 ? 1 : Math.min(1, Math.floor(x * n + (start ? 1 : 0)) / n));
    } else if (bezier) {
      const [a, b, c, d] = bezier[1].split(',').map(parseFloat);
      fn = cubicBezier(a, b, c, d);
    } else {
      warnOnce(`timing:${t}`, `timing function '${t}' is not supported (linear used)`);
      fn = (x) => x;
    }
    timingCache.set(t, fn);
    return fn;
  }

  // Per-property keyframe tracks of a @keyframes rule, built on first use.
  function tracksOf(keyframes) {
    if (keyframes.tracks) return keyframes.tracks;
    const tracks = new Map();
    for (const frame of keyframes.frames) {
      let timing = null;
      for (const [p, v] of frame.decls) if (p === 'animation-timing-function') timing = v;
      for (const offset of frame.offsets) {
        for (const [p, v] of frame.decls) {
          if (p.startsWith('animation-')) continue;
          if (!tracks.has(p)) tracks.set(p, []);
          tracks.get(p).push({ o: offset, v, timing });
        }
      }
    }
    for (const list of tracks.values()) list.sort((a, b) => a.o - b.o);
    keyframes.tracks = tracks;
    return tracks;
  }

  const lerp = (a, b, t) => a + (b - a) * t;

  // Interpolate a property between two specified strings.
  function interpolate(prop, from, to, t) {
    if (t <= 0) return from;
    if (t >= 1) return to;
    const kind = P[prop] ? P[prop].kind : null;
    if (kind === 'opacity' || kind === 'number') {
      const a = parseFloat(from);
      const b = parseFloat(to);
      if (Number.isFinite(a) && Number.isFinite(b)) return String(lerp(a, b, t));
    } else if (kind === 'color' || kind === 'paint') {
      const a = parseColor(from);
      const b = parseColor(to);
      if (Array.isArray(a) && Array.isArray(b)) {
        const c = a.map((x, i) => lerp(x, b[i], t));
        return `rgba(${c[0] * 255},${c[1] * 255},${c[2] * 255},${c[3]})`;
      }
    } else if (kind === 'length' || kind === 'spacing') {
      const a = parseLength(from);
      const b = parseLength(to);
      if (a && b && !a.calc && !b.calc && a.u === b.u) return `${lerp(a.n, b.n, t)}${a.u}`;
    } else if (prop === 'visibility') {
      // visible wins over hidden for any t in (0, 1).
      if (from.trim() === 'visible' || to.trim() === 'visible') return 'visible';
    }
    return t < 0.5 ? from : to;
  }

  // The animated property values of `el` at `now`, or null. `state` is the
  // element's animation bookkeeping ({starts: Map name -> start ms}).
  // Returns {values, active} where active says whether anything will still
  // change after now.
  function evaluateAnimations(cs, state, now, doc, underlying) {
    const names = cs['animation-name'];
    if (names.length === 1 && names[0] === 'none') {
      state.starts.clear();
      return null;
    }
    const seen = new Set();
    let values = null;
    let active = false;
    const at = (list, i) => list[i % list.length];
    for (let i = 0; i < names.length; i++) {
      const name = names[i].trim();
      if (name === 'none') continue;
      seen.add(name);
      const keyframes = doc._keyframes.get(name);
      if (!keyframes) {
        warnOnce(`keyframes:${name}`, `@keyframes ${name} is not defined`);
        continue;
      }
      if (!state.starts.has(name)) state.starts.set(name, now);
      const duration = at(cs['animation-duration'], i);
      const delay = at(cs['animation-delay'], i);
      const countText = at(cs['animation-iteration-count'], i).trim();
      const count = countText === 'infinite' ? Infinity : parseFloat(countText);
      const direction = at(cs['animation-direction'], i).trim();
      const fill = at(cs['animation-fill-mode'], i).trim();
      if (at(cs['animation-play-state'], i).trim() === 'paused') warnOnce('animation-paused', 'animation-play-state: paused is not supported (runs)');
      const elapsed = now - state.starts.get(name) - delay;
      let progress;
      let iteration;
      if (elapsed < 0) {
        if (fill !== 'backwards' && fill !== 'both') {
          active = true;
          continue;
        }
        progress = 0;
        iteration = 0;
        active = true;
      } else if (duration <= 0 || elapsed >= duration * count) {
        if (fill !== 'forwards' && fill !== 'both') continue;
        iteration = count === Infinity ? 0 : Math.max(0, Math.ceil(count) - 1);
        const frac = count - Math.floor(count);
        progress = duration <= 0 ? 1 : frac > 0 ? frac : 1;
      } else {
        iteration = Math.floor(elapsed / duration);
        progress = (elapsed - iteration * duration) / duration;
        active = true;
      }
      const reversed = direction === 'reverse' || (direction === 'alternate' && iteration % 2 === 1) || (direction === 'alternate-reverse' && iteration % 2 === 0);
      if (reversed) progress = 1 - progress;
      const easing = at(cs['animation-timing-function'], i);
      for (const [prop, track] of tracksOf(keyframes)) {
        const base = underlying(prop);
        let prev = { o: 0, v: base, timing: null };
        let next = { o: 1, v: base, timing: null };
        for (const k of track) {
          if (k.o <= progress) prev = k;
          if (k.o >= progress) {
            next = k;
            break;
          }
        }
        let value;
        if (next === prev || next.o === prev.o) value = prev.v;
        else {
          const local = (progress - prev.o) / (next.o - prev.o);
          value = interpolate(prop, prev.v, next.v, timingFunction(prev.timing ?? easing)(local));
        }
        if (value === undefined) continue;
        if (!values) values = Object.create(null);
        values[prop] = value;
      }
    }
    for (const name of [...state.starts.keys()]) if (!seen.has(name)) state.starts.delete(name);
    return { values, active };
  }

  // A computed value back to a CSS string (for animation underlying values
  // and style reads).
  function serialize(prop, v) {
    if (v === undefined || v === null) return '';
    if (typeof v === 'string') return v;
    if (typeof v === 'number') return P[prop] && (P[prop].kind === 'fontSize' || P[prop].kind === 'spacing' || P[prop].kind === 'borderWidth') ? `${v}px` : String(v);
    if (Array.isArray(v) && v.length === 4 && typeof v[0] === 'number') return `rgba(${v[0] * 255},${v[1] * 255},${v[2] * 255},${v[3]})`;
    if (Array.isArray(v)) return v.map((x) => (typeof x === 'object' ? serialize(prop, x) : String(x))).join(', ');
    if (v.url !== undefined) return `url(#${v.url})`;
    if (v.n !== undefined) return `${v.n}${v.u}`;
    if (v.mult !== undefined) return String(v.mult);
    return '';
  }

  D.css = {
    serialize,
    parseColor,
    parseLength,
    parseTransform,
    transformMatrix,
    multiply: mul,
    toPx,
    isPercent,
    splitTokens,
    splitCommas,
    expand,
    parseDeclarations,
    parseStyleSheet,
    computeStyle,
    diffStyles,
    evaluateAnimations,
    timingFunction,
    PROPS: P,
    INITIAL,
    INHERITED,
    PRESENTATION,
    CURRENT,
  };
})();
