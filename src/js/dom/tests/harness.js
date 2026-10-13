// Test harness for the DOM: a mock __host with fixed, simple font metrics
// (every character is half the font size wide; ascent 0.8 and descent 0.2
// of the size), a recorded submitDisplay, and a readable dump of a stream.
// The metrics are a test fixture, not a model of any font.
(() => {
  const submits = [];
  const images = { '/Images/test.png': [64, 32] };
  const host = globalThis.__host || (globalThis.__host = {});
  Object.assign(host, {
    measureText: (family, size, text) => 0.5 * size * [...text].length,
    fontMetrics: (family, size) => [0.8 * size, 0.2 * size],
    screenSize: (screen) => ({ PFD: [768, 1024], MFD: [1646, 1024], TEST: [200, 100] })[screen] ?? [300, 150],
    submitDisplay: (screen, ops, strings) => submits.push({ screen, ops: Array.from(ops), strings: strings.slice() }),
    imageSize: (url) => images[url] ?? [0, 0],
  });

  const NAMES = { 1: 'SAVE', 2: 'RESTORE', 3: 'TRANSFORM', 4: 'SET_TRANSFORM', 5: 'GLOBAL_ALPHA', 6: 'CLIP_RECT', 10: 'BEGIN_PATH', 11: 'MOVE_TO', 12: 'LINE_TO', 13: 'QUAD_TO', 14: 'CUBIC_TO', 15: 'ARC', 16: 'ELLIPSE', 17: 'RECT', 18: 'CLOSE_PATH', 20: 'FILL', 21: 'STROKE', 22: 'CLIP_PATH', 30: 'TEXT', 40: 'IMAGE', 50: 'LINEAR_GRADIENT_FILL' };
  const ARGS = { 1: 0, 2: 0, 3: 6, 4: 6, 5: 1, 6: 4, 10: 0, 11: 2, 12: 2, 13: 4, 14: 6, 15: 6, 16: 8, 17: 4, 18: 0, 20: 5, 22: 1, 30: 18, 40: 9 };
  const num = (v) => {
    const r = Math.round(v * 1000) / 1000;
    return Object.is(r, -0) ? '0' : String(r);
  };

  // One line per op; strings quoted in place of their indices.
  function dump(ops, strings) {
    const lines = [];
    let depth = 0;
    for (let i = 0; i < ops.length; ) {
      const o = ops[i];
      const name = NAMES[o];
      if (!name) {
        lines.push(`?? ${o} at ${i}`);
        break;
      }
      let n = ARGS[o];
      if (o === 21) n = 9 + ops[i + 9] + 1;
      if (o === 50) n = 5 + ops[i + 5] * 5 + 1;
      const args = ops.slice(i + 1, i + 1 + n);
      let text;
      if (o === 30) text = `${JSON.stringify(strings[args[0]])} ${JSON.stringify(strings[args[1]])} ${args.slice(2).map(num).join(' ')}`;
      else if (o === 40) text = `${JSON.stringify(strings[args[0]])} ${args.slice(1).map(num).join(' ')}`;
      else text = args.map(num).join(' ');
      if (o === 2) depth--;
      lines.push(`${'  '.repeat(Math.max(0, depth))}${name}${text ? ` ${text}` : ''}`);
      if (o === 1) depth++;
      i += 1 + n;
    }
    return lines.join('\n');
  }

  globalThis.__test = {
    submits,
    images,
    dump,
    last: (screen) => {
      for (let i = submits.length - 1; i >= 0; i--) if (submits[i].screen === screen) return submits[i];
      return null;
    },
    dumpLast: (screen) => {
      const s = globalThis.__test.last(screen);
      return s ? dump(s.ops, s.strings) : '(nothing submitted)';
    },
    fail: (message) => {
      throw new Error(message);
    },
    eq: (a, b, what) => {
      if (a !== b) throw new Error(`${what}: expected ${JSON.stringify(b)}, got ${JSON.stringify(a)}`);
    },
  };
})();
