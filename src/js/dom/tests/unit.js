// Unit tests for the DOM, run with the mock host (harness.js). Each test
// makes its own screen document; `__runUnitTests()` returns a report.
(() => {
  const SVG = 'http://www.w3.org/2000/svg';
  const { eq, dumpLast, submits } = globalThis.__test;
  let screenCounter = 0;
  let clock = 0;

  const fresh = (w = 200, h = 100, sheet = '') => {
    __destroyDocument(`U${screenCounter}`);
    const screen = `U${++screenCounter}`;
    const doc = __createDocument(screen, w, h);
    globalThis.document = doc;
    if (sheet) doc.__addStyleSheet(sheet);
    return doc;
  };
  const frame = (ms = 16) => {
    clock += ms;
    __tick(clock);
  };
  const svgEl = (doc, tag, attrs = {}, ...children) => {
    const el = doc.createElementNS(SVG, tag);
    for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
    for (const c of children) el.append(c);
    return el;
  };
  const html = (doc, tag, attrs = {}, ...children) => {
    const el = doc.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
    for (const c of children) el.append(c);
    return el;
  };
  const ok = (cond, what) => {
    if (!cond) throw new Error(what);
  };
  const has = (text, part, what) => ok(text.includes(part), `${what}: missing ${JSON.stringify(part)} in\n${text}`);
  const lacks = (text, part, what) => ok(!text.includes(part), `${what}: unexpected ${JSON.stringify(part)} in\n${text}`);
  const near = (a, b, what, tol = 1e-6) => ok(Math.abs(a - b) <= tol, `${what}: expected ${b}, got ${a}`);

  const tests = {
    'tree operations and text nodes'() {
      const doc = fresh();
      const div = html(doc, 'div');
      doc.body.append(div);
      for (let i = 0; i < 3; i++) div.appendChild(doc.createElement('span'));
      eq(div.childNodes.length, 3, 'children');
      while (div.firstChild) div.removeChild(div.firstChild);
      eq(div.childNodes.length, 0, 'emptied');
      div.insertAdjacentHTML('beforeend', 'A &amp; B');
      const text = div.lastChild;
      eq(text.nodeType, 3, 'text node');
      eq(text.nodeValue, 'A & B', 'entities');
      text.nodeValue = ' ';
      eq(div.textContent, ' ', 'nodeValue');
      const svg = svgEl(doc, 'svg');
      const t = svgEl(doc, 'text');
      svg.append(t);
      t.innerHTML = '<tspan class="Green">SPD</tspan> SEL';
      eq(t.firstChild.namespaceURI, SVG, 'tspan parsed in the SVG namespace');
      eq(t.firstChild.localName, 'tspan', 'tspan');
      eq(t.textContent, 'SPD SEL', 'textContent');
      ok(t instanceof SVGElement && div instanceof HTMLElement, 'instanceof');
      eq(div.tagName, 'DIV', 'tagName');
      const a = html(doc, 'i');
      div.append(a);
      const b = html(doc, 'b');
      a.insertAdjacentElement('beforebegin', b);
      eq(div.firstElementChild, b, 'beforebegin');
      eq(b.nextSibling, a, 'nextSibling');
      a.remove();
      eq(b.nextSibling, null, 'remove');
    },

    'attributes, classList, dataset and style'() {
      const doc = fresh();
      const el = html(doc, 'div');
      el.classList.add('A', 'B');
      el.classList.toggle('A', false);
      el.classList.toggle('C');
      el.classList.replace('B', 'D');
      eq(el.className, 'D C', 'classList');
      ok(el.classList.contains('C'), 'contains');
      el.dataset.fooBar = '7';
      eq(el.getAttribute('data-foo-bar'), '7', 'dataset');
      el.style.backgroundColor = 'red';
      el.style.setProperty('--x', '4px');
      el.style.display = 'none';
      eq(el.style.getPropertyValue('background-color'), 'red', 'camelCase');
      eq(el.style.display, 'none', 'display');
      has(el.getAttribute('style'), 'display: none;', 'style attribute');
      el.style.display = '';
      eq(el.style.display, '', 'removed');
      el.setAttribute('style', 'top: 3px; left: 4px');
      eq(el.style.top, '3px', 'style attribute parsed');
      const s = svgEl(doc, 'use');
      s.setAttributeNS('http://www.w3.org/1999/xlink', 'xlink:href', '#a');
      eq(s.getAttributeNS('http://www.w3.org/1999/xlink', 'href'), '#a', 'xlink');
      eq(svgEl(doc, 'svg', { viewBox: '0 0 1 1' }).getAttribute('viewBox'), '0 0 1 1', 'SVG attribute case');
    },

    'selectors'() {
      const doc = fresh();
      doc.body.innerHTML = '<vcockpit-panel id="panel"><a380x-pfd url="x"></a380x-pfd></vcockpit-panel><div class="a b"><p>1</p><p class="c">2</p><input type="radio"><label>l</label></div>';
      eq(doc.querySelectorAll('vcockpit-panel > *').length, 1, 'child combinator on custom tags');
      eq(doc.querySelectorAll('vcockpit-panel > *')[0].getAttribute('url'), 'x', 'url attribute');
      eq(doc.querySelector('.a.b > p:nth-child(2)').textContent, '2', 'nth-child');
      eq(doc.querySelector('p:not(.c)').textContent, '1', ':not');
      eq(doc.querySelector('p:last-of-type').textContent, '2', 'last-of-type');
      eq(doc.querySelector('input[type=radio] + label').textContent, 'l', 'adjacent sibling');
      eq(doc.querySelector('div ~ *'), null, 'general sibling none');
      eq(doc.getElementById('panel').localName, 'vcockpit-panel', 'getElementById');
      ok(doc.querySelector('p').closest('.a'), 'closest');
    },

    'events: capture, bubble, once, stopPropagation'() {
      const doc = fresh();
      const outer = html(doc, 'div');
      const inner = html(doc, 'span');
      outer.append(inner);
      doc.body.append(outer);
      const log = [];
      doc.body.addEventListener('click', () => log.push('body capture'), { capture: true });
      outer.addEventListener('click', () => log.push('outer bubble'));
      outer.addEventListener('click', () => log.push('outer capture'), true);
      inner.addEventListener('click', (e) => log.push(`inner ${e.eventPhase}`), { once: true });
      inner.dispatchEvent(new MouseEvent('click', { bubbles: true }));
      eq(log.join(','), 'body capture,outer capture,inner 2,outer bubble', 'order');
      log.length = 0;
      outer.addEventListener('click', (e) => e.stopPropagation(), true);
      inner.dispatchEvent(new MouseEvent('click', { bubbles: true }));
      eq(log.join(','), 'body capture,outer capture', 'stopped in capture, once removed');
      let custom = null;
      doc.addEventListener('update', (e) => (custom = e.detail));
      doc.dispatchEvent(new CustomEvent('update', { detail: 5 }));
      eq(custom, 5, 'CustomEvent detail');
    },

    'cascade: specificity, !important, presentation attributes, inline, inheritance'() {
      const doc = fresh(100, 100, `
        .Green { fill: #00ff00; }
        text.Amber { fill: #ffa500; }
        #t3 { fill: blue !important; }
        g { stroke: red; font-size: 10px; }
        .Big { font-size: 2em; }
      `);
      const svg = svgEl(doc, 'svg', { width: 100, height: 100 });
      const g = svgEl(doc, 'g', { fill: 'white' });
      const t1 = svgEl(doc, 'text', { class: 'Green', fill: 'red' });
      const t2 = svgEl(doc, 'text', { class: 'Green Amber' });
      const t3 = svgEl(doc, 'text', { id: 't3', class: 'Green', style: 'fill: black' });
      const t4 = svgEl(doc, 'text', { class: 'Big' });
      g.append(t1, t2, t3, t4);
      svg.append(g);
      doc.body.append(svg);
      doc._flush();
      eq(String(t1._cs.fill), String([0, 1, 0, 1]), 'a rule beats a presentation attribute');
      eq(String(t2._cs.fill), String([1, 165 / 255, 0, 1]), 'higher specificity wins');
      eq(String(t3._cs.fill), String([0, 0, 1, 1]), '!important beats inline');
      eq(String(t4._cs.stroke), String([1, 0, 0, 1]), 'stroke inherited');
      eq(t4._cs['font-size'], 20, 'em against the parent');
      t1.classList.remove('Green');
      doc._flush();
      eq(String(t1._cs.fill), String([1, 0, 0, 1]), 'restyled after a class change');
      g.setAttribute('fill', 'yellow');
      t4.style.fill = 'currentColor';
      t4.style.color = 'cyan';
      doc._flush();
      eq(t4._cs.fill, __dom.css.CURRENT, 'currentColor stays a keyword to inherit');
      eq(String(t4._cs.color), String([0, 1, 1, 1]), 'color');
    },

    'dirty-only painting and measured cost'() {
      const doc = fresh(100, 100);
      const svg = svgEl(doc, 'svg', { width: 100, height: 100 });
      const t = svgEl(doc, 'text', { x: 1, y: 2 });
      t.textContent = 'A';
      svg.append(t);
      doc.body.append(svg);
      const count = () => submits.filter((s) => s.screen === doc.__screen).length;
      const before = 0;
      frame();
      eq(count(), before + 1, 'first frame submits');
      frame();
      frame();
      eq(count(), before + 1, 'nothing changed: nothing submitted');
      t.firstChild.nodeValue = 'A';
      frame();
      eq(count(), before + 1, 'same text: nothing submitted');
      t.firstChild.nodeValue = 'B';
      frame();
      eq(count(), before + 2, 'changed text submits');
      has(dumpLast(doc.__screen), 'TEXT "B"', 'new text');
      ok(typeof doc.stats.paintMs === 'number' && doc.stats.ops > 0, 'stats');
    },

    'keyframe blinking on the engine clock'() {
      const doc = fresh(100, 100, `
        @keyframes blinking { 0% { opacity: 0; } 50% { opacity: 0; } 51% { opacity: 1; } 100% { opacity: 1; } }
        .BlinkInfinite { animation-name: blinking; animation-duration: 1s; animation-iteration-count: infinite; }
      `);
      const svg = svgEl(doc, 'svg', { width: 100, height: 100 });
      const p = svgEl(doc, 'path', { d: 'M0 0 L1 1', stroke: 'white', class: 'BlinkInfinite' });
      svg.append(p);
      doc.body.append(svg);
      frame(); // starts at this time
      const start = clock;
      const at = (ms) => {
        clock = start + ms - 16;
        frame(16);
        return dumpLast(doc.__screen);
      };
      lacks(at(250), 'STROKE', 'hidden in the first half');
      has(at(750), 'STROKE', 'shown in the second half');
      const n = submits.length;
      at(800);
      eq(submits.length, n, 'no submit while the value holds');
      lacks(at(1200), 'STROKE', 'hidden again in the next iteration');
    },

    'path data: every command'() {
      const b = __dom.geom.parsePathData('M10 10 h5 v5 H0 V0 l1 1 L2 2 c1 0 1 1 0 1 s-1 1 0 1 C0 0 0 0 0 0 S1 1 2 2 q1 0 1 1 t1 1 Q0 0 1 1 T2 2 a2 2 0 0 1 2 2 A1 1 0 1 0 0 0 z m1 1 -1-1e0 .5.5');
      const ops = Array.from(b.done().ops);
      const names = [];
      const sizes = { 11: 2, 12: 2, 13: 4, 14: 6, 16: 8, 18: 0 };
      for (let i = 0; i < ops.length; i += 1 + sizes[ops[i]]) names.push(ops[i]);
      eq(names.join(' '), '11 12 12 12 12 12 12 14 14 14 14 13 13 13 13 16 16 18 11 12 12', 'command sequence');
      // The S reflects the previous control point (3,3) about (2,3).
      const firstS = ops.indexOf(14, ops.indexOf(14) + 1);
      near(ops[firstS + 1], 1, 'S reflected control x');
      near(ops[firstS + 2], 3, 'S reflected control y');
      // A lenient transform: FlyByWire's altitude tape leaves out the ')'.
      const m = __dom.css.transformMatrix(__dom.css.parseTransform('translate(0 12.5'), 0, 0, 16, null);
      eq(m.join(','), '1,0,0,1,0,12.5', 'lenient translate');
      const r = __dom.css.transformMatrix(__dom.css.parseTransform('rotate(90 10 0)'), 0, 0, 16, null);
      near(r[4], 10, 'rotate about a point e');
      near(r[5], -10, 'rotate about a point f');
    },

    'svg text: anchors, tspans, baselines, white space, measurement'() {
      const doc = fresh(200, 100, 'text { font-size: 10px; font-family: Ecam; }');
      const svg = svgEl(doc, 'svg', { width: 200, height: 100 });
      const t = svgEl(doc, 'text', { x: 100, y: 50, 'text-anchor': 'middle', 'dominant-baseline': 'middle' });
      t.innerHTML = '\n  <tspan fill="cyan">AB</tspan>  <tspan dy="2" alignment-baseline="hanging">C</tspan>\n';
      svg.append(t);
      doc.body.append(svg);
      frame();
      const out = dumpLast(doc.__screen);
      // "AB" (10 px) + " " (5) + "C" (5) = 20 px wide, centred on 100.
      has(out, 'TEXT "AB" "Ecam" 10 400 0 90 50 0 1 0 1 1 1', 'first run');
      has(out, 'TEXT " " "Ecam" 10 400 0 100 50 0 1 0 0 0 1', 'the space between keeps the text style');
      has(out, 'TEXT "C" "Ecam" 10 400 0 105 52 0 4', 'dy and alignment-baseline');
      near(t.getComputedTextLength(), 20, 'getComputedTextLength');
      const bb = t.firstElementChild.getBBox();
      near(bb.x, 90, 'tspan bbox x');
      near(bb.width, 10, 'tspan bbox width');
    },

    'clipPath and linearGradient'() {
      const doc = fresh(100, 100);
      doc.body.innerHTML = `<svg width="100" height="100">
        <defs>
          <clipPath id="rect"><rect x="1" y="2" width="30" height="40"/></clipPath>
          <clipPath id="path"><path d="M0 0 L10 0 L10 10 z" transform="translate(5 5)"/></clipPath>
          <linearGradient id="grad" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="red"/><stop offset="100%" stop-color="blue" stop-opacity="0.5"/></linearGradient>
        </defs>
        <g clip-path="url(#rect)"><rect width="50" height="50" fill="url(#grad)"/></g>
        <g clip-path="url(#path)"><circle r="3" fill="white"/></g>
      </svg>`;
      frame();
      const out = dumpLast(doc.__screen);
      has(out, 'CLIP_RECT 1 2 30 40', 'rectangle clip');
      has(out, 'TRANSFORM 50 0 0 50 0 0', 'objectBoundingBox gradient');
      has(out, 'LINEAR_GRADIENT_FILL 0 0 0 1 2 0 1 0 0 1 1 0 0 1 0.5 0', 'gradient stops');
      has(out, 'CLIP_PATH 0', 'path clip');
      const lines = out.split('\n').map((l) => l.trim());
      const clipAt = lines.indexOf('CLIP_PATH 0');
      eq(lines.slice(clipAt - 8, clipAt).join('|'), 'BEGIN_PATH|SAVE|TRANSFORM 1 0 0 1 5 5|MOVE_TO 0 0|LINE_TO 10 0|LINE_TO 10 10|CLOSE_PATH|RESTORE', 'clip geometry');
      // Changing the clip path repaints its user.
      doc.querySelector('#rect rect').setAttribute('width', '20');
      frame();
      has(dumpLast(doc.__screen), 'CLIP_RECT 1 2 20 40', 'clip user repainted');
    },

    'visibility, display, opacity and hidden children'() {
      const doc = fresh(100, 100, '.SelfTestText { visibility: visible !important; }');
      doc.body.innerHTML = `<svg width="100" height="100"><g visibility="hidden">
        <path d="M0 0 L1 1" stroke="red"/><text class="SelfTestText" x="1" y="1">T</text>
        </g><g style="display:none"><path d="M0 0 L2 2" stroke="red"/></g><g opacity="0.5"><path d="M0 0 L3 3" stroke="red"/></g></svg>`;
      frame();
      const out = dumpLast(doc.__screen);
      lacks(out, 'LINE_TO 1 1', 'hidden path');
      has(out, 'TEXT "T"', 'visible child of a hidden group');
      lacks(out, 'LINE_TO 2 2', 'display none');
      has(out, 'GLOBAL_ALPHA 0.5', 'opacity');
    },

    'absolute layout, viewBox and getBoundingClientRect'() {
      const doc = fresh(768, 1024, `
        .pfd-svg { position: absolute; width: 768px; height: 1024px; background: #000; }
        #box { position: absolute; left: 10%; top: 20px; width: 100px; height: 50px; border: 2px solid white; }
      `);
      doc.body.style.margin = '0';
      doc.body.innerHTML = '<div id="box"></div><svg class="pfd-svg" viewBox="0 0 158.75 211.6"><rect id="r" x="10" y="10" width="10" height="10" fill="green"/></svg>';
      frame();
      const box = doc.getElementById('box').getBoundingClientRect();
      near(box.left, 76.8, 'percent left');
      near(box.width, 104, 'border box width');
      const r = doc.getElementById('r').getBoundingClientRect();
      const s = Math.min(768 / 158.75, 1024 / 211.6);
      near(r.left, 10 * s, 'viewBox scale x', 1e-6);
      near(r.width, 10 * s, 'viewBox scale width', 1e-6);
      const out = dumpLast(doc.__screen);
      has(out, 'FILL 0 0 0 1 0', 'svg background');
    },

    'flex layout'() {
      const doc = fresh(400, 100, `
        .row { display: flex; flex-direction: row; justify-content: space-between; align-items: center; width: 300px; height: 40px; }
        .a { width: 50px; height: 20px; } .b { flex: 1; height: 10px; } .c { width: 30px; height: 30px; margin-left: 10px; }
        .col { display: flex; flex-direction: column; width: 100px; }
        .col > div { height: 10px; }
        .center { display: flex; justify-content: center; align-items: center; width: 100px; height: 40px; font-size: 20px; font-family: Ecam; }
      `);
      doc.body.style.margin = '0';
      doc.body.innerHTML = '<div class="row"><div class="a"></div><div class="b"></div><div class="c"></div></div><div class="col"><div></div><div></div></div><div class="center">OK</div>';
      frame();
      const [a, b, c] = doc.querySelectorAll('.row > div').map((e) => e.getBoundingClientRect());
      near(a.left, 0, 'a left');
      near(a.top, 10, 'a centred');
      near(b.left, 50, 'b after a');
      near(b.width, 210, 'b grows into the free space');
      near(c.left, 270, 'c after its margin');
      const kids = doc.querySelectorAll('.col > div').map((e) => e.getBoundingClientRect());
      near(kids[1].top, 50, 'column stacks');
      near(kids[1].width, 100, 'column items stretch');
      // "OK" is 40 px wide and 20 px tall (ascent 16): centred in 100 x 40.
      has(dumpLast(doc.__screen), 'TEXT "OK" "Ecam" 20 400 0 40 26', 'anonymous flex item text centred (in its box)');
    },

    'grid layout'() {
      const doc = fresh(400, 200, `
        .grid { display: grid; grid-template-columns: repeat(3, 1fr); grid-auto-rows: 20px; column-gap: 10px; width: 320px; }
        .wide { grid-column: span 2; }
      `);
      doc.body.style.margin = '0';
      doc.body.innerHTML = '<div class="grid"><div class="wide"></div><div></div><div></div></div>';
      frame();
      const cells = doc.querySelectorAll('.grid > div').map((e) => e.getBoundingClientRect());
      near(cells[0].width, 210, 'span 2');
      near(cells[1].left, 220, 'third column');
      near(cells[2].top, 20, 'auto row');
      near(cells[2].width, 100, '1fr');
    },

    'inline text wraps and ::before content'() {
      const doc = fresh(400, 200, `
        .t { width: 50px; font-size: 10px; font-family: Ecam; line-height: 12px; }
        .l::before { content: attr(data-label); color: cyan; }
      `);
      doc.body.style.margin = '0';
      doc.body.innerHTML = '<div class="t">AAA BBB CCC</div><div class="l" data-label="CRS">1</div>';
      frame();
      const out = dumpLast(doc.__screen);
      has(out, 'TEXT "AAA BBB" "Ecam" 10 400 0 0 9 ', 'first line (half-leading 1, ascent 8)');
      has(out, 'TEXT "CCC" "Ecam" 10 400 0 0 21 ', 'wrapped line');
      has(out, 'TEXT "CRS"', 'before content from an attribute');
      eq(doc.querySelector('.t').getBoundingClientRect().height, 24, 'two lines tall');
    },

    'mouse input: hit testing, click, hover'() {
      const doc = fresh(200, 100, `
        .btn { position: absolute; left: 10px; top: 10px; width: 50px; height: 20px; background: gray; }
        .btn:hover { background: white; }
      `);
      doc.body.innerHTML = '<div class="btn"><span>GO</span></div><svg style="position:absolute;left:100px;top:0" width="100" height="100"><rect id="r" x="10" y="10" width="20" height="20" fill="red"/></svg>';
      const log = [];
      doc.querySelector('.btn').addEventListener('click', (e) => log.push(`click ${e.clientX},${e.clientY}`));
      doc.querySelector('.btn').addEventListener('mouseover', () => log.push('over'));
      doc.getElementById('r').addEventListener('mousedown', () => log.push('rect down'));
      frame();
      __screenEvent(doc.__screen, 'move', 20, 15, 0, 0);
      frame();
      has(dumpLast(doc.__screen), 'FILL 1 1 1 1 0', ':hover restyled');
      __screenEvent(doc.__screen, 'down', 20, 15, 0, 0);
      __screenEvent(doc.__screen, 'up', 21, 16, 0, 0);
      __screenEvent(doc.__screen, 'down', 115, 15, 0, 0);
      eq(log.join('|'), 'over|click 21,16|rect down', 'events');
    },

    'canvas 2D and Path2D'() {
      const doc = fresh(200, 100);
      doc.body.style.margin = '0';
      const canvas = html(doc, 'canvas', { width: 100, height: 50 });
      canvas.style.width = '200px';
      canvas.style.height = '100px';
      doc.body.append(canvas);
      const ctx = canvas.getContext('2d');
      ctx.translate(10, 10);
      ctx.rotate(Math.PI / 2);
      ctx.strokeStyle = '#00ff00';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(5, 0);
      ctx.stroke();
      ctx.resetTransform();
      ctx.font = '21px Ecam';
      ctx.fillStyle = 'white';
      ctx.textAlign = 'center';
      ctx.fillText('N', 50, 25);
      ctx.fill(new Path2D('M0 0 a5 5 0 0 1 10 0 z'));
      frame();
      let out = dumpLast(doc.__screen);
      has(out, 'CLIP_RECT 0 0 200 100', 'canvas box clip');
      has(out, 'TRANSFORM 2 0 0 2 0 0', 'canvas pixels to CSS px');
      has(out, 'LINE_TO 10 15', 'rotated path');
      has(out, 'STROKE 0 1 0 1 2 0 0 10 0 0', 'stroke');
      has(out, 'TEXT "N" "Ecam" 21 400 0 50 25 1 0 1 1 1 1', 'fillText');
      has(out, 'ELLIPSE 5 0 5 5 0 3.142 6.283 0', 'Path2D arc');
      ctx.clearRect(0, 0, 100, 50);
      frame();
      out = dumpLast(doc.__screen);
      lacks(out, 'STROKE', 'cleared');
    },

    'custom elements'() {
      const doc = fresh();
      const log = [];
      class Gauge extends HTMLElement {
        constructor() {
          super();
          log.push('constructed');
        }
        connectedCallback() {
          log.push(`connected ${this.localName}`);
        }
        disconnectedCallback() {
          log.push('disconnected');
        }
      }
      const early = doc.createElement('test-gauge');
      customElements.define('test-gauge', Gauge);
      const el = doc.createElement('test-gauge');
      ok(el instanceof Gauge, 'created from the definition');
      doc.body.append(el);
      el.remove();
      doc.body.append(early);
      ok(early instanceof Gauge, 'upgraded when connected');
      eq(log.join('|'), 'constructed|connected test-gauge|disconnected|connected test-gauge', 'callbacks');
    },

    '@font-face and stylesheets from <style>'() {
      const doc = fresh(100, 100);
      const style = doc.createElement('style');
      style.textContent = '@font-face { font-family: "Ecam"; src: url(/Fonts/fbw-a380x/FBW-Display-EIS-A380.ttf); } .x { fill: red }';
      doc.head.append(style);
      doc.body.innerHTML = '<svg width="10" height="10"><path class="x" d="M0 0 L1 1"/></svg>';
      frame();
      eq(doc._fontFaces[0].family, 'Ecam', 'family');
      eq(doc._fontFaces[0].src, '/Fonts/fbw-a380x/FBW-Display-EIS-A380.ttf', 'file');
      has(dumpLast(doc.__screen), 'FILL 1 0 0 1 0', 'rule from <style>');
      style.textContent = '.x { fill: blue }';
      frame();
      has(dumpLast(doc.__screen), 'FILL 0 0 1 1 0', 'style text change');
    },
  };

  globalThis.__runUnitTests = () => {
    const failures = [];
    let passed = 0;
    for (const [name, fn] of Object.entries(tests)) {
      try {
        fn();
        passed++;
      } catch (e) {
        failures.push(`${name}: ${e && e.message}\n${e && e.stack ? e.stack : ''}`);
      }
    }
    return JSON.stringify({ passed, failures });
  };
})();
