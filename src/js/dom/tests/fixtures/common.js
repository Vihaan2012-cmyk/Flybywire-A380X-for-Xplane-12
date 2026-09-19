// What msfs-sdk 2.3.3's FSComponent does to the DOM when it renders
// (docs/dom-survey.md, "How FSComponent builds and renders"), without the
// framework: SVG tags from its svgTags list get the SVG namespace, `class`
// strings and records go through classList, `style` records through
// style.setProperty, other props through setAttribute, and text children
// through insertAdjacentHTML with the node kept as `root`. Subject is a
// minimal subscribable so fixtures can change bound values as components do.
(() => {
  const SVG_TAGS = new Set(['circle', 'clipPath', 'defs', 'ellipse', 'g', 'image', 'line', 'linearGradient', 'marker', 'mask', 'path', 'pattern', 'polygon', 'polyline', 'radialGradient', 'rect', 'stop', 'svg', 'text', 'tspan']);

  class Subject {
    constructor(v) {
      this.v = v;
      this.subs = [];
    }
    get() {
      return this.v;
    }
    set(v) {
      if (v === this.v) return;
      this.v = v;
      for (const s of this.subs) s(v);
    }
    sub(fn) {
      this.subs.push(fn);
      fn(this.v);
    }
  }

  function h(tag, props, ...children) {
    const doc = globalThis.document;
    const el = SVG_TAGS.has(tag) ? doc.createElementNS('http://www.w3.org/2000/svg', tag) : doc.createElement(tag);
    for (const [key, value] of Object.entries(props || {})) {
      if (key === 'ref') value.instance = el;
      else if (key === 'class' && typeof value === 'object' && !(value instanceof Subject)) {
        for (const [name, on] of Object.entries(value)) {
          if (on instanceof Subject) on.sub((v) => el.classList.toggle(name, !!v));
          else el.classList.toggle(name, !!on);
        }
      } else if (key === 'style' && typeof value === 'object' && !(value instanceof Subject)) {
        for (const [name, v] of Object.entries(value)) {
          if (v instanceof Subject) v.sub((x) => (x === null || x === '' ? el.style.removeProperty(name) : el.style.setProperty(name, x)));
          else el.style.setProperty(name, v);
        }
      } else if (value instanceof Subject) value.sub((v) => el.setAttribute(key, v));
      else el.setAttribute(key, value);
    }
    for (const child of children.flat()) {
      if (child === null || child === undefined) continue;
      if (child instanceof Subject) {
        el.insertAdjacentHTML('beforeend', ' ');
        const root = el.lastChild;
        child.sub((v) => (root.nodeValue = v === '' ? ' ' : String(v)));
      } else if (typeof child === 'string' || typeof child === 'number') {
        el.insertAdjacentHTML('beforeend', String(child));
      } else {
        el.appendChild(child);
      }
    }
    return el;
  }

  let clock = 0;
  globalThis.__fixture = {
    h,
    Subject,
    screen(name, w, h2, css) {
      const doc = __createDocument(name, w, h2);
      globalThis.document = doc;
      const style = doc.createElement('style');
      style.textContent = css;
      doc.head.appendChild(style);
      return doc;
    },
    frame(ms = 16) {
      clock += ms;
      __tick(clock);
    },
    at(ms) {
      clock = ms;
      __tick(clock);
    },
    // The stream last submitted for a screen, and how many there were.
    report(doc, label) {
      const n = __test.submits.filter((s) => s.screen === doc.__screen).length;
      return `== ${label} (submits: ${n})\n${__test.dumpLast(doc.__screen)}\n`;
    },
  };
})();
