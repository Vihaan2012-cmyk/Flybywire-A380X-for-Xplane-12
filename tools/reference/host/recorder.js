// Records which DOM APIs, CSS properties and SVG features the instruments
// actually use (reference.mjs --record). Installed before any page script.
//
// Every method and accessor on the prototypes of the DOM/CSSOM/SVG/Canvas
// interfaces is wrapped with a counter. A call is attributed to the first
// script URL on its stack: the instrument bundles (/Pages/...), the harness
// host shims (/reference/..., e.g. diffAndSetAttribute on a bundle's behalf)
// or the runner (no URL). Once a key has only been seen from bundles, later
// calls skip the stack walk, so the cost stays small.
// __refCollectDomUsage() adds what is only visible at the end: the style
// sheets' rules, and the elements, attributes, inline style properties,
// transform functions, path commands and effective computed properties in
// the final document.
(function () {
  'use strict';
  const calls = new Map(); // key -> {bundle, host, runner}
  const details = new Map(); // "Iface.method arg" -> count (bundle calls only)
  const INTERFACES = /^(EventTarget|Node|Element|Document|DocumentFragment|CharacterData|Text|Attr|NamedNodeMap|DOMTokenList|DOMRect|DOMRectReadOnly|DOMMatrix|DOMMatrixReadOnly|DOMPoint|DOMPointReadOnly|DOMParser|Range|Selection|MutationObserver|ResizeObserver|IntersectionObserver|CustomElementRegistry|ShadowRoot|HTML\w*Element|HTMLCollection|NodeList|SVG\w*|CSS\w*|StyleSheet|MediaList|FontFace|FontFaceSet|CanvasRenderingContext2D|Path2D|CanvasGradient|CanvasPattern|ImageData|TextMetrics|OffscreenCanvas\w*|Animation|KeyframeEffect|AnimationEffect|DocumentTimeline|Event|UIEvent|MouseEvent|WheelEvent|KeyboardEvent|CustomEvent|FocusEvent|PointerEvent|XMLSerializer)$/;
  const DETAIL = {
    'Element.setAttribute': (a) => a[0],
    'Element.setAttributeNS': (a) => a[1],
    'Element.removeAttribute': (a) => a[0],
    'Element.getAttribute': (a) => a[0],
    'Element.insertAdjacentHTML': (a) => a[0],
    'Element.insertAdjacentElement': (a) => a[0],
    'Element.querySelector': (a) => a[0],
    'Element.querySelectorAll': (a) => a[0],
    'Document.querySelector': (a) => a[0],
    'Document.querySelectorAll': (a) => a[0],
    'Document.getElementById': () => '',
    'Document.createElement': (a) => a[0],
    'Document.createElementNS': (a) => `${a[0]} ${a[1]}`,
    'CSSStyleDeclaration.setProperty': (a) => a[0],
    'CSSStyleDeclaration.removeProperty': (a) => a[0],
    'CSSStyleDeclaration.getPropertyValue': (a) => a[0],
    'HTMLCanvasElement.getContext': (a) => a[0],
    'EventTarget.addEventListener': (a) => a[0],
    'DOMTokenList.add': (a) => a[0],
    'DOMTokenList.toggle': (a) => a[0],
  };

  let inside = false;
  function origin() {
    const stack = new Error().stack || '';
    for (const line of stack.split('\n').slice(3)) {
      const m = line.match(/(https?:\/\/[^\s)]+?):\d+:\d+/);
      if (!m) continue;
      return new URL(m[1]).pathname.startsWith('/reference/') ? 'host' : 'bundle';
    }
    return 'runner';
  }
  function note(key, args) {
    if (inside) return;
    inside = true;
    try {
      let e = calls.get(key);
      if (!e) calls.set(key, (e = { bundle: 0, host: 0, runner: 0 }));
      const who = e.bundle > 0 && e.host === 0 && e.runner === 0 ? 'bundle' : origin();
      e[who]++;
      if (who === 'bundle' && DETAIL[key] && args) {
        const d = `${key} ${String(DETAIL[key](args))}`;
        details.set(d, (details.get(d) || 0) + 1);
      }
    } finally {
      inside = false;
    }
  }

  const proxies = new WeakMap();
  function styleProxy(decl) {
    if (!decl) return decl;
    let p = proxies.get(decl);
    if (!p) {
      p = new Proxy(decl, {
        get(t, k) {
          const v = Reflect.get(t, k, t);
          return typeof v === 'function' ? v.bind(t) : v;
        },
        set(t, k, v) {
          if (typeof k === 'string') note(`CSSStyleDeclaration[${k}] set`);
          return Reflect.set(t, k, v, t);
        },
      });
      proxies.set(decl, p);
    }
    return p;
  }

  function wrapProto(ifaceName, proto) {
    for (const prop of Object.getOwnPropertyNames(proto)) {
      if (prop === 'constructor') continue;
      const desc = Object.getOwnPropertyDescriptor(proto, prop);
      if (!desc || !desc.configurable) continue;
      const key = `${ifaceName}.${prop}`;
      if (typeof desc.value === 'function') {
        const fn = desc.value;
        const wrapped = {
          [prop](...args) {
            note(key, args);
            return fn.apply(this, args);
          },
        }[prop];
        try {
          Object.defineProperty(proto, prop, { ...desc, value: wrapped });
        } catch (e) {
          /* non-writable */
        }
      } else if (desc.get || desc.set) {
        const nd = { configurable: true, enumerable: desc.enumerable };
        if (desc.get && prop === 'style') {
          // CSS properties set as el.style.name = value are named-property
          // interceptors, not prototype accessors; a proxy sees them.
          const g = desc.get;
          nd.get = function () {
            note(`${key} get`);
            return styleProxy(g.call(this));
          };
        } else if (desc.get) {
          const g = desc.get;
          nd.get = function () {
            note(`${key} get`);
            return g.call(this);
          };
        }
        if (desc.set) {
          const s = desc.set;
          nd.set = function (v) {
            note(`${key} set`);
            return s.call(this, v);
          };
        }
        try {
          Object.defineProperty(proto, prop, nd);
        } catch (e) {
          /* not redefinable */
        }
      }
    }
  }

  for (const name of Object.getOwnPropertyNames(window)) {
    if (!INTERFACES.test(name)) continue;
    const ctor = window[name];
    if (typeof ctor !== 'function' || !ctor.prototype) continue;
    wrapProto(name, ctor.prototype);
  }
  // Window's own members live on the instance.
  for (const prop of ['getComputedStyle', 'matchMedia', 'getSelection']) {
    const fn = window[prop];
    if (typeof fn !== 'function') continue;
    window[prop] = function (...args) {
      note(`Window.${prop}`, args);
      return fn.apply(window, args);
    };
  }
  for (const prop of ['innerWidth', 'innerHeight', 'devicePixelRatio']) {
    const desc = Object.getOwnPropertyDescriptor(window, prop);
    if (!desc || !desc.get || !desc.configurable) continue;
    Object.defineProperty(window, prop, {
      configurable: true,
      get() {
        note(`Window.${prop} get`);
        return desc.get.call(window);
      },
    });
  }

  const inc = (map, k, n = 1) => map.set(k, (map.get(k) || 0) + n);

  function selectorFeatures(sel, out) {
    for (const m of sel.matchAll(/::?[a-z-]+(\([^)]*\))?/gi)) inc(out, m[0].replace(/\(.*\)/, '()'));
    if (/\[[^\]]+\]/.test(sel)) inc(out, '[attribute]');
    const compact = sel.replace(/\s*([>+~,])\s*/g, '$1');
    if (/>/.test(compact)) inc(out, 'child combinator >');
    if (/\+/.test(compact)) inc(out, 'next-sibling combinator +');
    if (/~/.test(compact)) inc(out, 'subsequent-sibling combinator ~');
    if (/[\w\])*]\s+[\w.#*[:]/.test(compact)) inc(out, 'descendant combinator');
    if (/\*/.test(compact)) inc(out, 'universal *');
    if (/(^|[>+~,\s])[a-z][\w-]*/i.test(compact)) inc(out, 'type');
    if (/\.[\w-]+/.test(compact)) inc(out, '.class');
    if (/#[\w-]+/.test(compact)) inc(out, '#id');
    if (/,/.test(compact)) inc(out, 'selector list ,');
  }

  function collectRules(rules, out) {
    for (const rule of rules) {
      if (rule instanceof CSSStyleRule) {
        inc(out.ruleTypes, 'style rule');
        for (let i = 0; i < rule.style.length; i++) {
          const p = rule.style[i];
          inc(out.sheetProps, p);
          const v = rule.style.getPropertyValue(p);
          for (const f of v.matchAll(/([a-z-]+)\(/gi)) inc(out.valueFunctions, `${f[1].toLowerCase()}()`);
          if (rule.style.getPropertyPriority(p) === 'important') inc(out.valueFunctions, '!important');
        }
        selectorFeatures(rule.selectorText, out.selectorFeatures);
        if (rule.cssRules && rule.cssRules.length) collectRules(rule.cssRules, out);
      } else if (rule instanceof CSSKeyframesRule) {
        inc(out.ruleTypes, '@keyframes');
        for (const kf of rule.cssRules) for (let i = 0; i < kf.style.length; i++) inc(out.keyframeProps, kf.style[i]);
      } else if (rule instanceof CSSFontFaceRule) {
        inc(out.ruleTypes, '@font-face');
      } else if (rule instanceof CSSMediaRule) {
        inc(out.ruleTypes, '@media');
        collectRules(rule.cssRules, out);
      } else if (rule instanceof CSSImportRule) {
        inc(out.ruleTypes, '@import');
      } else {
        inc(out.ruleTypes, rule.constructor.name);
        if (rule.cssRules) collectRules(rule.cssRules, out);
      }
    }
  }

  window.__refCollectDomUsage = () => {
    inside = true;
    try {
      const out = {
        ruleTypes: new Map(),
        sheetProps: new Map(),
        keyframeProps: new Map(),
        valueFunctions: new Map(),
        selectorFeatures: new Map(),
        htmlElements: new Map(),
        svgElements: new Map(),
        attributes: new Map(),
        inlineStyleProps: new Map(),
        transformFunctions: new Map(),
        pathCommands: new Map(),
        computedNonInitial: new Map(),
        sheets: [],
      };
      for (const sheet of document.styleSheets) {
        const path = sheet.href ? new URL(sheet.href).pathname : '(inline <style>)';
        if (path.startsWith('/reference/')) continue;
        out.sheets.push(path);
        collectRules(sheet.cssRules, out);
      }
      // Computed values that differ from a bare element of the same tag in the
      // same kind of parent: the properties that actually take effect.
      const probeHost = document.createElement('div');
      probeHost.style.cssText = 'position:absolute;visibility:hidden;left:-10000px;top:0';
      document.body.appendChild(probeHost);
      const probeSvg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
      probeHost.appendChild(probeSvg);
      const baseline = new Map();
      const baseFor = (el, svg) => {
        const tag = `${svg ? 'svg' : 'html'}:${el.localName}`;
        if (!baseline.has(tag)) {
          const b = svg ? document.createElementNS('http://www.w3.org/2000/svg', el.localName) : document.createElement(el.localName);
          (svg && el.localName !== 'svg' ? probeSvg : probeHost).appendChild(b);
          const cs = getComputedStyle(b);
          const snap = {};
          for (let i = 0; i < cs.length; i++) snap[cs[i]] = cs.getPropertyValue(cs[i]);
          baseline.set(tag, snap);
        }
        return baseline.get(tag);
      };
      // Layout results, not authored properties.
      const DERIVED = new Set(['inline-size', 'block-size', 'perspective-origin', 'transform-origin', 'width', 'height', 'min-inline-size', 'min-block-size']);
      const panel = document.getElementById('panel');
      for (const el of panel ? panel.querySelectorAll('*') : []) {
        const svg = el instanceof SVGElement;
        inc(svg ? out.svgElements : out.htmlElements, el.localName);
        for (const a of el.attributes) {
          inc(out.attributes, `${svg ? 'svg' : 'html'} <${el.localName}> ${a.name}`);
          if (a.name === 'transform') for (const f of a.value.matchAll(/([a-zA-Z]+)\s*\(/g)) inc(out.transformFunctions, `transform attribute: ${f[1]}()`);
          if (a.name === 'd') for (const c of a.value.matchAll(/[MmLlHhVvCcSsQqTtAaZz]/g)) inc(out.pathCommands, c[0]);
        }
        for (let i = 0; i < el.style.length; i++) {
          const p = el.style[i];
          inc(out.inlineStyleProps, p);
          if (p === 'transform') for (const f of el.style.transform.matchAll(/([a-zA-Z0-9]+)\s*\(/g)) inc(out.transformFunctions, `CSS transform: ${f[1]}()`);
        }
        const cs = getComputedStyle(el);
        if (cs.display === 'none') {
          inc(out.computedNonInitial, '(elements with display: none)');
          continue;
        }
        const base = baseFor(el, svg);
        for (let i = 0; i < cs.length; i++) {
          const p = cs[i];
          if (DERIVED.has(p)) continue;
          if (cs.getPropertyValue(p) !== base[p]) inc(out.computedNonInitial, p);
        }
      }
      probeHost.remove();
      const obj = (m) => Object.fromEntries([...m.entries()].sort((a, b) => b[1] - a[1] || String(a[0]).localeCompare(String(b[0]))));
      const api = {};
      for (const [k, v] of [...calls.entries()].sort()) api[k] = v;
      return {
        api,
        details: obj(details),
        css: {
          sheets: out.sheets,
          ruleTypes: obj(out.ruleTypes),
          sheetProps: obj(out.sheetProps),
          keyframeProps: obj(out.keyframeProps),
          valueFunctions: obj(out.valueFunctions),
          selectorFeatures: obj(out.selectorFeatures),
          inlineStyleProps: obj(out.inlineStyleProps),
          computedNonInitial: obj(out.computedNonInitial),
        },
        dom: {
          htmlElements: obj(out.htmlElements),
          svgElements: obj(out.svgElements),
          attributes: obj(out.attributes),
          transformFunctions: obj(out.transformFunctions),
          pathCommands: obj(out.pathCommands),
        },
      };
    } finally {
      inside = false;
    }
  };
})();
