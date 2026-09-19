// A screen's document: stylesheets, the rule index, style resolution with
// invalidation, animations on the engine clock, custom elements, and the
// frame that restyles, lays out and paints only what changed.
//
// Mutations mark work instead of doing it. Each element carries
//   _cs       its computed style,
//   _matched  the declaration lists of the rules that match it,
//   _pending  MATCH (re-match rules) and/or STYLE (recompute from _matched),
//   _chunk    its subtree's display ops, or null when they must be rebuilt.
// A frame (ScreenDocument.frame) resolves pending styles in tree order,
// relays out HTML if a layout-affecting value changed, rebuilds the chunks
// that were cleared, and submits a stream only when a chunk was rebuilt.
(() => {
  const D = globalThis.__dom;
  const { css, selectors, warnOnce } = D;
  const HTML_NS = D.NS.HTML;
  const SVG_NS = D.NS.SVG;

  const MATCH = 1;
  const STYLE = 2;

  // The parts of the HTML specification's rendering section
  // (https://html.spec.whatwg.org/multipage/rendering.html, 15.3) that the
  // elements the instruments create use.
  const UA_CSS = `
    html, address, blockquote, body, center, dialog, div, figure, figcaption, footer, form, header, hr, legend,
    listing, main, p, plaintext, pre, search, xmp, article, aside, h1, h2, h3, h4, h5, h6, hgroup, nav, section,
    dir, dd, dl, dt, menu, ol, ul, fieldset, details, summary, optgroup { display: block; }
    li { display: list-item; }
    table { display: table; } tr { display: table-row; } td, th { display: table-cell; }
    area, base, basefont, datalist, head, link, meta, noembed, noframes, param, rp, script, style, template, title { display: none; }
    [hidden] { display: none; }
    body { margin: 8px; }
    p, blockquote, figure, listing, plaintext, pre, xmp, dl, ol, ul, menu, dir { margin-top: 1em; margin-bottom: 1em; }
    h1 { font-size: 2em; margin-top: 0.67em; margin-bottom: 0.67em; font-weight: bold; }
    h2 { font-size: 1.5em; margin-top: 0.83em; margin-bottom: 0.83em; font-weight: bold; }
    h3 { font-size: 1.17em; margin-top: 1em; margin-bottom: 1em; font-weight: bold; }
    b, strong { font-weight: bolder; }
    i, em, cite, var, dfn { font-style: italic; }
    pre, listing, plaintext, xmp, textarea { white-space: pre; }
    input, button, select, textarea, img, canvas, iframe, video { display: inline-block; }
    svg:not(:root) { overflow: hidden; }
    clipPath, defs, linearGradient, radialGradient, marker, mask, pattern, symbol, filter, stop, title, desc { display: none; }
  `;
  let uaSheet = null;

  // ---------------------------------------------------------------- custom elements

  const registry = new Map();
  const constructing = [];
  D.customElements = {
    define(name, ctor) {
      const key = String(name).toLowerCase();
      if (registry.has(key)) throw new Error(`customElements.define: '${name}' has already been defined`);
      if (typeof ctor !== 'function') throw new TypeError('customElements.define: the constructor is not a function');
      registry.set(key, ctor);
      for (const doc of D.documents.values()) {
        D.walkElements(doc, (el) => {
          if (el.localName === key && el.namespaceURI === HTML_NS) upgrade(el);
          return false;
        });
      }
    },
    get(name) {
      return registry.get(String(name).toLowerCase());
    },
    whenDefined(name) {
      const ctor = registry.get(String(name).toLowerCase());
      return ctor ? Promise.resolve(ctor) : new Promise(() => {});
    },
    upgrade(root) {
      D.walkElements(root, (el) => (upgrade(el), false));
    },
    _construct(doc, name) {
      const ctor = registry.get(name);
      if (!ctor) return null;
      constructing.push({ doc, name });
      try {
        return new ctor();
      } finally {
        constructing.pop();
      }
    },
    _pending(ctor) {
      if (constructing.length) return constructing[constructing.length - 1];
      for (const [name, c] of registry) if (c === ctor) return { doc: D.currentDocument(), name };
      throw new TypeError('Illegal constructor');
    },
  };

  // An element made before its definition takes the class on.
  function upgrade(el, connect = true) {
    const ctor = registry.get(el.localName);
    if (!ctor || el instanceof ctor) return;
    Object.setPrototypeOf(el, ctor.prototype);
    if (connect && el._connected) callback(el, 'connectedCallback');
  }

  function callback(el, name, ...args) {
    if (typeof el[name] !== 'function') return;
    try {
      el[name](...args);
    } catch (e) {
      console.error(e);
    }
  }

  // ---------------------------------------------------------------- the document

  D.documents = new Map();

  const isSvgText = (el) => el.namespaceURI === SVG_NS && (el.localName === 'text' || el.localName === 'tspan' || el.localName === 'textPath');
  const REFERENCED = new Set(['clipPath', 'linearGradient', 'radialGradient', 'stop']);

  class ScreenDocument extends D.Document {
    constructor(screen, width, height) {
      super();
      this.__screen = String(screen);
      this.__width = width;
      this.__height = height;
      this._rootFontSize = 16;
      this._sheetsDirty = true;
      this._extraSheets = [];
      this._rules = null;
      this._keys = null;
      this._keyframes = new Map();
      this._fontFaces = [];
      this._pendingSet = new Set();
      this._animated = new Set();
      this._layoutDirty = true;
      this._paintDirty = true;
      this._strings = [];
      this._stringIndex = new Map();
      this._refUsers = new Map();
      this._layoutPass = 0;
      this._hovered = null;
      this._pressed = null;
      this.title = '';
      this.stats = { frames: 0, submits: 0, styled: 0, styleMs: 0, layoutMs: 0, paintMs: 0, ops: 0, strings: 0 };
      this._markSubtree(this.documentElement, MATCH);
    }

    get defaultView() {
      return globalThis;
    }
    get styleSheets() {
      return [];
    }
    get fonts() {
      return { ready: Promise.resolve(), check: () => true, load: () => Promise.resolve([]), addEventListener() {} };
    }

    // Stylesheets not in the tree (a gauge's linked CSS, read by the runtime).
    __addStyleSheet(text) {
      this._extraSheets.push(css.parseStyleSheet(String(text)));
      this._sheetsDirty = true;
    }

    // ------------------------------------------------------------ marking

    _mark(el, bits) {
      if (!el._connected) return;
      el._pending = (el._pending | 0) | bits;
      this._pendingSet.add(el);
    }
    _markSubtree(el, bits) {
      this._mark(el, bits);
      D.walkElements(el, (c) => (this._mark(c, bits), false));
    }

    // Chunks from `node` up are stale. The walk goes to the root: an
    // element may keep no chunk while its ancestors keep theirs.
    _markPaint(node) {
      this._paintDirty = true;
      for (let n = node; n && n.nodeType !== 9; n = n.parentNode) n._chunk = null;
    }
    // Every chunk under `el` is stale (a viewport or string table changed).
    _markPaintDeep(el) {
      const clear = (n) => {
        n._chunk = null;
        for (let c = n._first; c; c = c._next) if (c.nodeType === 1) clear(c);
      };
      clear(el);
      this._markPaint(el);
    }

    // Which element's chunk draws `node`: SVG text is drawn by its <text>,
    // HTML inline content by the block that lays it out.
    _paintOwner(node) {
      let el = node.nodeType === 1 ? node : node.parentNode;
      if (!el || el.nodeType !== 1) return null;
      if (el.namespaceURI === SVG_NS) {
        if (isSvgText(el)) {
          for (let p = el; p && p.nodeType === 1; p = p.parentNode) if (p.localName === 'text') return p;
        }
        return el;
      }
      return el._ifc && el._ifc._connected ? el._ifc : el;
    }

    _referenceChanged(node) {
      for (let p = node.nodeType === 1 ? node : node.parentNode; p && p.nodeType === 1; p = p.parentNode) {
        if (p.namespaceURI === SVG_NS && REFERENCED.has(p.localName)) {
          const id = p.getAttribute('id');
          const users = id && this._refUsers.get(id);
          if (users) for (const u of users) if (u._connected) this._markPaint(u);
        }
      }
    }

    // A change that can move HTML boxes.
    _affectsLayout(node) {
      const el = node.nodeType === 1 ? node : node.parentNode;
      if (!el || el.nodeType !== 1) return false;
      if (el.namespaceURI !== SVG_NS) return true;
      return el.localName === 'svg' && !(el.parentNode && el.parentNode.namespaceURI === SVG_NS);
    }

    // ------------------------------------------------------------ mutation hooks (core.js)

    _nodeInserted(node) {
      const parent = node.parentNode;
      this._idCache = null;
      if (node.nodeType === 1) {
        this._markSubtree(node, MATCH);
        this._scanInserted(node);
        if (this._keys && (this._keys.structural || this._keys.siblingCombinator) && parent && parent.nodeType === 1) {
          for (let c = parent._first; c; c = c._next) if (c.nodeType === 1 && c !== node) this._mark(c, MATCH);
        }
      } else if (node.nodeType === 3 && parent && parent.nodeType === 1 && (parent.localName === 'style')) {
        this._sheetsDirty = true;
      }
      if (parent && parent.nodeType === 1) {
        this._markPaint(this._paintOwner(node.nodeType === 3 ? node : parent) ?? parent);
        this._markPaint(parent);
        if (this._affectsLayout(node.nodeType === 1 ? parent : node)) this._layoutDirty = true;
        this._referenceChanged(parent);
      } else {
        this._paintDirty = true;
        this._layoutDirty = true;
      }
    }
    _scanInserted(root) {
      const visit = (el) => {
        if (el.localName === 'style' || (el.localName === 'link' && /stylesheet/i.test(el.getAttribute('rel') ?? ''))) this._sheetsDirty = true;
        if (el.namespaceURI === HTML_NS && registry.has(el.localName)) upgrade(el, false);
      };
      visit(root);
      D.walkElements(root, (el) => (visit(el), false));
      const connect = (el) => {
        if (typeof el.connectedCallback === 'function' && el.namespaceURI === HTML_NS && registry.has(el.localName)) callback(el, 'connectedCallback');
      };
      connect(root);
      D.walkElements(root, (el) => (el._connected && connect(el), false));
    }
    _nodeRemoving(node) {
      if (node.nodeType !== 1) {
        if (node.parentNode && node.parentNode.localName === 'style') this._sheetsDirty = true;
        return;
      }
      const forget = (el) => {
        if (el.localName === 'style' || el.localName === 'link') this._sheetsDirty = true;
        this._animated.delete(el);
        this._pendingSet.delete(el);
        el._pending = 0;
        el._anim = null;
        el._cs = null;
        el._chunk = null;
        if (this._activeElement === el) this._activeElement = null;
        if (this._hovered === el) this._hovered = null;
      };
      forget(node);
      D.walkElements(node, (el) => (forget(el), false));
    }
    _nodeRemoved(parent, node) {
      this._idCache = null;
      const disconnect = (el) => {
        if (typeof el.disconnectedCallback === 'function' && registry.has(el.localName)) callback(el, 'disconnectedCallback');
      };
      if (node.nodeType === 1) {
        disconnect(node);
        D.walkElements(node, (el) => (disconnect(el), false));
      }
      if (parent.nodeType === 1) {
        this._markPaint(this._paintOwner(node.nodeType === 3 ? parent : parent) ?? parent);
        this._markPaint(parent);
        if (this._affectsLayout(node.nodeType === 1 ? parent : node) || parent.namespaceURI !== SVG_NS) this._layoutDirty = true;
        if (this._keys && (this._keys.structural || this._keys.siblingCombinator)) {
          for (let c = parent._first; c; c = c._next) if (c.nodeType === 1) this._mark(c, MATCH);
        }
        this._referenceChanged(parent);
      }
    }
    _attributeChanged(el, name, old) {
      const keys = this._keys;
      if (name === 'class') {
        const before = old ? old.split(/\s+/) : [];
        const after = el._classes;
        this._mark(el, MATCH);
        if (keys) {
          let deep = false;
          let siblings = false;
          for (const c of before) if (!after.includes(c)) ((deep ||= keys.ancestor.has(`.${c}`)), (siblings ||= keys.sibling.has(`.${c}`)));
          for (const c of after) if (!before.includes(c)) ((deep ||= keys.ancestor.has(`.${c}`)), (siblings ||= keys.sibling.has(`.${c}`)));
          if (deep) this._markSubtree(el, MATCH);
          if (siblings) this._markFollowing(el);
        }
      } else if (name === 'id') {
        this._mark(el, MATCH);
        if (keys && (keys.ancestor.has(`#${old}`) || keys.ancestor.has(`#${el.getAttribute('id')}`))) this._markSubtree(el, MATCH);
        this._idCache = null;
      } else if (keys && keys.attrs.has(name)) {
        this._mark(el, MATCH);
        if (keys.ancestor.has(`[${name}`)) this._markSubtree(el, MATCH);
        if (keys.sibling.has(`[${name}`)) this._markFollowing(el);
      }
      if (el.namespaceURI === SVG_NS) {
        if (css.PRESENTATION.has(name)) this._mark(el, STYLE);
        const owner = this._paintOwner(el);
        this._markPaint(owner);
        if (owner.localName === 'svg' || name === 'viewBox' || name === 'width' || name === 'height' || name === 'x' || name === 'y') {
          if (el.localName === 'svg') this._markPaintDeep(el);
        }
        if (this._affectsLayout(el)) this._layoutDirty = true;
        this._referenceChanged(el);
      } else {
        if (name === 'hidden' || ((name === 'width' || name === 'height' || name === 'src') && (el.localName === 'canvas' || el.localName === 'img'))) {
          this._layoutDirty = true;
          this._markPaint(el);
        }
        if (name === 'data-label' || name.startsWith('data-')) {
          // content: attr(...) reads attributes.
          this._layoutDirty = true;
        }
      }
      const observed = el.constructor && el.constructor.observedAttributes;
      if (observed && registry.has(el.localName) && Array.from(observed).includes(name)) {
        callback(el, 'attributeChangedCallback', name, old ?? null, el.getAttribute(name));
      }
    }
    _markFollowing(el) {
      for (let s = el._next; s; s = s._next) if (s.nodeType === 1) this._markSubtree(s, MATCH);
    }
    _textChanged(text) {
      const parent = text.parentNode;
      if (!parent || parent.nodeType !== 1) return;
      if (parent.localName === 'style') {
        this._sheetsDirty = true;
        return;
      }
      this._markPaint(this._paintOwner(text));
      if (parent.namespaceURI !== SVG_NS) this._layoutDirty = true;
    }
    _textChangedIn(el) {
      this._layoutDirty = true;
      this._markPaint(el);
    }
    _inlineStyleChanged(el) {
      this._mark(el, STYLE);
    }
    _stateChanged(el) {
      this._mark(el, MATCH);
      this._markFollowing(el);
    }
    _scrolled(el) {
      this._markPaint(el);
    }
    _focus(el) {
      const old = this._activeElement;
      if (old === el) return;
      this._activeElement = el;
      const keys = this._keys;
      for (const n of [old, el]) if (n && keys && keys.state) this._mark(n, MATCH);
      if (old) {
        old.dispatchEvent(new D.FocusEvent('blur', { relatedTarget: el }));
        old.dispatchEvent(new D.FocusEvent('focusout', { bubbles: true, relatedTarget: el }));
      }
      if (el) {
        el.dispatchEvent(new D.FocusEvent('focus', { relatedTarget: old }));
        el.dispatchEvent(new D.FocusEvent('focusin', { bubbles: true, relatedTarget: old }));
      }
    }
    // :hover changed for `el` and its ancestors.
    _hoverChanged(el) {
      const keys = this._keys;
      if (!keys || !keys.state) return;
      for (let n = el; n && n.nodeType === 1; n = n.parentNode) {
        if (keys.stateOutsideSubject) this._markSubtree(n, MATCH);
        else this._mark(n, MATCH);
      }
    }

    // ------------------------------------------------------------ strings

    _str(s) {
      let i = this._stringIndex.get(s);
      if (i === undefined) {
        i = this._strings.length;
        this._strings.push(s);
        this._stringIndex.set(s, i);
      }
      return i;
    }

    // ------------------------------------------------------------ stylesheets

    _collectSheets() {
      if (!uaSheet) uaSheet = css.parseStyleSheet(UA_CSS);
      const sheets = [];
      D.walkElements(this, (el) => {
        if (el.localName === 'style') {
          const text = el.textContent;
          if (el._sheetText !== text) {
            el._sheetText = text;
            el._sheet = css.parseStyleSheet(text);
          }
          sheets.push(el._sheet);
        } else if (el.localName === 'link' && /stylesheet/i.test(el.getAttribute('rel') ?? '')) {
          const href = el.getAttribute('href') ?? '';
          if (el._sheetHref !== href) {
            el._sheetHref = href;
            el._sheet = null;
            const text = D.readText(href);
            if (text !== null) el._sheet = css.parseStyleSheet(text);
          }
          if (el._sheet) sheets.push(el._sheet);
        }
        return false;
      });
      sheets.push(...this._extraSheets);
      const rules = [];
      const index = { id: new Map(), cls: new Map(), tag: new Map(), any: [] };
      const keys = { ancestor: new Set(), sibling: new Set(), attrs: new Set(), structural: false, state: false, stateOutsideSubject: false, siblingCombinator: false };
      let order = 0;
      const add = (sheet, origin) => {
        for (const rule of sheet.rules) {
          for (const sel of rule.selectors) {
            const entry = { sel, decls: rule.decls, origin, spec: sel.specificity, order: order++ };
            rules.push(entry);
            selectors.collectKeys(sel, keys);
            const subject = sel.compounds[0];
            const bucket = (map, key) => {
              let list = map.get(key);
              if (!list) map.set(key, (list = []));
              list.push(entry);
            };
            if (subject.id) bucket(index.id, subject.id);
            else if (subject.classes.length) bucket(index.cls, subject.classes[0]);
            else if (subject.tag) bucket(index.tag, subject.tag.toLowerCase());
            else index.any.push(entry);
          }
        }
      };
      add(uaSheet, 0);
      this._keyframes = new Map();
      this._fontFaces = [];
      for (const sheet of sheets) {
        add(sheet, 1);
        for (const [name, frames] of sheet.keyframes) this._keyframes.set(name, frames);
        this._fontFaces.push(...sheet.fontFaces);
      }
      this._rules = index;
      this._keys = keys;
      this._sheetsDirty = false;
      this._markSubtree(this.documentElement, MATCH);
      this._layoutDirty = true;
    }

    // Declaration lists of the rules matching `el` (or its pseudo-element),
    // lowest precedence first.
    _match(el, pseudo) {
      const index = this._rules;
      const found = [];
      const test = (list) => {
        if (!list) return;
        for (const entry of list) {
          if ((entry.sel.pseudoElement ?? null) !== pseudo) continue;
          if (selectors.matchComplex(el, entry.sel, null)) found.push(entry);
        }
      };
      const id = el._attrs.get('id');
      if (id) test(index.id.get(id));
      for (const c of el._classes) test(index.cls.get(c));
      test(index.tag.get(el.localName.toLowerCase()));
      test(index.any);
      if (found.length > 1) {
        // A rule reached through two of its classes is counted once.
        const seen = new Set();
        for (let i = found.length - 1; i >= 0; i--) {
          if (seen.has(found[i])) found.splice(i, 1);
          else seen.add(found[i]);
        }
        found.sort((a, b) => a.origin - b.origin || a.spec - b.spec || a.order - b.order);
      }
      return found.map((e) => e.decls);
    }

    // ------------------------------------------------------------ style resolution

    _styleOf(el, now) {
      if (el._pending & MATCH) {
        el._matched = this._match(el, null);
        el._matchedBefore = this._rules.any.length || this._rules.cls.size ? this._match(el, 'before') : null;
        el._matchedAfter = el._matchedBefore !== null ? this._match(el, 'after') : null;
      }
      el._pending = 0;
      this.stats.styled++;
      const parent = el.parentNode;
      const parentCs = parent && parent.nodeType === 1 ? parent._cs : null;
      let cs = css.computeStyle(el, parentCs, this, el._matched, null, null);
      el._baseCs = cs;
      const names = cs['animation-name'];
      if (!(names.length === 1 && names[0] === 'none')) {
        if (!el._anim) el._anim = { starts: new Map(), values: null };
        const r = css.evaluateAnimations(cs, el._anim, now, this, (p) => css.serialize(p, cs[p]));
        el._anim.values = r ? r.values : null;
        if (r && r.values) cs = css.computeStyle(el, parentCs, this, el._matched, null, r.values);
        if (r && r.active) this._animated.add(el);
        else this._animated.delete(el);
      } else if (el._anim) {
        el._anim = null;
        this._animated.delete(el);
      }
      const pseudo = (list) => (list && list.length ? css.computeStyle(el, cs, this, list, 'x', null) : null);
      const before = pseudo(el._matchedBefore);
      const after = pseudo(el._matchedAfter);
      const pseudoChanged = !samePseudo(el._csBefore, before) || !samePseudo(el._csAfter, after);
      el._csBefore = before;
      el._csAfter = after;
      const old = el._cs;
      el._cs = cs;
      const diff = css.diffStyles(old, cs);
      if (diff || pseudoChanged) {
        if (el.namespaceURI === SVG_NS) {
          this._markPaint(this._paintOwner(el));
          if (el.localName === 'stop' || REFERENCED.has(el.localName)) this._referenceChanged(el);
          if (this._affectsLayout(el) && (!diff || diff.layout)) this._layoutDirty = true;
        } else {
          this._markPaint(this._paintOwner(el));
          this._markPaint(el);
          if (!diff || diff.layout || pseudoChanged) this._layoutDirty = true;
        }
        if (diff && diff.inherited) for (let c = el._first; c; c = c._next) if (c.nodeType === 1) this._mark(c, STYLE);
        if (el === this.documentElement && diff && old && old['font-size'] !== cs['font-size']) this._rootFontSize = cs['font-size'];
      }
    }

    // Resolve every pending style, parents first.
    _resolveStyles(now) {
      if (this._sheetsDirty) this._collectSheets();
      // Running animations whose values moved.
      for (const el of this._animated) {
        if (!el._connected || !el._baseCs) continue;
        const r = css.evaluateAnimations(el._baseCs, el._anim, now, this, (p) => css.serialize(p, el._baseCs[p]));
        if (!sameValues(r && r.values, el._anim.values) || !(r && r.active)) this._mark(el, STYLE);
      }
      const set = this._pendingSet;
      const ensure = (el) => {
        const parent = el.parentNode;
        if (parent && parent.nodeType === 1 && parent._pending) ensure(parent);
        if (el._pending || !el._cs) this._styleOf(el, now);
      };
      for (const el of set) {
        if (el._connected && (el._pending || !el._cs)) ensure(el);
      }
      set.clear();
    }

    // Styles and layout, current (for geometry queries and before painting).
    _flush() {
      const now = globalThis.performance ? performance.now() : 0;
      if (this._pendingSet.size || this._sheetsDirty) this._resolveStyles(now);
      if (this._layoutDirty) {
        this._layoutDirty = false;
        D.layout.run(this);
      }
    }

    // One frame: resolve, lay out, paint, and submit if anything changed.
    frame() {
      this.stats.frames++;
      const clock = D.wallClock;
      const a = clock();
      const now = globalThis.performance ? performance.now() : 0;
      if (this._pendingSet.size || this._sheetsDirty || this._animated.size) this._resolveStyles(now);
      const b = clock();
      if (this._layoutDirty) {
        this._layoutDirty = false;
        D.layout.run(this);
      }
      const c = clock();
      this.stats.styleMs = b - a;
      this.stats.layoutMs = c - b;
      if (!this._paintDirty) {
        this.stats.paintMs = 0;
        return false;
      }
      this._paintDirty = false;
      const out = D.paint.document(this);
      this.stats.paintMs = clock() - c;
      this.stats.ops = out.length;
      this.stats.strings = this._strings.length;
      this.stats.submits++;
      const host = globalThis.__host;
      // A headless view (no VCockpit texture, e.g. systems-host/extras-host's
      // JS-only backplane instruments: js/msfs/mod.rs's `Cockpit::new` gives
      // these an empty screen name) has nothing to submit to; skip the call
      // instead of asking the host for a screen that was never meant to exist.
      if (this.__screen && host && typeof host.submitDisplay === 'function') {
        try {
          host.submitDisplay(this.__screen, out, this._strings);
        } catch (e) {
          warnOnce(`submit:${this.__screen}:${e}`, `submitDisplay(${this.__screen}) failed: ${e}`);
        }
      }
      this._lastOps = out;
      return true;
    }

    // Geometry for core.js.
    _clientRect(el) {
      this._flush();
      return D.paint.clientRect(this, el);
    }
    _bbox(el) {
      this._flush();
      return D.paint.bbox(this, el);
    }
    _textLength(el) {
      this._flush();
      return D.paint.textLength(this, el);
    }
    _offset(el) {
      this._flush();
      const box = el._box;
      if (!box) return { left: 0, top: 0 };
      let left = box.x;
      let top = box.y;
      const parent = el.parentElement;
      for (let p = parent; p && p._box; p = p.parentElement) {
        const cs = p._cs;
        if (cs && (cs.position !== 'static' || p === this.body)) break;
        left += p._box.x;
        top += p._box.y;
      }
      return { left, top };
    }
    _scrollHeight(el) {
      this._flush();
      const box = el._box;
      return box ? Math.max(box.h, box.contentH ?? 0) : 0;
    }
  }

  const samePseudo = (a, b) => {
    if (!a || !b) return a === b;
    return css.diffStyles(a, b) === null;
  };
  const sameValues = (a, b) => {
    if (!a || !b) return a === b;
    const ka = Object.keys(a);
    if (ka.length !== Object.keys(b).length) return false;
    for (const k of ka) if (a[k] !== b[k]) return false;
    return true;
  };

  // Text a stylesheet link names, through the host's file reader
  // (coui://html_ui paths).
  D.readText = (url) => {
    const host = globalThis.__host;
    if (host && typeof host.readFile === 'function') {
      try {
        return String(host.readFile(String(url)));
      } catch (e) {
        warnOnce(`readFile:${url}`, `stylesheet ${url} could not be read: ${e}`);
        return null;
      }
    }
    warnOnce('readFile', `stylesheet ${url} cannot be read: the host has no readFile (use document.__addStyleSheet)`);
    return null;
  };

  // Wall time in ms for the cost figures (the engine clock is the
  // simulator's and stands still within a frame).
  D.wallClock = () => {
    const host = globalThis.__host;
    return host && typeof host.wallClock === 'function' ? host.wallClock() : Date.now();
  };

  // A read-only view of an element's computed style.
  D.getComputedStyle = (el, pseudo) => {
    const doc = el && el.ownerDocument;
    if (doc && doc._flush) doc._flush();
    const which = pseudo === '::before' || pseudo === ':before' ? '_csBefore' : pseudo === '::after' || pseudo === ':after' ? '_csAfter' : '_cs';
    const cs = (el && el[which]) || css.INITIAL;
    const get = (name) => {
      const prop = name.startsWith('--') ? name : D.camelToKebab(name);
      if (prop === 'width' || prop === 'height') {
        const box = el && el._box;
        if (box) return `${prop === 'width' ? box.w : box.h}px`;
      }
      if (prop.startsWith('--')) return cs[prop] ?? '';
      if (css.PROPS[prop] === undefined) return '';
      return css.serialize(prop, cs[prop]);
    };
    return new Proxy(
      { getPropertyValue: (name) => get(String(name)) },
      {
        get(target, prop) {
          if (prop in target) return target[prop];
          return typeof prop === 'string' ? get(prop) : undefined;
        },
      },
    );
  };

  D.ScreenDocument = ScreenDocument;
  D.MATCH = MATCH;
  D.STYLE = STYLE;
})();
