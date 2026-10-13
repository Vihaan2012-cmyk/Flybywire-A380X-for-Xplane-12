// The DOM: nodes, elements (HTML and SVG, with namespaces), attributes,
// classList, style declarations, events with capture and bubble, and an
// HTML fragment parser for innerHTML/insertAdjacentHTML.
//
// Nodes keep their children as a linked list (FBW empties containers with
// `while (el.firstChild) el.removeChild(el.firstChild)`). Changes to
// connected nodes are reported to their document (document.js), which
// turns them into restyle, relayout and repaint work.
(() => {
  const D = globalThis.__dom;
  const warnOnce = D.warnOnce;
  const HTML_NS = 'http://www.w3.org/1999/xhtml';
  const SVG_NS = 'http://www.w3.org/2000/svg';
  const XLINK_NS = 'http://www.w3.org/1999/xlink';
  const XML_NS = 'http://www.w3.org/XML/1998/namespace';
  D.NS = { HTML: HTML_NS, SVG: SVG_NS, XLINK: XLINK_NS };

  // The document scripts see as `document`, for nodes made with `new`.
  D.currentDocument = () => (globalThis.document instanceof D.Document ? globalThis.document : null);

  const domError = (name, message) => {
    const e = new Error(message);
    e.name = name;
    return e;
  };

  // ---------------------------------------------------------------- events

  class Event {
    constructor(type, init = {}) {
      this.type = String(type);
      this.bubbles = !!init.bubbles;
      this.cancelable = !!init.cancelable;
      this.composed = !!init.composed;
      this.defaultPrevented = false;
      this.target = null;
      this.currentTarget = null;
      this.eventPhase = 0;
      this.isTrusted = false;
      this.timeStamp = typeof performance !== 'undefined' ? performance.now() : 0;
      this._stop = false;
      this._stopNow = false;
      this._path = [];
    }
    get srcElement() {
      return this.target;
    }
    get returnValue() {
      return !this.defaultPrevented;
    }
    set returnValue(v) {
      if (!v) this.preventDefault();
    }
    get cancelBubble() {
      return this._stop;
    }
    set cancelBubble(v) {
      if (v) this._stop = true;
    }
    preventDefault() {
      if (this.cancelable) this.defaultPrevented = true;
    }
    stopPropagation() {
      this._stop = true;
    }
    stopImmediatePropagation() {
      this._stop = true;
      this._stopNow = true;
    }
    composedPath() {
      return this._path.slice();
    }
    initEvent(type, bubbles, cancelable) {
      this.type = type;
      this.bubbles = !!bubbles;
      this.cancelable = !!cancelable;
    }
  }
  Event.NONE = 0;
  Event.CAPTURING_PHASE = 1;
  Event.AT_TARGET = 2;
  Event.BUBBLING_PHASE = 3;

  class CustomEvent extends Event {
    constructor(type, init = {}) {
      super(type, init);
      this.detail = init.detail === undefined ? null : init.detail;
    }
    initCustomEvent(type, bubbles, cancelable, detail) {
      this.initEvent(type, bubbles, cancelable);
      this.detail = detail;
    }
  }

  class UIEvent extends Event {
    constructor(type, init = {}) {
      super(type, init);
      this.view = init.view ?? null;
      this.detail = init.detail ?? 0;
    }
  }

  class MouseEvent extends UIEvent {
    constructor(type, init = {}) {
      super(type, init);
      this.screenX = init.screenX ?? 0;
      this.screenY = init.screenY ?? 0;
      this.clientX = init.clientX ?? 0;
      this.clientY = init.clientY ?? 0;
      this.button = init.button ?? 0;
      this.buttons = init.buttons ?? 0;
      this.ctrlKey = !!init.ctrlKey;
      this.shiftKey = !!init.shiftKey;
      this.altKey = !!init.altKey;
      this.metaKey = !!init.metaKey;
      this.relatedTarget = init.relatedTarget ?? null;
      this.offsetX = init.offsetX ?? this.clientX;
      this.offsetY = init.offsetY ?? this.clientY;
    }
    get pageX() {
      return this.clientX;
    }
    get pageY() {
      return this.clientY;
    }
    get x() {
      return this.clientX;
    }
    get y() {
      return this.clientY;
    }
    getModifierState() {
      return false;
    }
  }

  class PointerEvent extends MouseEvent {
    constructor(type, init = {}) {
      super(type, init);
      this.pointerId = init.pointerId ?? 1;
      this.pointerType = init.pointerType ?? 'mouse';
      this.isPrimary = init.isPrimary ?? true;
      this.width = 1;
      this.height = 1;
      this.pressure = init.pressure ?? 0;
    }
  }

  class WheelEvent extends MouseEvent {
    constructor(type, init = {}) {
      super(type, init);
      this.deltaX = init.deltaX ?? 0;
      this.deltaY = init.deltaY ?? 0;
      this.deltaZ = init.deltaZ ?? 0;
      this.deltaMode = init.deltaMode ?? 0;
    }
  }
  WheelEvent.DOM_DELTA_PIXEL = 0;
  WheelEvent.DOM_DELTA_LINE = 1;
  WheelEvent.DOM_DELTA_PAGE = 2;

  class KeyboardEvent extends UIEvent {
    constructor(type, init = {}) {
      super(type, init);
      this.key = init.key ?? '';
      this.code = init.code ?? '';
      this.keyCode = init.keyCode ?? 0;
      this.charCode = init.charCode ?? 0;
      this.which = init.which ?? this.keyCode;
      this.ctrlKey = !!init.ctrlKey;
      this.shiftKey = !!init.shiftKey;
      this.altKey = !!init.altKey;
      this.metaKey = !!init.metaKey;
      this.repeat = !!init.repeat;
    }
    getModifierState() {
      return false;
    }
  }

  class FocusEvent extends UIEvent {
    constructor(type, init = {}) {
      super(type, init);
      this.relatedTarget = init.relatedTarget ?? null;
    }
  }

  const listenerOptions = (options) => {
    if (typeof options === 'boolean') return { capture: options, once: false, passive: false };
    if (!options) return { capture: false, once: false, passive: false };
    return { capture: !!options.capture, once: !!options.once, passive: !!options.passive };
  };

  // addEventListener/removeEventListener/dispatchEvent for any object.
  const eventTargetMethods = {
    addEventListener(type, fn, options) {
      if (!fn) return;
      const o = listenerOptions(options);
      if (!this._listeners) this._listeners = new Map();
      let list = this._listeners.get(type);
      if (!list) this._listeners.set(type, (list = []));
      for (const l of list) if (l.fn === fn && l.capture === o.capture) return;
      list.push({ fn, capture: o.capture, once: o.once, passive: o.passive, removed: false });
    },
    removeEventListener(type, fn, options) {
      const list = this._listeners && this._listeners.get(type);
      if (!list) return;
      const capture = listenerOptions(options).capture;
      const i = list.findIndex((l) => l.fn === fn && l.capture === capture);
      if (i >= 0) {
        list[i].removed = true;
        list.splice(i, 1);
      }
    },
    dispatchEvent(event) {
      return dispatch(this, event);
    },
  };

  class EventTarget {}
  Object.assign(EventTarget.prototype, eventTargetMethods);

  const parentForEvents = (node) => {
    if (node.nodeType === 9) return node.defaultView ?? null;
    return node.parentNode ?? null;
  };

  // Run `node`'s listeners for the event: capture listeners when
  // `capture`, the others when not.
  function invoke(node, event, capture) {
    const list = node._listeners && node._listeners.get(event.type);
    event.currentTarget = node;
    if (list && list.length) {
      for (const l of list.slice()) {
        if (l.removed || l.capture !== capture) continue;
        if (l.once) eventTargetMethods.removeEventListener.call(node, event.type, l.fn, { capture: l.capture });
        try {
          if (typeof l.fn === 'function') l.fn.call(node, event);
          else if (l.fn && typeof l.fn.handleEvent === 'function') l.fn.handleEvent(event);
        } catch (e) {
          console.error(e);
        }
        if (event._stopNow) break;
      }
    }
    // on<type> handler properties, as a non-capture listener.
    if (!capture && !event._stopNow && node !== globalThis) {
      const handler = node[`on${event.type}`];
      if (typeof handler === 'function') {
        try {
          if (handler.call(node, event) === false) event.preventDefault();
        } catch (e) {
          console.error(e);
        }
      }
    }
  }

  function dispatch(target, event) {
    event.target = target;
    const path = [];
    for (let n = target; n; n = parentForEvents(n)) path.push(n);
    event._path = path;
    event._stop = false;
    event._stopNow = false;
    event.eventPhase = 1;
    for (let i = path.length - 1; i > 0 && !event._stop; i--) invoke(path[i], event, true);
    if (!event._stop) {
      // At the target, capture listeners run before the others.
      event.eventPhase = 2;
      invoke(target, event, true);
      if (!event._stopNow) invoke(target, event, false);
    }
    if (event.bubbles) {
      event.eventPhase = 3;
      for (let i = 1; i < path.length && !event._stop; i++) invoke(path[i], event, false);
    }
    event.eventPhase = 0;
    event.currentTarget = null;
    return !event.defaultPrevented;
  }

  // ---------------------------------------------------------------- nodes

  const docOf = (node) => (node.nodeType === 9 ? node : node.ownerDocument);

  const setConnected = (node, connected, doc) => {
    // The document's elements by tag name, once a query has asked for it.
    if (node.nodeType === 1) {
      const owner = doc ?? node.ownerDocument;
      const index = owner && owner._tagIndex;
      if (index && node._connected !== connected) {
        let set = index.get(node.localName);
        if (connected) {
          if (!set) index.set(node.localName, (set = new Set()));
          set.add(node);
        } else if (set) {
          set.delete(node);
        }
      }
    }
    node._connected = connected;
    if (doc && node.ownerDocument !== doc && node.nodeType !== 9) node.ownerDocument = doc;
    for (let c = node._first; c; c = c._next) setConnected(c, connected, doc);
  };

  // The connected elements with a tag name, from the document's index
  // (built on first use, kept by setConnected).
  const elementsByTag = (doc, tag) => {
    if (!doc._tagIndex) {
      const index = new Map();
      walkElements(doc, (el) => {
        let set = index.get(el.localName);
        if (!set) index.set(el.localName, (set = new Set()));
        set.add(el);
        return false;
      });
      doc._tagIndex = index;
    }
    return doc._tagIndex.get(tag);
  };

  // `tag > compound` from the document: the children of that tag's
  // elements, without walking the whole tree. `null` when the list is not
  // of that form or has more than one parent to order.
  const childQuery = (root, list, first) => {
    if (root.nodeType !== 9 || list.length !== 1) return null;
    const sel = list[0];
    if (sel.compounds.length !== 2 || sel.combinators[0] !== '>') return null;
    const parent = sel.compounds[1];
    if (!parent.tag || parent.pseudos.length) return null;
    const parents = elementsByTag(root, parent.tag);
    if (!parents || parents.size === 0) return [];
    if (parents.size > 1) return null;
    const [p] = parents;
    if (!p._connected || !D.selectors.matchCompound(p, parent, null)) return [];
    const out = [];
    for (let c = p._first; c; c = c._next) {
      if (c.nodeType === 1 && D.selectors.matchCompound(c, sel.compounds[0], null)) {
        out.push(c);
        if (first) break;
      }
    }
    return out;
  };

  class Node extends EventTarget {
    constructor(doc) {
      super();
      this.ownerDocument = doc;
      this.parentNode = null;
      this._first = null;
      this._last = null;
      this._next = null;
      this._prev = null;
      this._connected = false;
      this._childArray = null;
    }
    get firstChild() {
      return this._first;
    }
    get lastChild() {
      return this._last;
    }
    get nextSibling() {
      return this._next;
    }
    get previousSibling() {
      return this._prev;
    }
    get parentElement() {
      const p = this.parentNode;
      return p && p.nodeType === 1 ? p : null;
    }
    get isConnected() {
      return this._connected;
    }
    get childNodes() {
      if (!this._childArray) {
        const out = [];
        for (let c = this._first; c; c = c._next) out.push(c);
        this._childArray = out;
      }
      return this._childArray;
    }
    hasChildNodes() {
      return this._first !== null;
    }
    getRootNode() {
      let n = this;
      while (n.parentNode) n = n.parentNode;
      return n;
    }
    contains(other) {
      for (let n = other; n; n = n.parentNode) if (n === this) return true;
      return false;
    }
    appendChild(node) {
      return this._insert(node, null);
    }
    insertBefore(node, ref) {
      if (ref && ref.parentNode !== this) throw domError('NotFoundError', 'The node before which the new node is to be inserted is not a child of this node.');
      return this._insert(node, ref ?? null);
    }
    removeChild(node) {
      if (!node || node.parentNode !== this) throw domError('NotFoundError', 'The node to be removed is not a child of this node.');
      this._remove(node);
      return node;
    }
    replaceChild(node, old) {
      if (!old || old.parentNode !== this) throw domError('NotFoundError', 'The node to be replaced is not a child of this node.');
      if (node === old) return old;
      const ref = old._next === node ? node._next : old._next;
      this._remove(old);
      this._insert(node, ref);
      return old;
    }
    _insert(node, ref) {
      if (!node || !(node instanceof Node)) throw new TypeError("Failed to execute 'insertBefore' on 'Node': parameter 1 is not of type 'Node'.");
      if (node.nodeType === 11) {
        for (let c = node._first; c; ) {
          const next = c._next;
          this._insert(c, ref);
          c = next;
        }
        return node;
      }
      if (node.nodeType === 9 || node.contains(this)) throw domError('HierarchyRequestError', 'The new child element contains the parent.');
      if (node === ref) ref = node._next;
      if (node.parentNode) node.parentNode._remove(node);
      node.parentNode = this;
      if (ref) {
        node._next = ref;
        node._prev = ref._prev;
        if (ref._prev) ref._prev._next = node;
        else this._first = node;
        ref._prev = node;
      } else {
        node._prev = this._last;
        node._next = null;
        if (this._last) this._last._next = node;
        else this._first = node;
        this._last = node;
      }
      this._childArray = null;
      if (this.nodeType === 1) this._elementChildren = null;
      const doc = docOf(this);
      if (node.ownerDocument !== doc) setConnected(node, node._connected, doc);
      if (this._connected) {
        setConnected(node, true, doc);
        doc._nodeInserted(node);
      }
      return node;
    }
    _remove(node) {
      const wasConnected = node._connected;
      if (wasConnected) docOf(this)._nodeRemoving(node);
      if (node._prev) node._prev._next = node._next;
      else this._first = node._next;
      if (node._next) node._next._prev = node._prev;
      else this._last = node._prev;
      const next = node._next;
      node._prev = node._next = null;
      node.parentNode = null;
      this._childArray = null;
      if (this.nodeType === 1) this._elementChildren = null;
      if (wasConnected) {
        setConnected(node, false, null);
        docOf(this)._nodeRemoved(this, node, next);
      }
    }
    get textContent() {
      if (this.nodeType === 9) return null;
      let out = '';
      const walk = (n) => {
        for (let c = n._first; c; c = c._next) {
          if (c.nodeType === 3) out += c._data;
          else if (c.nodeType === 1 || c.nodeType === 11) walk(c);
        }
      };
      walk(this);
      return out;
    }
    set textContent(value) {
      if (this.nodeType === 9) return;
      const text = value === null || value === undefined ? '' : String(value);
      // Reuse a lone text child, which is what React and FSComponent update.
      if (this._first && this._first === this._last && this._first.nodeType === 3) {
        if (text !== '') {
          this._first.data = text;
          return;
        }
      }
      while (this._first) this._remove(this._first);
      if (text !== '') this._insert(docOf(this).createTextNode(text), null);
    }
    get nodeValue() {
      return null;
    }
    set nodeValue(_v) {}
    cloneNode(deep = false) {
      const copy = this._cloneShallow();
      if (deep) for (let c = this._first; c; c = c._next) copy._insert(c.cloneNode(true), null);
      return copy;
    }
    isSameNode(other) {
      return this === other;
    }
    compareDocumentPosition(other) {
      if (this === other) return 0;
      if (this.contains(other)) return 20;
      if (other.contains(this)) return 10;
      const order = (n) => {
        const chain = [];
        for (let x = n; x; x = x.parentNode) chain.unshift(x);
        return chain;
      };
      const a = order(this);
      const b = order(other);
      if (a[0] !== b[0]) return 1 | 32;
      let i = 0;
      while (a[i] === b[i]) i++;
      for (let s = a[i]; s; s = s._next) if (s === b[i]) return 4;
      return 2;
    }
    normalize() {}
  }
  const NODE_TYPES = {
    ELEMENT_NODE: 1, ATTRIBUTE_NODE: 2, TEXT_NODE: 3, CDATA_SECTION_NODE: 4, PROCESSING_INSTRUCTION_NODE: 7,
    COMMENT_NODE: 8, DOCUMENT_NODE: 9, DOCUMENT_TYPE_NODE: 10, DOCUMENT_FRAGMENT_NODE: 11,
  };
  Object.assign(Node, NODE_TYPES);
  Object.assign(Node.prototype, NODE_TYPES);

  class CharacterData extends Node {
    constructor(doc, data) {
      super(doc);
      this._data = String(data);
    }
    get data() {
      return this._data;
    }
    set data(value) {
      const v = value === null || value === undefined ? '' : String(value);
      if (v === this._data) return;
      this._data = v;
      if (this._connected && this.nodeType === 3) this.ownerDocument._textChanged(this);
    }
    get nodeValue() {
      return this._data;
    }
    set nodeValue(v) {
      this.data = v;
    }
    get textContent() {
      return this._data;
    }
    set textContent(v) {
      this.data = v;
    }
    get length() {
      return this._data.length;
    }
    appendData(s) {
      this.data = this._data + s;
    }
    remove() {
      if (this.parentNode) this.parentNode._remove(this);
    }
    before(...nodes) {
      insertNodes(this.parentNode, this, nodes, this.ownerDocument);
    }
    after(...nodes) {
      insertNodes(this.parentNode, this._next, nodes, this.ownerDocument);
    }
    replaceWith(...nodes) {
      const parent = this.parentNode;
      if (!parent) return;
      const next = this._next;
      parent._remove(this);
      insertNodes(parent, next, nodes, this.ownerDocument);
    }
    get nextElementSibling() {
      return D.selectors.nextElement(this);
    }
    get previousElementSibling() {
      return D.selectors.prevElement(this);
    }
  }

  class Text extends CharacterData {
    constructor(data = '') {
      super(D.currentDocument(), data);
    }
    get nodeType() {
      return 3;
    }
    get nodeName() {
      return '#text';
    }
    get wholeText() {
      return this._data;
    }
    _cloneShallow() {
      return this.ownerDocument.createTextNode(this._data);
    }
  }

  class Comment extends CharacterData {
    constructor(data = '') {
      super(D.currentDocument(), data);
    }
    get nodeType() {
      return 8;
    }
    get nodeName() {
      return '#comment';
    }
    _cloneShallow() {
      return this.ownerDocument.createComment(this._data);
    }
  }

  const insertNodes = (parent, ref, nodes, doc) => {
    if (!parent) return;
    for (const n of nodes) parent._insert(typeof n === 'string' ? doc.createTextNode(n) : n, ref);
  };

  // Mixin for Element, Document and DocumentFragment.
  const parentNodeMethods = {
    get children() {
      if (!this._elementChildren) {
        const out = [];
        for (let c = this._first; c; c = c._next) if (c.nodeType === 1) out.push(c);
        this._elementChildren = out;
      }
      return this._elementChildren;
    },
    get childElementCount() {
      return this.children.length;
    },
    get firstElementChild() {
      let c = this._first;
      while (c && c.nodeType !== 1) c = c._next;
      return c;
    },
    get lastElementChild() {
      let c = this._last;
      while (c && c.nodeType !== 1) c = c._prev;
      return c;
    },
    append(...nodes) {
      insertNodes(this, null, nodes, docOf(this));
    },
    prepend(...nodes) {
      insertNodes(this, this._first, nodes, docOf(this));
    },
    replaceChildren(...nodes) {
      while (this._first) this._remove(this._first);
      insertNodes(this, null, nodes, docOf(this));
    },
    querySelector(selector) {
      const list = D.selectors.parseList(String(selector));
      const quick = childQuery(this, list, true);
      if (quick) return quick[0] ?? null;
      const scope = this.nodeType === 1 ? this : null;
      let found = null;
      walkElements(this, (el) => {
        if (list.some((s) => D.selectors.matchComplex(el, s, scope))) {
          found = el;
          return true;
        }
        return false;
      });
      return found;
    },
    querySelectorAll(selector) {
      const list = D.selectors.parseList(String(selector));
      const quick = childQuery(this, list, false);
      if (quick) return quick;
      const scope = this.nodeType === 1 ? this : null;
      const out = [];
      walkElements(this, (el) => {
        if (list.some((s) => D.selectors.matchComplex(el, s, scope))) out.push(el);
        return false;
      });
      return out;
    },
    getElementsByTagName(name) {
      const out = [];
      const all = name === '*';
      const lower = String(name).toLowerCase();
      walkElements(this, (el) => {
        if (all || el.localName === name || (el.namespaceURI === HTML_NS && el.localName === lower)) out.push(el);
        return false;
      });
      return out;
    },
    getElementsByClassName(names) {
      const wanted = String(names).split(/\s+/).filter(Boolean);
      const out = [];
      walkElements(this, (el) => {
        if (wanted.every((c) => el._classes.includes(c))) out.push(el);
        return false;
      });
      return out;
    },
  };

  // Depth-first over descendant elements; stop when `fn` returns true.
  function walkElements(root, fn) {
    for (let c = root._first; c; c = c._next) {
      if (c.nodeType !== 1) continue;
      if (fn(c)) return true;
      if (walkElements(c, fn)) return true;
    }
    return false;
  }
  D.walkElements = walkElements;

  // ---------------------------------------------------------------- classList

  class DOMTokenList {
    constructor(el) {
      this._el = el;
    }
    get length() {
      return this._el._classes.length;
    }
    get value() {
      return this._el.getAttribute('class') ?? '';
    }
    set value(v) {
      this._el.setAttribute('class', v);
    }
    item(i) {
      return this._el._classes[i] ?? null;
    }
    contains(token) {
      return this._el._classes.includes(token);
    }
    _write(list) {
      this._el._setAttr('class', list.join(' '));
    }
    add(...tokens) {
      const list = this._el._classes.slice();
      let changed = false;
      for (const t of tokens) {
        checkToken(t);
        if (!list.includes(t)) {
          list.push(t);
          changed = true;
        }
      }
      if (changed || !this._el._attrs.has('class')) this._write(list);
    }
    remove(...tokens) {
      const list = this._el._classes.filter((c) => !tokens.includes(c));
      if (list.length !== this._el._classes.length) this._write(list);
    }
    toggle(token, force) {
      checkToken(token);
      const has = this.contains(token);
      const want = force === undefined ? !has : !!force;
      if (want && !has) this.add(token);
      else if (!want && has) this.remove(token);
      return want;
    }
    replace(oldToken, newToken) {
      const list = this._el._classes.slice();
      const i = list.indexOf(oldToken);
      if (i < 0) return false;
      if (list.includes(newToken)) list.splice(i, 1);
      else list[i] = newToken;
      this._write(list);
      return true;
    }
    supports() {
      return true;
    }
    forEach(fn, thisArg) {
      this._el._classes.slice().forEach((c, i) => fn.call(thisArg, c, i, this));
    }
    toString() {
      return this.value;
    }
    [Symbol.iterator]() {
      return this._el._classes.slice()[Symbol.iterator]();
    }
  }
  const checkToken = (t) => {
    if (t === '') throw domError('SyntaxError', 'The token provided must not be empty.');
    if (/\s/.test(t)) throw domError('InvalidCharacterError', `The token provided ('${t}') contains HTML space characters.`);
  };

  // ---------------------------------------------------------------- style

  const camelToKebab = (name) => {
    if (name === 'cssFloat') return 'float';
    if (name.startsWith('webkit') || name.startsWith('Webkit')) return `-webkit-${name.slice(6).replace(/[A-Z]/g, (c) => `-${c.toLowerCase()}`).replace(/^-/, '')}`;
    return name.replace(/[A-Z]/g, (c) => `-${c.toLowerCase()}`);
  };

  class CSSStyleDeclaration {
    constructor(el) {
      this._el = el;
      this._decls = new Map(); // longhand -> {value, important}
      this._shorthands = new Map(); // shorthand -> last value set, for reads
    }
    get length() {
      return this._decls.size;
    }
    item(i) {
      return [...this._decls.keys()][i] ?? '';
    }
    getPropertyValue(prop) {
      const name = prop.startsWith('--') ? prop : prop.toLowerCase();
      const d = this._decls.get(name);
      if (d) return d.value;
      return this._shorthands.get(name) ?? '';
    }
    getPropertyPriority(prop) {
      const d = this._decls.get(prop.toLowerCase());
      return d && d.important ? 'important' : '';
    }
    setProperty(prop, value, priority) {
      const name = String(prop).startsWith('--') ? String(prop) : String(prop).trim().toLowerCase();
      if (value === null || value === undefined || String(value).trim() === '') {
        this.removeProperty(name);
        return;
      }
      let text = String(value).trim();
      let important = priority === 'important';
      const bang = /!\s*important\s*$/i.exec(text);
      if (bang) {
        // Browsers reject "!important" inside a value; FBW never relies on it.
        text = text.slice(0, bang.index).trim();
        important = true;
      }
      const longhands = name.startsWith('--') ? [[name, text]] : D.css.expand(name, text);
      if (longhands.length !== 1 || longhands[0][0] !== name) this._shorthands.set(name, text);
      let changed = false;
      for (const [p, v] of longhands) {
        const old = this._decls.get(p);
        if (old && old.value === v && old.important === important) continue;
        // Re-setting a property moves it last, as in the browser's cssText.
        this._decls.delete(p);
        this._decls.set(p, { value: v, important });
        changed = true;
      }
      if (changed) this._changed();
    }
    removeProperty(prop) {
      const name = prop.startsWith('--') ? prop : prop.toLowerCase();
      const old = this.getPropertyValue(name);
      let changed = false;
      const longhands = name.startsWith('--') ? [[name]] : D.css.expand(name, 'initial');
      for (const [p] of longhands) if (this._decls.delete(p)) changed = true;
      this._shorthands.delete(name);
      if (changed) this._changed();
      return old;
    }
    get cssText() {
      const parts = [];
      for (const [p, d] of this._decls) parts.push(`${p}: ${d.value}${d.important ? ' !important' : ''};`);
      return parts.join(' ');
    }
    set cssText(text) {
      this._parse(String(text ?? ''));
      this._changed();
    }
    // Replace all declarations from a style attribute string.
    _parse(text) {
      this._decls.clear();
      this._shorthands.clear();
      for (const [p, v, important] of D.css.parseDeclarations(text)) {
        this._decls.delete(p);
        this._decls.set(p, { value: v, important });
      }
    }
    _changed() {
      const el = this._el;
      el._styleAttrStale = true;
      if (el._connected) el.ownerDocument._inlineStyleChanged(el);
    }
  }

  // camelCase and kebab-case accessors (style.display, style.backgroundColor,
  // style['stroke-width']) for every known property and shorthand.
  {
    const names = new Set([
      ...Object.keys(D.css.PROPS),
      'margin', 'padding', 'border', 'border-top', 'border-right', 'border-bottom', 'border-left', 'border-width',
      'border-style', 'border-color', 'border-radius', 'outline', 'background', 'flex', 'flex-flow', 'gap', 'grid-gap',
      'grid-column', 'grid-row', 'place-content', 'place-items', 'place-self', 'overflow', 'text-decoration', 'font',
      'animation', 'transition', 'inset', 'float', 'cursor', 'user-select', 'list-style', 'appearance',
    ]);
    for (const kebab of names) {
      const camel = kebab.replace(/-([a-z])/g, (_, c) => c.toUpperCase());
      const desc = {
        get() {
          return this.getPropertyValue(kebab);
        },
        set(v) {
          this.setProperty(kebab, v);
        },
        configurable: true,
      };
      Object.defineProperty(CSSStyleDeclaration.prototype, camel, desc);
      if (camel !== kebab) Object.defineProperty(CSSStyleDeclaration.prototype, kebab, desc);
    }
    Object.defineProperty(CSSStyleDeclaration.prototype, 'cssFloat', {
      get() {
        return this.getPropertyValue('float');
      },
      set(v) {
        this.setProperty('float', v);
      },
    });
  }
  D.camelToKebab = camelToKebab;

  // ---------------------------------------------------------------- elements

  const VOID = new Set(['area', 'base', 'br', 'col', 'embed', 'hr', 'img', 'input', 'link', 'meta', 'source', 'track', 'wbr']);

  class Element extends Node {
    constructor(doc, namespace, qualifiedName) {
      // A custom element's own constructor calls super() with no arguments:
      // its name and document are the ones being created or upgraded.
      if (qualifiedName === undefined) {
        const pending = D.customElements._pending(new.target);
        doc = pending.doc;
        namespace = HTML_NS;
        qualifiedName = pending.name;
      }
      super(doc);
      this.namespaceURI = namespace;
      const colon = qualifiedName.indexOf(':');
      this.prefix = colon >= 0 ? qualifiedName.slice(0, colon) : null;
      this.localName = colon >= 0 ? qualifiedName.slice(colon + 1) : qualifiedName;
      this._qualified = qualifiedName;
      this._attrs = new Map();
      this._attrNS = null;
      this._classes = [];
      this._style = null;
      this._classList = null;
      this._elementChildren = null;
      this._styleAttrStale = false;
    }
    get nodeType() {
      return 1;
    }
    get tagName() {
      return this.namespaceURI === HTML_NS ? this._qualified.toUpperCase() : this._qualified;
    }
    get nodeName() {
      return this.tagName;
    }
    _cloneShallow() {
      const copy = this.ownerDocument.createElementNS(this.namespaceURI, this._qualified);
      for (const [name, value] of this._attrs) copy._setAttr(name, name === 'style' ? this.getAttribute('style') : value);
      return copy;
    }

    // Attributes.
    _name(name) {
      if (this.namespaceURI !== HTML_NS) return typeof name === 'string' ? name : String(name);
      return String(name).toLowerCase();
    }
    getAttribute(name) {
      const key = this._name(name);
      if (key === 'style' && this._styleAttrStale) return this._style ? this._style.cssText : null;
      const v = this._attrs.get(key);
      return v === undefined ? null : v;
    }
    hasAttribute(name) {
      const key = this._name(name);
      if (key === 'style' && this._styleAttrStale) return !!this._style && this._style.length > 0;
      return this._attrs.has(key);
    }
    setAttribute(name, value) {
      this._setAttr(this._name(name), String(value));
    }
    _setAttr(name, value) {
      const old = this._attrs.get(name);
      if (name === 'style') {
        if (!this._style) this._style = new CSSStyleDeclaration(this);
        if (old === value && !this._styleAttrStale) return;
        this._attrs.set(name, value);
        this._styleAttrStale = false;
        this._style._parse(value);
        if (this._connected) this.ownerDocument._inlineStyleChanged(this);
        return;
      }
      if (old === value) return;
      this._attrs.set(name, value);
      if (name === 'class') this._classes = value.split(/\s+/).filter(Boolean);
      if (this._connected) this.ownerDocument._attributeChanged(this, name, old);
    }
    removeAttribute(name) {
      const key = this._name(name);
      if (!this._attrs.has(key) && !(key === 'style' && this._style)) return;
      const old = this._attrs.get(key);
      this._attrs.delete(key);
      if (key === 'class') this._classes = [];
      if (key === 'style') {
        if (this._style) this._style._parse('');
        this._styleAttrStale = false;
        if (this._connected) this.ownerDocument._inlineStyleChanged(this);
        return;
      }
      if (this._connected) this.ownerDocument._attributeChanged(this, key, old);
    }
    toggleAttribute(name, force) {
      const has = this.hasAttribute(name);
      const want = force === undefined ? !has : !!force;
      if (want && !has) this.setAttribute(name, '');
      if (!want && has) this.removeAttribute(name);
      return want;
    }
    getAttributeNames() {
      return [...this._attrs.keys()];
    }
    hasAttributes() {
      return this._attrs.size > 0;
    }
    get attributes() {
      const out = [];
      for (const name of this._attrs.keys()) {
        const colon = name.indexOf(':');
        out.push({ name, localName: colon >= 0 ? name.slice(colon + 1) : name, value: this.getAttribute(name), namespaceURI: this._attrNS?.get(name) ?? null });
      }
      out.getNamedItem = (n) => out.find((a) => a.name === n) ?? null;
      return out;
    }
    // Namespaced attributes are stored under their qualified name (so
    // xlink:href set either way reads back either way).
    setAttributeNS(ns, qualifiedName, value) {
      if (ns) {
        if (!this._attrNS) this._attrNS = new Map();
        this._attrNS.set(qualifiedName, ns);
      }
      this._setAttr(qualifiedName, String(value));
    }
    _nsKey(ns, localName) {
      if (this._attrNS) for (const [q, n] of this._attrNS) if (n === ns && (q === localName || q.endsWith(`:${localName}`))) return q;
      if (ns === XLINK_NS && this._attrs.has(`xlink:${localName}`)) return `xlink:${localName}`;
      if (ns === XML_NS && this._attrs.has(`xml:${localName}`)) return `xml:${localName}`;
      return localName;
    }
    getAttributeNS(ns, localName) {
      return this.getAttribute(ns ? this._nsKey(ns, localName) : localName);
    }
    hasAttributeNS(ns, localName) {
      return this.hasAttribute(ns ? this._nsKey(ns, localName) : localName);
    }
    removeAttributeNS(ns, localName) {
      this.removeAttribute(ns ? this._nsKey(ns, localName) : localName);
    }

    get id() {
      return this.getAttribute('id') ?? '';
    }
    set id(v) {
      this.setAttribute('id', v);
    }
    get className() {
      return this.getAttribute('class') ?? '';
    }
    set className(v) {
      this.setAttribute('class', v);
    }
    get classList() {
      if (!this._classList) this._classList = new DOMTokenList(this);
      return this._classList;
    }
    set classList(v) {
      this.setAttribute('class', v);
    }
    get style() {
      if (!this._style) this._style = new CSSStyleDeclaration(this);
      return this._style;
    }
    set style(v) {
      this.setAttribute('style', v);
    }
    get dataset() {
      const el = this;
      const key = (prop) => `data-${String(prop).replace(/[A-Z]/g, (c) => `-${c.toLowerCase()}`)}`;
      return new Proxy(
        {},
        {
          get: (_, prop) => (typeof prop === 'string' ? el.getAttribute(key(prop)) ?? undefined : undefined),
          set: (_, prop, value) => {
            el.setAttribute(key(prop), value);
            return true;
          },
          deleteProperty: (_, prop) => {
            el.removeAttribute(key(prop));
            return true;
          },
          has: (_, prop) => el.hasAttribute(key(prop)),
        },
      );
    }

    // Tree conveniences.
    get nextElementSibling() {
      return D.selectors.nextElement(this);
    }
    get previousElementSibling() {
      return D.selectors.prevElement(this);
    }
    remove() {
      if (this.parentNode) this.parentNode._remove(this);
    }
    before(...nodes) {
      insertNodes(this.parentNode, this, nodes, this.ownerDocument);
    }
    after(...nodes) {
      insertNodes(this.parentNode, this._next, nodes, this.ownerDocument);
    }
    replaceWith(...nodes) {
      const parent = this.parentNode;
      if (!parent) return;
      const next = this._next;
      parent._remove(this);
      insertNodes(parent, next, nodes, this.ownerDocument);
    }
    insertAdjacentElement(where, el) {
      switch (String(where).toLowerCase()) {
        case 'beforebegin':
          if (!this.parentNode) return null;
          this.parentNode._insert(el, this);
          return el;
        case 'afterbegin':
          this._insert(el, this._first);
          return el;
        case 'beforeend':
          this._insert(el, null);
          return el;
        case 'afterend':
          if (!this.parentNode) return null;
          this.parentNode._insert(el, this._next);
          return el;
        default:
          throw domError('SyntaxError', `'${where}' is not a valid position.`);
      }
    }
    insertAdjacentText(where, text) {
      this.insertAdjacentElement(where, this.ownerDocument.createTextNode(text));
    }
    insertAdjacentHTML(where, html) {
      const pos = String(where).toLowerCase();
      const context = pos === 'beforebegin' || pos === 'afterend' ? this.parentNode : this;
      if (!context || context.nodeType === 9) throw domError('NoModificationAllowedError', 'The element has no parent.');
      const fragment = parseHTML(String(html), context);
      this.insertAdjacentElement(pos, fragment);
    }
    get innerHTML() {
      let out = '';
      for (let c = this._first; c; c = c._next) out += serialize(c);
      return out;
    }
    set innerHTML(html) {
      while (this._first) this._remove(this._first);
      const text = String(html ?? '');
      if (text === '') return;
      if (!/[<&]/.test(text)) {
        this._insert(this.ownerDocument.createTextNode(text), null);
        return;
      }
      this._insert(parseHTML(text, this), null);
    }
    get outerHTML() {
      return serialize(this);
    }
    get innerText() {
      return this.textContent;
    }
    set innerText(v) {
      this.textContent = v;
    }
    matches(selector) {
      return D.selectors.parseList(String(selector)).some((s) => D.selectors.matchComplex(this, s, this));
    }
    webkitMatchesSelector(selector) {
      return this.matches(selector);
    }
    closest(selector) {
      const list = D.selectors.parseList(String(selector));
      for (let el = this; el && el.nodeType === 1; el = el.parentNode) {
        if (list.some((s) => D.selectors.matchComplex(el, s, this))) return el;
      }
      return null;
    }

    // Geometry, from the document's layout.
    getBoundingClientRect() {
      const doc = this.ownerDocument;
      if (!this._connected || !doc._clientRect) return new DOMRect(0, 0, 0, 0);
      return doc._clientRect(this);
    }
    getClientRects() {
      return [this.getBoundingClientRect()];
    }
    get scrollTop() {
      return this._scrollTop ?? 0;
    }
    set scrollTop(v) {
      const n = Math.max(0, Number(v) || 0);
      if (n === (this._scrollTop ?? 0)) return;
      this._scrollTop = n;
      if (this._connected) this.ownerDocument._scrolled(this);
    }
    get scrollLeft() {
      return this._scrollLeft ?? 0;
    }
    set scrollLeft(v) {
      const n = Math.max(0, Number(v) || 0);
      if (n === (this._scrollLeft ?? 0)) return;
      this._scrollLeft = n;
      if (this._connected) this.ownerDocument._scrolled(this);
    }
    scrollIntoView() {
      warnOnce('scrollIntoView', 'scrollIntoView() does not scroll');
    }
    scrollTo(x, y) {
      if (typeof x === 'object' && x) {
        if (x.top !== undefined) this.scrollTop = x.top;
        if (x.left !== undefined) this.scrollLeft = x.left;
      } else {
        this.scrollLeft = x;
        this.scrollTop = y;
      }
    }
    focus() {
      const doc = this.ownerDocument;
      if (doc && doc._focus) doc._focus(this);
    }
    blur() {
      const doc = this.ownerDocument;
      if (doc && doc._activeElement === this && doc._focus) doc._focus(null);
    }
  }
  Object.defineProperties(Element.prototype, Object.getOwnPropertyDescriptors(parentNodeMethods));

  class HTMLElement extends Element {
    get hidden() {
      return this.hasAttribute('hidden');
    }
    set hidden(v) {
      this.toggleAttribute('hidden', !!v);
    }
    get title() {
      return this.getAttribute('title') ?? '';
    }
    set title(v) {
      this.setAttribute('title', v);
    }
    get offsetParent() {
      return this.parentElement;
    }
    get offsetLeft() {
      return this.ownerDocument._offset ? this.ownerDocument._offset(this).left : 0;
    }
    get offsetTop() {
      return this.ownerDocument._offset ? this.ownerDocument._offset(this).top : 0;
    }
    get offsetWidth() {
      return this.getBoundingClientRect().width;
    }
    get offsetHeight() {
      return this.getBoundingClientRect().height;
    }
    get clientWidth() {
      return this.getBoundingClientRect().width;
    }
    get clientHeight() {
      return this.getBoundingClientRect().height;
    }
    get scrollWidth() {
      return this.getBoundingClientRect().width;
    }
    get scrollHeight() {
      return this.ownerDocument._scrollHeight ? this.ownerDocument._scrollHeight(this) : this.getBoundingClientRect().height;
    }
    get contentEditable() {
      return this.getAttribute('contenteditable') ?? 'inherit';
    }
    get isContentEditable() {
      return this.getAttribute('contenteditable') === 'true';
    }
    get tabIndex() {
      return parseInt(this.getAttribute('tabindex') ?? '-1', 10);
    }
    set tabIndex(v) {
      this.setAttribute('tabindex', v);
    }
    click() {
      this.dispatchEvent(new MouseEvent('click', { bubbles: true, cancelable: true }));
    }
  }

  const reflectBool = (cls, name) =>
    Object.defineProperty(cls.prototype, name, {
      get() {
        return this.hasAttribute(name);
      },
      set(v) {
        this.toggleAttribute(name, !!v);
      },
    });
  const reflectString = (cls, name, attr = name.toLowerCase()) =>
    Object.defineProperty(cls.prototype, name, {
      get() {
        return this.getAttribute(attr) ?? '';
      },
      set(v) {
        this.setAttribute(attr, v);
      },
    });

  class HTMLDivElement extends HTMLElement {}
  class HTMLSpanElement extends HTMLElement {}
  class HTMLParagraphElement extends HTMLElement {}
  class HTMLBRElement extends HTMLElement {}
  class HTMLHtmlElement extends HTMLElement {}
  class HTMLHeadElement extends HTMLElement {}
  class HTMLBodyElement extends HTMLElement {}
  class HTMLIFrameElement extends HTMLElement {}
  class HTMLUnknownElement extends HTMLElement {}
  class HTMLFormElement extends HTMLElement {
    submit() {
      warnOnce('form-submit', 'form.submit() does nothing');
    }
  }

  class HTMLStyleElement extends HTMLElement {}

  class HTMLButtonElement extends HTMLElement {}
  reflectBool(HTMLButtonElement, 'disabled');
  reflectString(HTMLButtonElement, 'type');

  class HTMLInputElement extends HTMLElement {
    get value() {
      return this._value ?? this.getAttribute('value') ?? '';
    }
    set value(v) {
      this._value = String(v);
      if (this._connected) this.ownerDocument._textChangedIn(this);
    }
    get checked() {
      return this._checked ?? this.hasAttribute('checked');
    }
    set checked(v) {
      if (!!v === this.checked) return;
      this._checked = !!v;
      if (this._connected) this.ownerDocument._stateChanged(this);
    }
    select() {}
    setSelectionRange() {}
  }
  reflectBool(HTMLInputElement, 'disabled');
  reflectString(HTMLInputElement, 'type');
  reflectString(HTMLInputElement, 'name');
  reflectString(HTMLInputElement, 'placeholder');

  class HTMLTextAreaElement extends HTMLInputElement {}

  class HTMLImageElement extends HTMLElement {
    get src() {
      return this.getAttribute('src') ?? '';
    }
    set src(v) {
      this.setAttribute('src', v);
      // The renderer loads images; scripts waiting for load carry on.
      Promise.resolve().then(() => this.dispatchEvent(new Event('load')));
    }
    get width() {
      return parseFloat(this.getAttribute('width') ?? '') || this.naturalWidth;
    }
    set width(v) {
      this.setAttribute('width', v);
    }
    get height() {
      return parseFloat(this.getAttribute('height') ?? '') || this.naturalHeight;
    }
    set height(v) {
      this.setAttribute('height', v);
    }
    get naturalWidth() {
      return D.imageSize(this.src)[0];
    }
    get naturalHeight() {
      return D.imageSize(this.src)[1];
    }
    get complete() {
      return true;
    }
    decode() {
      return Promise.resolve();
    }
  }
  reflectString(HTMLImageElement, 'alt');

  class HTMLCanvasElement extends HTMLElement {
    get width() {
      const v = parseInt(this.getAttribute('width') ?? '', 10);
      return Number.isFinite(v) && v >= 0 ? v : 300;
    }
    set width(v) {
      this.setAttribute('width', Math.max(0, Math.floor(Number(v) || 0)));
      if (this._context) this._context._reset();
    }
    get height() {
      const v = parseInt(this.getAttribute('height') ?? '', 10);
      return Number.isFinite(v) && v >= 0 ? v : 150;
    }
    set height(v) {
      this.setAttribute('height', Math.max(0, Math.floor(Number(v) || 0)));
      if (this._context) this._context._reset();
    }
    getContext(type) {
      if (type !== '2d') {
        warnOnce(`canvas-context:${type}`, `canvas getContext('${type}') is not supported`);
        return null;
      }
      if (!this._context) this._context = new D.CanvasRenderingContext2D(this);
      return this._context;
    }
    toDataURL() {
      warnOnce('toDataURL', 'canvas.toDataURL() is not supported');
      return 'data:,';
    }
  }

  class SVGElement extends Element {
    get ownerSVGElement() {
      for (let p = this.parentNode; p && p.nodeType === 1; p = p.parentNode) if (p.namespaceURI === SVG_NS && p.localName === 'svg') return p;
      return null;
    }
    get viewportElement() {
      return this.ownerSVGElement;
    }
  }
  class SVGGraphicsElement extends SVGElement {
    getBBox() {
      const doc = this.ownerDocument;
      return doc && doc._bbox ? doc._bbox(this) : new DOMRect(0, 0, 0, 0);
    }
    getCTM() {
      warnOnce('getCTM', 'SVG getCTM() is not supported');
      return null;
    }
    getScreenCTM() {
      warnOnce('getScreenCTM', 'SVG getScreenCTM() is not supported');
      return null;
    }
  }
  class SVGSVGElement extends SVGGraphicsElement {
    createSVGPoint() {
      return { x: 0, y: 0, matrixTransform: () => ({ x: 0, y: 0 }) };
    }
  }
  class SVGGElement extends SVGGraphicsElement {}
  class SVGDefsElement extends SVGGraphicsElement {}
  class SVGGeometryElement extends SVGGraphicsElement {
    getTotalLength() {
      warnOnce('getTotalLength', 'SVG getTotalLength() is not supported');
      return 0;
    }
  }
  class SVGPathElement extends SVGGeometryElement {}
  class SVGRectElement extends SVGGeometryElement {}
  class SVGCircleElement extends SVGGeometryElement {}
  class SVGEllipseElement extends SVGGeometryElement {}
  class SVGLineElement extends SVGGeometryElement {}
  class SVGPolylineElement extends SVGGeometryElement {}
  class SVGPolygonElement extends SVGGeometryElement {}
  class SVGTextContentElement extends SVGGraphicsElement {
    getComputedTextLength() {
      const doc = this.ownerDocument;
      return doc && doc._textLength ? doc._textLength(this) : 0;
    }
    getNumberOfChars() {
      return this.textContent.length;
    }
  }
  class SVGTextElement extends SVGTextContentElement {}
  class SVGTSpanElement extends SVGTextContentElement {}
  class SVGImageElement extends SVGGraphicsElement {}
  class SVGClipPathElement extends SVGElement {}
  class SVGGradientElement extends SVGElement {}
  class SVGLinearGradientElement extends SVGGradientElement {}
  class SVGStopElement extends SVGElement {}
  class SVGUseElement extends SVGGraphicsElement {}

  const HTML_CLASSES = {
    div: HTMLDivElement, span: HTMLSpanElement, p: HTMLParagraphElement, br: HTMLBRElement, html: HTMLHtmlElement,
    head: HTMLHeadElement, body: HTMLBodyElement, iframe: HTMLIFrameElement, style: HTMLStyleElement,
    button: HTMLButtonElement, input: HTMLInputElement, textarea: HTMLTextAreaElement, img: HTMLImageElement,
    canvas: HTMLCanvasElement, form: HTMLFormElement,
  };
  const SVG_CLASSES = {
    svg: SVGSVGElement, g: SVGGElement, defs: SVGDefsElement, path: SVGPathElement, rect: SVGRectElement,
    circle: SVGCircleElement, ellipse: SVGEllipseElement, line: SVGLineElement, polyline: SVGPolylineElement,
    polygon: SVGPolygonElement, text: SVGTextElement, tspan: SVGTSpanElement, image: SVGImageElement,
    clipPath: SVGClipPathElement, linearGradient: SVGLinearGradientElement, stop: SVGStopElement, use: SVGUseElement,
  };
  // Tags any HTML namespace element with an unknown name still gets
  // HTMLElement for (custom elements such as vcockpit-panel included).
  const KNOWN_HTML = new Set([
    'a', 'abbr', 'b', 'blockquote', 'caption', 'center', 'code', 'col', 'colgroup', 'dd', 'dl', 'dt', 'em', 'fieldset',
    'footer', 'h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'header', 'hr', 'i', 'label', 'legend', 'li', 'link', 'main', 'meta',
    'nav', 'ol', 'option', 'pre', 'script', 'section', 'select', 'small', 'strong', 'sub', 'sup', 'table', 'tbody', 'td',
    'tfoot', 'th', 'thead', 'title', 'tr', 'u', 'ul', 'video', 'audio', 'article', 'aside', 'template', 'slot',
  ]);

  function createElementNS(doc, ns, qualifiedName) {
    const name = String(qualifiedName);
    if (ns === SVG_NS) {
      const local = name.includes(':') ? name.slice(name.indexOf(':') + 1) : name;
      const Cls = SVG_CLASSES[local] ?? SVGElement;
      return new Cls(doc, SVG_NS, name);
    }
    if (ns === HTML_NS || ns === null || ns === undefined || ns === '') {
      const custom = D.customElements && D.customElements._construct(doc, name);
      if (custom) return custom;
      const Cls = HTML_CLASSES[name] ?? (KNOWN_HTML.has(name) || name.includes('-') ? HTMLElement : HTMLUnknownElement);
      return new Cls(doc, ns === HTML_NS || ns === undefined ? HTML_NS : ns, name);
    }
    return new Element(doc, ns, name);
  }

  class DocumentFragment extends Node {
    constructor() {
      super(D.currentDocument());
    }
    get nodeType() {
      return 11;
    }
    get nodeName() {
      return '#document-fragment';
    }
    getElementById(id) {
      let found = null;
      walkElements(this, (el) => (el.getAttribute('id') === id ? ((found = el), true) : false));
      return found;
    }
    _cloneShallow() {
      return this.ownerDocument.createDocumentFragment();
    }
  }
  Object.defineProperties(DocumentFragment.prototype, Object.getOwnPropertyDescriptors(parentNodeMethods));

  class Document extends Node {
    constructor() {
      super(null);
      this._connected = true;
      this._activeElement = null;
      const html = createElementNS(this, HTML_NS, 'html');
      html.appendChild(createElementNS(this, HTML_NS, 'head'));
      html.appendChild(createElementNS(this, HTML_NS, 'body'));
      this._insertRootQuietly(html);
    }
    // The initial <html> goes in before the document is set up to track
    // changes.
    _insertRootQuietly(html) {
      html.parentNode = this;
      this._first = this._last = html;
      html.ownerDocument = this;
      setConnected(html, true, this);
    }
    get nodeType() {
      return 9;
    }
    get nodeName() {
      return '#document';
    }
    get documentElement() {
      let c = this._first;
      while (c && c.nodeType !== 1) c = c._next;
      return c;
    }
    get head() {
      const html = this.documentElement;
      return html ? html.querySelector('head') : null;
    }
    get body() {
      const html = this.documentElement;
      if (!html) return null;
      for (let c = html._first; c; c = c._next) if (c.nodeType === 1 && c.localName === 'body') return c;
      return null;
    }
    get defaultView() {
      return globalThis;
    }
    get readyState() {
      return 'complete';
    }
    get activeElement() {
      return this._activeElement ?? this.body;
    }
    get compatMode() {
      return 'CSS1Compat';
    }
    get visibilityState() {
      return 'visible';
    }
    get hidden() {
      return false;
    }
    hasFocus() {
      return true;
    }
    createElement(tag) {
      const name = String(tag);
      return createElementNS(this, HTML_NS, /^[A-Za-z]/.test(name) && !name.includes(':') ? name.toLowerCase() : name);
    }
    createElementNS(ns, qualifiedName) {
      return createElementNS(this, ns, qualifiedName);
    }
    createTextNode(data) {
      const t = new Text(data);
      t.ownerDocument = this;
      return t;
    }
    createComment(data) {
      const c = new Comment(data);
      c.ownerDocument = this;
      return c;
    }
    createDocumentFragment() {
      const f = new DocumentFragment();
      f.ownerDocument = this;
      return f;
    }
    createEvent(kind) {
      const k = String(kind).toLowerCase();
      if (k.startsWith('customevent')) return new CustomEvent('');
      if (k.startsWith('mouseevent')) return new MouseEvent('');
      return new Event('');
    }
    importNode(node, deep) {
      const copy = node.cloneNode(deep);
      setConnected(copy, false, this);
      return copy;
    }
    adoptNode(node) {
      if (node.parentNode) node.parentNode._remove(node);
      setConnected(node, false, this);
      return node;
    }
    getElementById(id) {
      let found = null;
      const want = String(id);
      walkElements(this, (el) => (el._attrs.get('id') === want ? ((found = el), true) : false));
      return found;
    }
    _cloneShallow() {
      throw domError('NotSupportedError', 'Documents cannot be cloned.');
    }
    // Change hooks; document.js replaces these with real invalidation.
    _nodeInserted() {}
    _nodeRemoving() {}
    _nodeRemoved() {}
    _attributeChanged() {}
    _textChanged() {}
    _textChangedIn() {}
    _inlineStyleChanged() {}
    _stateChanged() {}
    _scrolled() {}
  }
  Object.defineProperties(Document.prototype, Object.getOwnPropertyDescriptors(parentNodeMethods));

  class DOMRect {
    constructor(x = 0, y = 0, width = 0, height = 0) {
      this.x = x;
      this.y = y;
      this.width = width;
      this.height = height;
    }
    get left() {
      return Math.min(this.x, this.x + this.width);
    }
    get top() {
      return Math.min(this.y, this.y + this.height);
    }
    get right() {
      return Math.max(this.x, this.x + this.width);
    }
    get bottom() {
      return Math.max(this.y, this.y + this.height);
    }
    toJSON() {
      return { x: this.x, y: this.y, width: this.width, height: this.height, left: this.left, top: this.top, right: this.right, bottom: this.bottom };
    }
  }

  // ---------------------------------------------------------------- HTML parser

  const ENTITIES = {
    amp: '&', lt: '<', gt: '>', quot: '"', apos: "'", nbsp: ' ', deg: '°', plusmn: '±', middot: '·',
    times: '×', divide: '÷', copy: '©', reg: '®', micro: 'µ', para: '¶', sect: '§',
    laquo: '«', raquo: '»', larr: '←', uarr: '↑', rarr: '→', darr: '↓', harr: '↔',
    ndash: '–', mdash: '—', hellip: '…', bull: '•', trade: '™', deg2: '°', Delta: 'Δ',
    ensp: ' ', emsp: ' ', thinsp: ' ',
  };
  const decodeEntities = (s) =>
    s.indexOf('&') < 0
      ? s
      : s.replace(/&(#x[0-9a-f]+|#\d+|[a-z][a-z0-9]*);/gi, (m, body) => {
          if (body[0] === '#') {
            const code = body[1] === 'x' || body[1] === 'X' ? parseInt(body.slice(2), 16) : parseInt(body.slice(1), 10);
            return Number.isFinite(code) ? String.fromCodePoint(code) : m;
          }
          return ENTITIES[body] ?? m;
        });

  const SVG_TAGS = {};
  for (const t of ['clipPath', 'linearGradient', 'radialGradient', 'foreignObject', 'textPath', 'feGaussianBlur', 'feOffset', 'feBlend', 'feColorMatrix', 'feMerge', 'feMergeNode', 'feFlood', 'feComposite', 'animateTransform', 'animateMotion']) SVG_TAGS[t.toLowerCase()] = t;
  const SVG_ATTRS = {};
  for (const a of ['viewBox', 'preserveAspectRatio', 'gradientUnits', 'gradientTransform', 'clipPathUnits', 'patternUnits', 'patternTransform', 'textLength', 'lengthAdjust', 'spreadMethod', 'markerWidth', 'markerHeight', 'markerUnits', 'refX', 'refY', 'pathLength', 'stdDeviation', 'startOffset', 'maskUnits', 'maskContentUnits', 'baseProfile', 'attributeName', 'repeatCount', 'keyTimes', 'keySplines', 'calcMode']) SVG_ATTRS[a.toLowerCase()] = a;
  const RAW_TEXT = new Set(['style', 'script', 'textarea', 'title']);

  // Parse an HTML fragment in the context of `context` (its namespace
  // decides whether tags are SVG) into a DocumentFragment.
  function parseHTML(html, context) {
    const doc = docOf(context);
    const fragment = doc.createDocumentFragment();
    const stack = [fragment];
    const nsOf = (parent) => {
      const p = parent === fragment ? context : parent;
      if (p && p.nodeType === 1 && p.namespaceURI === SVG_NS && p.localName !== 'foreignObject') return SVG_NS;
      return HTML_NS;
    };
    const top = () => stack[stack.length - 1];
    const attrRe = /([^\s"'>/=]+)(?:\s*=\s*(?:"([^"]*)"|'([^']*)'|([^\s>]+)))?/g;
    let i = 0;
    const n = html.length;
    let text = '';
    const flushText = () => {
      if (text) {
        top()._insert(doc.createTextNode(decodeEntities(text)), null);
        text = '';
      }
    };
    while (i < n) {
      const lt = html.indexOf('<', i);
      if (lt < 0) {
        text += html.slice(i);
        break;
      }
      text += html.slice(i, lt);
      const next = html[lt + 1];
      if (html.startsWith('<!--', lt)) {
        flushText();
        const end = html.indexOf('-->', lt + 4);
        const body = html.slice(lt + 4, end < 0 ? n : end);
        top()._insert(doc.createComment(body), null);
        i = end < 0 ? n : end + 3;
      } else if (next === '!' || next === '?') {
        flushText();
        const end = html.indexOf('>', lt);
        i = end < 0 ? n : end + 1;
      } else if (next === '/') {
        const m = /^<\/\s*([^\s>]+)\s*>/.exec(html.slice(lt, lt + 200));
        if (!m) {
          text += '<';
          i = lt + 1;
          continue;
        }
        flushText();
        const name = m[1].toLowerCase();
        for (let k = stack.length - 1; k > 0; k--) {
          if (stack[k].localName.toLowerCase() === name) {
            stack.length = k;
            break;
          }
        }
        i = lt + m[0].length;
      } else if (next && /[A-Za-z]/.test(next)) {
        const end = findTagEnd(html, lt);
        // A tag the input ends inside is dropped, as the HTML tokenizer
        // drops it (FlyByWire's MFD writes text such as '<RETURN').
        if (end >= n) {
          flushText();
          i = n;
          break;
        }
        const inside = html.slice(lt + 1, end);
        const nameMatch = /^[^\s/>]+/.exec(inside);
        flushText();
        let name = nameMatch[0];
        const lower = name.toLowerCase();
        const parent = top();
        let ns = nsOf(parent);
        if (lower === 'svg') ns = SVG_NS;
        name = ns === SVG_NS ? SVG_TAGS[lower] ?? lower : lower;
        const el = createElementNS(doc, ns, name);
        const attrText = inside.slice(nameMatch[0].length);
        const selfClosing = /\/\s*$/.test(attrText);
        attrRe.lastIndex = 0;
        let a;
        while ((a = attrRe.exec(attrText.replace(/\/\s*$/, '')))) {
          let attr = a[1];
          const value = decodeEntities(a[2] ?? a[3] ?? a[4] ?? '');
          if (ns === SVG_NS) attr = SVG_ATTRS[attr.toLowerCase()] ?? attr.toLowerCase();
          else attr = attr.toLowerCase();
          if (attr.startsWith('xlink:')) el.setAttributeNS(XLINK_NS, attr, value);
          else el._setAttr(attr, value);
        }
        parent._insert(el, null);
        i = end + 1;
        if (ns === HTML_NS && RAW_TEXT.has(lower) && !selfClosing) {
          const close = html.toLowerCase().indexOf(`</${lower}`, i);
          const body = html.slice(i, close < 0 ? n : close);
          if (body) el._insert(doc.createTextNode(lower === 'textarea' || lower === 'title' ? decodeEntities(body) : body), null);
          const gt = close < 0 ? n : html.indexOf('>', close);
          i = gt < 0 ? n : gt + 1;
        } else if (!(selfClosing && ns === SVG_NS) && !(ns === HTML_NS && VOID.has(lower))) {
          stack.push(el);
        }
      } else {
        text += '<';
        i = lt + 1;
      }
    }
    flushText();
    return fragment;
  }

  const findTagEnd = (html, start) => {
    let quote = null;
    for (let i = start + 1; i < html.length; i++) {
      const c = html[i];
      if (quote) {
        if (c === quote) quote = null;
      } else if (c === '"' || c === "'") quote = c;
      else if (c === '>') return i;
    }
    return html.length;
  };

  const escapeText = (s) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/ /g, '&nbsp;');
  const escapeAttr = (s) => s.replace(/&/g, '&amp;').replace(/"/g, '&quot;').replace(/ /g, '&nbsp;');

  function serialize(node) {
    switch (node.nodeType) {
      case 3: {
        const p = node.parentNode;
        return p && p.nodeType === 1 && RAW_TEXT.has(p.localName) && p.localName !== 'textarea' ? node._data : escapeText(node._data);
      }
      case 8:
        return `<!--${node._data}-->`;
      case 1: {
        const name = node._qualified;
        let out = `<${name}`;
        for (const attr of node._attrs.keys()) out += ` ${attr}="${escapeAttr(node.getAttribute(attr) ?? '')}"`;
        out += '>';
        if (node.namespaceURI === HTML_NS && VOID.has(name)) return out;
        for (let c = node._first; c; c = c._next) out += serialize(c);
        return `${out}</${name}>`;
      }
      case 11: {
        let out = '';
        for (let c = node._first; c; c = c._next) out += serialize(c);
        return out;
      }
      default:
        return '';
    }
  }

  // Natural image sizes, when the host can tell.
  const imageSizes = new Map();
  D.imageSize = (url) => {
    if (!url) return [0, 0];
    if (imageSizes.has(url)) return imageSizes.get(url);
    let size = [0, 0];
    const host = globalThis.__host;
    if (host && typeof host.imageSize === 'function') {
      try {
        const s = host.imageSize(url);
        if (s && s.length >= 2) size = [Number(s[0]) || 0, Number(s[1]) || 0];
      } catch (e) {
        warnOnce(`imageSize:${url}`, `__host.imageSize(${url}) failed: ${e}`);
      }
    }
    imageSizes.set(url, size);
    return size;
  };

  Object.assign(D, {
    Event, CustomEvent, UIEvent, MouseEvent, PointerEvent, WheelEvent, KeyboardEvent, FocusEvent, EventTarget,
    eventTargetMethods, dispatch, Node, CharacterData, Text, Comment, Element, HTMLElement, SVGElement, DocumentFragment,
    Document, DOMRect, DOMTokenList, CSSStyleDeclaration, parseHTML, serialize, createElementNS, decodeEntities,
    classes: {
      HTMLDivElement, HTMLSpanElement, HTMLParagraphElement, HTMLBRElement, HTMLHtmlElement, HTMLHeadElement,
      HTMLBodyElement, HTMLIFrameElement, HTMLUnknownElement, HTMLFormElement, HTMLStyleElement, HTMLButtonElement,
      HTMLInputElement, HTMLTextAreaElement, HTMLImageElement, HTMLCanvasElement, SVGGraphicsElement, SVGSVGElement,
      SVGGElement, SVGDefsElement, SVGGeometryElement, SVGPathElement, SVGRectElement, SVGCircleElement,
      SVGEllipseElement, SVGLineElement, SVGPolylineElement, SVGPolygonElement, SVGTextContentElement, SVGTextElement,
      SVGTSpanElement, SVGImageElement, SVGClipPathElement, SVGGradientElement, SVGLinearGradientElement,
      SVGStopElement, SVGUseElement,
    },
  });
})();
