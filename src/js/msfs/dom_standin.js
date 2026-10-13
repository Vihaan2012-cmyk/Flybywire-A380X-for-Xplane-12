// A stand-in DOM: the node tree and events, with no styles, layout or
// painting. It exists so instruments' logic can run before the real DOM
// (src/js/dom/, which paints to the display stream) is loaded in its place;
// it is replaced whole, not extended. Anything that needs layout
// (getBoundingClientRect, offsetWidth, canvas) is absent here and throws
// where it is used.
(() => {
  if (globalThis.document !== undefined) {
    return;
  }

  const SVG_NS = 'http://www.w3.org/2000/svg';
  const HTML_NS = 'http://www.w3.org/1999/xhtml';

  class Event {
    constructor(type, init = {}) {
      this.type = String(type);
      this.bubbles = !!init.bubbles;
      this.cancelable = !!init.cancelable;
      this.composed = !!init.composed;
      this.defaultPrevented = false;
      this.target = null;
      this.currentTarget = null;
      this.timeStamp = performance.now();
      this.stopped = false;
      this.stoppedImmediately = false;
    }
    preventDefault() {
      if (this.cancelable) {
        this.defaultPrevented = true;
      }
    }
    stopPropagation() {
      this.stopped = true;
    }
    stopImmediatePropagation() {
      this.stopped = true;
      this.stoppedImmediately = true;
    }
  }

  class CustomEvent extends Event {
    constructor(type, init = {}) {
      super(type, init);
      this.detail = init.detail === undefined ? null : init.detail;
    }
  }

  class EventTarget {
    addEventListener(type, listener, options) {
      if (!listener) {
        return;
      }
      const once = typeof options === 'object' && options !== null && !!options.once;
      const map = this.__listeners || (this.__listeners = new Map());
      let list = map.get(type);
      if (list === undefined) {
        list = [];
        map.set(type, list);
      }
      if (!list.some((l) => l.listener === listener)) {
        list.push({ listener, once });
      }
    }
    removeEventListener(type, listener) {
      const list = this.__listeners && this.__listeners.get(type);
      if (list) {
        const i = list.findIndex((l) => l.listener === listener);
        if (i >= 0) {
          list.splice(i, 1);
        }
      }
    }
    dispatchEvent(event) {
      event.target = this;
      const path = [];
      for (let node = this; node; node = node.parentNode) {
        path.push(node);
        if (!event.bubbles) {
          break;
        }
      }
      for (const node of path) {
        const list = node.__listeners && node.__listeners.get(event.type);
        if (list) {
          event.currentTarget = node;
          for (const entry of list.slice()) {
            if (entry.once) {
              node.removeEventListener(event.type, entry.listener);
            }
            try {
              if (typeof entry.listener === 'function') {
                entry.listener.call(node, event);
              } else {
                entry.listener.handleEvent(event);
              }
            } catch (e) {
              console.error(e);
            }
            if (event.stoppedImmediately) {
              break;
            }
          }
        }
        if (event.stopped) {
          break;
        }
      }
      return !event.defaultPrevented;
    }
  }

  class Node extends EventTarget {
    constructor() {
      super();
      this.parentNode = null;
      this.childNodes = [];
    }
    get parentElement() {
      return this.parentNode instanceof Element ? this.parentNode : null;
    }
    get firstChild() {
      return this.childNodes[0] || null;
    }
    get lastChild() {
      return this.childNodes[this.childNodes.length - 1] || null;
    }
    get nextSibling() {
      if (!this.parentNode) return null;
      const siblings = this.parentNode.childNodes;
      return siblings[siblings.indexOf(this) + 1] || null;
    }
    get previousSibling() {
      if (!this.parentNode) return null;
      const siblings = this.parentNode.childNodes;
      return siblings[siblings.indexOf(this) - 1] || null;
    }
    get ownerDocument() {
      return document;
    }
    get isConnected() {
      let node = this;
      while (node.parentNode) node = node.parentNode;
      return node === document;
    }
    hasChildNodes() {
      return this.childNodes.length > 0;
    }
    contains(other) {
      for (let node = other; node; node = node.parentNode) {
        if (node === this) return true;
      }
      return false;
    }
    appendChild(child) {
      return this.insertBefore(child, null);
    }
    insertBefore(child, reference) {
      if (child instanceof DocumentFragment) {
        for (const c of child.childNodes.slice()) {
          this.insertBefore(c, reference);
        }
        return child;
      }
      if (child.parentNode) {
        child.parentNode.removeChild(child);
      }
      const i = reference ? this.childNodes.indexOf(reference) : -1;
      if (i >= 0) {
        this.childNodes.splice(i, 0, child);
      } else {
        this.childNodes.push(child);
      }
      child.parentNode = this;
      if (this.isConnected) {
        connected(child);
      }
      return child;
    }
    removeChild(child) {
      const i = this.childNodes.indexOf(child);
      if (i < 0) {
        throw new Error('removeChild: not a child of this node');
      }
      const wasConnected = child.isConnected;
      this.childNodes.splice(i, 1);
      child.parentNode = null;
      if (wasConnected) {
        disconnected(child);
      }
      return child;
    }
    replaceChild(child, old) {
      this.insertBefore(child, old);
      return this.removeChild(old);
    }
    remove() {
      if (this.parentNode) {
        this.parentNode.removeChild(this);
      }
    }
    get textContent() {
      return this.childNodes.map((c) => c.textContent).join('');
    }
    set textContent(value) {
      for (const c of this.childNodes.slice()) this.removeChild(c);
      if (value !== null && value !== undefined && String(value) !== '') {
        this.appendChild(new Text(String(value)));
      }
    }
  }

  const connected = (node) => {
    if (node instanceof Element) {
      const ctor = registry.get(node.localName);
      if (ctor && !(node instanceof ctor)) {
        Object.setPrototypeOf(node, ctor.prototype);
      }
      if (typeof node.connectedCallback === 'function') {
        try {
          node.connectedCallback();
        } catch (e) {
          console.error(e);
        }
      }
    }
    for (const c of node.childNodes.slice()) connected(c);
  };
  const disconnected = (node) => {
    if (node instanceof Element && typeof node.disconnectedCallback === 'function') {
      try {
        node.disconnectedCallback();
      } catch (e) {
        console.error(e);
      }
    }
    for (const c of node.childNodes.slice()) disconnected(c);
  };

  class CharacterData extends Node {
    constructor(data) {
      super();
      this.data = String(data);
    }
    get nodeValue() {
      return this.data;
    }
    set nodeValue(v) {
      this.data = String(v);
    }
    get textContent() {
      return this.data;
    }
    set textContent(v) {
      this.data = String(v);
    }
  }
  class Text extends CharacterData {
    get nodeType() {
      return 3;
    }
    get nodeName() {
      return '#text';
    }
    cloneNode() {
      return new Text(this.data);
    }
  }
  class Comment extends CharacterData {
    get nodeType() {
      return 8;
    }
    get nodeName() {
      return '#comment';
    }
    get textContent() {
      return '';
    }
    cloneNode() {
      return new Comment(this.data);
    }
  }

  class DocumentFragment extends Node {
    get nodeType() {
      return 11;
    }
    get children() {
      return this.childNodes.filter((c) => c instanceof Element);
    }
    cloneNode(deep) {
      const f = new DocumentFragment();
      if (deep) for (const c of this.childNodes) f.appendChild(c.cloneNode(true));
      return f;
    }
    querySelector(s) {
      return select(this, s, true)[0] || null;
    }
    querySelectorAll(s) {
      return select(this, s, false);
    }
  }

  class DOMTokenList {
    constructor(element) {
      this.element = element;
    }
    get tokens() {
      return (this.element.getAttribute('class') || '').split(/\s+/).filter((t) => t !== '');
    }
    write(tokens) {
      this.element.setAttribute('class', tokens.join(' '));
    }
    get length() {
      return this.tokens.length;
    }
    item(i) {
      return this.tokens[i] || null;
    }
    contains(t) {
      return this.tokens.includes(t);
    }
    add(...ts) {
      const tokens = this.tokens;
      for (const t of ts) if (!tokens.includes(t)) tokens.push(t);
      this.write(tokens);
    }
    remove(...ts) {
      this.write(this.tokens.filter((t) => !ts.includes(t)));
    }
    toggle(t, force) {
      const has = this.contains(t);
      const want = force === undefined ? !has : !!force;
      if (want && !has) this.add(t);
      if (!want && has) this.remove(t);
      return want;
    }
    replace(a, b) {
      if (!this.contains(a)) return false;
      this.write(this.tokens.map((t) => (t === a ? b : t)));
      return true;
    }
    forEach(fn) {
      this.tokens.forEach(fn);
    }
    [Symbol.iterator]() {
      return this.tokens[Symbol.iterator]();
    }
    get value() {
      return this.tokens.join(' ');
    }
  }

  // Inline style as declared text: properties are kept and read back, never
  // computed.
  const styleName = (prop) => (prop.startsWith('--') ? prop : prop.replace(/[A-Z]/g, (c) => '-' + c.toLowerCase()));
  const makeStyle = (element) => {
    const read = () => {
      const map = new Map();
      for (const part of (element.getAttribute('style') || '').split(';')) {
        const i = part.indexOf(':');
        if (i > 0) map.set(part.slice(0, i).trim(), part.slice(i + 1).trim());
      }
      return map;
    };
    const write = (map) => {
      const text = [...map].map(([k, v]) => `${k}: ${v}`).join('; ');
      if (text === '') element.removeAttribute('style');
      else element.setAttribute('style', text);
    };
    const api = {
      setProperty(name, value) {
        const map = read();
        if (value === null || value === undefined || String(value) === '') map.delete(name);
        else map.set(name, String(value));
        write(map);
      },
      getPropertyValue(name) {
        return read().get(name) || '';
      },
      removeProperty(name) {
        const map = read();
        const old = map.get(name) || '';
        map.delete(name);
        write(map);
        return old;
      },
    };
    return new Proxy(api, {
      get(target, prop) {
        if (typeof prop !== 'string') return undefined;
        if (prop in target) return target[prop];
        if (prop === 'cssText') return element.getAttribute('style') || '';
        return read().get(styleName(prop)) || '';
      },
      set(target, prop, value) {
        if (prop === 'cssText') {
          element.setAttribute('style', String(value));
        } else {
          target.setProperty(styleName(String(prop)), value);
        }
        return true;
      },
    });
  };

  class Element extends Node {
    constructor(localName, namespaceURI) {
      super();
      this.localName = localName === undefined ? pendingName(new.target) : String(localName);
      this.namespaceURI = namespaceURI || HTML_NS;
      this.attributeMap = new Map();
    }
    get nodeType() {
      return 1;
    }
    get tagName() {
      return this.namespaceURI === HTML_NS ? this.localName.toUpperCase() : this.localName;
    }
    get nodeName() {
      return this.tagName;
    }
    get attributes() {
      return [...this.attributeMap].map(([name, value]) => ({ name, value }));
    }
    getAttribute(name) {
      const v = this.attributeMap.get(String(name).toLowerCase());
      return v === undefined ? null : v;
    }
    getAttributeNS(_ns, name) {
      return this.getAttribute(name);
    }
    setAttribute(name, value) {
      const key = String(name).toLowerCase();
      const old = this.getAttribute(key);
      this.attributeMap.set(key, String(value));
      if (typeof this.attributeChangedCallback === 'function') {
        this.attributeChangedCallback(key, old, String(value));
      }
    }
    setAttributeNS(_ns, name, value) {
      this.setAttribute(name, value);
    }
    removeAttribute(name) {
      this.attributeMap.delete(String(name).toLowerCase());
    }
    hasAttribute(name) {
      return this.attributeMap.has(String(name).toLowerCase());
    }
    toggleAttribute(name, force) {
      const want = force === undefined ? !this.hasAttribute(name) : !!force;
      if (want) this.setAttribute(name, '');
      else this.removeAttribute(name);
      return want;
    }
    get id() {
      return this.getAttribute('id') || '';
    }
    set id(v) {
      this.setAttribute('id', v);
    }
    get className() {
      return this.getAttribute('class') || '';
    }
    set className(v) {
      this.setAttribute('class', v);
    }
    get classList() {
      return this.__classList || (this.__classList = new DOMTokenList(this));
    }
    get style() {
      return this.__style || (this.__style = makeStyle(this));
    }
    get dataset() {
      const element = this;
      return new Proxy(
        {},
        {
          get: (_t, prop) => element.getAttribute('data-' + styleName(String(prop))) ?? undefined,
          set: (_t, prop, value) => {
            element.setAttribute('data-' + styleName(String(prop)), value);
            return true;
          },
        },
      );
    }
    get children() {
      return this.childNodes.filter((c) => c instanceof Element);
    }
    get childElementCount() {
      return this.children.length;
    }
    get firstElementChild() {
      return this.children[0] || null;
    }
    get lastElementChild() {
      const c = this.children;
      return c[c.length - 1] || null;
    }
    append(...nodes) {
      for (const n of nodes) this.appendChild(typeof n === 'string' ? new Text(n) : n);
    }
    prepend(...nodes) {
      const first = this.firstChild;
      for (const n of nodes) this.insertBefore(typeof n === 'string' ? new Text(n) : n, first);
    }
    replaceChildren(...nodes) {
      for (const c of this.childNodes.slice()) this.removeChild(c);
      this.append(...nodes);
    }
    insertAdjacentElement(position, element) {
      switch (String(position).toLowerCase()) {
        case 'beforebegin':
          return this.parentNode ? this.parentNode.insertBefore(element, this) : null;
        case 'afterbegin':
          return this.insertBefore(element, this.firstChild);
        case 'beforeend':
          return this.appendChild(element);
        case 'afterend':
          return this.parentNode ? this.parentNode.insertBefore(element, this.nextSibling) : null;
        default:
          throw new Error(`insertAdjacentElement: bad position ${position}`);
      }
    }
    cloneNode(deep) {
      const copy = document.createElementNS(this.namespaceURI, this.localName);
      for (const [k, v] of this.attributeMap) copy.attributeMap.set(k, v);
      if (deep) for (const c of this.childNodes) copy.appendChild(c.cloneNode(true));
      return copy;
    }
    get innerHTML() {
      return this.childNodes.map(serialise).join('');
    }
    set innerHTML(html) {
      for (const c of this.childNodes.slice()) this.removeChild(c);
      for (const n of parseHtml(String(html), this.namespaceURI === SVG_NS)) this.appendChild(n);
    }
    get outerHTML() {
      return serialise(this);
    }
    querySelector(s) {
      return select(this, s, true)[0] || null;
    }
    querySelectorAll(s) {
      return select(this, s, false);
    }
    getElementsByTagName(name) {
      const n = String(name).toLowerCase();
      return descendants(this).filter((e) => n === '*' || e.localName.toLowerCase() === n);
    }
    getElementsByClassName(names) {
      const wanted = String(names).split(/\s+/).filter((t) => t);
      return descendants(this).filter((e) => wanted.every((t) => e.classList.contains(t)));
    }
    matches(s) {
      return parseSelector(s).some((chain) => matchChain(this, chain, null));
    }
    closest(s) {
      for (let e = this; e instanceof Element; e = e.parentNode) {
        if (e.matches(s)) return e;
      }
      return null;
    }
    focus() {
      document.activeElement = this;
    }
    blur() {
      if (document.activeElement === this) document.activeElement = document.body;
    }
  }

  class HTMLElement extends Element {}
  class SVGElement extends Element {
    constructor(localName) {
      super(localName, SVG_NS);
    }
  }

  const descendants = (root) => {
    const out = [];
    const walk = (node) => {
      for (const c of node.childNodes) {
        if (c instanceof Element) {
          out.push(c);
          walk(c);
        }
      }
    };
    walk(root);
    return out;
  };

  // Selectors: compound selectors of tag, #id, .class, [attr], [attr=value],
  // joined by descendant or child combinators, in comma-separated lists, and
  // :scope.
  const parseSelector = (text) =>
    String(text)
      .split(',')
      .map((part) => {
        const tokens = part.trim().replace(/\s*>\s*/g, ' > ').split(/\s+/);
        const chain = [];
        let combinator = ' ';
        for (const token of tokens) {
          if (token === '>') {
            combinator = '>';
            continue;
          }
          chain.push({ combinator, compound: parseCompound(token) });
          combinator = ' ';
        }
        return chain;
      });
  const parseCompound = (token) => {
    const c = { tag: null, id: null, classes: [], attrs: [], scope: false };
    const re = /(:scope)|#([\w-]+)|\.([\w-]+)|\[([\w-:]+)(?:([~|^$*]?=)["']?([^"'\]]*)["']?)?\]|([\w-]+|\*)/g;
    let m;
    while ((m = re.exec(token)) !== null) {
      if (m[1]) c.scope = true;
      else if (m[2]) c.id = m[2];
      else if (m[3]) c.classes.push(m[3]);
      else if (m[4]) c.attrs.push({ name: m[4], op: m[5], value: m[6] });
      else if (m[7]) c.tag = m[7] === '*' ? null : m[7].toLowerCase();
    }
    return c;
  };
  const matchCompound = (e, c, scope) => {
    if (c.scope && e !== scope) return false;
    if (c.tag && e.localName.toLowerCase() !== c.tag) return false;
    if (c.id && e.id !== c.id) return false;
    for (const cls of c.classes) if (!e.classList.contains(cls)) return false;
    for (const a of c.attrs) {
      const v = e.getAttribute(a.name);
      if (v === null) return false;
      if (a.op === '=' && v !== a.value) return false;
      if (a.op === '^=' && !v.startsWith(a.value)) return false;
      if (a.op === '$=' && !v.endsWith(a.value)) return false;
      if (a.op === '*=' && !v.includes(a.value)) return false;
      if (a.op === '~=' && !v.split(/\s+/).includes(a.value)) return false;
    }
    return true;
  };
  const matchChain = (e, chain, scope) => {
    const match = (element, i) => {
      if (!matchCompound(element, chain[i].compound, scope)) return false;
      if (i === 0) return true;
      if (chain[i].combinator === '>') {
        return element.parentNode instanceof Element && match(element.parentNode, i - 1);
      }
      for (let p = element.parentNode; p instanceof Element; p = p.parentNode) {
        if (match(p, i - 1)) return true;
      }
      return false;
    };
    return match(e, chain.length - 1);
  };
  const select = (root, text, first) => {
    const chains = parseSelector(text);
    const out = [];
    for (const e of descendants(root)) {
      if (chains.some((chain) => matchChain(e, chain, root))) {
        out.push(e);
        if (first) break;
      }
    }
    return out;
  };

  // HTML: elements, attributes, text, comments, and raw text in script and
  // style elements. Entities beyond the basic five are left as written.
  const VOID = new Set(['area', 'base', 'br', 'col', 'embed', 'hr', 'img', 'input', 'link', 'meta', 'source', 'track', 'wbr']);
  const RAW = new Set(['script', 'style']);
  const decode = (s) =>
    s.replace(/&lt;/g, '<').replace(/&gt;/g, '>').replace(/&quot;/g, '"').replace(/&#39;/g, "'").replace(/&amp;/g, '&');
  const encode = (s) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  const parseHtml = (html, svg) => {
    const root = new DocumentFragment();
    const stack = [{ node: root, svg }];
    const top = () => stack[stack.length - 1];
    const re = /<!--([\s\S]*?)-->|<\/([\w-:]+)\s*>|<([\w-:]+)((?:\s+[^\s=>\/]+(?:\s*=\s*(?:"[^"]*"|'[^']*'|[^\s>]+))?)*)\s*(\/?)>|([^<]+)|(<)/g;
    let m;
    while ((m = re.exec(html)) !== null) {
      if (m[1] !== undefined) {
        top().node.appendChild(new Comment(m[1]));
      } else if (m[2] !== undefined) {
        const name = m[2].toLowerCase();
        for (let i = stack.length - 1; i > 0; i--) {
          if (stack[i].node.localName.toLowerCase() === name) {
            stack.length = i;
            break;
          }
        }
      } else if (m[3] !== undefined) {
        const name = m[3];
        const inSvg = top().svg || name.toLowerCase() === 'svg';
        const element = inSvg ? document.createElementNS(SVG_NS, name) : document.createElement(name);
        const attrRe = /([^\s=>\/]+)(?:\s*=\s*(?:"([^"]*)"|'([^']*)'|([^\s>]+)))?/g;
        let a;
        while ((a = attrRe.exec(m[4])) !== null) {
          element.attributeMap.set(a[1].toLowerCase(), decode(a[2] ?? a[3] ?? a[4] ?? ''));
        }
        top().node.appendChild(element);
        const lower = name.toLowerCase();
        if (RAW.has(lower) && m[5] !== '/') {
          const end = html.toLowerCase().indexOf(`</${lower}`, re.lastIndex);
          const stop = end < 0 ? html.length : end;
          const text = html.slice(re.lastIndex, stop);
          if (text !== '') element.appendChild(new Text(text));
          re.lastIndex = stop;
        } else if (m[5] !== '/' && !VOID.has(lower)) {
          stack.push({ node: element, svg: inSvg });
        }
      } else {
        const text = m[6] !== undefined ? m[6] : m[7];
        top().node.appendChild(new Text(decode(text)));
      }
    }
    return root.childNodes.slice();
  };
  const serialise = (node) => {
    if (node instanceof Text) return node.parentNode && RAW.has(node.parentNode.localName) ? node.data : encode(node.data);
    if (node instanceof Comment) return `<!--${node.data}-->`;
    if (node instanceof Element) {
      const attrs = [...node.attributeMap].map(([k, v]) => ` ${k}="${v.replace(/"/g, '&quot;')}"`).join('');
      if (VOID.has(node.localName)) return `<${node.localName}${attrs}>`;
      return `<${node.localName}${attrs}>${node.childNodes.map(serialise).join('')}</${node.localName}>`;
    }
    return node.childNodes.map(serialise).join('');
  };

  // Custom elements.
  const registry = new Map();
  const constructing = [];
  const pendingName = (ctor) => {
    const name = constructing.length > 0 ? constructing[constructing.length - 1] : [...registry].find(([, c]) => c === ctor)?.[0];
    if (name === undefined) {
      throw new TypeError('Illegal constructor: the element is not a defined custom element');
    }
    return name;
  };
  const customElements = {
    define(name, ctor) {
      const key = String(name).toLowerCase();
      if (registry.has(key)) {
        throw new Error(`customElements.define: ${name} is already defined`);
      }
      registry.set(key, ctor);
      for (const e of descendants(document)) {
        if (e.localName === key && !(e instanceof ctor)) connected(e);
      }
    },
    get(name) {
      return registry.get(String(name).toLowerCase());
    },
    whenDefined(name) {
      return registry.has(String(name).toLowerCase()) ? Promise.resolve(registry.get(String(name).toLowerCase())) : new Promise(() => {});
    },
  };

  class Document extends Node {
    constructor() {
      super();
      this.documentElement = null;
      this.head = null;
      this.body = null;
      this.activeElement = null;
      this.title = '';
    }
    get nodeType() {
      return 9;
    }
    createElement(name) {
      const key = String(name).toLowerCase();
      const ctor = registry.get(key);
      if (ctor) {
        constructing.push(key);
        try {
          return new ctor();
        } finally {
          constructing.pop();
        }
      }
      return new HTMLElement(key);
    }
    createElementNS(ns, name) {
      if (ns === SVG_NS) return new SVGElement(String(name));
      if (ns === HTML_NS || ns === null || ns === undefined) return this.createElement(name);
      return new Element(String(name), ns);
    }
    createTextNode(text) {
      return new Text(text);
    }
    createComment(text) {
      return new Comment(text);
    }
    createDocumentFragment() {
      return new DocumentFragment();
    }
    getElementById(id) {
      return descendants(this).find((e) => e.id === String(id)) || null;
    }
    querySelector(s) {
      return select(this, s, true)[0] || null;
    }
    querySelectorAll(s) {
      return select(this, s, false);
    }
    getElementsByTagName(name) {
      return Element.prototype.getElementsByTagName.call(this, name);
    }
    getElementsByClassName(names) {
      return Element.prototype.getElementsByClassName.call(this, names);
    }
  }

  const document = new Document();
  globalThis.document = document;
  const html = new HTMLElement('html');
  document.documentElement = html;
  document.childNodes.push(html);
  html.parentNode = document;
  document.head = html.appendChild(new HTMLElement('head'));
  document.body = html.appendChild(new HTMLElement('body'));
  document.activeElement = document.body;

  Object.assign(globalThis, {
    Event,
    CustomEvent,
    EventTarget,
    Node,
    Text,
    Comment,
    DocumentFragment,
    Element,
    HTMLElement,
    SVGElement,
    Document,
    customElements,
  });
})();
