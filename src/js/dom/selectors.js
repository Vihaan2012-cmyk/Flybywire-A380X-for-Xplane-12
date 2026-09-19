// CSS selectors: parsing, specificity and matching, shared by
// querySelector(All)/matches/closest and the stylesheet cascade.
//
// A selector list parses to complex selectors. A complex selector is kept
// right to left: `compounds[0]` is the subject and `combinators[i]` joins
// `compounds[i]` to `compounds[i + 1]` (' ', '>', '+' or '~').
(() => {
  const D = (globalThis.__dom = globalThis.__dom || {});

  // Log an unsupported feature once per key, naming it.
  const warned = new Set();
  D.warnOnce = (key, message) => {
    if (warned.has(key)) return;
    warned.add(key);
    try {
      console.warn(`[dom] ${message ?? key}`);
    } catch {
      // No console yet: nothing to report to.
    }
  };

  const IDENT = /[-\w -￿\\]/;

  class Parser {
    constructor(text) {
      this.s = text;
      this.i = 0;
    }
    peek() {
      return this.s[this.i];
    }
    eof() {
      return this.i >= this.s.length;
    }
    ws() {
      const start = this.i;
      while (this.i < this.s.length && /\s/.test(this.s[this.i])) this.i++;
      return this.i > start;
    }
    ident() {
      let out = '';
      while (this.i < this.s.length && IDENT.test(this.s[this.i])) {
        if (this.s[this.i] === '\\') {
          this.i++;
          out += this.s[this.i++] ?? '';
        } else {
          out += this.s[this.i++];
        }
      }
      return out;
    }
    // Text up to the parenthesis closing the one just opened.
    balanced() {
      let depth = 1;
      const start = this.i;
      while (this.i < this.s.length) {
        const c = this.s[this.i];
        if (c === '(') depth++;
        else if (c === ')' && --depth === 0) break;
        else if (c === '"' || c === "'") {
          const q = c;
          this.i++;
          while (this.i < this.s.length && this.s[this.i] !== q) this.i++;
        }
        this.i++;
      }
      const inner = this.s.slice(start, this.i);
      this.i++;
      return inner;
    }
  }

  const fail = (text) => {
    throw new SyntaxError(`'${text}' is not a valid selector`);
  };

  // Parse "an+b" (and odd/even) for :nth-child.
  const parseNth = (arg) => {
    const t = arg.replace(/\s+/g, '').toLowerCase();
    if (t === 'odd') return [2, 1];
    if (t === 'even') return [2, 0];
    const m = /^([+-]?\d*)n([+-]\d+)?$/.exec(t);
    if (m) {
      const a = m[1] === '' || m[1] === '+' ? 1 : m[1] === '-' ? -1 : parseInt(m[1], 10);
      return [a, m[2] ? parseInt(m[2], 10) : 0];
    }
    if (/^[+-]?\d+$/.test(t)) return [0, parseInt(t, 10)];
    return null;
  };

  const LIST_PSEUDOS = new Set(['not', 'is', 'where', 'matches', 'any']);
  const KNOWN_PSEUDOS = new Set([
    'hover', 'active', 'focus', 'focus-within', 'focus-visible', 'checked', 'disabled', 'enabled',
    'first-child', 'last-child', 'only-child', 'first-of-type', 'last-of-type', 'only-of-type',
    'nth-child', 'nth-last-child', 'nth-of-type', 'nth-last-of-type', 'empty', 'root', 'scope',
    'link', 'visited', 'placeholder-shown',
  ]);
  const PSEUDO_ELEMENTS = new Set(['before', 'after']);

  function parseCompound(p, text) {
    const c = { tag: null, id: null, classes: [], attrs: [], pseudos: [], pseudoElement: null };
    let any = false;
    if (p.peek() === '*') {
      p.i++;
      any = true;
    } else if (IDENT.test(p.peek() ?? '')) {
      c.tag = p.ident();
      any = true;
    }
    for (;;) {
      const ch = p.peek();
      if (ch === '#') {
        p.i++;
        c.id = p.ident();
        any = true;
      } else if (ch === '.') {
        p.i++;
        c.classes.push(p.ident());
        any = true;
      } else if (ch === '[') {
        p.i++;
        p.ws();
        const name = p.ident();
        p.ws();
        let op = null;
        let value = null;
        let insensitive = false;
        if (p.peek() !== ']') {
          const m = /^([~|^$*]?=)/.exec(p.s.slice(p.i));
          if (!m) fail(text);
          op = m[1];
          p.i += op.length;
          p.ws();
          const q = p.peek();
          if (q === '"' || q === "'") {
            const end = p.s.indexOf(q, p.i + 1);
            if (end < 0) fail(text);
            value = p.s.slice(p.i + 1, end);
            p.i = end + 1;
          } else {
            value = p.ident();
          }
          p.ws();
          if (/[iI]/.test(p.peek() ?? '')) {
            p.i++;
            insensitive = true;
            p.ws();
          }
        }
        if (p.peek() !== ']') fail(text);
        p.i++;
        c.attrs.push({ name, op, value, insensitive });
        any = true;
      } else if (ch === ':') {
        p.i++;
        let element = false;
        if (p.peek() === ':') {
          p.i++;
          element = true;
        }
        const name = p.ident().toLowerCase();
        let arg = null;
        if (p.peek() === '(') {
          p.i++;
          arg = p.balanced();
        }
        if (element || PSEUDO_ELEMENTS.has(name)) {
          c.pseudoElement = name;
        } else if (LIST_PSEUDOS.has(name)) {
          c.pseudos.push({ name: name === 'matches' || name === 'any' ? 'is' : name, list: parseList(arg ?? '') });
        } else if (name.startsWith('nth-')) {
          const nth = parseNth(arg ?? '');
          if (!nth) fail(text);
          c.pseudos.push({ name, nth });
        } else {
          if (!KNOWN_PSEUDOS.has(name)) D.warnOnce(`pseudo:${name}`, `selector pseudo-class :${name} is not supported (never matches)`);
          c.pseudos.push({ name, arg });
        }
        any = true;
      } else {
        break;
      }
    }
    if (!any) fail(text);
    return c;
  }

  function specificityOfCompound(c) {
    let a = c.id ? 1 : 0;
    let b = c.classes.length + c.attrs.length;
    let d = c.tag ? 1 : 0;
    for (const ps of c.pseudos) {
      if (ps.list) {
        if (ps.name === 'where') continue;
        let best = 0;
        for (const s of ps.list) best = Math.max(best, s.specificity);
        a += Math.floor(best / 1e6);
        b += Math.floor(best / 1e3) % 1e3;
        d += best % 1e3;
      } else {
        b++;
      }
    }
    if (c.pseudoElement) d++;
    return a * 1e6 + b * 1e3 + d;
  }

  function parseComplex(p, text) {
    const compounds = [];
    const combinators = [];
    p.ws();
    compounds.push(parseCompound(p, text));
    for (;;) {
      const hadWs = p.ws();
      const ch = p.peek();
      if (ch === undefined || ch === ',' || ch === ')') break;
      let comb = ' ';
      if (ch === '>' || ch === '+' || ch === '~') {
        comb = ch;
        p.i++;
        p.ws();
      } else if (!hadWs) {
        fail(text);
      }
      combinators.push(comb);
      compounds.push(parseCompound(p, text));
    }
    compounds.reverse();
    combinators.reverse();
    let specificity = 0;
    for (const c of compounds) specificity += specificityOfCompound(c);
    const subject = compounds[0];
    return { compounds, combinators, specificity, pseudoElement: subject.pseudoElement, text };
  }

  const listCache = new Map();

  // Parse a selector list ("a, b > c"). Throws SyntaxError when invalid.
  function parseList(text) {
    const cached = listCache.get(text);
    if (cached) return cached;
    const p = new Parser(text);
    const list = [];
    for (;;) {
      const start = p.i;
      const complex = parseComplex(p, text);
      complex.text = text.slice(start, p.i).trim();
      list.push(complex);
      p.ws();
      if (p.peek() === ',') {
        p.i++;
        continue;
      }
      if (!p.eof()) fail(text);
      break;
    }
    if (listCache.size > 2000) listCache.clear();
    listCache.set(text, list);
    return list;
  }

  const SVG_NS = 'http://www.w3.org/2000/svg';

  const tagMatches = (el, tag) => {
    if (el.namespaceURI === SVG_NS) return el.localName === tag || el.localName.toLowerCase() === tag.toLowerCase();
    return el.localName === tag.toLowerCase();
  };

  const prevElement = (el) => {
    let n = el._prev;
    while (n && n.nodeType !== 1) n = n._prev;
    return n;
  };
  const nextElement = (el) => {
    let n = el._next;
    while (n && n.nodeType !== 1) n = n._next;
    return n;
  };
  const parentElement = (el) => {
    const p = el.parentNode;
    return p && p.nodeType === 1 ? p : null;
  };

  const nthMatches = ([a, b], index) => {
    if (a === 0) return index === b;
    const n = (index - b) / a;
    return Number.isInteger(n) && n >= 0;
  };

  const indexAmong = (el, backwards, ofType) => {
    let i = 1;
    for (let n = backwards ? nextElement(el) : prevElement(el); n; n = backwards ? nextElement(n) : prevElement(n)) {
      if (!ofType || n.localName === el.localName) i++;
    }
    return i;
  };

  function matchPseudo(el, ps, scope) {
    switch (ps.name) {
      case 'not':
        return !ps.list.some((s) => matchComplex(el, s, scope));
      case 'is':
      case 'where':
        return ps.list.some((s) => matchComplex(el, s, scope));
      case 'hover':
        return !!el._hover;
      case 'active':
        return !!el._active;
      case 'focus':
      case 'focus-visible':
        return el.ownerDocument && el.ownerDocument._activeElement === el;
      case 'focus-within': {
        const active = el.ownerDocument && el.ownerDocument._activeElement;
        return !!active && el.contains(active);
      }
      case 'checked':
        return !!el.checked;
      case 'disabled':
        return el.hasAttribute('disabled');
      case 'enabled':
        return !el.hasAttribute('disabled');
      case 'first-child':
        return parentElement(el) !== null && !prevElement(el);
      case 'last-child':
        return parentElement(el) !== null && !nextElement(el);
      case 'only-child':
        return parentElement(el) !== null && !prevElement(el) && !nextElement(el);
      case 'first-of-type':
        return indexAmong(el, false, true) === 1;
      case 'last-of-type':
        return indexAmong(el, true, true) === 1;
      case 'only-of-type':
        return indexAmong(el, false, true) === 1 && indexAmong(el, true, true) === 1;
      case 'nth-child':
        return nthMatches(ps.nth, indexAmong(el, false, false));
      case 'nth-last-child':
        return nthMatches(ps.nth, indexAmong(el, true, false));
      case 'nth-of-type':
        return nthMatches(ps.nth, indexAmong(el, false, true));
      case 'nth-last-of-type':
        return nthMatches(ps.nth, indexAmong(el, true, true));
      case 'empty':
        return !el._first;
      case 'root':
        return el.ownerDocument && el.ownerDocument.documentElement === el;
      case 'scope':
        return scope ? el === scope : el.ownerDocument && el.ownerDocument.documentElement === el;
      default:
        return false;
    }
  }

  function matchCompound(el, c, scope) {
    if (c.tag !== null && !tagMatches(el, c.tag)) return false;
    if (c.id !== null && el.getAttribute('id') !== c.id) return false;
    if (c.classes.length) {
      const classes = el._classes;
      for (const cls of c.classes) if (!classes.includes(cls)) return false;
    }
    for (const a of c.attrs) {
      let v = el.getAttribute(a.name);
      if (v === null) return false;
      if (a.op === null) continue;
      let want = a.value;
      if (a.insensitive) {
        v = v.toLowerCase();
        want = want.toLowerCase();
      }
      switch (a.op) {
        case '=':
          if (v !== want) return false;
          break;
        case '~=':
          if (!v.split(/\s+/).includes(want)) return false;
          break;
        case '|=':
          if (v !== want && !v.startsWith(`${want}-`)) return false;
          break;
        case '^=':
          if (!want || !v.startsWith(want)) return false;
          break;
        case '$=':
          if (!want || !v.endsWith(want)) return false;
          break;
        case '*=':
          if (!want || !v.includes(want)) return false;
          break;
        default:
          return false;
      }
    }
    for (const ps of c.pseudos) if (!matchPseudo(el, ps, scope)) return false;
    return true;
  }

  function matchFrom(el, sel, i, scope) {
    if (!matchCompound(el, sel.compounds[i], scope)) return false;
    if (i === sel.compounds.length - 1) return true;
    switch (sel.combinators[i]) {
      case ' ':
        for (let a = parentElement(el); a; a = parentElement(a)) if (matchFrom(a, sel, i + 1, scope)) return true;
        return false;
      case '>': {
        const a = parentElement(el);
        return !!a && matchFrom(a, sel, i + 1, scope);
      }
      case '+': {
        const s = prevElement(el);
        return !!s && matchFrom(s, sel, i + 1, scope);
      }
      case '~':
        for (let s = prevElement(el); s; s = prevElement(s)) if (matchFrom(s, sel, i + 1, scope)) return true;
        return false;
      default:
        return false;
    }
  }

  // Whether `el` matches complex selector `sel` (ignoring its pseudo-element).
  function matchComplex(el, sel, scope) {
    return matchFrom(el, sel, 0, scope);
  }

  // Keys a selector depends on outside its subject: classes, ids and
  // attributes in ancestor or sibling position, and whether structural or
  // state pseudo-classes appear. The cascade uses them to decide how far a
  // change has to re-match.
  function collectKeys(sel, keys) {
    sel.compounds.forEach((c, i) => {
      const where = i === 0 ? null : sel.combinators[i - 1] === ' ' || sel.combinators[i - 1] === '>' ? 'ancestor' : 'sibling';
      const note = (k) => {
        if (where === 'ancestor') keys.ancestor.add(k);
        else if (where === 'sibling') keys.sibling.add(k);
      };
      for (const cls of c.classes) note(`.${cls}`);
      if (c.id) note(`#${c.id}`);
      if (c.tag) note(`<${c.tag}`);
      for (const a of c.attrs) {
        keys.attrs.add(a.name);
        note(`[${a.name}`);
      }
      for (const ps of c.pseudos) {
        if (ps.list) {
          for (const s of ps.list) collectKeys(s, keys);
          // A class inside :not() on the subject still only concerns the subject.
        }
        if (/child|of-type|empty/.test(ps.name)) keys.structural = true;
        if (ps.name === 'hover' || ps.name === 'active' || ps.name.startsWith('focus')) {
          keys.state = true;
          if (where) keys.stateOutsideSubject = true;
        }
        if (ps.name === 'checked' || ps.name === 'disabled' || ps.name === 'enabled') keys.attrs.add(ps.name);
      }
    });
    if (sel.combinators.some((c) => c === '+' || c === '~')) keys.siblingCombinator = true;
  }

  D.selectors = { parseList, matchComplex, matchCompound, collectKeys, parentElement, prevElement, nextElement };
})();
