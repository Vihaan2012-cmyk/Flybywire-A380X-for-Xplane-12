// The window of an MSFS view: a VCockpit page at
// coui://html_ui/Pages/VCockpit/Core/VCockpit.html (asobo-vcockpits-core),
// a top-level document, so its parent and top are itself. Also URL and
// URLSearchParams, which instruments use to read their gauge URL, as the
// WHATWG URL standard parses the parts they read.
(() => {
  globalThis.window = globalThis;
  globalThis.self = globalThis;
  globalThis.parent = globalThis;
  globalThis.top = globalThis;
  globalThis.closed = false;
  globalThis.name = '';

  class URLSearchParams {
    constructor(init = '') {
      this._list = [];
      if (typeof init === 'string') {
        const query = init.startsWith('?') ? init.slice(1) : init;
        for (const pair of query.split('&')) {
          if (pair === '') continue;
          const eq = pair.indexOf('=');
          const name = eq < 0 ? pair : pair.slice(0, eq);
          const value = eq < 0 ? '' : pair.slice(eq + 1);
          this._list.push([decode(name), decode(value)]);
        }
      } else if (init && typeof init === 'object') {
        const entries = Symbol.iterator in init ? Array.from(init) : Object.entries(init);
        for (const [k, v] of entries) this._list.push([String(k), String(v)]);
      }
    }
    get(name) {
      const e = this._list.find(([k]) => k === String(name));
      return e === undefined ? null : e[1];
    }
    getAll(name) {
      return this._list.filter(([k]) => k === String(name)).map(([, v]) => v);
    }
    has(name) {
      return this._list.some(([k]) => k === String(name));
    }
    set(name, value) {
      const i = this._list.findIndex(([k]) => k === String(name));
      if (i < 0) {
        this._list.push([String(name), String(value)]);
      } else {
        this._list[i][1] = String(value);
        this._list = this._list.filter(([k], j) => j <= i || k !== String(name));
      }
    }
    append(name, value) {
      this._list.push([String(name), String(value)]);
    }
    delete(name) {
      this._list = this._list.filter(([k]) => k !== String(name));
    }
    forEach(fn, thisArg) {
      for (const [k, v] of this._list) fn.call(thisArg, v, k, this);
    }
    keys() {
      return this._list.map(([k]) => k)[Symbol.iterator]();
    }
    values() {
      return this._list.map(([, v]) => v)[Symbol.iterator]();
    }
    entries() {
      return this._list.map(([k, v]) => [k, v])[Symbol.iterator]();
    }
    [Symbol.iterator]() {
      return this.entries();
    }
    toString() {
      return this._list.map(([k, v]) => `${encode(k)}=${encode(v)}`).join('&');
    }
  }
  const decode = (s) => {
    try {
      return decodeURIComponent(s.replace(/\+/g, ' '));
    } catch {
      return s;
    }
  };
  const encode = (s) => encodeURIComponent(s).replace(/%20/g, '+');

  // scheme://host/path?query#fragment, and paths relative to a base.
  const PATTERN = /^([a-zA-Z][a-zA-Z0-9+.-]*):(?:\/\/([^/?#]*))?([^?#]*)(\?[^#]*)?(#.*)?$/;
  const removeDots = (path) => {
    const out = [];
    const parts = path.split('/');
    for (let i = 0; i < parts.length; i++) {
      const p = parts[i];
      if (p === '..') {
        if (out.length > 1) out.pop();
        if (i === parts.length - 1) out.push('');
      } else if (p === '.') {
        if (i === parts.length - 1) out.push('');
      } else {
        out.push(p);
      }
    }
    return out.join('/');
  };
  class URL {
    constructor(url, base) {
      let text = String(url);
      let m = PATTERN.exec(text);
      if (!m) {
        if (base === undefined) {
          throw new TypeError(`Failed to construct 'URL': Invalid URL ${text}`);
        }
        const b = new URL(base);
        if (text.startsWith('//')) {
          text = `${b.protocol}${text}`;
        } else if (text.startsWith('/')) {
          text = `${b.protocol}//${b.host}${text}`;
        } else if (text.startsWith('?')) {
          text = `${b.protocol}//${b.host}${b.pathname}${text}`;
        } else if (text.startsWith('#')) {
          text = `${b.protocol}//${b.host}${b.pathname}${b.search}${text}`;
        } else {
          text = `${b.protocol}//${b.host}${b.pathname.replace(/[^/]*$/, '')}${text}`;
        }
        m = PATTERN.exec(text);
      }
      this.protocol = `${m[1].toLowerCase()}:`;
      this.host = m[2] ?? '';
      this.hostname = this.host.replace(/:\d+$/, '');
      this.port = (/:(\d+)$/.exec(this.host) || [])[1] || '';
      this.pathname = removeDots(m[3] || '/') || '/';
      this.search = m[4] && m[4] !== '?' ? m[4] : '';
      this.hash = m[5] && m[5] !== '#' ? m[5] : '';
      this.searchParams = new URLSearchParams(this.search);
    }
    get origin() {
      return `${this.protocol}//${this.host}`;
    }
    get href() {
      return `${this.protocol}//${this.host}${this.pathname}${this.search}${this.hash}`;
    }
    toString() {
      return this.href;
    }
    toJSON() {
      return this.href;
    }
  }
  globalThis.URL = URL;
  globalThis.URLSearchParams = URLSearchParams;

  const page = new URL('coui://html_ui/Pages/VCockpit/Core/VCockpit.html');
  globalThis.location = {
    href: page.href,
    protocol: page.protocol,
    host: page.host,
    hostname: page.hostname,
    port: '',
    origin: page.origin,
    pathname: page.pathname,
    search: '',
    hash: '',
    reload: () => console.warn('MSFS runtime: location.reload is not supported here'),
    toString: () => page.href,
  };
})();
