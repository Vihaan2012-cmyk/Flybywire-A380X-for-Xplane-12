// What scripts see: the DOM's classes and functions as globals,
// `__createDocument(screen, width, height)` for the runtime, mouse input
// through `__screenEvent`, and a frame after every engine tick.
(() => {
  const D = globalThis.__dom;
  const g = globalThis;
  const HTML_NS = D.NS.HTML;

  Object.assign(g, {
    Event: D.Event, CustomEvent: D.CustomEvent, UIEvent: D.UIEvent, MouseEvent: D.MouseEvent, PointerEvent: D.PointerEvent,
    WheelEvent: D.WheelEvent, KeyboardEvent: D.KeyboardEvent, FocusEvent: D.FocusEvent, EventTarget: D.EventTarget,
    Node: D.Node, CharacterData: D.CharacterData, Text: D.Text, Comment: D.Comment, Element: D.Element,
    HTMLElement: D.HTMLElement, SVGElement: D.SVGElement, DocumentFragment: D.DocumentFragment, Document: D.Document,
    DOMRect: D.DOMRect, DOMTokenList: D.DOMTokenList, CSSStyleDeclaration: D.CSSStyleDeclaration,
    CanvasRenderingContext2D: D.CanvasRenderingContext2D, Path2D: D.Path2D, CanvasGradient: D.CanvasGradient,
    customElements: D.customElements,
    getComputedStyle: D.getComputedStyle,
  });
  Object.assign(g, D.classes);

  // new Image(width, height)
  g.Image = function Image(width, height) {
    const doc = D.currentDocument() ?? D.documents.values().next().value;
    if (!doc) throw new TypeError('Image: there is no document');
    const img = doc.createElementNS(HTML_NS, 'img');
    if (width !== undefined) img.setAttribute('width', width);
    if (height !== undefined) img.setAttribute('height', height);
    return img;
  };

  // ---------------------------------------------------------------- documents

  // A document for a screen, painting to `__host.submitDisplay(screen, ...)`.
  // Size defaults to what the renderer expects for the screen.
  g.__createDocument = (screen, width, height) => {
    const id = String(screen);
    if (width === undefined || height === undefined) {
      const host = g.__host;
      const size = host && typeof host.screenSize === 'function' ? host.screenSize(id) : null;
      if (!size) throw new TypeError(`__createDocument: no size for screen ${id}`);
      [width, height] = size;
    }
    const doc = new D.ScreenDocument(id, Number(width), Number(height));
    D.documents.set(id, doc);
    return doc;
  };
  g.__destroyDocument = (screen) => {
    D.documents.delete(String(screen));
  };

  // Every document's frame: styles, layout and paint where something
  // changed. What it costs is kept in `__dom.cost` and, when the host wants
  // it, reported through `__host.domCost(ms)`.
  D.cost = { lastMs: 0, maxMs: 0, totalMs: 0, frames: 0 };
  D.frameAll = () => {
    const started = D.wallClock();
    for (const doc of D.documents.values()) {
      try {
        doc.frame();
      } catch (e) {
        console.error(`[dom] ${doc.__screen}: ${e}`, e && e.stack);
      }
    }
    const ms = D.wallClock() - started;
    const c = D.cost;
    c.lastMs = ms;
    c.maxMs = Math.max(c.maxMs, ms);
    c.totalMs += ms;
    c.frames++;
    const host = g.__host;
    if (host && typeof host.domCost === 'function') host.domCost(ms);
  };
  const tick = g.__tick;
  if (typeof tick === 'function' && !tick.__domFrame) {
    const wrapped = (ms) => {
      const r = tick(ms);
      D.frameAll();
      return r;
    };
    wrapped.__domFrame = true;
    g.__tick = wrapped;
  }

  // ---------------------------------------------------------------- mouse input

  const BUTTON_BITS = { 0: 1, 1: 4, 2: 2 };
  // Chromium's double-click interval and slop on Windows (GetDoubleClickTime
  // defaults to 500 ms; SM_CXDOUBLECLK to 4 px).
  const DOUBLE_CLICK_MS = 500;
  const DOUBLE_CLICK_PX = 4;

  const documentFor = (screen) => {
    const id = String(screen);
    const doc = D.documents.get(id);
    if (doc) return doc;
    const key = id.replace(/^\$/, '').toLowerCase();
    for (const [name, d] of D.documents) if (name.replace(/^\$/, '').toLowerCase() === key) return d;
    return null;
  };

  const chain = (el) => {
    const out = [];
    for (let n = el; n && n.nodeType === 1; n = n.parentNode) out.push(n);
    return out;
  };

  const focusable = (el) => {
    for (let n = el; n && n.nodeType === 1; n = n.parentNode) {
      if (n.hasAttribute('tabindex') || ((n.localName === 'input' || n.localName === 'button' || n.localName === 'textarea' || n.localName === 'select') && !n.hasAttribute('disabled'))) return n;
      if (n.getAttribute('contenteditable') === 'true') return n;
    }
    return null;
  };

  g.__screenEvent = (screen, type, x, y, button, delta) => {
    const doc = documentFor(screen);
    if (!doc) return;
    const state = doc._mouse || (doc._mouse = { buttons: 0, down: null, lastClick: null });
    const target = D.paint.hitTest(doc, x, y) ?? doc.body ?? doc.documentElement;
    const init = (extra) => {
      const rect = target && target.nodeType === 1 ? D.paint.clientRect(doc, target) : { left: 0, top: 0 };
      return { bubbles: true, cancelable: true, composed: true, clientX: x, clientY: y, screenX: x, screenY: y, offsetX: x - rect.left, offsetY: y - rect.top, button: button | 0, buttons: state.buttons, view: g, ...extra };
    };
    const fire = (el, Cls, name, extra) => el && el.dispatchEvent(new Cls(name, init(extra)));

    // Hover: out/leave on what the pointer left, over/enter on what it reached.
    if (target !== doc._hovered) {
      const old = doc._hovered;
      const oldChain = old && old._connected ? chain(old) : [];
      const newChain = chain(target);
      if (old && old._connected) {
        fire(old, D.PointerEvent, 'pointerout', { relatedTarget: target });
        fire(old, D.MouseEvent, 'mouseout', { relatedTarget: target });
        for (const n of oldChain) {
          if (newChain.includes(n)) break;
          n._hover = false;
          n.dispatchEvent(new D.PointerEvent('pointerleave', init({ bubbles: false, cancelable: false, relatedTarget: target })));
          n.dispatchEvent(new D.MouseEvent('mouseleave', init({ bubbles: false, cancelable: false, relatedTarget: target })));
        }
        doc._hoverChanged(old);
      }
      fire(target, D.PointerEvent, 'pointerover', { relatedTarget: old });
      fire(target, D.MouseEvent, 'mouseover', { relatedTarget: old });
      for (const n of newChain.slice().reverse()) {
        if (oldChain.includes(n)) continue;
        n._hover = true;
        n.dispatchEvent(new D.PointerEvent('pointerenter', init({ bubbles: false, cancelable: false, relatedTarget: old })));
        n.dispatchEvent(new D.MouseEvent('mouseenter', init({ bubbles: false, cancelable: false, relatedTarget: old })));
      }
      for (const n of newChain) n._hover = true;
      doc._hovered = target;
      doc._hoverChanged(target);
    }

    switch (type) {
      case 'move':
        fire(target, D.PointerEvent, 'pointermove', {});
        fire(target, D.MouseEvent, 'mousemove', {});
        break;
      case 'down': {
        state.buttons |= BUTTON_BITS[button] ?? 0;
        fire(target, D.PointerEvent, 'pointerdown', {});
        const allowed = fire(target, D.MouseEvent, 'mousedown', {});
        state.down = { target, button };
        if (allowed) {
          const f = focusable(target);
          if (f) f.focus();
          else if (doc._activeElement) doc._focus(null);
        }
        if (button === 2) fire(target, D.MouseEvent, 'contextmenu', {});
        break;
      }
      case 'up': {
        state.buttons &= ~(BUTTON_BITS[button] ?? 0);
        fire(target, D.PointerEvent, 'pointerup', {});
        fire(target, D.MouseEvent, 'mouseup', {});
        const down = state.down;
        state.down = null;
        if (down && down.button === button && down.target._connected) {
          // The click goes to the nearest element both presses were in.
          const upChain = chain(target);
          const common = chain(down.target).find((n) => upChain.includes(n));
          if (common) {
            const now = g.performance ? performance.now() : 0;
            const last = state.lastClick;
            const detail = last && last.target === common && now - last.at <= DOUBLE_CLICK_MS && Math.abs(last.x - x) <= DOUBLE_CLICK_PX && Math.abs(last.y - y) <= DOUBLE_CLICK_PX ? last.detail + 1 : 1;
            if (button === 0) {
              common.dispatchEvent(new D.PointerEvent('click', init({ detail })));
              if (detail === 2) common.dispatchEvent(new D.MouseEvent('dblclick', init({ detail })));
            } else {
              common.dispatchEvent(new D.PointerEvent('auxclick', init({ detail })));
            }
            state.lastClick = { target: common, at: now, x, y, detail };
          }
        }
        break;
      }
      case 'wheel':
        target.dispatchEvent(new D.WheelEvent('wheel', init({ deltaY: Number(delta) || 0, deltaMode: 0 })));
        break;
      default:
        D.warnOnce(`screen-event:${type}`, `screen event '${type}' is not known`);
    }
  };
})();
