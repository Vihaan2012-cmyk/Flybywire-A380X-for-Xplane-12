// Copyright (c) 2026 -- Study pages for the FlyByWire A380X EFB.
// SPDX-License-Identifier: GPL-3.0
//
// The pages are the X-Plane Study app, app/ui/index.html, copied verbatim by
// build.sh. There is one implementation of them, not two that drift apart.
//
// It is injected into the EFB's own document rather than an iframe. Coherent
// GT -- the EFB's browser -- renders a nested document's markup but does not
// run its scripts or route input into it, so an iframe gives a page that
// looks right, does nothing, and cannot be clicked. Injected into the same
// document, the app's own code runs and its own handlers receive clicks.
//
// efb.js is never touched. The only change to a FlyByWire file is one
// import-script line in efb.html.

(function () {
  'use strict';

  var BASE = '/Pages/VCockpit/Instruments/A380X/EFB/';
  var PAGE = BASE + 'study-app.html';
  var HOST = 'ds-app';

  // The width the app is laid out in, which is not the width of the screen.
  //
  // The app was written for a desktop window a metre away. This is a small
  // object in a 3D cockpit, and FlyByWire's own EFB pages are set about 1.4
  // times larger than the app's to suit it -- put their dashboard beside
  // our component list and that difference is the whole of it. Resolution
  // does not close it; 12px text is 12px text however finely it is drawn.
  //
  // So the app gets a 1000px-wide box and the box is scaled up to fill the
  // screen. Every length grows by the same factor and nothing is lost,
  // because the scaling happens when it is drawn rather than when it is
  // measured. build.py resolves the app's own lengths against this same
  // number, so the two must not drift.
  var APP_W = 1000;

  var shell = null;
  var loaded = false;

  // The size the document has been pinned to when the render target is
  // larger than the page was written for -- see applyScale. Null means the
  // target is its original size and the viewport can be trusted.
  var pinned = null;

  function build() {
    shell = document.createElement('div');
    shell.id = 'deepstudy';
    // Not 100vw/100vh when the document is pinned: a viewport unit measures
    // the enlarged render target, not the box the page was pinned into, and
    // the overlay would lay itself out at twice the width before being
    // scaled twice again. This is set inline because a stylesheet rule
    // cannot override an inline style without !important, and fighting our
    // own declaration is a worse way to say it.
    var screenW = pinned ? pinned[0] : window.innerWidth;
    var screenH = pinned ? pinned[1] : window.innerHeight;
    shell.style.cssText =
      'position:fixed;top:0;left:0;width:' + screenW + 'px;height:' + screenH +
      'px;z-index:2147483000;display:none;background:#0d1218;overflow:hidden;';

    // The app's own box: APP_W wide, whatever height that leaves at the
    // screen's shape, drawn at the size of the screen.
    var zoom = screenW / APP_W;
    var host = document.createElement('div');
    host.id = HOST;
    host.style.cssText =
      'width:' + APP_W + 'px;height:' + Math.round(screenH / zoom) + 'px;' +
      'transform:scale(' + zoom + ');transform-origin:0 0;' +
      'display:flex;flex-direction:column;overflow:hidden;';
    shell.appendChild(host);

    // Sits above the app, in the screen's own coordinates rather than the
    // app's -- so its size is scaled by hand to match what the app around
    // it ends up looking like, instead of staying stubbornly 13px.
    var close = document.createElement('div');
    close.textContent = 'CLOSE';
    close.style.cssText =
      'position:absolute;right:' + Math.round(10 * zoom) + 'px;bottom:' +
      Math.round(10 * zoom) + 'px;z-index:10;cursor:pointer;' +
      'font:600 ' + Math.round(13 * zoom) + 'px "Segoe UI",Tahoma,sans-serif;color:#eef4f9;' +
      'border:2px solid #5c6b7a;padding:' + Math.round(8 * zoom) + 'px ' +
      Math.round(18 * zoom) + 'px;background:#0d1218;';
    close.addEventListener('click', hide);
    shell.appendChild(close);

    document.body.appendChild(shell);
    return host;
  }

  // -------------------------------------------------- keeping our CSS ours
  //
  // The app's stylesheet has to live in document.head: there is one document
  // here, the EFB's, and a <style> applies only where it is. But the app was
  // written for a window of its own, so it styles `button`, `input`,
  // `table`, `.box`, `.tabs` -- names FlyByWire's own pages use as well.
  // Left alone, every one of those rules reaches their whole EFB and quietly
  // restyles it. Their Settings page is where it showed first, its labels
  // and fields laid over each other, but nothing was safe.
  //
  // So every selector gets our host prefixed onto it, by walking the text
  // rather than by a regular expression: rules nest inside @media, and
  // @keyframes' `from`, `to` and `50%` are not selectors at all -- prefixing
  // those would break every animation the app has.

  // Split a selector list on the commas that are not inside brackets, so
  // `:not(a, b)` and `:is(.x, .y)` survive as one selector.
  function splitSelectors(list) {
    var parts = [];
    var depth = 0;
    var start = 0;
    for (var i = 0; i < list.length; i++) {
      var c = list.charAt(i);
      if (c === '(' || c === '[') depth++;
      else if (c === ')' || c === ']') depth--;
      else if (c === ',' && depth === 0) {
        parts.push(list.slice(start, i));
        start = i + 1;
      }
    }
    parts.push(list.slice(start));
    return parts;
  }

  function prefixSelectors(list, sel) {
    return splitSelectors(list).map(function (one) {
      var t = one.trim();
      // Already ours: the html/body/:root rewrite turns those into the host
      // itself, and prefixing again would aim at a descendant of it.
      if (!t || t === sel || t.indexOf(sel) === 0) return t;
      return sel + ' ' + t;
    }).join(', ');
  }

  function scopeRules(css, sel) {
    var out = '';
    var i = 0;
    while (i < css.length) {
      var open = css.indexOf('{', i);
      if (open < 0) { out += css.slice(i); break; }

      var depth = 1;
      var j = open + 1;
      while (j < css.length && depth > 0) {
        var c = css.charAt(j);
        if (c === '{') depth++;
        else if (c === '}') depth--;
        j++;
      }
      var head = css.slice(i, open).trim();
      var body = css.slice(open + 1, j - 1);

      if (head.charAt(0) === '@') {
        if (/^@(media|supports|layer|container|document)\b/i.test(head)) {
          // These hold rules of their own, so go in.
          out += head + '{' + scopeRules(body, sel) + '}';
        } else {
          // @keyframes, @font-face, @page: their contents are not selectors.
          out += head + '{' + body + '}';
        }
      } else {
        out += prefixSelectors(head, sel) + '{' + body + '}';
      }
      i = j;
    }
    return out;
  }

  function scopeCss(css, sel) {
    return scopeRules(
      css
        // Comments first: a `}` inside one would derail the brace walk.
        .replace(/\/\*[\s\S]*?\*\//g, '')
        // The app's own frame. Here html and body are the EFB's, so these
        // become the host element itself rather than a descendant of it.
        .replace(/(^|[\s,}])html\s*,\s*body\b/g, '$1' + sel)
        .replace(/(^|[\s,}])body\b/g, '$1' + sel)
        .replace(/(^|[\s,}]):root\b/g, '$1' + sel),
      sel
    );
  }

  // ----------------------------------------------------- what the DOM lacks
  //
  // Coherent GT 2.9.5 is not Chromium; it is Coherent Labs' own engine with
  // its own subset of the DOM, and replaceChildren is not in it. The app
  // calls it in seven places, every one of them the first thing a page does
  // to clear its container. The call throws, the rest of that render never
  // runs, and the page comes up blank under a heading that says how many
  // rows it is about to draw -- which is exactly what Flight Controls,
  // Landing Gear and the breaker list were doing.
  //
  // append() is here, so the replacement is the obvious one.
  function polyfill() {
    ['Element', 'Document', 'DocumentFragment'].forEach(function (name) {
      var proto = window[name] && window[name].prototype;
      if (!proto || proto.replaceChildren) return;
      proto.replaceChildren = function () {
        while (this.firstChild) this.removeChild(this.firstChild);
        if (arguments.length) this.append.apply(this, arguments);
      };
    });
  }

  // ------------------------------------------------- giving the SVGs a size
  //
  // The synoptics are `.study-svg { width: 100%; height: auto; }`. A browser
  // that works out a replaced element's intrinsic ratio from its viewBox
  // gives them the right height; this engine does not, so they fall back to
  // the default 150px and the diagram is drawn tiny and centred inside a
  // letterbox. That is the "separate image" on the Fuel page.
  //
  // The ratio is written on every one of them, so read it and set the
  // height. The app redraws its pages as values change, which replaces the
  // element, so this runs on a timer rather than once.
  function sizeDiagrams() {
    var svgs = document.querySelectorAll('#' + HOST + ' svg.study-svg');
    for (var i = 0; i < svgs.length; i++) {
      var svg = svgs[i];
      var box = (svg.getAttribute('viewBox') || '').split(/[\s,]+/);
      if (box.length !== 4) continue;
      var w = parseFloat(box[2]);
      var h = parseFloat(box[3]);
      var width = svg.clientWidth || svg.parentNode.clientWidth;
      if (!(w > 0 && h > 0 && width > 0)) continue;
      var want = Math.round(width * h / w) + 'px';
      if (svg.style.height !== want) svg.style.height = want;
    }
  }

  function inject(host, html) {
    var doc = new DOMParser().parseFromString(html, 'text/html');

    doc.querySelectorAll('style').forEach(function (st) {
      var out = document.createElement('style');
      out.setAttribute('data-deepstudy', '');
      out.textContent = scopeCss(st.textContent, '#' + HOST);
      document.head.appendChild(out);
    });

    // The app's code is NOT in this document as text. Coherent GT will not
    // run a <script> element made at runtime and throws on the Function
    // constructor, so build.py writes the app's block out as study-app.js,
    // efb.html loads it the same way it loads this file, and it leaves a
    // starter behind for us to call once the markup is here.
    doc.querySelectorAll('script').forEach(function (sc) { sc.remove(); });
    host.innerHTML = doc.body.innerHTML;

    if (typeof window.__deepStudyStart !== 'function') {
      fail(host, 'study-app.js did not load: no __deepStudyStart. Check that efb.html imports it and layout.json lists it.');
      return;
    }
    polyfill();
    try {
      window.__deepStudyStart();
      setInterval(sizeDiagrams, 400);
    } catch (e) {
      fail(host, 'Study app failed to start: ' + (e && e.stack ? e.stack : e));
    }
  }

  function fail(host, text) {
    var err = document.createElement('div');
    err.style.cssText = 'padding:16px;color:#e8a33d;font:13px "Segoe UI",system-ui,sans-serif;white-space:pre-wrap;';
    err.textContent = text;
    host.insertBefore(err, host.firstChild);
  }

  function show() {
    var host = shell ? document.getElementById(HOST) : build();
    shell.style.display = 'block';
    if (loaded) return;
    loaded = true;
    fetch(PAGE)
      .then(function (r) {
        if (!r.ok) throw new Error('study-app.html: HTTP ' + r.status);
        return r.text();
      })
      .then(function (html) { inject(host, html); })
      .catch(function (e) {
        loaded = false;
        host.innerHTML = '';
        var m = document.createElement('div');
        m.style.cssText = 'padding:20px;color:#e8a33d;font:14px "Segoe UI",system-ui,sans-serif;';
        m.textContent = e.message;
        host.appendChild(m);
      });
  }

  function hide() { if (shell) shell.style.display = 'none'; }

  // ---------------------------------------------- a button in THEIR toolbar

  var ICON =
    '<svg xmlns="http://www.w3.org/2000/svg" width="35" height="35" viewBox="0 0 16 16" fill="currentColor">' +
    '<path d="M8 1.5 1 5l7 3.5L15 5 8 1.5zM1 8.2V11l7 3.5L15 11V8.2l-7 3.5-7-3.5z"/></svg>';

  var btn = null;

  function placeButton() {
    if (btn && btn.isConnected) return;
    // Matching their failures link by href rather than by class keeps this
    // working when their bundle is rebuilt and class names change.
    var link = document.querySelector('a[href$="/failures"], a[href*="failures"]');
    if (!link) return;
    var slot = link.parentElement || link;
    if (!slot.parentElement) return;

    btn = document.createElement('div');
    btn.id = 'deepstudy-btn';
    btn.title = 'Study';
    btn.innerHTML = ICON;
    btn.style.cssText =
      'display:flex;align-items:center;justify-content:center;padding:10px 0;cursor:pointer;color:#93a3b2;';
    btn.addEventListener('mouseenter', function () { btn.style.color = '#eef4f9'; });
    btn.addEventListener('mouseleave', function () { btn.style.color = '#93a3b2'; });
    btn.addEventListener('click', function (ev) { ev.preventDefault(); ev.stopPropagation(); show(); });
    slot.parentElement.insertBefore(btn, slot.nextSibling);
  }

  // -------------------------------------------------------- a sharper screen
  //
  // The tablet is a 1430x1000 render target -- panel.cfg, [VCockpit15] --
  // and that is every pixel it has, ours and FlyByWire's alike. Text on it
  // is as sharp as that and no sharper.
  //
  // install.ps1 -Scale enlarges the target. On its own that buys nothing:
  // the page would simply lay itself out in more CSS pixels, so a 15px line
  // would still be 15 device pixels tall, just physically smaller on the
  // panel. FlyByWire's own EFB would shrink with it -- its sizes are fixed,
  // 282 rem lengths against a default root -- so this is not optional.
  //
  // Instead the document is pinned back to the 1430x1000 it was written for
  // and scaled up to fill the larger target. The layout is unchanged, every
  // element is drawn with the factor's worth of extra pixels, and neither
  // their pages nor ours have to know it happened.
  function applyScale() {
    var scale = window.innerWidth / 1430;
    if (!(scale > 1.01)) return;
    pinned = [1430, 1000];
    var st = document.createElement('style');
    st.id = 'deepstudy-scale';
    st.textContent =
      'html, body { width: 1430px; height: 1000px; margin: 0; overflow: hidden; }' +
      'body { transform: scale(' + scale + '); transform-origin: 0 0; }' +
      // Tailwind's viewport-sized utilities would still measure the real
      // target rather than the box just pinned, so restate those two.
      '.h-screen { height: 1000px; } .w-screen { width: 1430px; }';
    document.head.appendChild(st);
  }

  function attach() {
    if (!document.body) { setTimeout(attach, 150); return; }
    applyScale();
    placeButton();
    setInterval(placeButton, 1500);
  }
  attach();
})();
