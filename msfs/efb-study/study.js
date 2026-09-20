// Copyright (c) 2026 -- systems Study overlay for the FlyByWire A380X EFB.
// SPDX-License-Identifier: GPL-3.0
//
// Deliberately standalone: this does NOT modify efb.js. It is loaded as its
// own script beside the EFB and draws its own panel, so the aircraft's own
// bundle is whatever shipped with it and stays that way across updates.
// The only change to FlyByWire's files is one import-script line in efb.html.
//
// Everything shown comes from catalogue.json, which is static -- the
// systems model's failures, components, breakers and MEL references. Live
// state (what is armed, what is open) is NOT shown, because the deep model
// is not running in MSFS yet and a plausible-looking number would be a lie.

(function () {
  'use strict';

  var CAT = null;
  var tab = 'failures';
  var query = '';
  var selected = null;

  var C = {
    body: '#171c22',
    panel: '#1e252d',
    accent: '#2c3641',
    line: '#3a4653',
    text: '#e6edf3',
    dim: '#8b99a7',
    hi: '#00c2cc',
    amber: '#e8a33d',
  };

  function el(tag, style, text) {
    var e = document.createElement(tag);
    if (style) e.setAttribute('style', style);
    if (text !== undefined) e.textContent = text;
    return e;
  }

  function chapterName(n) {
    if (!CAT) return 'ATA ' + n;
    for (var i = 0; i < CAT.chapters.length; i++) {
      if (CAT.chapters[i].number === n) return CAT.chapters[i].name;
    }
    return 'ATA ' + n;
  }

  function groupByChapter(rows) {
    var m = {};
    rows.forEach(function (r) {
      (m[r.ataChapterNumber] = m[r.ataChapterNumber] || []).push(r);
    });
    return Object.keys(m)
      .map(Number)
      .sort(function (a, b) { return a - b; })
      .map(function (n) { return [n, m[n]]; });
  }

  function humanise(s) {
    return s.replace(/([A-Z])/g, ' $1').replace(/^./, function (c) { return c.toUpperCase(); });
  }

  // ---------------------------------------------------------------- shell

  var root = el('div',
    'position:fixed;inset:0;z-index:99999;display:none;background:' + C.body +
    ';color:' + C.text + ';font-family:system-ui,sans-serif;font-size:15px;flex-direction:column;');

  var header = el('div',
    'display:flex;align-items:center;gap:16px;padding:12px 18px;border-bottom:1px solid ' + C.line + ';');
  var title = el('div', 'font-weight:700;font-size:20px;', 'Study');
  var counts = el('div', 'color:' + C.dim + ';font-size:13px;flex:1;', 'loading…');

  var search = el('input',
    'background:' + C.panel + ';border:1px solid ' + C.line + ';color:' + C.text +
    ';padding:6px 10px;border-radius:5px;width:260px;outline:none;');
  search.placeholder = 'Search';
  search.addEventListener('input', function () { query = search.value.trim().toUpperCase(); selected = null; render(); });

  var closeBtn = el('div',
    'cursor:pointer;padding:6px 14px;border:1px solid ' + C.line + ';border-radius:5px;', 'Close');
  closeBtn.addEventListener('click', function () { root.style.display = 'none'; });

  header.appendChild(title); header.appendChild(counts); header.appendChild(search); header.appendChild(closeBtn);

  var tabBar = el('div', 'display:flex;gap:8px;padding:10px 18px;border-bottom:1px solid ' + C.line + ';');
  var body = el('div', 'flex:1;overflow:auto;padding:14px 18px;');

  root.appendChild(header); root.appendChild(tabBar); root.appendChild(body);

  var TABS = [
    ['failures', 'Failures'],
    ['components', 'Components'],
    ['alerts', 'ECAM'],
    ['breakers', 'Breakers'],
    ['mel', 'MEL'],
    ['systems', 'Systems'],
  ];

  function renderTabs() {
    tabBar.innerHTML = '';
    TABS.forEach(function (t) {
      var active = tab === t[0];
      var b = el('div',
        'cursor:pointer;padding:6px 16px;border-radius:5px;border:1px solid ' +
        (active ? C.hi : C.line) + ';' + (active ? 'background:' + C.hi + ';color:' + C.body + ';font-weight:600;' : ''),
        t[1]);
      b.addEventListener('click', function () { tab = t[0]; selected = null; render(); });
      tabBar.appendChild(b);
    });
  }

  // ---------------------------------------------------------------- pages

  function card() {
    return el('div',
      'border:1px solid ' + C.line + ';border-radius:6px;padding:10px 12px;margin-bottom:6px;background:' + C.panel + ';');
  }

  function chapterHeading(n, count) {
    var h = el('div',
      'display:flex;justify-content:space-between;background:' + C.accent +
      ';padding:7px 12px;border-radius:5px;margin:14px 0 6px;font-weight:700;');
    h.appendChild(el('span', null, chapterName(n)));
    h.appendChild(el('span', 'color:' + C.dim + ';font-weight:400;', String(count)));
    return h;
  }

  function pageFailures() {
    var rows = CAT.failures.filter(function (f) {
      return query === '' ||
        f.name.toUpperCase().indexOf(query) >= 0 ||
        f.component.toUpperCase().indexOf(query) >= 0 ||
        f.cause.toUpperCase().indexOf(query) >= 0 ||
        String(f.id).indexOf(query) >= 0;
    });
    if (query === '') {
      body.appendChild(el('div', 'color:' + C.dim + ';margin-bottom:8px;',
        'Search to narrow ' + rows.length.toLocaleString() + ' failures. Showing the first 400.'));
      rows = rows.slice(0, 400);
    }
    groupByChapter(rows).forEach(function (g) {
      body.appendChild(chapterHeading(g[0], g[1].length));
      g[1].forEach(function (f) {
        var c = card();
        var top = el('div', 'display:flex;justify-content:space-between;gap:12px;');
        top.appendChild(el('span', 'font-weight:600;', f.name));
        top.appendChild(el('span', 'color:' + C.dim + ';font-family:monospace;font-size:13px;', String(f.id)));
        c.appendChild(top);
        c.appendChild(el('div', 'color:' + C.dim + ';font-size:13px;margin-top:3px;', f.cause));
        var meta = el('div', 'margin-top:4px;font-size:13px;');
        meta.appendChild(el('span', 'color:' + C.hi + ';', f.component));
        if (f.magnitudeSemantics) {
          meta.appendChild(el('span', 'color:' + C.dim + ';margin-left:14px;', 'magnitude: ' + f.magnitudeSemantics));
        }
        c.appendChild(meta);
        body.appendChild(c);
      });
    });
  }

  function pageComponents() {
    var rows = CAT.components.filter(function (c) {
      return query === '' || c.name.toUpperCase().indexOf(query) >= 0 || c.id.toUpperCase().indexOf(query) >= 0;
    });
    var byId = {};
    CAT.failures.forEach(function (f) { byId[f.id] = f; });

    if (query === '') {
      body.appendChild(el('div', 'color:' + C.dim + ';margin-bottom:8px;',
        'Search to narrow ' + rows.length.toLocaleString() + ' components. Showing the first 250.'));
      rows = rows.slice(0, 250);
    }
    groupByChapter(rows).forEach(function (g) {
      body.appendChild(chapterHeading(g[0], g[1].length));
      g[1].forEach(function (comp) {
        var c = card();
        var top = el('div', 'display:flex;justify-content:space-between;gap:12px;cursor:pointer;');
        top.appendChild(el('span', 'font-weight:600;',
          comp.name + (comp.instance !== null ? ' ' + comp.instance : '')));
        top.appendChild(el('span', 'color:' + C.dim + ';font-size:13px;',
          comp.failures.length + (comp.failures.length === 1 ? ' failure' : ' failures')));
        c.appendChild(top);

        var open = selected === comp.id;
        top.addEventListener('click', function () { selected = open ? null : comp.id; render(); });

        if (open) {
          c.appendChild(el('div', 'color:' + C.dim + ';font-family:monospace;font-size:12px;margin-top:3px;', comp.id));
          if (comp.parameters.length) {
            c.appendChild(el('div', 'margin-top:8px;font-weight:600;', 'What the model solves for it'));
            comp.parameters.forEach(function (p) {
              var r = el('div', 'margin-top:4px;padding-left:10px;border-left:2px solid ' + C.line + ';');
              var l = el('div', 'display:flex;justify-content:space-between;gap:12px;font-size:13px;');
              l.appendChild(el('span', 'font-family:monospace;', p.name));
              l.appendChild(el('span', 'color:' + C.dim + ';', 'healthy: ' + p.healthyValue));
              r.appendChild(l);
              r.appendChild(el('div', 'color:' + C.dim + ';font-size:13px;', p.meaning));
              c.appendChild(r);
            });
          }
          c.appendChild(el('div', 'margin-top:8px;font-weight:600;', 'What can go wrong with it'));
          comp.failures.forEach(function (id) {
            var f = byId[id];
            var r = el('div', 'margin-top:4px;padding-left:10px;border-left:2px solid ' + C.line + ';font-size:13px;');
            r.appendChild(el('div', null, f ? f.name : 'Failure ' + id));
            if (f) r.appendChild(el('div', 'color:' + C.dim + ';', f.cause));
            c.appendChild(r);
          });
        }
        body.appendChild(c);
      });
    });
  }

  function pageBreakers() {
    var rows = CAT.breakers.filter(function (b) {
      return query === '' ||
        b.label.toUpperCase().indexOf(query) >= 0 ||
        b.consumer.toUpperCase().indexOf(query) >= 0 ||
        b.bus.toUpperCase().indexOf(query) >= 0 ||
        humanise(b.panel).toUpperCase().indexOf(query) >= 0;
    });

    var panels = {};
    rows.forEach(function (b) { (panels[b.panel] = panels[b.panel] || []).push(b); });

    body.appendChild(el('div', 'color:' + C.dim + ';margin-bottom:10px;',
      rows.length + ' breakers, drawn on the panels they sit on. Dimmed = protects nothing the model solves. ' +
      'Open/closed state is not shown: the systems model is not running here yet.'));

    Object.keys(panels).sort().forEach(function (p) {
      var list = panels[p];
      body.appendChild(chapterHeading2(humanise(p), list.length));
      var grid = el('div', 'display:flex;flex-wrap:wrap;gap:4px;margin-bottom:12px;');
      list.sort(function (a, b) { return a.row - b.row || a.column - b.column; });
      list.forEach(function (b) {
        var t = el('div',
          'width:120px;border:1px solid ' + C.line + ';border-radius:4px;padding:5px;text-align:center;' +
          'font-size:11px;background:' + C.panel + ';cursor:default;' + (b.protectsModelledLoad ? '' : 'opacity:0.45;'));
        t.title = b.consumer + ' — ' + b.ratingAmperes + ' A on ' + b.bus + ' (row ' + b.row + ', col ' + b.column + ')\n' + b.basis;
        t.appendChild(el('div', 'font-family:monospace;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;', b.label));
        t.appendChild(el('div', 'color:' + C.dim + ';', b.ratingAmperes + ' A · ' + b.bus));
        grid.appendChild(t);
      });
      body.appendChild(grid);
    });
  }

  function chapterHeading2(text, count) {
    var h = el('div',
      'display:flex;justify-content:space-between;background:' + C.accent +
      ';padding:7px 12px;border-radius:5px;margin:14px 0 6px;font-weight:700;');
    h.appendChild(el('span', null, text));
    h.appendChild(el('span', 'color:' + C.dim + ';font-weight:400;', String(count)));
    return h;
  }

  function pageMel() {
    var byId = {};
    CAT.failures.forEach(function (f) { byId[f.id] = f; });
    var rows = CAT.melEntries.filter(function (e) {
      if (query === '') return true;
      if (e.melReference.toUpperCase().indexOf(query) >= 0) return true;
      return e.failures.some(function (id) {
        return byId[id] && byId[id].name.toUpperCase().indexOf(query) >= 0;
      });
    });
    body.appendChild(el('div', 'color:' + C.dim + ';margin-bottom:10px;',
      'The cross-reference only — which modelled failures each MEL item covers. The operator’s own MEL text is not shipped.'));
    groupByChapter(rows).forEach(function (g) {
      body.appendChild(chapterHeading(g[0], g[1].length));
      g[1].forEach(function (e) {
        var c = card();
        var top = el('div', 'display:flex;justify-content:space-between;gap:12px;');
        top.appendChild(el('span', 'font-family:monospace;font-weight:700;', e.melReference));
        top.appendChild(el('span', 'color:' + C.dim + ';font-size:13px;', e.failures.length + ' failures'));
        c.appendChild(top);
        e.failures.forEach(function (id) {
          var f = byId[id];
          var r = el('div', 'display:flex;justify-content:space-between;gap:12px;font-size:13px;margin-top:2px;');
          r.appendChild(el('span', null, f ? f.name : 'Failure ' + id));
          r.appendChild(el('span', 'color:' + C.dim + ';font-family:monospace;', String(id)));
          c.appendChild(r);
        });
        body.appendChild(c);
      });
    });
  }


  function levelColour(level) {
    var l = (level || '').toUpperCase();
    if (l.indexOf('WARN') >= 0 || l.indexOf('RED') >= 0) return '#e05c5c';
    if (l.indexOf('CAUT') >= 0 || l.indexOf('AMBER') >= 0) return C.amber;
    return C.dim;
  }

  function pageAlerts() {
    var byId = {};
    CAT.failures.forEach(function (f) { byId[f.id] = f; });

    var rows = CAT.alerts.filter(function (a) {
      return query === '' ||
        a.title.toUpperCase().indexOf(query) >= 0 ||
        a.key.toUpperCase().indexOf(query) >= 0;
    });

    body.appendChild(el('div', 'color:' + C.dim + ';margin-bottom:10px;',
      rows.length + ' ECAM alerts the systems model can raise, with what each one needs to fire. ' +
      'Every one is reachable: an alert nothing can trigger is a bug this project tests against.'));

    groupByChapter(rows).forEach(function (g) {
      body.appendChild(chapterHeading(g[0], g[1].length));
      g[1].forEach(function (a) {
        var c = card();
        var top = el('div', 'display:flex;justify-content:space-between;gap:12px;align-items:baseline;cursor:pointer;');
        top.appendChild(el('span', 'font-weight:600;color:' + levelColour(a.level) + ';', a.title));
        var badges = el('span', 'color:' + C.dim + ';font-size:12px;');
        badges.textContent = [a.level, a.aural, a.masterLight].filter(Boolean).join(' . ');
        top.appendChild(badges);
        c.appendChild(top);

        var open = selected === a.key;
        top.addEventListener('click', function () { selected = open ? null : a.key; render(); });

        if (open) {
          var bits = [];
          if (a.confirmSeconds !== null && a.confirmSeconds !== undefined) bits.push('confirm ' + a.confirmSeconds + ' s');
          if (a.inhibitedInPhases && a.inhibitedInPhases.length) bits.push('inhibited in phases ' + a.inhibitedInPhases.join(', '));
          if (a.procedureLineCount) bits.push(a.procedureLineCount + ' procedure lines');
          if (bits.length) c.appendChild(el('div', 'font-size:13px;color:' + C.dim + ';margin-top:4px;', bits.join(' . ')));

          if (a.inoperativeSystemsListEntries && a.inoperativeSystemsListEntries.length) {
            c.appendChild(el('div', 'margin-top:6px;font-weight:600;font-size:13px;', 'INOP SYS'));
            a.inoperativeSystemsListEntries.forEach(function (line) {
              c.appendChild(el('div', 'font-size:13px;color:' + C.dim + ';padding-left:10px;', line));
            });
          }
          if (a.statusPageLines && a.statusPageLines.length) {
            c.appendChild(el('div', 'margin-top:6px;font-weight:600;font-size:13px;', 'STATUS'));
            a.statusPageLines.forEach(function (line) {
              c.appendChild(el('div', 'font-size:13px;color:' + C.dim + ';padding-left:10px;', line));
            });
          }
          if (a.failures && a.failures.length) {
            c.appendChild(el('div', 'margin-top:6px;font-weight:600;font-size:13px;', 'Raised by'));
            a.failures.forEach(function (id) {
              var f = byId[id];
              var r = el('div', 'display:flex;justify-content:space-between;gap:12px;font-size:13px;padding-left:10px;');
              r.appendChild(el('span', null, f ? f.name : 'Failure ' + id));
              r.appendChild(el('span', 'color:' + C.dim + ';font-family:monospace;', String(id)));
              c.appendChild(r);
            });
          }
          c.appendChild(el('div', 'margin-top:6px;font-family:monospace;font-size:11px;color:' + C.dim + ';', a.key));
        }
        body.appendChild(c);
      });
    });
  }

  function pageSystems() {
    var per = {};
    function bump(n, k) {
      per[n] = per[n] || { components: 0, failures: 0, alerts: 0, breakers: 0 };
      per[n][k]++;
    }
    CAT.components.forEach(function (c) { bump(c.ataChapterNumber, 'components'); });
    CAT.failures.forEach(function (f) { bump(f.ataChapterNumber, 'failures'); });
    CAT.alerts.forEach(function (a) { bump(a.ataChapterNumber, 'alerts'); });
    CAT.breakers.forEach(function (b) { bump(b.ataChapterNumber, 'breakers'); });

    var chapters = Object.keys(per).map(Number).sort(function (a, b) { return a - b; });
    if (query !== '') {
      chapters = chapters.filter(function (n) { return chapterName(n).toUpperCase().indexOf(query) >= 0; });
    }

    var maxF = 0;
    chapters.forEach(function (n) { if (per[n].failures > maxF) maxF = per[n].failures; });

    body.appendChild(el('div', 'color:' + C.dim + ';margin-bottom:10px;',
      'What is actually modelled, per ATA chapter. The bar is failures relative to the deepest chapter, ' +
      'so it shows where the model has real depth and where it is thin.'));

    var head = el('div', 'display:flex;gap:10px;padding:6px 12px;color:' + C.dim + ';font-size:12px;font-weight:600;');
    head.appendChild(el('span', 'flex:1;', 'CHAPTER'));
    ['PARTS', 'FAILURES', 'ECAM', 'BREAKERS'].forEach(function (h) {
      head.appendChild(el('span', 'width:80px;text-align:right;', h));
    });
    body.appendChild(head);

    chapters.forEach(function (n) {
      var pc = per[n];
      var row = el('div', 'display:flex;gap:10px;align-items:center;padding:7px 12px;border-bottom:1px solid ' + C.line + ';');
      var nameCell = el('div', 'flex:1;');
      nameCell.appendChild(el('div', null, chapterName(n)));
      var barWrap = el('div', 'height:3px;background:' + C.accent + ';border-radius:2px;margin-top:4px;width:100%;');
      var pct = maxF ? Math.max(2, Math.round((pc.failures / maxF) * 100)) : 0;
      barWrap.appendChild(el('div', 'height:3px;width:' + pct + '%;background:' + C.hi + ';border-radius:2px;'));
      nameCell.appendChild(barWrap);
      row.appendChild(nameCell);
      [pc.components, pc.failures, pc.alerts, pc.breakers].forEach(function (v) {
        row.appendChild(el('span',
          'width:80px;text-align:right;font-variant-numeric:tabular-nums;' + (v === 0 ? 'color:' + C.dim + ';' : ''),
          String(v)));
      });
      body.appendChild(row);
    });

    var tot = el('div', 'display:flex;gap:10px;padding:10px 12px;font-weight:700;');
    tot.appendChild(el('span', 'flex:1;', 'Total'));
    [CAT.components.length, CAT.failures.length, CAT.alerts.length, CAT.breakers.length].forEach(function (v) {
      tot.appendChild(el('span', 'width:80px;text-align:right;font-variant-numeric:tabular-nums;', v.toLocaleString()));
    });
    body.appendChild(tot);
  }

  function render() {
    renderTabs();
    body.innerHTML = '';
    if (!CAT) { body.appendChild(el('div', 'color:' + C.dim + ';', 'Loading the systems catalogue…')); return; }
    if (tab === 'failures') pageFailures();
    else if (tab === 'components') pageComponents();
    else if (tab === 'alerts') pageAlerts();
    else if (tab === 'breakers') pageBreakers();
    else if (tab === 'systems') pageSystems();
    else pageMel();
  }

  // ------------------------------------------------------------- launcher

  var launcher = el('div',
    'position:fixed;left:10px;bottom:10px;z-index:99998;background:' + C.hi + ';color:' + C.body +
    ';padding:8px 16px;border-radius:6px;font-family:system-ui,sans-serif;font-weight:700;' +
    'cursor:pointer;font-size:14px;box-shadow:0 2px 10px rgba(0,0,0,.5);', 'STUDY');
  launcher.addEventListener('click', function () {
    root.style.display = 'flex';
    if (!CAT) load();
    render();
  });

  function load() {
    fetch('/Pages/VCockpit/Instruments/A380X/EFB/catalogue.json')
      .then(function (r) {
        if (!r.ok) throw new Error('HTTP ' + r.status);
        return r.json();
      })
      .then(function (d) {
        CAT = d;
        counts.textContent =
          d.failures.length.toLocaleString() + ' failures · ' +
          d.components.length.toLocaleString() + ' components · ' +
          d.breakers.length.toLocaleString() + ' breakers · ' +
          d.melEntries.length + ' MEL refs';
        render();
      })
      .catch(function (e) {
        body.innerHTML = '';
        body.appendChild(el('div', 'color:' + C.amber + ';', 'Could not load catalogue.json: ' + e.message));
      });
  }

  function attach() {
    if (!document.body) { setTimeout(attach, 200); return; }
    document.body.appendChild(root);
    document.body.appendChild(launcher);
  }
  attach();
})();
