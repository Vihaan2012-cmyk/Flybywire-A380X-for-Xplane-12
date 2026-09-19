// Modelled on FlyByWire's A380X EWD EGT gauge (EWD/elements/EGT.tsx with
// MsfsAvionicsCommon/gauges.tsx: GaugeComponent's arc path, the red-line
// arc nested in it, the indicator line, and the value text with `inherit`
// visibility), and the vertical display's clipped tape
// (ND/VerticalDisplay/VerticalDisplay.tsx:620). CSS copied from the
// package's EWD/ewd.css.
(() => {
  const { h, Subject, screen, at, report } = __fixture;
  const doc = screen('EWD', 768, 1024, `
    .Amber { stroke: #e68000; }
    text.Amber, tspan.Amber, .AmberFill { fill: #e68000; color: #e68000; }
    text.Green, tspan.Green, .GreenFill { fill: #00ff00; }
    .Large, .F23 { font-size: 23px !important; }
    .F26 { font-size: 26px !important; }
    .End { text-anchor: end !important; }
    .ThickRedLine { stroke: #ff0000 !important; stroke-width: 8 !important; fill: #ff0000; }
    .GaugeComponent .Gauge { stroke: #ffffff; stroke-width: 2; fill: none; }
    .GaugeComponent .GaugeIndicator { stroke: #00ff00; stroke-width: 3; fill: none; stroke-linecap: round; }
    .WhiteLine { stroke: #ffffff !important; stroke-width: 2; fill: none; }
  `);
  doc.body.style.margin = '0';
  // gauges.tsx polarToCartesian and describeArc.
  const polar = (cx, cy, r, deg) => {
    const a = ((deg - 90) * Math.PI) / 180;
    return { x: cx + r * Math.cos(a), y: cy + r * Math.sin(a) };
  };
  const arc = (x, y, r, startAngle, endAngle, largeArc, sweep) => {
    const s = polar(x, y, r, endAngle);
    const e = polar(x, y, r, startAngle);
    return ['M', s.x, s.y, 'A', r, r, 0, largeArc, sweep, e.x, e.y].join(' ');
  };
  const egt = new Subject('620');
  const indicator = new Subject(arc(100, 100, 58, 250, 270, 0, 0));
  const svg = h('svg', { id: 'ewd-main', viewBox: '0 0 768 1024', width: '768', height: '1024' },
    h('g', { id: 'EGT-indicator-1' },
      h('g', { visibility: 'hidden' },
        h('path', { class: 'WhiteLine', d: arc(100, 100, 60, 250, 90, 1, 0) }),
        h('text', { class: 'F26 End Amber', x: 117, y: 111.7 }, 'XX')),
      h('g', { visibility: 'inherit', class: 'GaugeComponent' },
        h('text', { class: 'Large End Green', x: 133, y: 111.7 }, egt),
        h('g', {},
          h('path', { class: 'Gauge', d: arc(100, 100, 60, 250, 90, 1, 0) }),
          h('path', { class: 'Gauge ThickRedLine', d: arc(100, 100, 58, 70, 90, 0, 0) }),
          h('path', { class: 'GaugeIndicator', d: indicator })))),
    h('defs', {}, h('clipPath', { id: 'AltitudeTapeMask' }, h('path', { d: 'm 0,1000 v-200 h130 v200 z' }))),
    h('g', { 'clip-path': 'url(#AltitudeTapeMask)' },
      h('line', { x1: '105', x2: '105', y1: '800', y2: '1000', stroke: '#fff', 'stroke-width': '2' }),
      h('polygon', { points: '100,900 110,895 110,905', fill: 'cyan', transform: 'rotate(45 105 900)' })));
  doc.body.appendChild(svg);
  const out = [];
  at(0);
  out.push(report(doc, 'EGT 620'));
  egt.set('645');
  indicator.set(arc(100, 100, 58, 250, 300, 0, 0));
  at(16);
  out.push(report(doc, 'EGT 645, indicator moved'));
  return out.join('\n');
})();
