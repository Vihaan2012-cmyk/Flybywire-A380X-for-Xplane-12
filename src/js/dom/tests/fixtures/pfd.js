// Modelled on FlyByWire's A380X PFD: the root <svg> (PFD/PFD.tsx:225), the
// fixed attitude indicator's upper part (PFD/AttitudeIndicatorFixed.tsx,
// AttitudeIndicatorFixedUpper), an FMA cell written with innerHTML
// (PFD/FMA.tsx:570) with its blinking box (FMA.tsx:596), and a speed
// readout bound to a subject. CSS copied from the package's PFD/pfd.css.
(() => {
  const { h, Subject, screen, at, report } = __fixture;
  const doc = screen('PFD', 768, 1024, `
    @font-face { font-family: "Ecam"; src: url(/Fonts/fbw-a380x/FBW-Display-EIS-A380.ttf) format("truetype"); font-weight: normal; font-style: normal; }
    @keyframes blinking { 0% { opacity: 0; } 50% { opacity: 0; } 51% { opacity: 1; } 100% { opacity: 1; } }
    .BlinkInfinite { animation-name: blinking; animation-duration: 1s; animation-iteration-count: infinite; }
    .pfd-svg { position: absolute; width: 768px; height: 1024px; background: #000000; font-family: "Ecam", monospace !important; }
    .SmallStroke { stroke-width: 0.1mm; stroke-linecap: round; }
    .NormalStroke { stroke-width: 0.16mm; stroke-linecap: round; }
    .CornerRound { stroke-linejoin: round; }
    .FontMedium { font-size: 6px; }
    .MiddleAlign { text-align: center; text-anchor: middle; }
    .White { fill: none; stroke: #ffffff; }
    .Green { stroke: #00ff00; fill: none; }
    text.Green { fill: #00ff00; stroke: none; }
    .Amber { stroke: #e68000; fill: none; }
    .Yellow { stroke: #ffff00; fill: none; }
  `);
  doc.body.style.margin = '0';
  const visibility = new Subject('hidden');
  const normalLaw = new Subject('block');
  const speed = new Subject('250');
  const fmaRef = {};
  const svg = h('svg', { class: 'pfd-svg', version: '1.1', viewBox: '0 0 158.75 211.6', xmlns: 'http://www.w3.org/2000/svg' },
    h('g', { id: 'AttitudeUpperInfoGroup', visibility },
      h('g', { id: 'RollProtGroup', class: 'SmallStroke Green', style: { display: normalLaw } },
        h('path', { id: 'RollProtRight', d: 'm105.64 62.887 1.5716-0.8008m-1.5716-0.78293 1.5716-0.8008' })),
      h('g', { class: 'SmallStroke White' },
        h('path', { d: 'm90.858 44.839a42.133 42.158 0 0 0-43.904 0' })),
      h('path', { class: 'NormalStroke Yellow CornerRound', d: 'm68.906 38.650-2.5184-3.7000h5.0367l-2.5184 3.7000' })),
    h('g', { ref: fmaRef, transform: 'translate(3.25 0)' }),
    h('text', { class: 'FontMedium MiddleAlign Green', x: '20', y: '100' }, speed));
  doc.body.appendChild(svg);
  const out = [];
  at(0);
  out.push(report(doc, 'first frame: attitude hidden'));
  visibility.set('visible');
  fmaRef.instance.innerHTML = '<text  class="FontMedium MiddleAlign Green" x="16.782249" y="7.1280665">SPEED</text><path class="NormalStroke Amber BlinkInfinite" d="m0.70556 1.8143h30.927v6.0476h-30.927z" />';
  at(100);
  out.push(report(doc, 'attitude shown, FMA written, blink box in its hidden half'));
  at(700);
  out.push(report(doc, 'blink box in its shown half'));
  at(716);
  out.push(report(doc, 'nothing changed: no new submit'));
  speed.set('251');
  normalLaw.set('none');
  at(732);
  out.push(report(doc, 'speed 251, roll protection hidden'));
  return out.join('\n');
})();
