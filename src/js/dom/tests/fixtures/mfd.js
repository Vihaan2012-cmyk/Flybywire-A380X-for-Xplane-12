// Modelled on FlyByWire's A380X MFD widgets: a Button (MsfsAvionicsCommon/
// UiWidgets/Button.tsx: a flex span with a label span) in a header row
// laid out like the F-PLN header (.mfd-fms-fpln-header), a label/value
// pair (.mfd-label-value-container) and a PERF-style grid. CSS copied from
// the package's MFD/mfd.css, except: the button border is solid here
// (mfd.css has outset, drawn solid with a warning), and the grid and
// :hover rules, which use the grid properties and :hover the survey lists
// for the MFD.
(() => {
  const { h, Subject, screen, at, report } = __fixture;
  const doc = screen('MFD', 768, 1024, `
    .mfd-label { font-size: 20px; color: #ffffff; font-family: "FBW-Display-EIS-A380-SlashedZero", monospace; }
    .mfd-value { color: #00ff00; font-size: 25px; font-family: "FBW-Display-EIS-A380-SlashedZero", monospace; }
    .mfd-button { display: flex; font-size: 22px; background-color: #272525; color: #ffffff; font-family: "FBW-Display-EIS-A380-SlashedZero", monospace; text-align: center; align-items: center; justify-content: center; border: 2px solid #c4c6cf; padding: 9px 12px 5px 12px; }
    .mfd-button:hover { border-color: cyan; }
    .mfd-label-value-container { padding: 7px; display: flex; flex-direction: row; align-items: center; }
    .mfd-fms-fpln-header { display: flex; flex-direction: row; justify-content: flex-start; margin-top: 5px; border-bottom: 1px solid #c4c6cf; }
    .mfd-fms-fpln-header-from { display: flex; width: 25%; justify-content: flex-start; align-items: center; padding-left: 3px; }
    .perf-grid { display: grid; grid-template-columns: 200px repeat(2, 1fr); row-gap: 4px; width: 600px; }
    .perf-grid > .wide { grid-column: span 3; }
  `);
  doc.body.style.margin = '0';
  const clicks = [];
  const value = new Subject('FL350');
  const buttonRef = {};
  const page = h('div', { style: { width: '768px', height: '1024px', 'background-color': '#040405' } },
    h('div', { class: 'mfd-fms-fpln-header' },
      h('div', { class: 'mfd-fms-fpln-header-from' }, h('span', { class: 'mfd-label' }, 'FROM')),
      h('span', { class: 'mfd-button', ref: buttonRef }, h('span', {}, 'INIT'))),
    h('div', { class: 'mfd-label-value-container' },
      h('span', { class: 'mfd-label' }, 'CRZ'),
      h('span', { class: 'mfd-value', style: { 'margin-left': '15px' } }, value)),
    h('div', { class: 'perf-grid' },
      h('div', { class: 'mfd-label wide' }, 'THR RED/ACC'),
      h('div', { class: 'mfd-label' }, 'T.O'),
      h('div', { class: 'mfd-value' }, '1500'),
      h('div', { class: 'mfd-value' }, '3000')));
  doc.body.appendChild(page);
  buttonRef.instance.addEventListener('click', () => clicks.push('INIT'));
  const out = [];
  at(0);
  out.push(report(doc, 'MFD page'));
  const r = buttonRef.instance.getBoundingClientRect();
  __screenEvent('MFD', 'move', r.left + 5, r.top + 5, 0, 0);
  __screenEvent('MFD', 'down', r.left + 5, r.top + 5, 0, 0);
  __screenEvent('MFD', 'up', r.left + 6, r.top + 5, 0, 0);
  value.set('FL370');
  at(16);
  out.push(report(doc, `hovered, clicked (${clicks.join(',')}), CRZ FL370; button at ${r.left},${r.top} ${r.width}x${r.height}`));
  return out.join('\n');
})();
