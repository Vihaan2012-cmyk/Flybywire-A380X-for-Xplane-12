// Modelled on FlyByWire's ND canvas map (fbw-common ND/shared/map/
// CanvasMap.tsx and its layers): a canvas sized by attributes and CSS,
// redrawn after a whole-canvas clearRect, with translate/rotate, dashed
// strokes, an arc, a symbol from an SVG path string through Path2D, and a
// label in '21px Ecam'.
(() => {
  const { h, screen, at, report } = __fixture;
  const doc = screen('ND', 768, 1024, '#map { position: absolute; top: 0; left: 0; width: 768px; height: 1024px; }');
  doc.body.style.margin = '0';
  const canvas = h('canvas', { id: 'map', width: '768', height: '1024' });
  doc.body.appendChild(canvas);
  const ctx = canvas.getContext('2d');
  const waypoint = new Path2D('M -7 0 L 0 -7 L 7 0 L 0 7 Z');
  const draw = (heading) => {
    ctx.clearRect(0, 0, 768, 1024);
    ctx.resetTransform();
    ctx.translate(384, 620);
    ctx.rotate((-heading * Math.PI) / 180);
    ctx.strokeStyle = '#ff94ff';
    ctx.lineWidth = 1.75;
    ctx.setLineDash([12, 6]);
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(0, -250);
    ctx.arc(-50, -250, 50, 0, -Math.PI / 2, true);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.translate(0, -250);
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#fff';
    ctx.stroke(waypoint);
    ctx.resetTransform();
    ctx.font = '21px Ecam';
    ctx.fillStyle = '#fff';
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    ctx.fillText('TOPOX', 400, 370);
  };
  const out = [];
  draw(0);
  at(0);
  out.push(report(doc, 'heading 0'));
  draw(90);
  at(16);
  out.push(report(doc, 'heading 90'));
  return out.join('\n');
})();
