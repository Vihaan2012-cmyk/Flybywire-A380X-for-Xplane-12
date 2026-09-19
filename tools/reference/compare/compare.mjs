#!/usr/bin/env node
// Compares the native pipeline's snapshots with the Chromium references.
//
//   npm run compare                                  every state and screen found
//   npm run compare -- --state approach --screen PFD_L,EWD
//
// Reference: D:/fbw-build/reference-png/<state>/<SCREEN>.png
// Ours:      D:/fbw-build/display-snapshots/<state>/<SCREEN>.png
// Output:    D:/fbw-build/reference-report/index.html, report.json and, per
//            pair, <state>/<SCREEN>.{diff,overlay,ssim}.png plus copies of both
//            inputs so the report stays valid when either side is re-rendered.
import fs from 'node:fs';
import path from 'node:path';
import { PNG } from 'pngjs';
import { REFERENCE_PNG_DIR, REPORT_DIR, SNAPSHOT_DIR } from '../src/config.mjs';

function parseArgs(argv) {
  const a = { state: null, screen: null, ref: REFERENCE_PNG_DIR, ours: SNAPSHOT_DIR, out: REPORT_DIR, threshold: 32, failBelow: null };
  for (let i = 0; i < argv.length; i++) {
    const k = argv[i];
    const next = () => argv[++i];
    if (k === '--state') a.state = next().split(',');
    else if (k === '--screen') a.screen = next().split(',');
    else if (k === '--ref') a.ref = next();
    else if (k === '--ours') a.ours = next();
    else if (k === '--out') a.out = next();
    else if (k === '--threshold') a.threshold = Number(next());
    else if (k === '--fail-below') a.failBelow = Number(next());
    else if (k === '--help' || k === '-h') {
      console.log('usage: npm run compare -- [--state a,b] [--screen A,B] [--ref dir] [--ours dir] [--out dir] [--threshold 0-255] [--fail-below ssim]');
      process.exit(0);
    } else throw new Error(`unknown argument ${k}`);
  }
  return a;
}

const readPng = (file) => PNG.sync.read(fs.readFileSync(file));
function writePng(file, width, height, rgba) {
  const png = new PNG({ width, height });
  rgba.copy ? rgba.copy(png.data) : png.data.set(rgba);
  fs.writeFileSync(file, PNG.sync.write(png));
}

// Composite over black (the emissive screen), as sRGB bytes.
function toRgb(png) {
  const n = png.width * png.height;
  const rgb = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const a = png.data[i * 4 + 3] / 255;
    rgb[i * 3] = png.data[i * 4] * a;
    rgb[i * 3 + 1] = png.data[i * 4 + 1] * a;
    rgb[i * 3 + 2] = png.data[i * 4 + 2] * a;
  }
  return rgb;
}

// Bilinear resample to the reference size, when our texture size differs.
function resample(rgb, w, h, W, H) {
  const out = new Float32Array(W * H * 3);
  for (let y = 0; y < H; y++) {
    const sy = Math.min(h - 1, Math.max(0, ((y + 0.5) * h) / H - 0.5));
    const y0 = Math.floor(sy);
    const y1 = Math.min(h - 1, y0 + 1);
    const fy = sy - y0;
    for (let x = 0; x < W; x++) {
      const sx = Math.min(w - 1, Math.max(0, ((x + 0.5) * w) / W - 0.5));
      const x0 = Math.floor(sx);
      const x1 = Math.min(w - 1, x0 + 1);
      const fx = sx - x0;
      for (let c = 0; c < 3; c++) {
        const p = (yy, xx) => rgb[(yy * w + xx) * 3 + c];
        out[(y * W + x) * 3 + c] = (p(y0, x0) * (1 - fx) + p(y0, x1) * fx) * (1 - fy) + (p(y1, x0) * (1 - fx) + p(y1, x1) * fx) * fy;
      }
    }
  }
  return out;
}

const luma = (rgb, n) => {
  const l = new Float32Array(n);
  for (let i = 0; i < n; i++) l[i] = 0.2126 * rgb[i * 3] + 0.7152 * rgb[i * 3 + 1] + 0.0722 * rgb[i * 3 + 2];
  return l;
};

// Separable Gaussian blur (11 taps, sigma 1.5: Wang et al. 2004), edges clamped.
const KERNEL = (() => {
  const k = [];
  let s = 0;
  for (let i = -5; i <= 5; i++) {
    const v = Math.exp(-(i * i) / (2 * 1.5 * 1.5));
    k.push(v);
    s += v;
  }
  return k.map((v) => v / s);
})();
function blur(src, w, h) {
  const tmp = new Float32Array(w * h);
  const out = new Float32Array(w * h);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      let s = 0;
      for (let k = -5; k <= 5; k++) s += KERNEL[k + 5] * src[y * w + Math.min(w - 1, Math.max(0, x + k))];
      tmp[y * w + x] = s;
    }
  }
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      let s = 0;
      for (let k = -5; k <= 5; k++) s += KERNEL[k + 5] * tmp[Math.min(h - 1, Math.max(0, y + k)) * w + x];
      out[y * w + x] = s;
    }
  }
  return out;
}

// SSIM on luminance (0..255, K1 0.01, K2 0.03). Returns the mean and the map.
function ssim(a, b, w, h) {
  const n = w * h;
  const aa = new Float32Array(n);
  const bb = new Float32Array(n);
  const ab = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    aa[i] = a[i] * a[i];
    bb[i] = b[i] * b[i];
    ab[i] = a[i] * b[i];
  }
  const ma = blur(a, w, h);
  const mb = blur(b, w, h);
  const saa = blur(aa, w, h);
  const sbb = blur(bb, w, h);
  const sab = blur(ab, w, h);
  const C1 = (0.01 * 255) ** 2;
  const C2 = (0.03 * 255) ** 2;
  const map = new Float32Array(n);
  let sum = 0;
  for (let i = 0; i < n; i++) {
    const va = saa[i] - ma[i] * ma[i];
    const vb = sbb[i] - mb[i] * mb[i];
    const cov = sab[i] - ma[i] * mb[i];
    const s = ((2 * ma[i] * mb[i] + C1) * (2 * cov + C2)) / ((ma[i] * ma[i] + mb[i] * mb[i] + C1) * (va + vb + C2));
    map[i] = s;
    sum += s;
  }
  return { mean: sum / n, map };
}

// Sobel edge magnitude, thresholded.
function edges(l, w, h, threshold = 48) {
  const e = new Uint8Array(w * h);
  for (let y = 1; y < h - 1; y++) {
    for (let x = 1; x < w - 1; x++) {
      const p = (dx, dy) => l[(y + dy) * w + x + dx];
      const gx = -p(-1, -1) - 2 * p(-1, 0) - p(-1, 1) + p(1, -1) + 2 * p(1, 0) + p(1, 1);
      const gy = -p(-1, -1) - 2 * p(0, -1) - p(1, -1) + p(-1, 1) + 2 * p(0, 1) + p(1, 1);
      if (Math.hypot(gx, gy) > threshold * 4) e[y * w + x] = 1;
    }
  }
  return e;
}
function dilate(m, w, h, r) {
  const out = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      if (!m[y * w + x]) continue;
      for (let dy = -r; dy <= r; dy++) {
        const yy = y + dy;
        if (yy < 0 || yy >= h) continue;
        for (let dx = -r; dx <= r; dx++) {
          const xx = x + dx;
          if (xx >= 0 && xx < w) out[yy * w + xx] = 1;
        }
      }
    }
  }
  return out;
}
// Edge F1 with a tolerance of `r` pixels: how much of the line work is where it should be.
function edgeScore(ea, eb, w, h, r = 2) {
  const da = dilate(ea, w, h, r);
  const db = dilate(eb, w, h, r);
  let na = 0;
  let nb = 0;
  let hitA = 0;
  let hitB = 0;
  for (let i = 0; i < w * h; i++) {
    if (ea[i]) {
      na++;
      if (db[i]) hitA++;
    }
    if (eb[i]) {
      nb++;
      if (da[i]) hitB++;
    }
  }
  const recall = na ? hitA / na : 1;
  const precision = nb ? hitB / nb : 1;
  const f1 = recall + precision ? (2 * recall * precision) / (recall + precision) : 0;
  return { recall, precision, f1, refEdgePixels: na, ourEdgePixels: nb };
}

function comparePair(refFile, ourFile, outBase, args) {
  const refPng = readPng(refFile);
  const ourPng = readPng(ourFile);
  const W = refPng.width;
  const H = refPng.height;
  const n = W * H;
  const ref = toRgb(refPng);
  let ours = toRgb(ourPng);
  const sizeMatches = ourPng.width === W && ourPng.height === H;
  if (!sizeMatches) ours = resample(ours, ourPng.width, ourPng.height, W, H);

  let absSum = 0;
  let sqSum = 0;
  let differing = 0;
  let refLit = 0;
  let ourLit = 0;
  const diff = Buffer.alloc(n * 4);
  const overlay = Buffer.alloc(n * 4);
  const lr = luma(ref, n);
  const lo = luma(ours, n);
  for (let i = 0; i < n; i++) {
    let maxd = 0;
    for (let c = 0; c < 3; c++) {
      const d = Math.abs(ref[i * 3 + c] - ours[i * 3 + c]);
      absSum += d;
      sqSum += d * d;
      if (d > maxd) maxd = d;
    }
    if (maxd > args.threshold) differing++;
    if (lr[i] > 24) refLit++;
    if (lo[i] > 24) ourLit++;
    // Diff heat map: black = equal, then red -> yellow -> white.
    const t = Math.min(1, maxd / 128);
    diff[i * 4] = Math.round(255 * Math.min(1, t * 3));
    diff[i * 4 + 1] = Math.round(255 * Math.min(1, Math.max(0, t * 3 - 1)));
    diff[i * 4 + 2] = Math.round(255 * Math.max(0, t * 3 - 2));
    diff[i * 4 + 3] = 255;
    // Overlay: reference in magenta, ours in green; where both draw it is white.
    overlay[i * 4] = Math.round(lr[i]);
    overlay[i * 4 + 1] = Math.round(lo[i]);
    overlay[i * 4 + 2] = Math.round(lr[i]);
    overlay[i * 4 + 3] = 255;
  }
  const mae = absSum / (n * 3);
  const rmse = Math.sqrt(sqSum / (n * 3));
  const psnr = rmse === 0 ? Infinity : 20 * Math.log10(255 / rmse);
  const s = ssim(lr, lo, W, H);
  const ssimImg = Buffer.alloc(n * 4);
  for (let i = 0; i < n; i++) {
    // Dark where structure differs.
    const v = Math.round(255 * Math.max(0, Math.min(1, s.map[i])));
    ssimImg[i * 4] = v;
    ssimImg[i * 4 + 1] = v;
    ssimImg[i * 4 + 2] = v;
    ssimImg[i * 4 + 3] = 255;
  }
  const edge = edgeScore(edges(lr, W, H), edges(lo, W, H), W, H);

  writePng(`${outBase}.diff.png`, W, H, diff);
  writePng(`${outBase}.overlay.png`, W, H, overlay);
  writePng(`${outBase}.ssim.png`, W, H, ssimImg);
  fs.copyFileSync(refFile, `${outBase}.ref.png`);
  fs.copyFileSync(ourFile, `${outBase}.ours.png`);

  return {
    width: W,
    height: H,
    ourSize: [ourPng.width, ourPng.height],
    sizeMatches,
    mae,
    rmse,
    psnr,
    differingPixels: differing / n,
    ssim: s.mean,
    edges: edge,
    litPixels: { ref: refLit / n, ours: ourLit / n },
  };
}

const listDirs = (d) => (fs.existsSync(d) ? fs.readdirSync(d).filter((f) => fs.statSync(path.join(d, f)).isDirectory()) : []);
const listPngs = (d) => (fs.existsSync(d) ? fs.readdirSync(d).filter((f) => /^[A-Za-z0-9_]+\.png$/.test(f)).map((f) => f.slice(0, -4)) : []);
const escHtml = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');

function report(results, args) {
  const fmt = (v, d = 3) => (v === Infinity ? '&infin;' : Number.isFinite(v) ? v.toFixed(d) : '');
  const grade = (r) => (r.status !== 'compared' ? 'missing' : r.metrics.ssim >= 0.95 && r.metrics.edges.f1 >= 0.9 ? 'good' : r.metrics.ssim >= 0.8 ? 'fair' : 'poor');
  const rows = results
    .map((r) => {
      const base = `${r.state}/${r.screen}`;
      const img = (suffix, label) => `<a href="${base}.${suffix}.png" target="_blank"><img loading="lazy" src="${base}.${suffix}.png" alt="${label}"><span>${label}</span></a>`;
      if (r.status !== 'compared') {
        const only = r.status === 'no snapshot' ? img('ref', 'reference') : img('ours', 'ours');
        return `<tr class="missing"><td>${escHtml(r.state)}</td><td>${escHtml(r.screen)}</td><td colspan="7">${escHtml(r.status)}</td><td class="imgs">${only}</td></tr>`;
      }
      const m = r.metrics;
      return `<tr class="${grade(r)}"><td>${escHtml(r.state)}</td><td>${escHtml(r.screen)}${m.sizeMatches ? '' : `<br><small>ours ${m.ourSize.join('x')} resampled to ${m.width}x${m.height}</small>`}</td>
<td data-v="${m.ssim}">${fmt(m.ssim)}</td><td data-v="${m.edges.f1}">${fmt(m.edges.f1)}<br><small>R ${fmt(m.edges.recall, 2)} P ${fmt(m.edges.precision, 2)}</small></td>
<td data-v="${m.mae}">${fmt(m.mae, 2)}</td><td data-v="${m.rmse}">${fmt(m.rmse, 2)}</td><td data-v="${m.psnr === Infinity ? 999 : m.psnr}">${fmt(m.psnr, 1)}</td>
<td data-v="${m.differingPixels}">${(m.differingPixels * 100).toFixed(2)}%</td><td data-v="${m.litPixels.ours - m.litPixels.ref}">${(m.litPixels.ref * 100).toFixed(1)}% / ${(m.litPixels.ours * 100).toFixed(1)}%</td>
<td class="imgs">${img('ref', 'reference')}${img('ours', 'ours')}${img('diff', 'diff')}${img('overlay', 'overlay')}${img('ssim', 'SSIM map')}</td></tr>`;
    })
    .join('\n');
  const compared = results.filter((r) => r.status === 'compared');
  const mean = (f) => (compared.length ? compared.reduce((s, r) => s + f(r.metrics), 0) / compared.length : NaN);
  return `<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Display reference report</title>
<style>
:root { --bg: #fff; --fg: #1d1d1f; --muted: #6b6b70; --line: #dcdce0; --good: #e3f4e6; --fair: #fbf1d9; --poor: #f9e0de; --missing: #efeff2; }
@media (prefers-color-scheme: dark) { :root { --bg: #151517; --fg: #e8e8ea; --muted: #9a9aa0; --line: #333338; --good: #173222; --fair: #372e14; --poor: #3b1c1a; --missing: #242428; } }
body { background: var(--bg); color: var(--fg); font: 14px/1.4 system-ui, sans-serif; margin: 16px; }
h1 { font-size: 20px; margin: 0 0 4px; } p { color: var(--muted); margin: 4px 0 12px; max-width: 70em; }
table { border-collapse: collapse; width: 100%; } th, td { border-bottom: 1px solid var(--line); padding: 6px 8px; text-align: left; vertical-align: top; }
th { cursor: pointer; position: sticky; top: 0; background: var(--bg); white-space: nowrap; } td small { color: var(--muted); }
tr.good td { background: var(--good); } tr.fair td { background: var(--fair); } tr.poor td { background: var(--poor); } tr.missing td { background: var(--missing); }
.imgs { display: flex; gap: 6px; flex-wrap: wrap; } .imgs a { display: flex; flex-direction: column; align-items: center; color: var(--muted); text-decoration: none; font-size: 11px; }
.imgs img { height: 140px; background: #000; border: 1px solid var(--line); image-rendering: pixelated; }
.wrap { overflow-x: auto; }
</style></head><body>
<h1>Display reference report</h1>
<p>Reference: <code>${escHtml(args.ref)}</code> (Chromium). Ours: <code>${escHtml(args.ours)}</code>. Generated ${new Date().toISOString()}.
${compared.length} pairs compared; mean SSIM ${fmt(mean((m) => m.ssim))}, mean edge F1 ${fmt(mean((m) => m.edges.f1))}.</p>
<p>SSIM: luminance structural similarity (11-tap Gaussian, 1 = identical). Edge F1: Sobel edges matched within 2 px (R = share of the reference's edges we draw, P = share of our edges that the reference has). MAE/RMSE: per channel, 0-255, composited over black. Differing: pixels with a channel off by more than ${args.threshold}. Lit: pixels brighter than 24 (reference / ours). Overlay: reference magenta, ours green, both white. Rows: green SSIM &ge; 0.95 and F1 &ge; 0.9, amber SSIM &ge; 0.8, red below. Click a column to sort.</p>
<div class="wrap"><table id="t"><thead><tr><th>state</th><th>screen</th><th>SSIM</th><th>edge F1</th><th>MAE</th><th>RMSE</th><th>PSNR dB</th><th>differing</th><th>lit ref / ours</th><th>images</th></tr></thead>
<tbody>
${rows}
</tbody></table></div>
<script>
document.querySelectorAll('#t th').forEach((th, col) => th.addEventListener('click', () => {
  const body = document.querySelector('#t tbody');
  const dir = th.dataset.dir === 'asc' ? 'desc' : 'asc';
  th.dataset.dir = dir;
  const key = (tr) => { const td = tr.children[col]; if (!td) return ''; return td.dataset.v !== undefined ? Number(td.dataset.v) : td.textContent; };
  [...body.rows].sort((a, b) => { const x = key(a), y = key(b); const c = typeof x === 'number' && typeof y === 'number' ? x - y : String(x).localeCompare(String(y)); return dir === 'asc' ? c : -c; }).forEach((tr) => body.appendChild(tr));
}));
</script>
</body></html>
`;
}

function main() {
  const args = parseArgs(process.argv.slice(2));
  const states = [...new Set([...listDirs(args.ref), ...listDirs(args.ours)])].filter((s) => !args.state || args.state.includes(s)).sort();
  if (!states.length) {
    console.error(`nothing to compare: no state folders in ${args.ref} or ${args.ours}`);
    process.exit(1);
  }
  fs.mkdirSync(args.out, { recursive: true });
  const results = [];
  for (const state of states) {
    const refDir = path.join(args.ref, state);
    const ourDir = path.join(args.ours, state);
    const screens = [...new Set([...listPngs(refDir), ...listPngs(ourDir)])].filter((s) => !args.screen || args.screen.includes(s)).sort();
    fs.mkdirSync(path.join(args.out, state), { recursive: true });
    for (const screen of screens) {
      const refFile = path.join(refDir, `${screen}.png`);
      const ourFile = path.join(ourDir, `${screen}.png`);
      const outBase = path.join(args.out, state, screen);
      const r = { state, screen };
      if (!fs.existsSync(ourFile)) {
        r.status = 'no snapshot';
        fs.copyFileSync(refFile, `${outBase}.ref.png`);
      } else if (!fs.existsSync(refFile)) {
        r.status = 'no reference';
        fs.copyFileSync(ourFile, `${outBase}.ours.png`);
      } else {
        const t0 = Date.now();
        r.status = 'compared';
        r.metrics = comparePair(refFile, ourFile, outBase, args);
        const m = r.metrics;
        console.log(`${state}/${screen}: SSIM ${m.ssim.toFixed(4)}  edge F1 ${m.edges.f1.toFixed(3)}  MAE ${m.mae.toFixed(2)}  differing ${(m.differingPixels * 100).toFixed(2)}%${m.sizeMatches ? '' : '  (resampled)'}  ${Date.now() - t0} ms`);
      }
      if (r.status !== 'compared') console.log(`${state}/${screen}: ${r.status}`);
      results.push(r);
    }
  }
  fs.writeFileSync(path.join(args.out, 'report.json'), JSON.stringify({ format: 'fbw-reference-report/1', generated: new Date().toISOString(), ref: args.ref, ours: args.ours, threshold: args.threshold, results }, null, 1));
  fs.writeFileSync(path.join(args.out, 'index.html'), report(results, args));
  console.log(`\n${path.join(args.out, 'index.html')}`);
  if (args.failBelow != null && results.some((r) => r.status === 'compared' && r.metrics.ssim < args.failBelow)) process.exit(2);
}

main();
