import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { chromium } from 'playwright';

const roots = [path.resolve('../../html_ui'), process.env.FBW_HTML_UI || 'D:/Steam Games/steamapps/common/X-Plane 12/Aircraft/A380 for X-Plane 12/html_ui'];
const out = process.argv[2] || 'snd-preview';
const types = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.ttf': 'font/ttf' };
const page0 = `<!doctype html><html><head><style>body{margin:0;background:#000}</style></head><body>
${fs.readFileSync(path.join(roots[0], 'Pages/VCockpit/Instruments/A380X/SND/snd.html'), 'utf8').replace(/<script type="text\/html" import-script[^>]*><\/script>/g, '')}
<script src="/Pages/VCockpit/Instruments/A380X/SND/snd.js"></script></body></html>`;
const server = http.createServer((req, res) => {
  const p = decodeURIComponent(new URL(req.url, 'http://x').pathname);
  if (p === '/preview.html') {
    res.writeHead(200, { 'content-type': 'text/html' }).end(page0);
    return;
  }
  for (const root of roots) {
    const file = path.join(root, p);
    if (fs.existsSync(file) && fs.statSync(file).isFile()) {
      res.writeHead(200, { 'content-type': types[path.extname(file)] || 'application/octet-stream' }).end(fs.readFileSync(file));
      return;
    }
  }
  res.writeHead(404).end();
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));

const stub = () => {
  const word = (v) => {
    const f = new Float32Array([v]);
    return new Uint32Array(f.buffer)[0] + 3 * 4294967296;
  };
  const vars = {
    'L:A32NX_ELEC_DC_ESS_BUS_IS_POWERED': 1,
    'L:A32NX_ADIRS_IR_3_HEADING': word(5),
    'L:A32NX_ADIRS_IR_3_TRUE_HEADING': word(5.5),
    'L:A32NX_ADIRS_IR_3_TRACK': word(8),
    'L:A32NX_ADIRS_IR_3_TRUE_TRACK': word(8.5),
    'L:A32NX_ADIRS_IR_3_GROUND_SPEED': word(241),
    'L:A32NX_ADIRS_IR_3_LATITUDE': word(50 + 58.2 / 60),
    'L:A32NX_ADIRS_IR_3_LONGITUDE': word(3 + 41.5 / 60),
    'GLASSCOCKPIT AUTOMATIC BRIGHTNESS': 100,
  };
  window.SimVar = { GetSimVarValue: (n) => vars[n] || 0, SetSimVarValue: (n, u, v) => { vars[n] = v; } };
  const state = {
    wpts: [{ lat: 50 + 54 / 60, lon: 3 + 38 / 60 }, { lat: 51 + 14 / 60, lon: 3 + 52 / 60 }, { lat: 51 + 20 / 60, lon: 3 + 21 / 60 }],
    fix: { lat: 51 + 17.5 / 60, lon: 3 + 42 / 60 },
    nav: { from: { lat: 50 + 54 / 60, lon: 3 + 38 / 60 }, to: 1 },
  };
  window.GetStoredData = () => JSON.stringify(state);
  window.SetStoredData = () => {};
  window.BaseInstrument = class extends HTMLElement {
    connectedCallback() {
      this.innerHTML = document.getElementById(this.templateID).textContent;
      const loop = () => {
        if (!this.ready) {
          this.ready = true;
          this.Init();
        }
        this.Update();
        requestAnimationFrame(loop);
      };
      requestAnimationFrame(loop);
    }
    Init() {}
    Update() {}
  };
  window.registerInstrument = (name, cls) => {
    customElements.define(name, cls);
    window.snd = document.body.appendChild(document.createElement(name));
  };
};

const browser = await chromium.launch();
const page = await browser.newPage({ viewport: { width: 512, height: 512 } });
page.on('pageerror', (e) => console.log('pageerror', e.message));
await page.addInitScript(stub);
await page.goto(`http://127.0.0.1:${server.address().port}/preview.html`);
await page.waitForTimeout(800);
await page.screenshot({ path: `${out}-nav.png` });
const send = async (...events) => {
  for (const e of events) {
    await page.evaluate((e) => window.snd.onInteractionEvent([`A380X_SND_${e}`]), e);
    await page.waitForTimeout(60);
  }
  await page.waitForTimeout(200);
};
await send('MENU_PRESSED', 'KNOB_CLOCKWISE');
await page.screenshot({ path: `${out}-menu.png` });
await send('KNOB_PRESSED', 'KNOB_PRESSED', 'KNOB_CLOCKWISE');
await page.screenshot({ path: `${out}-edit.png` });
await send('MENU_PRESSED', 'DIR_TO_PRESSED', 'KNOB_CLOCKWISE');
await page.screenshot({ path: `${out}-dirto.png` });
await browser.close();
server.close();
console.log('wrote', out);
