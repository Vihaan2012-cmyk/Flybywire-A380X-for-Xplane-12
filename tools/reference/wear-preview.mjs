import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { chromium } from 'playwright';

const root = path.resolve('../../app/ui');
const out = process.argv[2] || 'wear-preview.png';
const types = { '.html': 'text/html', '.json': 'application/json', '.js': 'text/javascript', '.css': 'text/css', '.svg': 'image/svg+xml', '.png': 'image/png' };
const server = http.createServer((req, res) => {
  const file = path.join(root, decodeURIComponent(new URL(req.url, 'http://x').pathname));
  if (!file.startsWith(root) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) {
    res.writeHead(404).end();
    return;
  }
  res.writeHead(200, { 'content-type': types[path.extname(file)] || 'application/octet-stream' }).end(fs.readFileSync(file));
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const browser = await chromium.launch();
const page = await browser.newPage({ viewport: { width: 1280, height: 1400 } });
page.on('pageerror', (e) => console.log('pageerror', e.message));
await page.goto(`http://127.0.0.1:${server.address().port}/index.html?fixture=1`);
await page.click('.tab-btn[data-tab="wear"]');
await page.waitForTimeout(1500);
const scroll = Number(process.argv[3] || 0);
if (scroll) {
  await page.evaluate((y) => {
    for (const e of document.querySelectorAll('*')) if (e.scrollHeight > e.clientHeight + 10 && getComputedStyle(e).overflowY !== 'visible') e.scrollTop = y;
  }, scroll);
  await page.waitForTimeout(300);
}
await page.screenshot({ path: out, fullPage: true });
await browser.close();
server.close();
console.log('wrote', out);
