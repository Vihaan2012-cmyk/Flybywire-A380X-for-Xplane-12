// A static server that lays FBW's files out the way MSFS's virtual file
// system does, rooted at html_ui:
//   /Pages/VCockpit/Instruments/A380X/...  FBW's built bundles (mach output)
//   /Fonts, /Images, /JS/fbw-a380x         FBW's tracked base package
//   /JS/dataStorage.js                     our reimplementation (host/)
//   /reference/host/...                    the VCockpit host page and MSFS shims
// Nothing from Asobo's install is served.
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { BASE_HTML_UI, BUNDLE_HTML_UI_CANDIDATES, TOOL_ROOT } from './config.mjs';

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json',
  '.map': 'application/json',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.gif': 'image/gif',
  '.ttf': 'font/ttf',
  '.otf': 'font/otf',
  '.woff': 'font/woff',
  '.woff2': 'font/woff2',
};

export function bundleHtmlUi() {
  const found = BUNDLE_HTML_UI_CANDIDATES.find((p) => fs.existsSync(path.join(p, 'Pages/VCockpit/Instruments/A380X')));
  if (!found) {
    throw new Error(`No built A380X instruments in: ${BUNDLE_HTML_UI_CANDIDATES.join(', ')}. See docs/reference-harness.md, "Bundles".`);
  }
  return found;
}

// MSFS's VFS is case-insensitive (panel.cfg names A380X/FCU/FCU.html, the file is fcu.html).
function resolveCaseInsensitive(root, rel) {
  let cur = root;
  for (const part of rel.split('/').filter(Boolean)) {
    if (part === '..') return null;
    const direct = path.join(cur, part);
    if (fs.existsSync(direct)) {
      cur = direct;
      continue;
    }
    let entries;
    try {
      entries = fs.readdirSync(cur);
    } catch {
      return null;
    }
    const hit = entries.find((e) => e.toLowerCase() === part.toLowerCase());
    if (!hit) return null;
    cur = path.join(cur, hit);
  }
  return cur;
}

export function startServer({ port = 0, log = () => {} } = {}) {
  const bundles = bundleHtmlUi();
  const roots = [
    { prefix: '/reference/', dir: TOOL_ROOT },
    { prefix: '/JS/dataStorage.js', dir: path.join(TOOL_ROOT, 'host/dataStorage.js'), file: true },
    { prefix: '/Pages/', dir: path.join(bundles, 'Pages') },
    { prefix: '/Pages/', dir: path.join(BASE_HTML_UI, 'Pages') },
    { prefix: '/Fonts/', dir: path.join(BASE_HTML_UI, 'Fonts') },
    { prefix: '/Images/', dir: path.join(BASE_HTML_UI, 'Images') },
    { prefix: '/JS/', dir: path.join(BASE_HTML_UI, 'JS') },
    { prefix: '/JS/', dir: path.join(bundles, 'JS') },
  ];
  const misses = new Set();

  const server = http.createServer((req, res) => {
    const url = new URL(req.url, 'http://x');
    const pathname = decodeURIComponent(url.pathname);
    for (const r of roots) {
      if (r.file ? pathname.toLowerCase() !== r.prefix.toLowerCase() : !pathname.startsWith(r.prefix)) continue;
      const file = r.file ? r.dir : resolveCaseInsensitive(r.dir, pathname.slice(r.prefix.length));
      if (!file || !fs.existsSync(file) || !fs.statSync(file).isFile()) continue;
      res.writeHead(200, {
        'content-type': MIME[path.extname(file).toLowerCase()] ?? 'application/octet-stream',
        'cache-control': 'no-store',
        'access-control-allow-origin': '*',
      });
      fs.createReadStream(file).pipe(res);
      return;
    }
    if (!misses.has(pathname)) {
      misses.add(pathname);
      log(`404 ${pathname}`);
    }
    res.writeHead(404, { 'content-type': 'text/plain' });
    res.end('not found');
  });

  return new Promise((resolve) => {
    server.listen(port, '127.0.0.1', () => {
      const { port: actual } = server.address();
      resolve({
        origin: `http://127.0.0.1:${actual}`,
        bundles,
        misses,
        close: () => new Promise((r) => server.close(r)),
      });
    });
  });
}
