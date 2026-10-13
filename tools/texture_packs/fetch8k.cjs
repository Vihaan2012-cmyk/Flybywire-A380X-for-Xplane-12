const path = require('path');
const { FragmenterContext, FragmenterInstaller } = require('D:/A380/fbw-installer-xp/node_modules/@flybywiresim/fragmenter');

const url = 'https://flybywirecdn.com/addons/msfs2020/a380x/stable-8k';
const dest = 'E:/fbw-texpacks/fbw-8k-package';
const temp = 'E:/fbw-texpacks/fbw-8k-tmp';

(async () => {
  const ctx = new FragmenterContext({ useConsoleLog: false }, new AbortController().signal);
  const installer = new FragmenterInstaller(ctx, url, dest, { temporaryDirectory: temp, forceManifestCacheBust: true });
  let last = 0;
  installer.on('downloadStarted', (m) => console.log(new Date().toISOString(), 'download', m.name));
  installer.on('downloadProgress', (m, p) => {
    const now = Date.now();
    if (now - last > 30000) {
      last = now;
      console.log(new Date().toISOString(), m.name, `${p.percent}%`);
    }
  });
  installer.on('unzipStarted', (m) => console.log(new Date().toISOString(), 'unzip', m.name));
  installer.on('retryScheduled', (m, n, s) => console.log('retry', m.name, n, s));
  installer.on('error', (e) => console.log('error', e && e.message));
  await installer.install();
  console.log(new Date().toISOString(), 'done');
})().catch((e) => {
  console.error('failed', e && e.stack);
  process.exit(1);
});
