'use strict';
const path = require('path');

const ci = { AIRCRAFT_PROJECT_PREFIX: 'a380x', AIRCRAFT_VARIANT: 'a380-842', VITE_BUILD: 'false', CLIENT_ID: '', CLIENT_SECRET: '' };
if (process.env.XP_RELEASE === '1') {
  delete ci.CLIENT_ID;
  delete ci.CLIENT_SECRET;
  delete process.env.CLIENT_ID;
  delete process.env.CLIENT_SECRET;
}
delete process.env.XP_RELEASE;
const needed = new Set(['PATH', 'SYSTEMROOT', 'WINDIR', 'TEMP', 'TMP', 'USERPROFILE', 'APPDATA', 'LOCALAPPDATA', 'NODE_OPTIONS']);
const backslash = String.fromCharCode(92);
const definable = (value) => !value.includes(backslash) && !value.includes('"') && !/[\u0000-\u001f]/.test(value);

for (const name of Object.keys(process.env)) {
  const value = process.env[name];
  if (needed.has(name.toUpperCase())) {
    process.env[name] = value.split(backslash).join('/').split('"').join('');
  } else if (!definable(value)) {
    delete process.env[name];
  }
}
for (const [name, value] of Object.entries(ci)) {
  if (process.env[name] === undefined) process.env[name] = value;
}

const script = path.resolve(process.argv[2]);
process.argv = [process.argv[0], script, ...process.argv.slice(3)];
require(script);
