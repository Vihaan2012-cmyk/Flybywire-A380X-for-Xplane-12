// Runs one of FlyByWire's esbuild scripts (their build.js, unchanged) on a
// Windows host.
//
// FlyByWire's build-utils.js turns every environment variable into an
// esbuild `define` (`process.env.NAME` -> "value"). Their builds run in a
// Linux container, where values are plain; on Windows many values are paths
// with backslashes (C:\WINDOWS), which esbuild rejects as define values, and
// the whole build fails. This keeps the variables esbuild's own process
// needs, with forward slashes, drops every other variable a define cannot
// hold, and sets the variables FlyByWire's CI sets for the A380X
// (.github/workflows/master.yml).
//
// Usage, from the FlyByWire workspace root:
//   node D:/fbw-xp-systems/tools/js-build/fbw-env.cjs fbw-a380x/src/systems/systems-host/build.js
'use strict';

const path = require('path');

const ci = {
  AIRCRAFT_PROJECT_PREFIX: 'a380x',
  AIRCRAFT_VARIANT: 'a380-842',
  VITE_BUILD: 'false',
};

const needed = new Set(['PATH', 'SYSTEMROOT', 'WINDIR', 'TEMP', 'TMP', 'USERPROFILE', 'APPDATA', 'LOCALAPPDATA']);
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
  if (process.env[name] === undefined) {
    process.env[name] = value;
  }
}

const script = process.argv[2];
if (!script) {
  console.error('usage: node fbw-env.cjs <build script>');
  process.exit(2);
}
require(path.resolve(script));
