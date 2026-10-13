#!/usr/bin/env node
// Downloads the Chromium build the installed Playwright expects into
// PLAYWRIGHT_BROWSERS_PATH (D:/A380/fbw-build/browsers; C: is nearly full).
import { spawnSync } from 'node:child_process';
import { createRequire } from 'node:module';
import path from 'node:path';
import './config.mjs';

const require = createRequire(import.meta.url);
const cli = path.join(path.dirname(require.resolve('playwright/package.json')), 'cli.js');
console.log(`installing chromium into ${process.env.PLAYWRIGHT_BROWSERS_PATH}`);
const r = spawnSync(process.execPath, [cli, 'install', 'chromium'], { stdio: 'inherit', env: process.env });
process.exit(r.status ?? 1);
