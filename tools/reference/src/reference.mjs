#!/usr/bin/env node
// Renders FBW A380X instruments in Chromium for a named simulator state.
//
//   npm run reference -- --state approach            all default screens
//   npm run reference -- --state all --screen PFD_L,EWD
//   npm run reference -- --state approach --record   also record DOM/CSS/SVG usage
//
// Output: D:/A380/fbw-build/reference-png/<state>/<SCREEN>.png, plus
// <SCREEN>.access.json (every variable/storage/Coherent access and errors)
// and state.resolved.json (the flat state both renderers load).
import fs from 'node:fs';
import path from 'node:path';
import { DOM_USAGE_DIR, REFERENCE_PNG_DIR, TOOL_ROOT } from './config.mjs';
import { loadScreens } from './panel.mjs';
import { startServer } from './server.mjs';
import { listStates, loadState } from './state.mjs';

const DEFAULT_SCREENS = ['PFD_L', 'PFD_R', 'ND_L', 'ND_R', 'EWD', 'SD', 'FCU', 'CLOCK'];

function parseArgs(argv) {
  const args = { state: 'all', screen: null, record: false, headed: false, settle: null, out: REFERENCE_PNG_DIR, verbose: false };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    const next = () => argv[++i];
    if (a === '--state') args.state = next();
    else if (a === '--screen') args.screen = next();
    else if (a === '--record') args.record = true;
    else if (a === '--headed') args.headed = true;
    else if (a === '--settle') args.settle = Number(next());
    else if (a === '--out') args.out = next();
    else if (a === '--verbose' || a === '-v') args.verbose = true;
    else if (a === '--help' || a === '-h') {
      console.log('usage: npm run reference -- [--state <name|all|none>] [--screen <A,B|all>] [--record] [--settle ms] [--out dir] [--headed] [-v]');
      process.exit(0);
    } else throw new Error(`unknown argument ${a}`);
  }
  return args;
}

async function renderScreen({ browser, server, screen, state, args, hostScripts }) {
  const context = await browser.newContext({
    viewport: { width: screen.width, height: screen.height },
    deviceScaleFactor: 1,
    colorScheme: 'dark',
    reducedMotion: 'no-preference',
  });
  const page = await context.newPage();
  const consoleLines = [];
  page.on('console', (m) => consoleLines.push(`[${m.type()}] ${m.text()}`));
  page.on('pageerror', (e) => consoleLines.push(`[pageerror] ${e.stack || e.message}`));

  const settleMs = args.settle ?? state.time.settleMs;
  const frameMs = state.time.frameMs;
  const start = state.time.epochMs - settleMs;

  // The virtual clock (host/freeze.js) starts settleMs before the state's UTC
  // time and does not move while the panel loads; then exactly settleMs of
  // frames run, ending at the state's time.
  const cfg = { screen, state: { vars: state.vars, storage: state.storage, time: state.time }, record: args.record, clockStart: start };
  await page.addInitScript({ content: `window.__REFERENCE__ = ${JSON.stringify(cfg)};` });
  await page.addInitScript({ content: hostScripts.freeze });
  if (args.record) await page.addInitScript({ content: hostScripts.recorder });

  await page.goto(`${server.origin}/reference/host/vcockpit.html`);
  const loadDeadline = Date.now() + 120000;
  while (!(await page.evaluate(() => window.__refPanelLoaded === true))) {
    if (Date.now() > loadDeadline) throw new Error('panel did not load in 120 s');
    await page.evaluate(() => window.__refIdle(10));
  }
  // Fonts must be ready before the first frame measures text.
  await page.evaluate(() => document.fonts.ready.then(() => true));

  const frames = Math.round(settleMs / frameMs);
  for (let f = 0; f < frames; f++) {
    await page.evaluate((ms) => window.__refStep(ms).then(() => window.__refNoteAnimations()), frameMs);
  }
  await page.evaluate(() => document.fonts.ready.then(() => true));
  const frozen = await page.evaluate(() => window.__refFreezeAnimations());

  const dir = path.join(args.out, state.name);
  fs.mkdirSync(dir, { recursive: true });
  const png = path.join(dir, `${screen.name}.png`);
  await page.screenshot({ path: png, animations: 'allow', caret: 'hide', scale: 'css', timeout: 60000 });

  const access = await page.evaluate(() => window.__refAccessLog());
  access.animationsFrozen = frozen;
  access.animations = await page.evaluate(() => window.__refAnimationUsage());
  access.screen = { name: screen.name, width: screen.width, height: screen.height, gauges: screen.gauges, skippedGauges: screen.skippedGauges };
  access.clock = { start, end: state.time.epochMs, frames, frameMs, ...(await page.evaluate(() => window.__refClockStats())) };
  const stateKeys = new Set(Object.keys(state.vars).map((k) => k.toUpperCase()));
  access.missingInputs = access.reads
    .filter((r) => !r.found && !stateKeys.has(r.key.toUpperCase()))
    .map((r) => ({ key: r.key, units: r.units }))
    .sort((a, b) => a.key.localeCompare(b.key));
  fs.writeFileSync(path.join(dir, `${screen.name}.access.json`), JSON.stringify(access, null, 1));
  fs.writeFileSync(path.join(dir, `${screen.name}.console.log`), consoleLines.join('\n'));

  if (args.record) {
    const usage = await page.evaluate(() => window.__refCollectDomUsage());
    fs.mkdirSync(DOM_USAGE_DIR, { recursive: true });
    fs.writeFileSync(path.join(DOM_USAGE_DIR, `${state.name}.${screen.name}.json`), JSON.stringify(usage, null, 1));
  }

  await context.close();
  return { png, reads: access.reads.length, missing: access.missingInputs.length, errors: access.errors.length, console: consoleLines };
}

async function main() {
  const args = parseArgs(process.argv.slice(2));
  const { chromium } = await import('playwright');
  const { file: panelFile, screens } = loadScreens();
  const states = args.state === 'all' ? listStates() : args.state.split(',');
  const hostScripts = {
    freeze: fs.readFileSync(path.join(TOOL_ROOT, 'host/freeze.js'), 'utf8'),
    recorder: fs.readFileSync(path.join(TOOL_ROOT, 'host/recorder.js'), 'utf8'),
  };

  const server = await startServer({ log: (m) => args.verbose && console.log(`  [server] ${m}`) });
  console.log(`panel.cfg: ${panelFile}\nbundles:   ${server.bundles}\nserver:    ${server.origin}`);
  const browser = await chromium.launch({
    headless: !args.headed,
    args: ['--font-render-hinting=none', '--disable-lcd-text', '--force-color-profile=srgb', '--disable-gpu', '--hide-scrollbars'],
  });

  let failures = 0;
  try {
    for (const stateName of states) {
      const state = loadState(stateName);
      const wanted = (args.screen && args.screen !== 'all' ? args.screen.split(',') : state.screens ?? DEFAULT_SCREENS).map((s) => s.trim());
      const outDir = path.join(args.out, state.name);
      fs.mkdirSync(outDir, { recursive: true });
      fs.writeFileSync(path.join(outDir, 'state.resolved.json'), JSON.stringify({ format: 'fbw-reference-state-resolved/1', name: state.name, time: state.time, vars: state.vars, storage: state.storage, meta: state.meta, sources: state.sources }, null, 1));
      console.log(`\nstate ${state.name}: ${Object.keys(state.vars).length} vars`);
      for (const name of wanted) {
        const screen = screens.find((s) => s.name === name);
        if (!screen) {
          console.log(`  ${name}: not in panel.cfg`);
          failures++;
          continue;
        }
        const t0 = Date.now();
        try {
          const r = await renderScreen({ browser, server, screen, state, args, hostScripts });
          console.log(`  ${name} ${screen.width}x${screen.height}: ${r.png} (${r.reads} vars read, ${r.missing} not in state, ${r.errors} errors, ${((Date.now() - t0) / 1000).toFixed(1)}s)`);
          if (args.verbose) r.console.filter((l) => /error|warn/i.test(l)).slice(0, 20).forEach((l) => console.log(`    ${l}`));
        } catch (e) {
          failures++;
          console.log(`  ${name}: FAILED ${e.stack || e}`);
        }
      }
    }
  } finally {
    await browser.close();
    await server.close();
  }
  if (server.misses.size) console.log(`\nfiles not found (${server.misses.size}): ${[...server.misses].slice(0, 20).join(', ')}`);
  process.exit(failures ? 1 : 0);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});
