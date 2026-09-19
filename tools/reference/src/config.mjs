// Paths the harness reads and writes. Each can be overridden with an
// environment variable so the harness runs on another machine.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const env = (name, fallback) => process.env[name] ?? fallback;

export const TOOL_ROOT = path.resolve(here, '..');

// FlyByWire checkout. Its own build (`fbw-a380x/out/...`) is served as-is.
export const FBW_ROOT = env('FBW_ROOT', 'D:/fbw-aircraft');

// Built instrument bundles: html_ui of FBW's mach build output. Falls back to
// D:/fbw-build/reference-bundles when the runtime engineer's build is absent.
export const BUNDLE_HTML_UI_CANDIDATES = [
  process.env.FBW_BUNDLE_HTML_UI,
  path.join(FBW_ROOT, 'fbw-a380x/out/flybywire-aircraft-a380-842/html_ui'),
  'D:/fbw-build/reference-bundles/flybywire-aircraft-a380-842/html_ui',
].filter(Boolean);

// Fonts, images and FBW's small plain-JS helpers (A380X_Simvars.js) live in
// FBW's tracked base package, not in the build output.
export const BASE_HTML_UI = env('FBW_BASE_HTML_UI', path.join(FBW_ROOT, 'fbw-a380x/src/base/flybywire-aircraft-a380-842/html_ui'));

// panel.cfg of the installed aircraft (sizes and gauge URLs). FBW's tracked
// copy is used when the MSFS package is missing.
export const PANEL_CFG_CANDIDATES = [
  process.env.FBW_PANEL_CFG,
  'D:/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/Community/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842/panel/panel.cfg',
  path.join(FBW_ROOT, 'fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842/panel/panel.cfg'),
].filter(Boolean);

// FBW's flight files (apron.FLT, Climb.flt, ...): the aircraft's own spawn states.
export const FLT_DIR = env('FBW_FLT_DIR', path.dirname(path.dirname(PANEL_CFG_CANDIDATES.find((p) => fs.existsSync(p)) ?? PANEL_CFG_CANDIDATES[0])));

export const STATES_DIR = env('REFERENCE_STATES_DIR', path.join(TOOL_ROOT, 'states'));
export const REFERENCE_PNG_DIR = env('REFERENCE_PNG_DIR', 'D:/fbw-build/reference-png');
export const SNAPSHOT_DIR = env('DISPLAY_SNAPSHOT_DIR', 'D:/fbw-build/display-snapshots');
export const REPORT_DIR = env('REFERENCE_REPORT_DIR', 'D:/fbw-build/reference-report');
export const DOM_USAGE_DIR = env('DOM_USAGE_DIR', 'D:/fbw-build/reference-dom-usage');
export const DOCS_DIR = env('FBW_XP_DOCS_DIR', path.resolve(TOOL_ROOT, '../../docs'));

// Chromium lives on D:, C: is nearly full.
process.env.PLAYWRIGHT_BROWSERS_PATH ??= 'D:/fbw-build/browsers';
