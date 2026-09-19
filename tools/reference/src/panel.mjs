// Reads panel.cfg the way MSFS's VCockpit host does: one [VCockpitNN] section
// is one screen (one Coherent view, one texture) holding one or more
// htmlgaugeNN, each "url, x, y, w, h".
import fs from 'node:fs';
import { PANEL_CFG_CANDIDATES } from './config.mjs';

// Screen names shared with the native renderer (docs/reference-harness.md).
// Keyed by the texture the VCockpit paints.
const SCREEN_BY_TEXTURE = {
  $SCREEN_DU_MFD: 'MFD',
  $SCREEN_DU_EWD: 'EWD',
  $SCREEN_DU_SD: 'SD',
  $SCREEN_DU_PFDL: 'PFD_L',
  $SCREEN_DU_PFDR: 'PFD_R',
  $SCREEN_DU_NDL: 'ND_L',
  $SCREEN_DU_NDR: 'ND_R',
  $FCU: 'FCU',
  SCREEN_ISIS_1: 'ISIS',
  $Clock: 'CLOCK',
  $RTPI: 'RTPI',
  $BAT: 'BAT',
  $SCREEN_EFB: 'EFB',
  SCREEN_DU_RMP_1: 'RMP_1',
  SCREEN_DU_RMP_2: 'RMP_2',
  SCREEN_DU_RMP_3: 'RMP_3',
  $SCREEN_OIT_LEFT: 'OIT_L',
  $SCREEN_OIT_RIGHT: 'OIT_R',
};

export function panelCfgPath() {
  const found = PANEL_CFG_CANDIDATES.find((p) => fs.existsSync(p));
  if (!found) throw new Error(`panel.cfg not found in: ${PANEL_CFG_CANDIDATES.join(', ')}`);
  return found;
}

export function parsePanelCfg(text) {
  const sections = [];
  let cur = null;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.replace(/[;].*$/, '').trim();
    if (!line) continue;
    const head = line.match(/^\[(.+)\]$/);
    if (head) {
      cur = { section: head[1], values: {}, gauges: [] };
      sections.push(cur);
      continue;
    }
    if (!cur) continue;
    const eq = line.indexOf('=');
    if (eq < 0) continue;
    const key = line.slice(0, eq).trim();
    const value = line.slice(eq + 1).trim();
    const gauge = key.match(/^htmlgauge(\d+)$/i);
    if (gauge) {
      const parts = value.split(',').map((s) => s.trim());
      cur.gauges.push({
        slot: Number(gauge[1]),
        url: parts[0],
        x: Number(parts[1] ?? 0),
        y: Number(parts[2] ?? 0),
        w: Number(parts[3] ?? 0),
        h: Number(parts[4] ?? 0),
        extra: parts.slice(5),
      });
    } else {
      cur.values[key.toLowerCase()] = value;
    }
  }
  return sections;
}

export function loadScreens() {
  const file = panelCfgPath();
  const screens = [];
  for (const s of parsePanelCfg(fs.readFileSync(file, 'utf8'))) {
    if (!/^VCockpit/i.test(s.section)) continue;
    const texture = s.values.texture;
    const name = SCREEN_BY_TEXTURE[texture];
    const size = (s.values.pixel_size ?? '0,0').split(',').map(Number);
    if (!name || !(size[0] > 0)) continue;
    const gauges = s.gauges.sort((a, b) => a.slot - b.slot);
    screens.push({
      name,
      section: s.section,
      texture,
      width: size[0],
      height: size[1],
      // WASM gauges (terrain display) are native code and cannot run in a browser.
      gauges: gauges.filter((g) => !/^WasmInstrument\//i.test(g.url)),
      skippedGauges: gauges.filter((g) => /^WasmInstrument\//i.test(g.url)),
    });
  }
  return { file, screens };
}
