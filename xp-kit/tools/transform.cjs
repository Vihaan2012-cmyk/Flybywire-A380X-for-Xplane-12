'use strict';
const fs = require('fs');
const path = require('path');

const KIT_ROOT = path.resolve(__dirname, '..');
const names = JSON.parse(fs.readFileSync(path.join(KIT_ROOT, 'names.json'), 'utf8'));
const SDK = '@microsoft/msfs-sdk';
const FACTORIES = { Subject: '$cell', MappedSubject: '$derive', ConsumerSubject: '$feed', ArraySubject: '$list' };
const FACTORY_EXPORTS = { $cell: 'cell', $derive: 'derive', $feed: 'feed', $list: 'list' };

const memberPairs = Object.entries(names.members).filter(([k]) => k !== 'pub').sort((a, b) => b[0].length - a[0].length);
const memberMap = new Map(memberPairs);
const memberRe = new RegExp('\\b(' + memberPairs.map(([k]) => k).join('|') + ')\\b', 'g');
const pubAccessRe = /\.pub(\s*[(<])/g;
const pubDeclRe = /^(\s*(?:(?:public|protected|private|override|async|static)\s+)*)pub(\s*[(<])/gm;
const importRe = /(import|export)\s+(type\s+)?\{([^}]*)\}\s*from\s*(['"])@microsoft\/msfs-sdk\4/g;

const unknown = new Set();

function rewriteSpecifiers(list, extra) {
  const out = [];
  for (const raw of list.split(',')) {
    const spec = raw.trim();
    if (!spec) continue;
    const m = spec.match(/^(type\s+)?(\w+)(?:\s+as\s+(\w+))?$/);
    if (!m) {
      unknown.add(`unparsed specifier: ${spec}`);
      out.push(spec);
      continue;
    }
    const [, typeKw, name, alias] = m;
    const local = alias || name;
    const mapped = names.exports[name];
    if (!mapped) {
      unknown.add(name);
      out.push(spec);
      continue;
    }
    out.push(`${typeKw || ''}${mapped} as ${local}`);
  }
  for (const e of extra) out.push(`${FACTORY_EXPORTS[e]} as ${e}`);
  return out.join(', ');
}

function transform(source, file) {
  let s = source;
  const hasSdk = s.includes(SDK);
  if (hasSdk) {
    const locals = new Map();
    s.replace(importRe, (all, kw, typeKw, list) => {
      if (kw === 'import' && !typeKw) {
        for (const raw of list.split(',')) {
          const m = raw.trim().match(/^(\w+)(?:\s+as\s+(\w+))?$/);
          if (m && FACTORIES[m[1]]) locals.set(m[2] || m[1], FACTORIES[m[1]]);
        }
      }
      return all;
    });
    const used = new Set();
    for (const [local, fn] of locals) {
      const re = new RegExp('\\b' + local + '\\.create\\(', 'g');
      if (re.test(s)) {
        used.add(fn);
        s = s.replace(re, fn + '(');
      }
    }
    const kitIndex = 'xp-kit';
    let first = true;
    s = s.replace(importRe, (all, kw, typeKw, list, q) => {
      const extra = kw === 'import' && !typeKw && first ? [...used] : [];
      if (extra.length) first = false;
      return `${kw} ${typeKw || ''}{ ${rewriteSpecifiers(list, extra)} } from ${q}${kitIndex}${q}`;
    });
    if (first && used.size) unknown.add(`factory import not placed in ${file}`);
    if (s.includes(SDK)) unknown.add(`unhandled msfs-sdk reference in ${file}`);
    if (/\.tsx$/.test(file) && /\bFSComponent\b/.test(s)) {
      s = '/** @jsx FSComponent.h */\n/** @jsxFrag FSComponent.Fragment */\n' + s;
    }
  }
  s = s.replace(memberRe, (m) => memberMap.get(m));
  s = s.replace(pubAccessRe, '.send$1').replace(pubDeclRe, '$1send$2');
  return s;
}

function plugin() {
  return {
    name: 'xp-kit',
    setup(build) {
      if (!process.env.XP_KIT) return;
      build.initialOptions.minifyWhitespace = true;
      build.initialOptions.legalComments = 'none';
      build.onLoad({ filter: /\.tsx?$/ }, async (args) => {
        if (args.path.includes('node_modules') || args.path.endsWith('.d.ts')) return undefined;
        const source = await fs.promises.readFile(args.path, 'utf8');
        const contents = transform(source, args.path);
        return { contents, loader: args.path.endsWith('x') ? 'tsx' : 'ts', resolveDir: path.dirname(args.path) };
      });
      build.onResolve({ filter: /^xp-kit$/ }, () => ({ path: path.resolve(process.env.XP_KIT, 'index.js') }));
      build.onResolve({ filter: /^@microsoft\/msfs-sdk$/ }, (args) => ({ errors: [{ text: `msfs-sdk import left after the xp-kit transform in ${args.importer}` }] }));
      build.onEnd(() => {
        if (unknown.size) console.log(`xp-kit: ${unknown.size} unmapped: ${[...unknown].join(' | ')}`);
      });
    },
  };
}

module.exports = { transform, plugin, unknown };
