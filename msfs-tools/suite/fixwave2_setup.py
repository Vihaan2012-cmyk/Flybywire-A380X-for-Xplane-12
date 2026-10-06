import csv
import json
import os
import re
import subprocess
from concurrent.futures import ThreadPoolExecutor

REPO = 'D:/A380/fbw-xp-worktrees/fs2020-672384b'
WT = 'D:/A380/fbw-wt'
OUT = 'E:/fix-wave2'
RES = 'E:/test1results'
os.makedirs(OUT, exist_ok=True)

patches = json.load(open('D:/A380/fbw-build/wasm-fs2020/ecam-msfs/patches-msfs.json', encoding='utf-8'))
bridge_text = ''.join(x['replace'] for x in patches if 'bridge' in x.get('reason', ''))
bridge_ids = set(re.findall(r'\b(\d{9})\b', bridge_text))
bridge_vars = set(re.findall(r'L:A32NX_([A-Z0-9_:]+)', bridge_text))

diff = subprocess.run(['git', 'diff', '8ea9bc2', 'HEAD', '--', 'fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/FwsAbnormalSensed.ts'],
                      cwd=REPO, capture_output=True, text=True, encoding='utf-8').stdout
today = sorted(set(re.findall(r'^\+\s+(\d{9}):', diff, re.M)))
overlap = [i for i in today if i in bridge_ids]
a = [i for i in overlap if i[:2] in ('24', '27', '28')]
b = [i for i in overlap if i not in a]
open(f'{OUT}/dedupe-a.txt', 'w').write('\n'.join(a) + '\n')
open(f'{OUT}/dedupe-b.txt', 'w').write('\n'.join(b) + '\n')
open(f'{OUT}/today-new-ids.txt', 'w').write('\n'.join(today) + '\n')

moved = {}
for line in open(f'{RES}/outcomes-runway.jsonl', encoding='utf-8'):
    v = json.loads(line)
    moved[v['items'][0]] = v['ever']
rows = list(csv.reader(open(f'{RES}/ECAM-runs.tsv', encoding='utf-8'), delimiter='\t'))[1:]
cov = {'coverage-a': [], 'coverage-b': []}
for r in rows:
    if not r[0].startswith('[runway] F') or ' + ' in r[0]:
        continue
    lab = r[0][9:]
    fid = int(re.match(r'F(\d+)', lab).group(1))
    if fid < 100000 or any(r[k] for k in (1, 3, 4, 5, 6)):
        continue
    ever = moved.get(f'F{fid}', [])
    if not ever:
        continue
    covered = [v for v in ever if v in bridge_vars or f'A32NX_{v}' in bridge_vars or v.replace('A32NX_', '') in bridge_vars]
    ata = (fid // 1000) % 1000
    key = 'coverage-a' if ata < 30 else 'coverage-b'
    cov[key].append(f'{lab}\tATA {ata}\tbridge reads: {", ".join(covered[:6]) or "none"}\tmoved: {", ".join(ever[:20])}')
for k, v in cov.items():
    open(f'{OUT}/{k}.txt', 'w', encoding='utf-8').write('label\tata\tbridge alerts read these moved vars\tvariables moved on the runway\n' + '\n'.join(v) + '\n')

packages = ['dedupe-a', 'dedupe-b', 'replay-bridge', 'ata21', 'eng-fuel-flags', 'ra-probes', 'gear-locks', 'slats-power', 'fuel-combined', 'fcu-apu',
            'oil-sd', 'inhibit-a', 'inhibit-b', 'deep-tests', 'env-alerts', 'cleanup', 'falsealarm-a', 'falsealarm-b', 'coverage-a', 'coverage-b']
sparse = ['fbw-a380x/src/systems', 'fbw-a380x/src/wasm', 'fbw-common/src/wasm', 'fbw-common/src/systems', 'fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects']


def make(name):
    path = f'{WT}/fixwave2-{name}'
    if os.path.isdir(path):
        return name, 'exists'
    run = lambda *x, cwd=REPO: subprocess.run(['git', *x], cwd=cwd, capture_output=True, text=True)
    r = run('worktree', 'add', '--no-checkout', '-b', f'fixwave2/{name}', path, 'HEAD')
    if r.returncode:
        return name, r.stderr.strip()[:200]
    run('sparse-checkout', 'init', '--cone', cwd=path)
    run('sparse-checkout', 'set', *sparse, cwd=path)
    r = run('checkout', cwd=path)
    return name, 'ok' if r.returncode == 0 else r.stderr.strip()[:200]


with ThreadPoolExecutor(10) as ex:
    res = list(ex.map(make, packages))
print('worktrees:', sum(1 for _, s in res if s == 'ok'), [r for r in res if r[1] != 'ok'])
print('today ids', len(today), 'overlap', len(overlap), 'dedupe-a', len(a), 'dedupe-b', len(b))
print({k: len(v) for k, v in cov.items()}, 'uncovered by bridge vars:', {k: sum(1 for l in v if 'bridge reads: none' in l) for k, v in cov.items()})
