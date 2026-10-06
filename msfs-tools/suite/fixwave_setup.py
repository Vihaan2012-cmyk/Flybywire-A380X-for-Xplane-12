import collections
import csv
import json
import os
import re
import subprocess
from concurrent.futures import ThreadPoolExecutor

REPO = 'D:/A380/fbw-xp-worktrees/fs2020-672384b'
WT = 'D:/A380/fbw-wt'
OUT = 'E:/fix-wave'
RES = 'E:/test1results'
os.makedirs(OUT, exist_ok=True)

moved = {}
for line in open(f'{RES}/outcomes-runway.jsonl', encoding='utf-8'):
    v = json.loads(line)
    moved[v['items'][0]] = sorted(v['ever'])
labels = {}
for line in open(f'{RES}/fws-runway/runs.jsonl', encoding='utf-8'):
    r = json.loads(line)
    if ' + ' not in r['label']:
        labels[r['label'].split(' ', 1)[0]] = r['label']


def fid(label):
    m = re.match(r'F(\d+)', label)
    return int(m.group(1)) if m else 0


ecam_groups = {
    'ecam-28': ['28'], 'ecam-24': ['24'], 'ecam-21': ['21'], 'ecam-36': ['36'], 'ecam-32': ['32'],
    'ecam-72-73': ['72', '73'], 'ecam-76-77-79': ['76', '77', '79'], 'ecam-29': ['29'], 'ecam-34': ['34'],
    'ecam-misc': ['26', '30', '27', '49', '22', '23', '25', '31', '33', '35', '38', '45', '46', '52', '56', '70', '71', '74', '75', '78', '80'],
}
rows = list(csv.reader(open(f'{RES}/ECAM-runs.tsv', encoding='utf-8'), delimiter='\t'))[1:]
for name, atas in ecam_groups.items():
    with open(f'{OUT}/{name}.txt', 'w', encoding='utf-8') as f:
        f.write('label\tvariables that moved on the runway (first 25)\n')
        for r in rows:
            if not r[0].startswith('[runway] F') or ' + ' in r[0]:
                continue
            lab = r[0][9:]
            i = fid(lab)
            if i >= 100000 or str(i)[:2] not in atas:
                continue
            key = f'F{i}'
            if not any(r[k] for k in (1, 3, 4, 5, 6)) and moved.get(key):
                f.write(f'{lab}\t{", ".join(moved[key][:25])}\n')

triage = list(csv.reader(open(f'{RES}/DEAD-TRIAGE.tsv', encoding='utf-8'), delimiter='\t'))
header, triage = triage[0], triage[1:]


def area(label):
    i = fid(label)
    if i < 100000:
        return 'FBW', 0
    if i < 2_000_000:
        return 'elecmode', 0
    return i // 1_000_000, (i // 1000) % 1000


latent_groups = {
    'verify-elecmode': lambda a: a[0] == 'elecmode',
    'verify-area17': lambda a: a[0] == 17,
    'verify-area6-a': lambda a: a[0] == 6 and a[1] in (26, 52, 77),
    'verify-area6-b': lambda a: a[0] == 6 and a[1] not in (26, 52, 77),
    'verify-area2': lambda a: a[0] == 2,
    'verify-area3-4-5-11': lambda a: a[0] in (3, 4, 5, 11),
    'verify-area7-8-10-15-19-20': lambda a: a[0] in (7, 8, 10, 15, 19, 20),
    'verify-fbw': lambda a: a[0] == 'FBW',
}
for name, pred in latent_groups.items():
    with open(f'{OUT}/{name}.tsv', 'w', encoding='utf-8') as f:
        f.write('\t'.join(header) + '\n')
        for r in triage:
            if r[1] != 'dead' and pred(area(r[0])):
                f.write('\t'.join(r) + '\n')

packages = list(ecam_groups) + list(latent_groups) + ['slats', 'overbroad']
assert len(packages) == 20, len(packages)
sparse = ['fbw-a380x/src/systems', 'fbw-a380x/src/wasm', 'fbw-common/src/wasm', 'fbw-common/src/systems', 'fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects']


def make(name):
    path = f'{WT}/fixwave-{name}'
    if os.path.isdir(path):
        return name, 'exists'
    run = lambda *a, cwd=REPO: subprocess.run(['git', *a], cwd=cwd, capture_output=True, text=True)
    r = run('worktree', 'add', '--no-checkout', '-b', f'fixwave/{name}', path, 'HEAD')
    if r.returncode:
        return name, r.stderr.strip()[:200]
    run('sparse-checkout', 'init', '--cone', cwd=path)
    run('sparse-checkout', 'set', *sparse, cwd=path)
    r = run('checkout', cwd=path)
    return name, 'ok' if r.returncode == 0 else r.stderr.strip()[:200]


with ThreadPoolExecutor(10) as ex:
    for name, status in ex.map(make, packages):
        print(name, status)
for p in packages:
    n = 0
    for ext in ('.txt', '.tsv'):
        fp = f'{OUT}/{p}{ext}'
        if os.path.exists(fp):
            n = sum(1 for _ in open(fp, encoding='utf-8')) - 1
    print(f'{p}: {n} failures')
