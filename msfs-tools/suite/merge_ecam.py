import collections
import glob
import json
import os
import re
import sys

out = sys.argv[1]
msg_dir = r'D:/A380/fbw-xp-worktrees/fs2020-672384b/fbw-a380x/src/systems/instruments/src/MsfsAvionicsCommon/EcamMessages'


def clean(text):
    text = re.sub(r'\\x1b<\d+m|\\x1b\d*m|\\x1b\[\d+m', '', text)
    return re.sub(r'\s+', ' ', text.replace("\\'", "'")).strip()


titles = {}
for path in glob.glob(os.path.join(msg_dir, '**', '*.ts'), recursive=True):
    src = open(path, encoding='utf-8', errors='ignore').read()
    for m in re.finditer(r"\b(\d{1,10}):\s*\{\s*title:\s*'((?:[^'\\]|\\.)*)'", src):
        titles.setdefault(m.group(1), clean(m.group(2)))
    for m in re.finditer(r"\b(\d{1,10}):\s*'((?:[^'\\]|\\.)*)'", src):
        titles.setdefault(m.group(1), clean(m.group(2)))


def name(key):
    t = titles.get(str(key))
    return f'{t} ({key})' if t else str(key)


lines = [
    '# ECAM and display replay',
    '',
    'Each run\'s end state from the headless campaign is replayed through the A380 FWS (FwsCore) for 20 s after a 20 s healthy settle,',
    'and compared with the healthy reference. Ground replays use the stationary ADR airspeed (CAS below 30 kt: 0, NCD, as adirs.rs outputs);',
    'the bench left the sim\'s indicated airspeed at the test-bed default of 250 kt on the ground. Display units are dark when every DC bus',
    'feeding them is unpowered or their breaker is open (CdsDisplayUnit rules).',
    '',
]
table = ['label\traised\tcleared\tmemos\tinop_sys\tlimitations\tdark\tmaster_warning\tmaster_caution']
for tag in ('runway', 'apron'):
    rows = []
    seen_labels = set()
    for path in sorted(glob.glob(os.path.join(out, f'fws-{tag}', '**', 'fws-*.jsonl'), recursive=True)):
        with open(path, encoding='utf-8', errors='ignore') as f:
            for line in f:
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                if row.get('label') in seen_labels:
                    continue
                seen_labels.add(row.get('label'))
                rows.append(row)
    if not rows:
        continue
    total_runs = sum(1 for _ in open(os.path.join(out, f'fws-{tag}', 'runs.jsonl'), encoding='utf-8', errors='ignore'))
    errors = [r for r in rows if 'error' in r]
    ok = [r for r in rows if 'error' not in r]
    singles = [r for r in ok if ' + ' not in r['label']]
    combos = [r for r in ok if ' + ' in r['label']]
    raised = collections.Counter(a for r in ok for a in r.get('raised', []))
    inop = collections.Counter(a for r in ok for a in r.get('inopSys', []))
    dark = collections.Counter(d for r in ok for d in r.get('dark', []))
    with_ecam = [r for r in ok if r.get('raised') or r.get('memos') or r.get('inopSys') or r.get('limitations')]
    silent = [r for r in singles if not (r.get('raised') or r.get('memos') or r.get('inopSys') or r.get('limitations') or r.get('cleared') or r.get('dark'))]
    healthy = {}
    for hp in [os.path.join(out, f'fws-{tag}', 'healthy.json')] + sorted(glob.glob(os.path.join(out, f'fws-{tag}', 'q', '*', 'healthy.json'))):
        try:
            healthy = json.load(open(hp, encoding='utf-8'))
            break
        except (OSError, ValueError):
            pass
    lines += [f'## {tag}', '']
    if healthy:
        lines += ['Already present in the healthy reference, so masked in every run below (mostly parts the headless bench does not run: FMS, RMPs, BTV, ROW/ROP):', '']
        lines += [f'- procedure: {name(k)}' for k in healthy.get('abnormal', [])]
        lines += [f'- INOP SYS: {name(k)}' for k in healthy.get('inopSys', [])]
        lines += [f'- limitation: {name(k)}' for k in healthy.get('limitations', [])]
        lines += [f'- display already dark: {d}' for d in healthy.get('dark', [])]
        lines += [f'- master caution lit: {healthy.get("masterCaution")}, master warning lit: {healthy.get("masterWarning")} (a lit light cannot be counted as new per run)', '']
    lines += [f'- runs replayed: {len(ok)} of {total_runs} ({len(singles)} single failures, {len(combos)} anomalous combinations; replay errors: {len(errors)})',
              f'- runs raising ECAM procedures, memos, INOP SYS or limitations: {len(with_ecam)}',
              f'- runs where a display unit went dark: {sum(1 for r in ok if r.get("dark"))}',
              f'- single failures with no ECAM or display consequence at all: {len(silent)}',
              f'- master warning: {sum(1 for r in ok if r.get("masterWarning"))}, master caution: {sum(1 for r in ok if r.get("masterCaution"))}', '']
    lines += ['### Most-raised procedures', ''] + [f'- {name(k)}: {v}' for k, v in raised.most_common(60)] + ['']
    lines += ['### Most-reported INOP SYS', ''] + [f'- {name(k)}: {v}' for k, v in inop.most_common(40)] + ['']
    lines += ['### Displays going dark', ''] + [f'- {k}: {v}' for k, v in dark.most_common()] + ['']
    lines += ['### Single failures with no ECAM or display consequence (first 300)', ''] + [f'- {r["label"]}' for r in silent[:300]] + ['']
    if errors:
        lines += ['### Replay errors (first 50)', ''] + [f'- {r["label"]}: {r["error"][:200]}' for r in errors[:50]] + ['']
    for r in ok:
        table.append('\t'.join([f'[{tag}] {r["label"]}'] + ['; '.join(name(k) for k in r.get(f, [])) for f in ('raised', 'cleared', 'memos', 'inopSys', 'limitations')]
                               + ['; '.join(r.get('dark', [])), str(bool(r.get('masterWarning'))), str(bool(r.get('masterCaution')))]))
with open(os.path.join(out, 'ECAM.md'), 'w', encoding='utf-8') as f:
    f.write('\n'.join(lines))
with open(os.path.join(out, 'ECAM-runs.tsv'), 'w', encoding='utf-8') as f:
    f.write('\n'.join(table))
print(f'ECAM.md written ({len(titles)} titles known, {len(table) - 1} runs in ECAM-runs.tsv)')
