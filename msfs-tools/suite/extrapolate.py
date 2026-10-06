import collections
import json
import math
import os
import re
import sys

out = sys.argv[1]
EXCLUDE = re.compile(os.environ['EXCLUDE_ITEMS']) if os.environ.get('EXCLUDE_ITEMS') else None
IGNORE_VARS = set(filter(None, os.environ.get('IGNORE_VARS', '').split(',')))
REPORT = os.environ.get('REPORT', 'EXTRAPOLATION.md')
TITLE = os.environ.get('TITLE', 'Extrapolation to the untested combinations')
Z = 1.959964
CONTINUOUS = re.compile(r'(CHARGE_FRACTION|_QTY_|_KG_|_KG$|TEMPERATURE|_TEMP_|_TEMP$|_C$|_PA$|_PSI|_W$|_A$|CURRENT|LOAD|POTENTIAL|PRESSURE|_FRAC$|FRACTION|_AGE_S$|_S$|FLOW|RPM|_N1|_N2|EGT|LEVEL|ANGLE|DEFLECTION|POSITION_PERCENT|_DEG$|_FT$|POSITION$|TARGET$|_TEMP$|_RATIO$)')


def wilson(k, n):
    if n == 0:
        return 0.0, 0.0, 1.0
    p = k / n
    d = 1 + Z * Z / n
    c = (p + Z * Z / (2 * n)) / d
    h = Z * math.sqrt(p * (1 - p) / n + Z * Z / (4 * n * n)) / d
    return p, max(0.0, c - h), min(1.0, c + h)


def discrete(v):
    names = [n for n in (v.get('emergent_sample') or []) + (v.get('masked_sample') or []) if n not in IGNORE_VARS]
    return any(not CONTINUOUS.search(n) for n in names)


def anomalous(v):
    e = [n for n in (v.get('emergent_sample') or []) if n not in IGNORE_VARS]
    m = [n for n in (v.get('masked_sample') or []) if n not in IGNORE_VARS]
    more = v.get('emergent', 0) > len(v.get('emergent_sample') or []) or v.get('masked', 0) > len(v.get('masked_sample') or [])
    return bool(e or m or more)


progress = json.load(open(f'{out}/progress.json', encoding='utf-8'))
population = {g['name']: g['population'] for g in progress.get('groups', [])}
stats = collections.defaultdict(lambda: collections.Counter())
ecam = {}
try:
    for line in open(f'{out}/ECAM-runs.tsv', encoding='utf-8'):
        cols = line.rstrip('\n').split('\t')
        if len(cols) >= 9 and cols[0].startswith('['):
            ecam[cols[0].split('] ', 1)[1]] = frozenset(f'{i}:{x}' for i in (1, 3, 4, 5, 6) for x in cols[i].split('; ') if x)
except OSError:
    pass
for line in open(f'{out}/pairs.jsonl', encoding='utf-8', errors='ignore'):
    try:
        v = json.loads(line)
    except ValueError:
        continue
    g, ph = v.get('group'), v.get('phase')
    if g not in population:
        continue
    if EXCLUDE and any(EXCLUDE.match(p) for p in v['label'].split(' + ')):
        continue
    bug = bool(v.get('panic') or v.get('cpp') or v.get('non_finite') or v.get('runaway'))
    hit = bug or anomalous(v)
    s = stats[g]
    s[f'{ph}_n'] += 1
    s[f'{ph}_hit'] += hit
    s[f'{ph}_disc'] += hit and discrete(v)
    s[f'{ph}_bug'] += bug
    parts = v['label'].split(' + ')
    if v['label'] in ecam and all(p in ecam for p in parts):
        union = frozenset().union(*(ecam[p] for p in parts))
        s[f'{ph}_ecam_n'] += 1
        s[f'{ph}_ecam'] += ecam[v['label']] != union

lines = [f'# {TITLE}', '',
         'Each group\'s anomaly rate comes from its pilot: pairs drawn uniformly at random from the group, so it is an unbiased sample of the whole group. '
         'The greedy phase then ran the highest-coupling pairs of the best groups first, so those pairs are richer than the rest; the remaining anomalies are therefore estimated as '
         '(pilot rate x group population) minus the anomalies already found, not as pilot rate x unrun pairs. Bounds are 95% Wilson intervals on the pilot rate. '
         '"Discrete" means at least one state flag, valve, contactor, breaker, fault or status word was involved, not only slow analogue drift.', '',
         '| group | population | pilot | anomaly rate (95%) | discrete rate (95%) | run | anomalies found | estimated anomalies in group (95%) | estimated still unfound (95%) | discrete still unfound (95%) |',
         '|---|---|---|---|---|---|---|---|---|---|']
tot = collections.Counter()
for g, n_pop in population.items():
    s = stats[g]
    n = s['pilot_n']
    p, lo, hi = wilson(s['pilot_hit'], n)
    dp, dlo, dhi = wilson(s['pilot_disc'], n)
    run = s['pilot_n'] + s['exhaust_n']
    found = s['pilot_hit'] + s['exhaust_hit']
    found_d = s['pilot_disc'] + s['exhaust_disc']
    est = [p * n_pop, lo * n_pop, hi * n_pop]
    left = [max(0.0, x - found) for x in est]
    left_d = [max(0.0, x * n_pop - found_d) for x in (dp, dlo, dhi)]
    tot['pop'] += n_pop
    tot['run'] += run
    tot['found'] += found
    for i, k in enumerate(('left', 'left_lo', 'left_hi')):
        tot[k] += left[i]
    for i, k in enumerate(('dleft', 'dleft_lo', 'dleft_hi')):
        tot[k] += left_d[i]
    lines.append(f'| {g} | {n_pop:,} | {n:,} | {100*p:.1f}% ({100*lo:.1f}-{100*hi:.1f}) | {100*dp:.1f}% ({100*dlo:.1f}-{100*dhi:.1f}) | {run:,} | {found:,} | '
                 f'{est[0]:,.0f} ({est[1]:,.0f}-{est[2]:,.0f}) | {left[0]:,.0f} ({left[1]:,.0f}-{left[2]:,.0f}) | {left_d[0]:,.0f} ({left_d[1]:,.0f}-{left_d[2]:,.0f}) |')
lines += ['', f'Total: {tot["pop"]:,} failure pairs, {tot["run"]:,} run ({100*tot["run"]/max(1,tot["pop"]):.2f}%), {tot["found"]:,} anomalies found; '
          f'estimated still unfound {tot["left"]:,.0f} ({tot["left_lo"]:,.0f}-{tot["left_hi"]:,.0f}), of which discrete {tot["dleft"]:,.0f} ({tot["dleft_lo"]:,.0f}-{tot["dleft_hi"]:,.0f}).', '']
bugs = sum(stats[g]['pilot_bug'] for g in population)
npil = sum(stats[g]['pilot_n'] for g in population)
p, lo, hi = wilson(bugs, npil)
lines += [f'Definite bugs (panic, C++ trap, NaN, runaway): {bugs} in {npil:,} random pilot pairs, so the rate is below {100*hi:.3f}% at 95% confidence; '
          f'over {tot["pop"]:,} pairs that bounds the expected number of crashing pairs at {hi*tot["pop"]:,.0f}.', '']
ecam_rows = [(g, stats[g]['pilot_n'], stats[g]['pilot_ecam_n'], stats[g]['pilot_ecam']) for g in population if stats[g]['pilot_ecam_n']]
if ecam_rows:
    lines += ['## Emergent ECAM', '',
              'A pair has emergent ECAM when its replayed ECAM (procedures, memos, INOP SYS, limitations, dark displays) differs from the union of the ECAM of its two singles. '
              'Only anomalous pilot pairs were replayed; a pair without any physical anomaly ends in the superposition of its singles and is counted as having none.', '',
              '| group | pilot pairs | anomalous ones replayed | with emergent ECAM, rate over the pilot (95%) | estimated pairs in group with emergent ECAM (95%) |', '|---|---|---|---|---|']
    for g, n_all, n, k in ecam_rows:
        p, lo, hi = wilson(k, n_all)
        lines.append(f'| {g} | {n_all:,} | {n:,} | {k:,}: {100*p:.2f}% ({100*lo:.2f}-{100*hi:.2f}) | {p*population[g]:,.0f} ({lo*population[g]:,.0f}-{hi*population[g]:,.0f}) |')
if EXCLUDE or IGNORE_VARS:
    lines += ['', f'Pairs with a part matching `{EXCLUDE.pattern if EXCLUDE else ""}` were left out because the fixes changed that failure\'s behaviour, '
              f'and these variables were ignored when deciding whether a pair is anomalous: {", ".join(sorted(IGNORE_VARS)) or "none"}. '
              'Group populations are unchanged, so the rates are applied to whole groups.']
open(f'{out}/{REPORT}', 'w', encoding='utf-8').write('\n'.join(lines))
print('\n'.join(lines))
