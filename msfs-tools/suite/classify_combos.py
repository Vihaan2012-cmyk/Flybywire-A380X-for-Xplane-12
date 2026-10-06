import collections
import json
import re
import sys

out = sys.argv[1]
CONTINUOUS = re.compile(r'(CHARGE_FRACTION|_QTY_|_KG_|_KG$|TEMPERATURE|_TEMP_|_TEMP$|_C$|_PA$|_PSI|_W$|_A$|CURRENT|LOAD|POTENTIAL|PRESSURE|_FRAC$|FRACTION|_AGE_S$|_S$|FLOW|RPM|_N1|_N2|EGT|LEVEL|ANGLE|DEFLECTION|POSITION_PERCENT|_DEG$|COUNT$)')


def kind(var):
    return 'continuous' if CONTINUOUS.search(var) else 'discrete'


by_phase = collections.defaultdict(lambda: collections.Counter())
emergent_vars = collections.Counter()
masked_vars = collections.Counter()
discrete_examples = []
for line in open(f'{out}/pairs.jsonl', encoding='utf-8', errors='ignore'):
    try:
        v = json.loads(line)
    except ValueError:
        continue
    phase = v.get('phase')
    e = v.get('emergent_sample') or []
    m = v.get('masked_sample') or []
    if not (e or m):
        continue
    kinds = {kind(x) for x in e + m}
    label = 'discrete' if 'discrete' in kinds else 'continuous only'
    by_phase[phase][label] += 1
    emergent_vars.update(e)
    masked_vars.update(m)
    if 'discrete' in kinds and len(discrete_examples) < 4000:
        discrete_examples.append((v['label'], [x for x in e if kind(x) == 'discrete'], [x for x in m if kind(x) == 'discrete']))

lines = ['# Combination anomalies by kind', '',
         'continuous only: every emergent or masked variable is a slow analogue quantity (charge, temperature, quantity, current, age); discrete: at least one state flag, valve, contactor, breaker, failure or status word changed.', '',
         '| phase | discrete | continuous only |', '|---|---|---|']
for phase, c in by_phase.items():
    lines.append(f'| {phase} | {c["discrete"]} | {c["continuous only"]} |')
lines += ['', '## Variables most often emergent', ''] + [f'- {k} ({kind(k)}): {n}' for k, n in emergent_vars.most_common(40)]
lines += ['', '## Variables most often masked', ''] + [f'- {k} ({kind(k)}): {n}' for k, n in masked_vars.most_common(40)]
lines += ['', '## Discrete interactions (first 300)', '']
for label, e, m in discrete_examples[:300]:
    lines.append(f'- {label}: ' + '; '.join(filter(None, [('emergent ' + ', '.join(e[:5])) if e else '', ('masked ' + ', '.join(m[:5])) if m else ''])))
open(f'{out}/COMBOS.md', 'w', encoding='utf-8').write('\n'.join(lines))
print('\n'.join(lines[:12]))
print('top emergent:', emergent_vars.most_common(8))
