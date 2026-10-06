import collections
import json
import sys

out = sys.argv[1]
tag = sys.argv[2] if len(sys.argv) > 2 else 'runway'

labels = {}
for line in open(f'{out}/fws-{tag}/runs.jsonl', encoding='utf-8', errors='ignore'):
    try:
        r = json.loads(line)
    except ValueError:
        continue
    if ' + ' not in r['label']:
        labels[r['label'].split(' ', 1)[0]] = r['label']


def low(end, names, limit):
    return all(n in end and end[n] < limit for n in names)


checks = {
    'all four engines stopped': lambda e: low(e, [f'ENGINE_N2:{i}' for i in range(1, 5)], 5),
    'all four engine generators lost': lambda e: low(e, [f'ELEC_ENG_GEN_{i}_POTENTIAL' for i in range(1, 5)], 10),
    'all four AC buses lost': lambda e: low(e, [f'ELEC_AC_{i}_BUS_IS_POWERED' for i in range(1, 5)], 0.5),
    'green and yellow hydraulics lost': lambda e: low(e, ['HYD_GREEN_SYSTEM_1_SECTION_PRESSURE', 'HYD_YELLOW_SYSTEM_1_SECTION_PRESSURE'], 1000),
    'all three PRIMs lost': lambda e: low(e, [f'PRIM_{i}_HEALTHY' for i in range(1, 4)], 0.5),
    'DC ESS and DC 1 and DC 2 lost': lambda e: low(e, ['ELEC_DC_ESS_BUS_IS_POWERED', 'ELEC_DC_1_BUS_IS_POWERED', 'ELEC_DC_2_BUS_IS_POWERED'], 0.5),
}
hits = collections.defaultdict(list)
breadth = []
for line in open(f'{out}/outcomes-{tag}.jsonl', encoding='utf-8', errors='ignore'):
    v = json.loads(line)
    item = v['items'][0]
    end = v.get('end', {})
    name = labels.get(item, item)
    breadth.append((len(v.get('ever', [])), name))
    for check, f in checks.items():
        if f(end):
            hits[check].append(name)
lines = [f'# Over-broad single failures ({tag})', '']
for check in checks:
    lines += [f'## {check}: {len(hits[check])}', ''] + [f'- {n}' for n in sorted(hits[check])] + ['']
breadth.sort(reverse=True)
lines += ['## Widest single failures (variables moved)', ''] + [f'- {n}: {b}' for b, n in breadth[:80]]
open(f'{out}/SCAN-{tag}.md', 'w', encoding='utf-8').write('\n'.join(lines))
for check in checks:
    print(f'{check}: {len(hits[check])}')
