import collections
import json
import re
import sys

GOLDEN = 'E:/fbw-int/xpsdk-runs/typecheck-golden.txt'
api = json.load(open('E:/fbw-int/xp-avionics/tools/api-groups.json'))
owner = {n: g for g, ns in api['groups'].items() for n in ns}
owner.update({n: g for g, ns in api['type_groups'].items() for n in ns})
is_error = re.compile(r'error TS\d+')
path = sys.argv[1].replace('\\', '/')
errors = [l for l in open(path, encoding='utf-8', errors='replace').read().splitlines() if is_error.search(l)]
if path != GOLDEN:
    golden = {l for l in open(GOLDEN, encoding='utf-8', errors='replace').read().splitlines() if is_error.search(l)}
    errors = [l for l in errors if l not in golden]
    open(path + '.new.txt', 'w', encoding='utf-8').write('\n'.join(errors) + '\n')
per_name = collections.Counter()
per_group = collections.Counter()
for l in errors:
    hit = next((n for n in re.findall(r"'([A-Za-z_$][\w$]*)", l) if n in owner), None)
    if hit:
        per_name[hit] += 1
        per_group[owner[hit]] += 1
    else:
        per_group['(unattributed)'] += 1
print(f'{len(errors)} type errors beyond the golden baseline; by group: {dict(per_group)}; list in {path}.new.txt')
print('top names:', ', '.join(f'{n} {c}' for n, c in per_name.most_common(25)))
