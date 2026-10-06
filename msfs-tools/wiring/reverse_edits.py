import json, os, sys

E = json.load(open('D:/A380/msfs-a380/wiring/code-wave-edits.json', encoding='utf-8'))
family = sys.argv[1]
only = sys.argv[2] if len(sys.argv) > 2 else None
for e in reversed(E[family]):
    if e['tool'] != 'Edit':
        print('skip non-Edit', e['tool'])
        continue
    p = e['file_path']
    if only and only not in os.path.normpath(p).replace(os.sep, '/'):
        continue
    s = open(p, encoding='utf-8').read()
    new, old = e['new_string'], e['old_string']
    n = s.count(new)
    if n == 0:
        print('NOT FOUND', os.path.basename(p), new[:60].replace('\n', ' '))
        continue
    if n > 1 and not e.get('replace_all'):
        print('AMBIGUOUS', os.path.basename(p))
        continue
    s = s.replace(new, old)
    open(p, 'w', encoding='utf-8', newline='').write(s)
    print('reverted', os.path.basename(p))
