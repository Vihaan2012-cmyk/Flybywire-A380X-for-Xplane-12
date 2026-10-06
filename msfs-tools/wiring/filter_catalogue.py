import json, re, shutil, sys, os

CAT = 'D:/A380/msfs-a380/install/out/EFB/catalogue.json'
log = open(sys.argv[1], encoding='utf-8', errors='ignore').read()
m = re.search(r'(\d+) failures the EFB lists would be rejected:\n(.*?)\n\n', log, re.S)
if not m:
    print('no unarmable failures in the log -- nothing to filter')
    sys.exit(0)
ids = {int(l.split()[0]) for l in m.group(2).splitlines() if l.strip() and l.split()[0].isdigit()}
if not os.path.exists(CAT + '.pre-filter'):
    shutil.copy(CAT, CAT + '.pre-filter')
cat = json.load(open(CAT + '.pre-filter', encoding='utf-8'))
removed = [f for f in cat['failures'] if f['id'] in ids]
cat['failures'] = [f for f in cat['failures'] if f['id'] not in ids]
for c in cat.get('components', []):
    if isinstance(c, dict) and 'failures' in c:
        c['failures'] = [x for x in c['failures'] if (x if isinstance(x, int) else x.get('id')) not in ids]
json.dump(cat, open(CAT, 'w', encoding='utf-8'), separators=(',', ':'))
with open('D:/A380/msfs-a380/wiring/not-armable-in-msfs.txt', 'w', encoding='utf-8') as out:
    out.write('Failures taken out of the MSFS EFB catalogue: an X-Plane-era id with no model\n')
    out.write('behind it in MSFS and no equivalent failure to alias it to. Arming one was rejected.\n\n')
    for f in sorted(removed, key=lambda f: f['id']):
        out.write(f"{f['id']}  {f['name']}  ({f['component']})\n")
print(f'removed {len(removed)} unarmable failures; catalogue now {len(cat["failures"])}')
