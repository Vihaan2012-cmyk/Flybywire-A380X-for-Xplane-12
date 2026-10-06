import json
import shutil
import sys
import time

CAT = sys.argv[3] if len(sys.argv) > 3 else 'D:/A380/msfs-a380/install/out/EFB/catalogue.json'
export = json.load(open(sys.argv[1], encoding='utf-8'))
prefixes = tuple(sys.argv[2].split(','))


def ours(component_id):
    return component_id.startswith(prefixes)


cat = json.load(open(CAT, encoding='utf-8'))
shutil.copy(CAT, f'{CAT}.bak-{time.strftime("%Y%m%d-%H%M%S")}')
chapters = {f['ataChapterNumber']: f['ataChapterName'] for f in cat['failures'] if 'ataChapterNumber' in f}

reg_f = {f['id']: f for f in export['failures'] if ours(f['component'])}
by_id = {f['id']: f for f in cat['failures']}
added = updated = 0
for i, r in reg_f.items():
    entry = {'ataChapterName': chapters.get(r['ata'], str(r['ata'])), 'ataChapterNumber': r['ata'], 'cause': r['effect'], 'component': r['component'], 'id': i, 'name': r['name']}
    if i in by_id:
        if any(by_id[i].get(k) != v for k, v in entry.items()):
            by_id[i].update(entry)
            updated += 1
    else:
        cat['failures'].append(entry)
        added += 1
stale = {i for i, f in by_id.items() if ours(f.get('component', '')) and i not in reg_f}
cat['failures'] = sorted((f for f in cat['failures'] if f['id'] not in stale), key=lambda f: f['id'])

reg_c = {c['id']: c for c in export['components'] if ours(c['id'])}
comps = {c['id']: c for c in cat['components']}
c_added = c_updated = 0
for cid, r in reg_c.items():
    entry = {'ataChapterName': chapters.get(r['ata'], str(r['ata'])), 'ataChapterNumber': r['ata'], 'failures': r['failures'], 'id': cid, 'name': r['name'], 'parameters': r['parameters']}
    if cid in comps:
        old = comps[cid]
        if any(old.get(k) != v for k, v in entry.items()):
            old.update(entry)
            c_updated += 1
    else:
        entry['instance'] = None
        cat['components'].append(entry)
        c_added += 1
c_stale = {cid for cid in comps if ours(cid) and cid not in reg_c}
cat['components'] = [c for c in cat['components'] if c['id'] not in c_stale]
cat['generatedAtUnixSeconds'] = int(time.time())
json.dump(cat, open(CAT, 'w', encoding='utf-8'), separators=(',', ':'))
print(f'failures: {added} added, {updated} updated, {len(stale)} removed; components: {c_added} added, {c_updated} updated, {len(c_stale)} removed; '
      f'catalogue now {len(cat["failures"])} failures, {len(cat["components"])} components')
