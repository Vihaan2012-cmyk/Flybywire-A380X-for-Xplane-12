import json
import re
import sys

sys.path.insert(0, 'E:/fix-wave3')
import fbw_catalog as c
import fcom_titles

ps = json.load(open('D:/A380/fbw-build/wasm-fs2020/ecam-msfs/patches-msfs.json', encoding='utf-8'))
js = ''.join(p['replace'][len(p['find']):] for p in ps if 'bridge' in p['reason'])
bridge_ids = set(re.findall(r'\{id:(\d{9}),', js))
ts_ids = set(re.findall(r'\n    (\d{9}): ', open('D:/A380/fbw-xp-worktrees/fs2020-672384b/fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/FwsAbnormalSensed.ts', encoding='utf-8').read()))
titles = sorted(set(re.findall(r"\d{10}:\{title:'([^']*)'", js)))


def level(f):
    im = ' '.join(f.get('images') or [])
    if '54437' in im or '54438' in im:
        return 3
    if '54424' in im or '54425' in im:
        return 2
    return None


out = {}
for t in titles:
    f = fcom_titles.lookup(t)
    if not f or f['phases'] is None:
        continue
    entry = {'phases': f['phases'], 'fcom_title': f['title'], 'page': f['page']}
    lv = level(f)
    if lv:
        entry['level'] = lv
    ids = [i for i in c.BY_TITLE.get(t.upper(), []) if i in bridge_ids or i in ts_ids]
    if ids:
        entry['suppressed_by'] = [int(i) for i in ids]
    out[t] = entry
json.dump(out, open('D:/A380/msfs-a380/wiring/registry-fcom.json', 'w', encoding='utf-8'), indent=1)
print(len(out), 'registry titles mapped to the FCOM;', sum(1 for v in out.values() if 'suppressed_by' in v), 'suppressed by an implemented FlyByWire alert')
