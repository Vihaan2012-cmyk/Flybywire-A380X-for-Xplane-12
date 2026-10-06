import json
import re
import sys

sys.path.insert(0, 'E:/fix-wave3')
import fbw_catalog as c

ps = json.load(open('D:/A380/fbw-build/wasm-fs2020/ecam-msfs/patches-msfs.json', encoding='utf-8'))
js = ''.join(p['replace'][len(p['find']):] for p in ps if 'bridge' in p['reason'])
bridge_ids = set(re.findall(r'\{id:(\d{9}),', js))
ts_ids = set(re.findall(r'\n    (\d{9}): ', open('D:/A380/fbw-xp-worktrees/fs2020-672384b/fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/FwsAbnormalSensed.ts', encoding='utf-8').read()))
syn = {m.group(1): m.group(2) for m in re.finditer(r"(\d{10}):\{title:'([^']*)'", js)}
dup, unimpl = [], []
for sid, title in sorted(syn.items()):
    ids = c.BY_TITLE.get(title.upper())
    if not ids:
        continue
    fid = ids[0]
    where = 'bridge' if fid in bridge_ids else ('ts' if fid in ts_ids else None)
    (dup if where else unimpl).append((sid, title, fid, where))
print('synthetic alerts whose title is a FlyByWire catalogue alert:', len(dup) + len(unimpl))
print('  already implemented under the FlyByWire id (shown twice):', len(dup))
for r in dup:
    print('   ', r)
print('  not implemented under the FlyByWire id:', len(unimpl))
for r in unimpl:
    print('   ', r)
