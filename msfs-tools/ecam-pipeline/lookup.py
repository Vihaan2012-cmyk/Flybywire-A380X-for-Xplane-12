import json
import re
import sys

sys.path.insert(0, 'E:/fix-wave3')
import fbw_catalog
import fcom_titles

REPO = 'D:/A380/fbw-xp-worktrees/fs2020-672384b'
ps = json.load(open('D:/A380/fbw-build/wasm-fs2020/ecam-msfs/patches-msfs.json', encoding='utf-8'))
js = ''.join(p['replace'][len(p['find']):] for p in ps if 'bridge' in p['reason'])
BRIDGE = set(re.findall(r'\{id:(\d{9}),', js))
TS = set(re.findall(r'\n    (\d{9}): ', open(REPO + '/fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/FwsAbnormalSensed.ts', encoding='utf-8').read()))


def level(f):
    im = ' '.join(f.get('images') or [])
    if '54437' in im or '54438' in im:
        return '3 (MASTER WARN)'
    if '54424' in im or '54425' in im:
        return '2 (MASTER CAUT)'
    return 'no master light (1)'


def search(words):
    words = [w.upper() for w in words]
    for a in fcom_titles.FA:
        if all(w in a['title'].upper() for w in words):
            print(f"  FCOM: {a['title']} | p.{a['page']} | phases {a['phases']} | level {level(a)} | {' '.join(a['triggering'])[:200]}")


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == '--search':
        search(sys.argv[2:])
        sys.exit()
    for title in sys.argv[1:]:
        print(title)
        f = fcom_titles.lookup(title)
        if f:
            print(f"  FCOM: {f['title']} | p.{f['page']} | phases {f['phases']} | level {level(f)}")
            print(f"  FCOM trigger: {' '.join(f['triggering'])[:300]}")
        else:
            print('  FCOM: no procedure with this concrete title (try --search WORD WORD)')
        ids = fbw_catalog.BY_TITLE.get(title.upper().strip(), [])
        if not ids:
            print('  FlyByWire catalogue: no id with exactly this title')
        for i in ids:
            where = 'implemented in the deep ECAM bridge (FbwProc)' if i in BRIDGE else ('implemented in TS (FwsAbnormalSensed.ts)' if i in TS else 'NOT implemented anywhere')
            print(f'  FlyByWire id {i}: {where}')
