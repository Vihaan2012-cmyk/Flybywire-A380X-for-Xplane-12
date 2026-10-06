import json, re, glob, sys
sys.path.insert(0, 'E:/fix-wave3')
import fcom_titles

REPO = 'D:/A380/fbw-xp-worktrees/fs2020-672384b'
ps = json.load(open('D:/A380/fbw-build/wasm-fs2020/ecam-msfs/patches-msfs.json', encoding='utf-8'))
bridge_js = ''.join(p['replace'][len(p['find']):] for p in ps if 'bridge' in p['reason'])

bridge = {}
for m in re.finditer(r'\{id:(\d+),failure:(\d),sysPage:(-?\d+),flightPhaseInhib:\[([\d,]*)\]', bridge_js):
    bridge[m.group(1)] = (int(m.group(2)), [int(x) for x in m.group(4).split(',') if x])

titles = {}
esc = re.compile('(?:' + chr(92)*2 + 'x1b|' + chr(27) + ')<?[0-9]*m')
for f in glob.glob(REPO + '/fbw-a380x/src/systems/instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/*.ts'):
    s = open(f, encoding='utf-8').read()
    for m in re.finditer(r"(\d{9}): \{\s*title:\s*'([^']*)'", s):
        titles[m.group(1)] = re.sub(r'\s+', ' ', esc.sub('', m.group(2))).strip()
for m in re.finditer(r"(\d{10}):\{title:'([^']*)'", bridge_js):
    titles[m.group(1)] = m.group(2)

ts = {}
s = open(REPO + '/fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/FwsAbnormalSensed.ts', encoding='utf-8').read()
for m in re.finditer(r'\n    (\d{9}): \{\s*(?://[^\n]*\s*)?flightPhaseInhib: \[([\d,\s]*)\]', s):
    body = s[m.end():m.end() + 3000]
    lv = re.search(r'failure: (\d)', body)
    ts[m.group(1)] = (int(lv.group(1)) if lv else None, [int(x) for x in re.findall(r'\d+', m.group(2))])

rows = []
for i in sorted(set(ts) | set(bridge)):
    src = 'bridge' if i in bridge else 'ts'
    lvl, inh = bridge.get(i) or ts[i]
    t = titles.get(i, '?')
    f = fcom_titles.lookup(t) if t != '?' else None
    if f is None:
        rows.append({'id': i, 'src': src, 'title': t, 'status': 'no_fcom', 'have': inh})
        continue
    want = f['phases']
    st = 'match' if want is None or sorted(want) == sorted(inh) else 'mismatch'
    rows.append({'id': i, 'src': src, 'title': t, 'status': st, 'have': inh, 'fcom': want, 'fcom_title': f['title'], 'page': f['page'], 'level': lvl, 'also_ts': i in ts and i in bridge})
json.dump(rows, open('E:/fix-wave3/inhib_audit.json', 'w'), indent=1)
from collections import Counter
print(Counter((r['src'], r['status']) for r in rows))
for r in rows:
    if r['status'] == 'mismatch' and r['id'] in ('240800061','240800062','701800085','320800039','320800045','211800051','340800053'):
        print(r)
