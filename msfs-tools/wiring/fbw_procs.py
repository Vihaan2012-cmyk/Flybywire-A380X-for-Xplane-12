import re, os, json

ROOT = 'D:/A380/fbw-xp-worktrees/fs2020-672384b/'
base = ROOT + 'fbw-a380x/src/systems/instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/'
procs = {}
pat = re.compile(r"^\s+(\d{9}): \{\s*\n\s*title:\s*('(?:[^'\\]|\\.)*'|\"(?:[^\"\\]|\\.)*\")", re.M)
for f in os.listdir(base):
    s = open(base + f, encoding='utf-8-sig').read()
    for m in pat.finditer(s):
        t = m.group(2)[1:-1]
        t = re.sub(r'\\x1b<\d+m', '', t)
        t = re.sub(r'\\x1b\d*m', '', t)
        procs[int(m.group(1))] = ' '.join(t.split())
fws = open(ROOT + 'fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/FwsAbnormalSensed.ts', encoding='utf-8').read()
wired_fbw = set(int(x) for x in re.findall(r'^\s+(\d{9}): \{', fws, re.M)) & procs.keys()
deep = set()
d = ROOT + 'fbw-a380x/src/wasm/systems/deep_systems/src/deep/ecam/fbw/'
for f in os.listdir(d):
    if f.endswith('.rs'):
        s = open(d + f, encoding='utf-8').read()
        for x in re.findall(r'\b(\d[\d_]{8,12})\b', s):
            v = int(x.replace('_', ''))
            if v in procs:
                deep.add(v)
items = {}
blk = re.compile(r'^  (\d{9}): \{', re.M)
for f in os.listdir(base):
    s = open(base + f, encoding='utf-8-sig').read()
    starts = [(m.start(), int(m.group(1))) for m in blk.finditer(s)]
    for k, (a, i) in enumerate(starts):
        b = starts[k + 1][0] if k + 1 < len(starts) else len(s)
        items[i] = len(re.findall(r'^\s+name:', s[a:b], re.M))
free = {i: t for i, t in sorted(procs.items()) if i not in wired_fbw and i not in deep}
print('defined', len(procs), 'fbw-triggered', len(wired_fbw), 'deep-triggered', len(deep), 'untriggered', len(free))
json.dump({'items': items, 'procs': procs, 'fbw_wired': sorted(wired_fbw), 'deep_wired': sorted(deep), 'free': free},
          open('D:/A380/msfs-a380/wiring/fbw_procs.json', 'w'), indent=0)
