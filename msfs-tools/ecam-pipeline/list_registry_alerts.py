import json
import re

W = 'D:/A380/msfs-a380/wiring/'
p = json.load(open(W + 'ecam-patches-live.json', encoding='utf-8'))
fc = json.load(open(W + 'registry-fcom.json', encoding='utf-8'))
s = ''.join(e['replace'] for e in p)
seen = {}
for m in re.finditer(r"(10\d{8}):\{title:'((?:[^'\\]|\\.)*)'", s):
    seen[m.group(1)] = m.group(2)
for k, t in sorted(seen.items()):
    print(k, 'FCOM' if t in fc else '----', t)
