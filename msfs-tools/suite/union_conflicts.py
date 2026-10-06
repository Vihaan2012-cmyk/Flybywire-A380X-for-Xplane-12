import re
import sys

path = sys.argv[1]
raw = open(path, encoding='utf-8', newline='').read()
nl = '\r\n' if '\r\n' in raw else '\n'
s = raw.replace('\r\n', '\n')
key = re.compile(r'^\s*\("([^"]+)",')
count = 0


def merge(m):
    global count
    count += 1
    ours = m.group(1).splitlines(keepends=True)
    theirs = m.group(2).splitlines(keepends=True)
    seen = {key.match(l).group(1) for l in ours if key.match(l)}
    out = ours[:]
    for l in theirs:
        k = key.match(l)
        if k and k.group(1) in seen:
            continue
        if k:
            seen.add(k.group(1))
        out.append(l)
    return ''.join(out)


s = re.sub(r'^<<<<<<< [^\n]*\n(.*?)^=======\n(.*?)^>>>>>>> [^\n]*\n', merge, s, flags=re.S | re.M)
assert '<<<<<<<' not in s and '>>>>>>>' not in s
ids = [key.match(l).group(1) for l in s.split('\n') if key.match(l)]
dups = sorted({i for i in ids if ids.count(i) > 1})
open(path, 'w', encoding='utf-8', newline='').write(s.replace('\n', nl) if nl == '\r\n' else s)
print(f'{count} hunks merged; duplicate subjects in file: {dups[:20]}')
