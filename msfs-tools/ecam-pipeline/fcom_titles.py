import json, re, itertools

FA = json.load(open('E:/fbw-debug/ecam/fcom_alerts.json', encoding='utf-8'))


def norm(t):
    return re.sub(r'\s+', ' ', t.upper()).strip()


def variants(title):
    t = norm(title)
    base = [t, norm(re.sub(r'\s\([^()]*\s[^()]*\)', '', t))]
    out = set()
    for t in base:
        prev = None
        while prev != t:
            prev = t
            t = re.sub(r'(\S)\s+\(([^()\s]+)\)', r'\1(\2)', t)
        toks = t.split(' ')
        opts = []
        for tok in toks:
            m = re.fullmatch(r'([^()]*)((?:\([^()]*\))+)([^()]*)', tok)
            if m and ' ' not in m.group(2):
                alts = [m.group(1)] + re.findall(r'\(([^()]*)\)', m.group(2))
                opts.append([a + m.group(3) for a in alts])
            else:
                opts.append([tok])
        n = 1
        for o in opts:
            n *= len(o)
        if n > 400:
            out.add(t)
            continue
        for combo in itertools.product(*opts):
            out.add(norm(' '.join(c for c in combo if c)))
    return out


INDEX = {}
for a in FA:
    for v in variants(a['title']):
        INDEX.setdefault(v, a)


def lookup(title):
    t = norm(title)
    return INDEX.get(t) or INDEX.get(norm(re.sub(r'\s\([^()]*\)$', '', t)))
