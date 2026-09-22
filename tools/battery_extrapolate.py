"""Extrapolate the whole combination space from the battery's tier data.

Model: a combination's reach is its members' reach, plus what each PAIR inside it
adds, minus what each pair masks -- all learned from tier 1 (singles) and tier 2
(pairs) only. Tiers 3, 4 and 5 are never learned from; they are held out and used
to measure the error.

    reach(S) ~= U reach(e)  U  U added(a,b)  \  U removed(a,b)
                e in S        {a,b} in S       {a,b} in S

    added(a,b)   = reach(a,b) \ (reach(a) U reach(b))
    removed(a,b) = (reach(a) U reach(b)) \ reach(a,b)
"""
import io, os, glob, itertools, collections

DUMPS = r'E:/fbw-battery/run-20260921-2220/dumps'

def load_tier(tier):
    """-> {case_index: (preset, frozenset(elements), passed, frozenset(reach))}"""
    plan = {}
    pf = os.path.join(DUMPS, tier, 'plan.txt')
    if not os.path.exists(pf):
        return {}
    with io.open(pf, encoding='utf8', errors='replace') as f:
        for i, line in enumerate(f):
            line = line.rstrip('\n').rstrip('\r')
            if not line:
                continue
            parts = line.split('|')
            if len(parts) < 3:
                continue
            preset = int(parts[0])
            els = frozenset([f'f:{x}' for x in parts[1].split(',') if x]
                            + [f'b:{x}' for x in parts[2].split(',') if x])
            plan[i] = (preset, els)
    out = {}
    for rf in glob.glob(os.path.join(DUMPS, tier, 'reach_*.txt')):
        with io.open(rf, encoding='utf8', errors='replace') as f:
            for line in f:
                line = line.rstrip('\n').rstrip('\r')
                if not line:
                    continue
                bits = line.split('\t')
                if len(bits) < 2:
                    continue
                idx = int(bits[0])
                passed = bits[1] == 'true'
                reach = frozenset(v for v in (bits[2].split(',') if len(bits) > 2 and bits[2] else []) if v)
                if idx in plan:
                    preset, els = plan[idx]
                    out[(tier, idx)] = (preset, els, passed, reach)
    return out

print('loading tiers...')
t1 = load_tier('tier_1')
t2 = {**load_tier('tier_2'), **load_tier('tier_2_split')}
assert len(t2) > 100_000, len(t2)
held = {t: load_tier(t) for t in ('tier_3', 'tier_4', 'tier_5')}
print(f'  tier 1 {len(t1):,}   tier 2 {len(t2):,}   '
      + '   '.join(f'{t} {len(v):,}' for t, v in held.items()))

# Noise: what moves with NO fault armed. tier 1 carries empty cases for this.
noise = {}
for preset, els, passed, reach in t1.values():
    if not els:
        noise[preset] = noise.get(preset, frozenset()) | reach
print('noise per preset: ' + ', '.join(f'{p}:{len(v)}' for p, v in sorted(noise.items())))
def clean(preset, r):
    return frozenset(r) - noise.get(preset, frozenset())

# --- learn from tier 1 + tier 2 only -------------------------------------
single = {}          # (preset, elem) -> reach
for preset, els, passed, reach in t1.values():
    if len(els) == 1:
        single[(preset, next(iter(els)))] = clean(preset, reach)

added, removed = {}, {}   # (preset, a, b) -> frozenset
pairs_learned = 0
for preset, els, passed, reach in t2.values():
    if len(els) != 2:
        continue
    a, b = sorted(els)
    ra, rb = single.get((preset, a)), single.get((preset, b))
    if ra is None or rb is None:
        continue
    u = ra | rb
    rc = clean(preset, reach)
    add, rem = rc - u, u - rc
    if add:
        added[(preset, a, b)] = add
    if rem:
        removed[(preset, a, b)] = rem
    pairs_learned += 1
print(f'learned from {pairs_learned:,} explicit pairs: '
      f'{len(added):,} with additions, {len(removed):,} with masking')

def predict(preset, els):
    r = set()
    for e in els:
        s = single.get((preset, e))
        if s is None:
            return None
        r |= s
    for a, b in itertools.combinations(sorted(els), 2):
        r |= added.get((preset, a, b), frozenset())
    for a, b in itertools.combinations(sorted(els), 2):
        r -= removed.get((preset, a, b), frozenset())
    return r

# --- measure on held-out tiers -------------------------------------------
print('\n=== held-out accuracy (never learned from) ===')
for tier, cases in held.items():
    n = exact = 0
    fp = fn = 0
    union_exact = 0
    jac = []
    for preset, els, passed, reach in cases.values():
        if len(els) < 2:
            continue
        p = predict(preset, els)
        if p is None:
            continue
        reach = clean(preset, reach)
        n += 1
        if p == reach:
            exact += 1
        else:
            fp += len(p - reach)
            fn += len(reach - p)
        # baseline: pure union of singles, no interaction terms
        u = set()
        ok = True
        for e in els:
            s = single.get((preset, e))
            if s is None:
                ok = False
                break
            u |= s
        if ok and u == reach:
            union_exact += 1
        jac.append(len(p & reach) / max(1, len(p | reach)))
    if n:
        print(f'  {tier:8s} n={n:6,}  exact={exact:6,} ({100*exact/n:5.1f}%)   '
              f'pure-union baseline={100*union_exact/n:5.1f}%   '
              f'over={fp:,} under={fn:,}  mean-overlap={100*sum(jac)/len(jac):.1f}%')

# --- what the model now covers -------------------------------------------
print('\n=== reach of the model ===')
per_preset = collections.Counter(p for (p, _) in single)
for p in sorted(per_preset):
    print(f'  preset {p}: {per_preset[p]} elements')
