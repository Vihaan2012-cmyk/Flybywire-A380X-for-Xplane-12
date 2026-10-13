import difflib
import json
import os
import re
import sys

import numpy as np
from PIL import Image

ORIGIN = re.compile(r'http://127\.0\.0\.1:\d+')
BUNDLE_LINE = re.compile(r'(\.js):\d+(:\d+)?')


def dom(path):
    return ORIGIN.sub('ORIGIN', open(path, encoding='utf-8').read()) if os.path.exists(path) else None


def first_dom_diffs(a, b, limit=6):
    ta, tb = re.split(r'(?=<)', a), re.split(r'(?=<)', b)
    out = []
    for op, i1, i2, j1, j2 in difflib.SequenceMatcher(None, ta, tb, autojunk=False).get_opcodes():
        if op != 'equal':
            out.append({'op': op, 'golden': ''.join(ta[i1:i2])[:400], 'ours': ''.join(tb[j1:j2])[:400]})
            if len(out) >= limit:
                break
    return out


def keyset(entries):
    return {e['key'] for e in entries}


def compare_screen(g, o, state, screen):
    r = {'state': state, 'screen': screen}
    gp, op = os.path.join(g, state, f'{screen}.png'), os.path.join(o, state, f'{screen}.png')
    if not os.path.exists(op):
        r['missing'] = True
        return r
    ga, oa = np.asarray(Image.open(gp).convert('RGB')).astype(int), np.asarray(Image.open(op).convert('RGB')).astype(int)
    r['png_diff_pixels'] = int((np.abs(ga - oa).sum(axis=2) > 0).sum()) if ga.shape == oa.shape else -1
    gd, od = dom(os.path.join(g, state, f'{screen}.dom.html')), dom(os.path.join(o, state, f'{screen}.dom.html'))
    r['dom_equal'] = gd == od
    if not r['dom_equal'] and gd is not None and od is not None:
        r['dom_diffs'] = first_dom_diffs(gd, od)
    ga_, oa_ = json.load(open(os.path.join(g, state, f'{screen}.access.json'))), json.load(open(os.path.join(o, state, f'{screen}.access.json')))
    gw = {e['key']: e.get('last') for e in ga_['writes']}
    ow = {e['key']: e.get('last') for e in oa_['writes']}
    r['writes_diff'] = {k: [gw.get(k), ow.get(k)] for k in sorted(set(gw) | set(ow)) if gw.get(k) != ow.get(k)}
    for kind in ('reads', 'coherentCalls', 'coherentTriggers', 'coherentOn', 'keyEvents', 'viewListeners', 'gameVars'):
        a, b = keyset(ga_[kind]), keyset(oa_[kind])
        if a != b:
            r[f'{kind}_diff'] = {'only_golden': sorted(a - b)[:30], 'only_ours': sorted(b - a)[:30]}
    norm = lambda e: BUNDLE_LINE.sub(r'\1', ORIGIN.sub('ORIGIN', json.dumps(e, sort_keys=True)))
    gerr = {norm(e)[:300] for e in ga_['errors']}
    oerr = [norm(e)[:600] for e in oa_['errors'] if norm(e)[:300] not in gerr]
    if oerr:
        r['new_errors'] = oerr[:10]
    r['identical'] = r['png_diff_pixels'] == 0 and r['dom_equal'] and not r['writes_diff'] and not any(k.endswith('_diff') and k != 'writes_diff' for k in r) and not oerr
    return r


def main(golden, ours, report):
    noisy = set()
    gb = golden.rstrip('/').rsplit('-', 1)[0] + '-b'
    if os.path.isdir(gb):
        for st in os.listdir(golden):
            if os.path.isdir(os.path.join(gb, st)):
                for f in os.listdir(os.path.join(golden, st)):
                    if f.endswith('.png') and os.path.exists(os.path.join(gb, st, f)):
                        a = np.asarray(Image.open(os.path.join(golden, st, f)).convert('RGB'))
                        b = np.asarray(Image.open(os.path.join(gb, st, f)).convert('RGB'))
                        if a.shape != b.shape or (a != b).any():
                            noisy.add((st, f[:-4]))
    results = []
    for state in sorted(os.listdir(ours)):
        sd = os.path.join(ours, state)
        if not os.path.isdir(sd) or not os.path.isdir(os.path.join(golden, state)):
            continue
        for f in sorted(os.listdir(sd)):
            if f.endswith('.png') and os.path.exists(os.path.join(golden, state, f)):
                r = compare_screen(golden, ours, state, f[:-4])
                if (state, f[:-4]) in noisy and r.get('png_diff_pixels', 0) > 0 and not r.get('missing'):
                    r['png_noisy_in_golden'] = r.pop('png_diff_pixels')
                    r['png_diff_pixels'] = 0
                    r['identical'] = r['dom_equal'] and not r['writes_diff'] and not any(k.endswith('_diff') and k != 'writes_diff' for k in r) and not r.get('new_errors')
                results.append(r)
    same = sum(1 for r in results if r.get('identical'))
    summary = {'identical': same, 'total': len(results), 'not_identical': [f"{r['state']}/{r['screen']}" for r in results if not r.get('identical')]}
    json.dump({'summary': summary, 'results': results}, open(report, 'w'), indent=1)
    print(f'identical {same}/{len(results)}')
    for r in results:
        if not r.get('identical'):
            bits = []
            if r.get('missing'):
                bits.append('NOT RENDERED')
            else:
                if r.get('png_diff_pixels'):
                    bits.append(f"{r['png_diff_pixels']} px")
                if not r.get('dom_equal'):
                    bits.append('dom')
                if r.get('writes_diff'):
                    bits.append(f"{len(r['writes_diff'])} writes")
                bits += [k[:-5] for k in r if k.endswith('_diff') and k not in ('writes_diff',)]
                if r.get('new_errors'):
                    bits.append(f"{len(r['new_errors'])} new errors")
            print(f"  {r['state']}/{r['screen']}: {', '.join(bits)}")


if __name__ == '__main__':
    main(*sys.argv[1:4])
