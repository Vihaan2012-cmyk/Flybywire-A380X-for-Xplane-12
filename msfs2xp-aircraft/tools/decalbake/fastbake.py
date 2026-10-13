import collections
import json
import multiprocessing as mp
import os
import pickle
import re
import subprocess
import sys
import time

import numpy as np
from PIL import Image

import bake
from bake import (CELL, DECALS, MAX_GAP, MIN_DOT, NOT_PANELS, OVERLAP_RES, SKIP_TEXTURE_PREFIX, area2, bary2, bary3, basis, ccw,
                  clip_area, composite, edge_groups, pair_groups, grid, light_canon, obj_path, overlap_count, rebuild_base, run, sample, signed_area, uv_px)
from objparse import triangles

OFF = 1 << 20
W = {}


def pack(x, y, z):
    return ((x + OFF) << 42) | ((y + OFF) << 21) | (z + OFF)


def parse_one(args):
    objdir, n = args
    return n, triangles(obj_path(objdir, n + '.obj'), n)


def overlap_one(args):
    shared, tid, uvs_path = args
    out = os.path.join(shared, f'ov_{tid}.npy')
    if not os.path.exists(out):
        np.save(out, overlap_count(uv_px(np.load(uvs_path), OVERLAP_RES)).astype(np.int16))
    return tid


def init_match(shared, objdir, main_lua):
    bake.CANON.update(light_canon(main_lua))
    for k in ('P_pos', 'P_n', 'P_uv', 'P_sig', 'P_tex', 'P_obj', 'P_batch', 'P_k', 'keys', 'starts', 'idx', 'alpha'):
        W[k] = np.load(os.path.join(shared, k + '.npy'), mmap_mode='r')
    meta = pickle.load(open(os.path.join(shared, 'meta.pkl'), 'rb'))
    W.update(meta)
    W['shared'] = shared
    W['decals'] = {dn: triangles(obj_path(objdir, dn + '.obj'), dn)[1] for dn in DECALS}
    W['ov'] = {}


def cells_for(lo, hi):
    keys, starts, idx = W['keys'], W['starts'], W['idx']
    want = np.array([pack(x, y, z) for x in range(lo[0], hi[0] + 1) for y in range(lo[1], hi[1] + 1) for z in range(lo[2], hi[2] + 1)], np.int64)
    pos = np.searchsorted(keys, want)
    found = []
    for p, w in zip(pos, want):
        if p < len(keys) and keys[p] == w:
            found.append(np.asarray(idx[starts[p]:starts[p + 1]]))
    return found


def overlap(tid):
    if tid not in W['ov']:
        W['ov'][tid] = np.load(os.path.join(W['shared'], f'ov_{tid}.npy'), mmap_mode='r')
    return W['ov'][tid]


def match(task):
    P_pos, P_n, P_uv, P_sig, P_tex = W['P_pos'], W['P_n'], W['P_uv'], W['P_sig'], W['P_tex']
    alpha = W['alpha']
    report = collections.Counter()
    accepted, offedge, shared_rejects, removed, bad = [], [], [], [], []
    for dn, bi in task:
        b = W['decals'][dn][bi]
        group = bake.bake_group(b['light'])
        moving = b['animated'] or b['conditional']
        sid = W['sig_ids'].get(b['sig'] if moving else ())
        if sid is None:
            report[f'{dn} kept: moving, no part moves the same'] += len(b['pos'])
            continue
        tag = ' (moving)' if moving else ''
        grp = edge_groups(b['pos'])
        pair = pair_groups(b['pos'])
        for k in range(len(b['pos'])):
            D, Duv, nd = b['pos'][k], b['uv'][k], b['normal'][k]
            if not np.isfinite(nd).all() or np.linalg.norm(nd) < 0.5:
                report[f'{dn}{tag} kept: degenerate'] += 1
                continue
            lo = np.floor(D.min(axis=0) / CELL).astype(np.int64)
            hi = np.floor(D.max(axis=0) / CELL).astype(np.int64)
            found = cells_for(lo, hi)
            if not found:
                report[f'{dn}{tag} kept: no panel'] += 1
                bad.append((dn, b['batch'], pair[k]))
                continue
            cand = np.unique(np.concatenate(found))
            cand = cand[(P_n[cand] @ nd >= MIN_DOT) & (P_sig[cand] == sid)]
            B = basis(nd)
            d2raw = D @ B.T
            d2 = ccw(d2raw)
            d_area = signed_area(d2)
            if d_area <= 1e-12:
                report[f'{dn}{tag} kept: degenerate'] += 1
                continue
            c2 = P_pos[cand] @ B.T
            dlo, dhi = d2.min(axis=0), d2.max(axis=0)
            cand = cand[((c2.max(axis=1) >= dlo) & (c2.min(axis=1) <= dhi)).all(axis=1)]
            near = [i for i in cand.tolist() if clip_area(d2, ccw(P_pos[i] @ B.T)) > 1e-6 * d_area]
            if not near:
                report[f'{dn}{tag} kept: no panel'] += 1
                bad.append((dn, b['batch'], pair[k]))
                continue
            n_div = int(np.clip(np.ceil(np.linalg.norm(D - np.roll(D, 1, axis=0), axis=1).max() / 0.0001), 24, 240))
            g = grid(n_div)
            pts = g @ d2raw
            pts3 = g @ D
            ink = sample(alpha, g @ uv_px(Duv, alpha.shape[1]))[:, 0] > 0.03
            if not ink.any():
                removed.append([dn, b['batch'], k])
                report[f'{dn}{tag} removed: no ink'] += 1
                continue
            best = np.full(len(pts), np.inf)
            owner = np.full(len(pts), -1)
            for i in near:
                l = bary2(pts, ccw(P_pos[i] @ B.T))
                if l is None:
                    continue
                t = np.abs((P_pos[i][0] - pts3) @ P_n[i] / float(nd @ P_n[i]))
                upd = (l >= -1e-6).all(axis=1) & (t <= MAX_GAP) & (t < best)
                best[upd] = t[upd]
                owner[upd] = i
            loose = ink & (owner < 0)
            if loose.any():
                frac = 1 - loose.sum() / ink.sum()
                report[f'{dn}{tag} kept: off panel edge, {int(frac * 10) * 10}%+ on panel'] += 1
                offedge.append([dn, b['batch'], k, D.tolist(), nd.tolist(), float(frac)])
                bad.append((dn, b['batch'], pair[k]))
                continue
            pairs = sorted(set(owner[ink].tolist()))
            if any(float(P_n[i] @ nd) < 0.9 for i in pairs):
                report[f'{dn}{tag} wraps onto a bevel'] += 1
            shared = False
            maps = collections.defaultdict(list)
            for i in pairs:
                tid = int(P_tex[i])
                ov = overlap(tid)
                a3, b3, c3 = np.asarray(P_pos[i])
                pn = np.asarray(P_n[i])
                lam = [bary3(p + ((a3 - p) @ pn) / float(nd @ pn) * nd, a3, b3, c3) for p in D]
                if any(l is None for l in lam):
                    shared = True
                    break
                lam = np.array(lam)
                pp = uv_px(np.asarray(P_uv[i]), OVERLAP_RES)
                dp = lam @ pp
                lo = np.clip(np.floor(np.maximum(dp.min(axis=0), pp.min(axis=0))).astype(int) - 1, 0, OVERLAP_RES - 1)
                hi = np.clip(np.ceil(np.minimum(dp.max(axis=0), pp.max(axis=0))).astype(int) + 1, 0, OVERLAP_RES)
                if (hi - lo).min() > 0:
                    ys, xs = np.mgrid[lo[1]:hi[1], lo[0]:hi[0]]
                    c = np.stack([xs.ravel() + 0.5, ys.ravel() + 0.5], axis=1)
                    lP, lD = bary2(c, pp), bary2(c, dp)
                    if lP is not None and lD is not None:
                        opp = np.linalg.norm(np.roll(pp, -1, axis=0) - np.roll(pp, -2, axis=0), axis=1)
                        near_tex = (lP * area2(pp) / np.maximum(opp, 1e-9) >= -1.5).all(axis=1) & (lD >= -0.05).all(axis=1)
                        if near_tex.any():
                            a_at = sample(alpha, lD[near_tex] @ uv_px(Duv, alpha.shape[1]))[:, 0]
                            hot = near_tex.copy()
                            hot[near_tex] = a_at > 0.03
                            if (ov[ys.ravel()[hot], xs.ravel()[hot]] > 1).any():
                                shared = True
                                break
                maps[W['tex_names'][tid]].append((i, lam))
            if shared:
                report[f'{dn}{tag} kept: shared texture area'] += 1
                shared_rejects.append(dict(dn=dn, batch=b['batch'], k=k, faces=[[W['obj_names'][int(W['P_obj'][i])], int(W['P_batch'][i]), int(W['P_k'][i]), W['tex_names'][int(P_tex[i])]] for i in pairs]))
                bad.append((dn, b['batch'], pair[k]))
                continue
            accepted.append(dict(Duv=Duv, dn=dn, batch=b['batch'], k=k, group=group, maps=dict(maps), grp=(dn, b['batch'], grp[k]), pair=(dn, b['batch'], pair[k]), tag=tag))
    return accepted, report, offedge, shared_rejects, removed, bad


def init_tex(shared, objdir, work, stage):
    W['P_uv'] = np.load(os.path.join(shared, 'P_uv.npy'), mmap_mode='r')
    W['atlases'] = {t: np.load(os.path.join(shared, f'atlas_{t}.npy'), mmap_mode='r') for t in pickle.load(open(os.path.join(shared, 'atlas_names.pkl'), 'rb'))}
    W['emis'] = np.load(os.path.join(shared, 'emis.npy'), mmap_mode='r')
    W.update(objdir=objdir, work=work, stage=stage)


def bake_tex(args):
    tex, items = args
    stem = os.path.splitext(tex)[0]
    base, best = rebuild_base(W['objdir'], W['work'], tex)
    if base is None:
        return tex, None
    rgba, glow, _ = composite(base, items, W['P_uv'], W['atlases'], W['emis'])
    out_png = os.path.join(W['work'], f'{stem}_baked.png')
    Image.fromarray((rgba * 255 + 0.5).astype(np.uint8), 'RGBA').save(out_png)
    run('dds', out_png, os.path.join(W['stage'], tex))
    lit = None
    if glow.max() > 1e-3:
        lit = f'{stem}_TXTLIT.dds'
        lit_png = os.path.join(W['work'], f'{stem}_TXTLIT.png')
        Image.fromarray((np.clip(glow, 0, 1) * 255 + 0.5).astype(np.uint8), 'RGB').save(lit_png)
        run('dds', lit_png, os.path.join(W['stage'], lit))
    return tex, dict(lit=lit, ao=best[1], parity=round(best[0], 2), decals=len(items))


def main(objdir, acf_path, work, stage, workers='12'):
    workers = int(workers)
    t0 = time.time()
    os.makedirs(work, exist_ok=True)
    os.makedirs(stage, exist_ok=True)
    shared = os.path.join(work, 'shared')
    os.makedirs(shared, exist_ok=True)
    main_lua = os.path.join(objdir, '..', 'plugins', 'sasl', 'data', 'modules', 'main.lua')
    bake.CANON.update(light_canon(main_lua))
    acf = open(acf_path, encoding='utf-8', errors='replace').read()
    names = [os.path.splitext(f)[0] for f in re.findall(r'_v10_att_file_stl (\S+)', acf)]
    cockpit = [n for n in names if n.startswith('a380_cockpit_') and n[13:].isdigit()]
    with mp.Pool(workers) as pool:
        loaded = dict(pool.map(parse_one, [(objdir, n) for n in cockpit]))
    print(f'parsed {len(loaded)} objects in {time.time() - t0:.0f}s', flush=True)

    atlases, emis = bake.load_atlases(work)
    for t, a in atlases.items():
        np.save(os.path.join(shared, f'atlas_{t}.npy'), a)
    pickle.dump(list(atlases), open(os.path.join(shared, 'atlas_names.pkl'), 'wb'))
    np.save(os.path.join(shared, 'emis.npy'), emis)
    np.save(os.path.join(shared, 'alpha.npy'), np.ascontiguousarray(atlases['-'][..., 3:4]))

    tex_names, tex_id = [], {}
    obj_names = list(loaded)
    obj_id = {n: i for i, n in enumerate(obj_names)}
    sig_ids = {(): 0}
    cols = collections.defaultdict(list)
    by_texture = collections.defaultdict(list)
    for n, (h, batches) in loaded.items():
        if h['texture']:
            by_texture[h['texture']].append(n)
        tex = h['texture'] or ''
        num = int(n[13:])
        if n in NOT_PANELS or tex.startswith(SKIP_TEXTURE_PREFIX) or num >= 97 or num in (4, 8, 9, 10):
            continue
        tid = tex_id.setdefault(tex, len(tex_names))
        if tid == len(tex_names):
            tex_names.append(tex)
        for b in batches:
            s = sig_ids.setdefault(b['sig'] if (b['animated'] or b['conditional']) else (), len(sig_ids))
            m = len(b['pos'])
            cols['P_pos'].append(b['pos'])
            cols['P_uv'].append(b['uv'])
            cols['P_sig'].append(np.full(m, s, np.int64))
            cols['P_tex'].append(np.full(m, tid, np.int32))
            cols['P_obj'].append(np.full(m, obj_id[n], np.int32))
            cols['P_batch'].append(np.full(m, b['batch'], np.int32))
            cols['P_k'].append(np.arange(m, dtype=np.int32))
    arr = {k: np.concatenate(v) for k, v in cols.items()}
    P = arr['P_pos']
    nrm = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    arr['P_n'] = nrm / np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
    lo = np.floor((P.min(axis=1) - 0.007) / CELL).astype(np.int64)
    hi = np.floor((P.max(axis=1) + 0.007) / CELL).astype(np.int64)
    ext = hi - lo + 1
    cnt = ext.prod(axis=1)
    tri = np.repeat(np.arange(len(P)), cnt)
    local = np.arange(cnt.sum()) - np.repeat(np.cumsum(cnt) - cnt, cnt)
    ex = ext[tri]
    dx = local % ex[:, 0]
    dy = (local // ex[:, 0]) % ex[:, 1]
    dz = local // (ex[:, 0] * ex[:, 1])
    key = pack(lo[tri, 0] + dx, lo[tri, 1] + dy, lo[tri, 2] + dz)
    order = np.argsort(key, kind='stable')
    key = key[order]
    arr['keys'], first = np.unique(key, return_index=True)
    arr['starts'] = np.r_[first, len(key)].astype(np.int64)
    arr['idx'] = tri[order].astype(np.int64)
    for k, v in arr.items():
        np.save(os.path.join(shared, k + '.npy'), v)
    pickle.dump(dict(sig_ids=sig_ids, tex_names=tex_names, obj_names=obj_names), open(os.path.join(shared, 'meta.pkl'), 'wb'))
    print(f'{len(P)} panel triangles, {len(arr["keys"])} cells, prepared in {time.time() - t0:.0f}s', flush=True)

    for f in os.listdir(shared):
        if f.startswith('ov_'):
            os.remove(os.path.join(shared, f))
    ov_jobs = []
    for tid, tex in enumerate(tex_names):
        p = os.path.join(shared, f'uvs_{tid}.npy')
        np.save(p, np.concatenate([bt['uv'] for n in by_texture[tex] for bt in loaded[n][1]]))
        ov_jobs.append((shared, tid, p))

    tasks = []
    report = collections.Counter()
    for dn in DECALS:
        for b in loaded[dn][1]:
            if bake.bake_group(b['light']) is None:
                report[f'{dn} kept: own light'] += len(b['pos'])
                continue
            tasks.append(((dn, b['batch']), len(b['pos'])))
    tasks.sort(key=lambda t: -t[1])
    n_chunks = workers * 6
    chunks = [[] for _ in range(n_chunks)]
    load = [0] * n_chunks
    for t, c in tasks:
        j = load.index(min(load))
        chunks[j].append(t)
        load[j] += c
    with mp.Pool(workers) as pool:
        pool.map(overlap_one, ov_jobs)
    print(f'overlap maps in {time.time() - t0:.0f}s', flush=True)
    with mp.Pool(workers, initializer=init_match, initargs=(shared, objdir, main_lua)) as pool:
        results = pool.map(match, [c for c in chunks if c])
    print(f'matched in {time.time() - t0:.0f}s', flush=True)

    accepted, offedge, shared_rejects = [], [], []
    baked = collections.defaultdict(list)
    bad = set()
    for acc, rep, off, sh, rem, bd in results:
        bad.update(bd)
        accepted += acc
        report.update(rep)
        offedge += off
        shared_rejects += sh
        for dn, b, k in rem:
            baked[dn].append([b, k, []])
    P_obj, P_batch, P_k = arr['P_obj'], arr['P_batch'], arr['P_k']

    def drop_bad(acc):
        keep = []
        for a in acc:
            if a['pair'] in bad:
                report[f"{a['dn']}{a['tag']} kept: other half of a quad that stays a decal"] += 1
            else:
                keep.append(a)
        return keep
    accepted = drop_bad(accepted)
    while True:
        on_tri = collections.defaultdict(collections.Counter)
        for a in accepted:
            if a['group'] != '(unlit)':
                for m in a['maps'].values():
                    for i, _ in m:
                        on_tri[i][a['group']] += 1
        losers = set()
        for i, c in on_tri.items():
            if len(c) > 1:
                win = c.most_common(1)[0][0]
                losers.update((i, g) for g in c if g != win)
        if not losers:
            break
        for a in accepted:
            if any((i, a['group']) in losers for m in a['maps'].values() for i, _ in m):
                report[f"{a['dn']}{a['tag']} kept: another light on the same face"] += 1
                bad.add(a['pair'])
        accepted = drop_bad(accepted)
    jobs = collections.defaultdict(list)
    glow_objects = collections.defaultdict(dict)
    for a in accepted:
        for tex, m in a['maps'].items():
            jobs[tex].append(dict(Duv=a['Duv'], dn=a['dn'], group=a['group'], maps=m, grp=a['grp']))
            if a['group'] != '(unlit)':
                for i, _ in m:
                    glow_objects[obj_names[P_obj[i]]][(int(P_batch[i]), int(P_k[i]))] = (a['group'], tex)
        baked[a['dn']].append([a['batch'], a['k'], sorted(a['maps'])])
        report[f"{a['dn']} baked{a['tag']}" + (' across textures' if len(a['maps']) > 1 else '')] += 1
    for k_, v in sorted(report.items()):
        print(f'  {k_}: {v}', flush=True)
    json.dump(offedge, open(os.path.join(work, 'offedge.json'), 'w'))
    json.dump(shared_rejects, open(os.path.join(work, 'shared.json'), 'w'))
    pickle.dump((jobs, baked, glow_objects, arr['P_uv']), open(os.path.join(work, 'jobs.pkl'), 'wb'))

    order = sorted(jobs.items(), key=lambda kv: -len(kv[1]))
    with mp.Pool(min(workers, 8), initializer=init_tex, initargs=(shared, objdir, work, stage)) as pool:
        out = pool.map(bake_tex, order, chunksize=1)
    results = {}
    for tex, r in out:
        if r is None:
            print(f'{tex}: no recipe matched the install, skipped', flush=True)
            continue
        results[tex] = r
        print(f'{tex}: {r["decals"]} decal triangles, parity {r["parity"]}, glow {bool(r["lit"])}', flush=True)
    glow = {o: [[b, k, g, t_] for (b, k), (g, t_) in sorted(t.items())] for o, t in glow_objects.items()}
    json.dump(dict(results=results, baked=baked, glow_tris=glow), open(os.path.join(work, 'bake_result.json'), 'w'), indent=1)
    print(f'total {time.time() - t0:.0f}s', flush=True)


if __name__ == '__main__':
    main(*sys.argv[1:6])
