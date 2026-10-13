import json
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image

INSTALLED = Path(r'D:\Steam Games\steamapps\common\X-Plane 12\Aircraft\A380 for X-Plane 12')
ROOT = Path(r'E:\fbw-texpacks')


def load_at(p: Path, size):
    im = Image.open(p)
    im.load()
    im = im.convert('RGBA')
    if size and im.size != size:
        im = im.resize(size, Image.BOX)
    return np.asarray(im, dtype=np.float32)


def main():
    label = sys.argv[1]
    pack = ROOT / f'pack-{label}'
    rejected = ROOT / f'pack-{label}-rejected'
    report = []
    for p in sorted(pack.rglob('*')):
        if not p.is_file():
            continue
        rel = p.relative_to(pack).as_posix()
        inst = INSTALLED / rel
        a = load_at(inst, None)
        b = load_at(p, (a.shape[1], a.shape[0]))
        d = np.abs(a - b)[..., :3].max(axis=2)
        r = {'rel': rel, 'mean': round(float(d.mean()), 2), 'p999': round(float(np.percentile(d, 99.9)), 1), 'frac10': round(float((d > 10).mean()), 4)}
        report.append(r)
        if r['frac10'] > 0.05 or r['mean'] > 6:
            dest = rejected / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(p), str(dest))
            r['rejected'] = True
    (ROOT / f'verify-{label}.json').write_text(json.dumps(report, indent=1))
    bad = [r for r in report if r.get('rejected')]
    print(f'{label}: {len(report)} checked, {len(bad)} moved out as not matching the installed art')
    for r in sorted(report, key=lambda r: -r['frac10'])[:12]:
        print(f"  {r['rel']:64} mean {r['mean']:6} p999 {r['p999']:6} frac10 {r['frac10']:.4f}{'  REJECTED' if r.get('rejected') else ''}")


main()
