import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

INSTALLED = Path(r'D:\Steam Games\steamapps\common\X-Plane 12\Aircraft\A380 for X-Plane 12')
ROOT = Path(r'E:\fbw-texpacks')


def load(p: Path) -> np.ndarray:
    im = Image.open(p)
    im.load()
    return np.asarray(im.convert('RGBA'), dtype=np.float32)


def compare(installed: Path, converted: Path) -> dict:
    a = load(installed)
    b_img = Image.open(converted)
    b_img.load()
    b_img = b_img.convert('RGBA')
    if b_img.size != (a.shape[1], a.shape[0]):
        b_img = b_img.resize((a.shape[1], a.shape[0]), Image.BOX)
    b = np.asarray(b_img, dtype=np.float32)
    d = np.abs(a - b)[..., :3].max(axis=2)
    return {'mean': round(float(d.mean()), 2), 'p999': round(float(np.percentile(d, 99.9)), 1), 'frac10': round(float((d > 10).mean()), 4)}


def main():
    name = sys.argv[1]
    report = json.loads((ROOT / f'compare-{name}.json').read_text())
    conv = ROOT / f'conv-{name}' / 'A380 for X-Plane 12'
    out = {}
    for rel, a_size, b_size in report['edited']:
        try:
            out[rel] = {'installed': a_size, 'converted': b_size, **compare(INSTALLED / rel, conv / rel)}
        except Exception as e:
            out[rel] = {'installed': a_size, 'converted': b_size, 'error': str(e)}
    (ROOT / f'similar-{name}.json').write_text(json.dumps(out, indent=1))
    for rel, r in sorted(out.items(), key=lambda kv: -kv[1].get('frac10', 9)):
        print(f"{rel:62} {str(r['installed']):13} {str(r['converted']):13} {r.get('mean')} {r.get('p999')} {r.get('frac10')} {r.get('error', '')}")


main()
