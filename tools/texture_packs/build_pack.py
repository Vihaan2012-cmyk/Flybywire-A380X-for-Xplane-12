import json
import shutil
import struct
import subprocess
import sys
from pathlib import Path

INSTALLED = Path(r'D:\Steam Games\steamapps\common\X-Plane 12\Aircraft\A380 for X-Plane 12')
ROOT = Path(r'E:\fbw-texpacks')
BAKES = Path(r'E:\fbw-gse\decalbake\work')
TEXPIPE = r'D:\A380\fbw-build\target\release\examples\texpipe.exe'


def size(p: Path):
    with open(p, 'rb') as f:
        h = f.read(24)
    if h[:4] == b'DDS ':
        hgt, wid = struct.unpack('<II', h[12:20])
        return wid, hgt
    if h[:4] == b'\x89PNG':
        return struct.unpack('>II', h[16:24])
    raise ValueError(p)


def pristine_set() -> set[str]:
    same = set(json.loads((ROOT / 'compare-base.json').read_text())['same'])
    similar = json.loads((ROOT / 'similar-base.json').read_text())
    close = {rel for rel, r in similar.items() if 'error' not in r and r['frac10'] <= 0.05 and r['p999'] <= 45}
    return same | close


def main():
    label = sys.argv[1]
    conv = ROOT / f'conv-{label}' / 'A380 for X-Plane 12'
    pack = ROOT / f'pack-{label}'
    if pack.exists():
        shutil.rmtree(pack)
    pristine = pristine_set()
    taken, baked, kept_edit, not_larger, missing = [], [], [], 0, 0
    for p in sorted(conv.rglob('*')):
        if not p.is_file() or p.suffix.lower() not in ('.dds', '.png'):
            continue
        rel = p.relative_to(conv).as_posix()
        if not rel.startswith(('objects/', 'liveries/')):
            continue
        inst = INSTALLED / rel
        if not inst.is_file():
            missing += 1
            continue
        have = max(size(inst))
        dest = pack / rel
        if rel in pristine:
            if max(size(p)) <= have:
                not_larger += 1
                continue
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, dest)
            taken.append((rel, have, max(size(p))))
            continue
        bake = BAKES / f'{inst.stem}_baked.png'
        if bake.is_file() and inst.suffix.lower() == '.dds' and max(size(bake)) > have:
            dest.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run([TEXPIPE, 'dds', str(bake), str(dest)], check=True, capture_output=True)
            baked.append((rel, have, max(size(dest))))
            continue
        kept_edit.append(rel)
    total = sum(f.stat().st_size for f in pack.rglob('*') if f.is_file()) if pack.exists() else 0
    (ROOT / f'pack-{label}.json').write_text(json.dumps({'taken': taken, 'baked': baked, 'kept_edit': kept_edit}, indent=1))
    print(f'{label}: {len(taken)} upgraded, {len(baked)} baked panels, {len(kept_edit)} edited kept, {not_larger} not larger, {missing} not in the install, {total / 1e6:.0f} MB')
    for rel, a, b in baked:
        print('  baked', rel, a, '->', b)


main()
