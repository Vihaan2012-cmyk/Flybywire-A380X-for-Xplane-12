import hashlib
import json
import os
import struct
import sys
from pathlib import Path

INSTALLED = Path(r'D:\Steam Games\steamapps\common\X-Plane 12\Aircraft\A380 for X-Plane 12')
ROOT = Path(r'E:\fbw-texpacks')


def textures(base: Path) -> dict[str, Path]:
    out = {}
    for sub in ('objects', 'liveries'):
        for p in (base / sub).rglob('*'):
            if p.is_file() and p.suffix.lower() in ('.dds', '.png'):
                out[p.relative_to(base).as_posix()] = p
    return out


def size(p: Path):
    with open(p, 'rb') as f:
        h = f.read(24)
    if h[:4] == b'DDS ':
        hgt, wid = struct.unpack('<II', h[12:20])
        return wid, hgt
    if h[:4] == b'\x89PNG':
        return struct.unpack('>II', h[16:24])
    return None


def digest(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    conv = ROOT / f'conv-{sys.argv[1]}' / 'A380 for X-Plane 12'
    installed = textures(INSTALLED)
    converted = textures(conv)
    report = {'same': [], 'edited': [], 'only_converted': [], 'only_installed': []}
    for rel, p in sorted(converted.items()):
        q = installed.get(rel)
        if q is None:
            report['only_converted'].append(rel)
        elif digest(p) == digest(q):
            report['same'].append(rel)
        else:
            report['edited'].append([rel, size(q), size(p)])
    report['only_installed'] = sorted(set(installed) - set(converted))
    (ROOT / f'compare-{sys.argv[1]}.json').write_text(json.dumps(report, indent=1))
    print({k: len(v) for k, v in report.items()})
    for rel, a, b in report['edited'][:60]:
        print('  edited', rel, a, b)


main()
