import numpy as np


class Obj:
    def __init__(self, path):
        self.lines = open(path, encoding='latin-1').read().split('\n')
        self.vt = np.array([[float(v) for v in l.split()[1:9]] for l in self.lines if l.startswith('VT ')], dtype=np.float64).reshape(-1, 8)
        self.idx = np.array([int(v) for l in self.lines if l.startswith(('IDX10 ', 'IDX ')) for v in l.split()[1:]], dtype=np.int64)
        self.batches = [(i, int(l.split()[1]), int(l.split()[2])) for i, l in enumerate(self.lines) if l.startswith('TRIS ')]

    def texture(self, kind='TEXTURE'):
        return next((l.split()[1] for l in self.lines if l.split()[:1] == [kind] and len(l.split()) > 1), None)

    def triangles(self):
        return [self.idx[s:s + c].reshape(-1, 3) for _, s, c in self.batches]

    def write(self, path, keep, uv=None, edit=None):
        tris = self.triangles()
        new_vt, seen = [], {}
        new_idx, spans = [], []
        for b, t in enumerate(tris):
            start = len(new_idx)
            for k in np.flatnonzero(keep[b]):
                remap = uv(b, k) if uv else None
                for c, v in enumerate(t[k]):
                    key = (int(v), None if remap is None else tuple(np.round(remap[c], 6)))
                    if key not in seen:
                        row = self.vt[v].copy()
                        if remap is not None:
                            row[6:8] = remap[c]
                        seen[key] = len(new_vt)
                        new_vt.append(row)
                    new_idx.append(seen[key])
            spans.append((start, len(new_idx) - start))
        out = []
        batch_of = {line: b for b, (line, _, _) in enumerate(self.batches)}
        for i, line in enumerate(self.lines):
            if line.startswith(('VT ', 'IDX10 ', 'IDX ')):
                continue
            if line.startswith('POINT_COUNTS'):
                p = line.split()
                out.append(f'POINT_COUNTS {len(new_vt)} {p[2]} {p[3]} {len(new_idx)}')
                out.extend('VT ' + ' '.join(f'{x:.6f}' if j < 6 else f'{x:.6f}' for j, x in enumerate(r)) for r in new_vt)
                for j in range(0, len(new_idx) - len(new_idx) % 10, 10):
                    out.append('IDX10 ' + ' '.join(map(str, new_idx[j:j + 10])))
                out.extend(f'IDX {v}' for v in new_idx[len(new_idx) - len(new_idx) % 10:])
                continue
            if i in batch_of:
                s, c = spans[batch_of[i]]
                if c:
                    out.extend(edit(batch_of[i], f'TRIS {s} {c}') if edit else [f'TRIS {s} {c}'])
                continue
            out.append(line)
        with open(path, 'w', encoding='latin-1', newline='\n') as f:
            f.write('\n'.join(out))
        return len(new_idx) // 3


def wrap(path, begin, end='ANIM_end'):
    lines = open(path, encoding='latin-1').read().split('\n')
    last_data = max(i for i, l in enumerate(lines) if l.startswith(('VT', 'IDX', 'POINT_COUNTS')))
    out = lines[:last_data + 1]
    opened = False
    for line in lines[last_data + 1:]:
        word = line.split()[0] if line.split() else ''
        if word == 'ATTR_LOD':
            if opened:
                out.append(end)
            out.append(line)
            out.extend(begin)
            opened = True
            continue
        if word and not opened:
            out.extend(begin)
            opened = True
        out.append(line)
    while out and out[-1].strip() == '':
        out.pop()
    if opened:
        out.append(end)
    out.append('')
    with open(path, 'w', encoding='latin-1', newline='\n') as f:
        f.write('\n'.join(out))


def sample(img, uv):
    h, w = img.shape[:2]
    x = np.clip((np.mod(uv[..., 0], 1.0) * w).astype(np.int64), 0, w - 1)
    y = np.clip(((1.0 - np.mod(uv[..., 1], 1.0)) * h).astype(np.int64), 0, h - 1)
    return img[y, x]


def png(path, rgb):
    import struct
    import zlib
    rgb = np.ascontiguousarray(rgb.astype(np.uint8))
    h, w = rgb.shape[:2]
    raw = b''.join(b'\x00' + rgb[y].tobytes() for y in range(h))

    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)

    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)) + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def decode(ddstool, path):
    import os
    import subprocess
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        raw = os.path.join(tmp, 'x.rgba')
        w, h = map(int, subprocess.run([ddstool, 'decode', path, raw], capture_output=True, text=True, check=True).stdout.split()[:2])
        return np.fromfile(raw, np.uint8).reshape(h, w, 4)
