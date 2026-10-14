import argparse
import hashlib
import re
import sys
from pathlib import Path

MAGIC = 'XP-ACF-PATCH 1'
TRIVIAL = re.compile(r'^(-?[01](\.0+)?|-?0\.0+|)$')
OWN = ('_obja/',)


def properties(text: str) -> dict[str, str]:
    out = {}
    for line in text.split('\n'):
        if line.startswith('P '):
            key, _, value = line[2:].partition(' ')
            out[key] = value
    return out


def check(value: str) -> str:
    return hashlib.sha256(value.encode('latin-1')).hexdigest()[:8]


def make(acf: Path, template: Path) -> str:
    template_bytes = template.read_bytes()
    base = properties(template_bytes.decode('latin-1'))
    out = [MAGIC, f'TEMPLATE {hashlib.sha256(template_bytes).hexdigest()}']
    referenced = 0
    for line in acf.read_bytes().decode('latin-1').split('\n'):
        if line.startswith('='):
            raise ValueError(f'{acf}: a line starts with "="')
        if line.startswith('P '):
            key, _, value = line[2:].partition(' ')
            if base.get(key) == value and not TRIVIAL.match(value) and not key.startswith(OWN):
                out.append(f'={key} {check(value)}')
                referenced += 1
                continue
        out.append(line)
    print(f'{acf.name}: {referenced} values taken from {template.name}, {len(out) - 2 - referenced} lines of our own', file=sys.stderr)
    return '\n'.join(out)


def apply(patch: str, template: Path) -> tuple[str, int]:
    lines = patch.split('\n')
    if lines[0] != MAGIC:
        raise ValueError('not an acf patch')
    base = properties(template.read_bytes().decode('latin-1'))
    out, mismatched = [], 0
    for line in lines[2:]:
        if line.startswith('='):
            key, _, expected = line[1:].partition(' ')
            if key not in base:
                raise ValueError(f'{template}: has no {key}')
            if check(base[key]) != expected:
                mismatched += 1
            out.append(f'P {key} {base[key]}')
        else:
            out.append(line)
    return '\n'.join(out), mismatched


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='command', required=True)
    m = sub.add_parser('make')
    m.add_argument('acf', type=Path)
    m.add_argument('template', type=Path)
    m.add_argument('out', type=Path)
    a = sub.add_parser('apply')
    a.add_argument('patch', type=Path)
    a.add_argument('template', type=Path)
    a.add_argument('out', type=Path)
    args = parser.parse_args()
    if args.command == 'make':
        args.out.write_bytes(make(args.acf, args.template).encode('latin-1'))
    else:
        text, mismatched = apply(args.patch.read_bytes().decode('latin-1'), args.template)
        args.out.write_bytes(text.encode('latin-1'))
        print(f'{args.out.name}: {mismatched} template values differ from the ones the patch was made against', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
