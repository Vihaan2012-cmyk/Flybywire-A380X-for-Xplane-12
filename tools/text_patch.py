import argparse
import hashlib
import sys
from pathlib import Path

MAGIC = 'XP-TEXT-PATCH 1'


def split(data: bytes) -> tuple[list[str], str]:
    text = data.decode('latin-1')
    eol = '\r\n' if '\r\n' in text else '\n'
    return text.split(eol), eol


def make(target: Path, template: Path) -> str:
    template_bytes = template.read_bytes()
    base, _ = split(template_bytes)
    first: dict[str, int] = {}
    for i, line in enumerate(base):
        first.setdefault(line, i)
    lines, eol = split(target.read_bytes())
    out = [MAGIC, f'TEMPLATE {hashlib.sha256(template_bytes).hexdigest()}', 'EOL CRLF' if eol == '\r\n' else 'EOL LF']
    run_start, run_len = None, 0
    referenced = 0

    def flush():
        nonlocal run_start, run_len
        if run_start is not None:
            out.append(f'={run_start} {run_len}')
        run_start, run_len = None, 0

    for line in lines:
        n = first.get(line)
        if n is not None:
            referenced += 1
            if run_start is not None and n == run_start + run_len:
                run_len += 1
            else:
                flush()
                run_start, run_len = n, 1
        else:
            flush()
            out.append('+' + line)
    flush()
    print(f'{target.name}: {referenced} lines from {template.name}, {len(lines) - referenced} of our own', file=sys.stderr)
    return '\n'.join(out)


def apply(patch: str, template_bytes: bytes) -> bytes:
    lines = patch.split('\n')
    if lines[0] != MAGIC:
        raise ValueError('not a text patch')
    if lines[1] != f'TEMPLATE {hashlib.sha256(template_bytes).hexdigest()}':
        raise ValueError('the template is not the one the patch was made from')
    eol = '\r\n' if lines[2] == 'EOL CRLF' else '\n'
    base, _ = split(template_bytes)
    out: list[str] = []
    for line in lines[3:]:
        if line.startswith('='):
            start, count = (int(x) for x in line[1:].split())
            out.extend(base[start:start + count])
        elif line.startswith('+'):
            out.append(line[1:])
        else:
            raise ValueError(f'bad patch line: {line!r}')
    return eol.join(out).encode('latin-1')


def main() -> int:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='command', required=True)
    m = sub.add_parser('make')
    m.add_argument('target', type=Path)
    m.add_argument('template', type=Path)
    m.add_argument('out', type=Path)
    a = sub.add_parser('apply')
    a.add_argument('patch', type=Path)
    a.add_argument('template', type=Path)
    a.add_argument('out', type=Path)
    args = parser.parse_args()
    if args.command == 'make':
        args.out.write_bytes(make(args.target, args.template).encode('latin-1'))
    else:
        args.out.write_bytes(apply(args.patch.read_bytes().decode('latin-1'), args.template.read_bytes()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
