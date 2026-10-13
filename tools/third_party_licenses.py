import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LICENSE_FILE = re.compile(r'^(licen[cs]e|copying|notice|unlicense)([-_.].*)?$', re.IGNORECASE)
PATH_COMMENT = re.compile(r'^\s*//\s+(\S*node_modules/\S+)', re.MULTILINE)
PACKAGE_IN_PATH = re.compile(r'node_modules/((?:@[^/]+/)?[^/]+)')
MIT_TEXT = '''MIT License

Copyright (c) {holder}

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
USE OR OTHER DEALINGS IN THE SOFTWARE.'''
CRATES = (
    ('the systems plugin (win.xpl, fbw_a380_systems_server.exe)', ROOT, 'x86_64-pc-windows-gnu', ['--features', 'js']),
    ('XPHFBW (XPHFBW.exe)', ROOT / 'app', 'x86_64-pc-windows-msvc', []),
    ('the installer and updater', ROOT / 'installer', 'x86_64-pc-windows-msvc', []),
)


def license_texts(directory: Path) -> list[tuple[str, str]]:
    out = []
    if not directory.is_dir():
        return out
    for entry in sorted(directory.iterdir()):
        if entry.is_file() and LICENSE_FILE.match(entry.name):
            out.append((entry.name, entry.read_text(encoding='utf-8', errors='replace').strip()))
    return out


def npm_packages(aircraft: Path, node_modules: Path) -> dict[str, dict]:
    found: dict[str, dict] = {}
    for js in (aircraft / 'html_ui').rglob('*.js'):
        for match in PATH_COMMENT.finditer(js.read_text(encoding='utf-8', errors='replace')):
            parts = PACKAGE_IN_PATH.findall(match.group(1))
            if not parts:
                continue
            directory = node_modules.joinpath(*('/node_modules/'.join(parts)).split('/'))
            key = '/'.join(parts)
            if key in found:
                continue
            manifest = directory / 'package.json'
            info = json.loads(manifest.read_text(encoding='utf-8')) if manifest.exists() else {}
            licence = info.get('license') or info.get('licenses') or 'not stated'
            if not isinstance(licence, str):
                licence = json.dumps(licence)
            author = info.get('author') or ''
            if isinstance(author, dict):
                author = author.get('name', '')
            found[key] = {
                'name': info.get('name', parts[-1]),
                'version': info.get('version', '?'),
                'license': licence,
                'holder': author or f"the {info.get('name', parts[-1])} authors",
                'texts': license_texts(directory),
            }
    return found


def crates(directory: Path, triple: str, extra: list[str]) -> tuple[dict[str, dict], list[str]]:
    env = dict(os.environ)
    meta = subprocess.run(
        ['cargo', 'metadata', '--format-version', '1', '--filter-platform', triple, *extra],
        cwd=directory, env=env, capture_output=True, text=True, check=True,
    )
    m = json.loads(meta.stdout)
    nodes = {n['id']: n for n in m['resolve']['nodes']}
    packages = {p['id']: p for p in m['packages']}
    seen, stack = set(), [m['resolve']['root']]
    while stack:
        current = stack.pop()
        if current in seen:
            continue
        seen.add(current)
        for dep in nodes[current]['deps']:
            if any(k['kind'] in (None, 'build') for k in dep['dep_kinds']):
                stack.append(dep['pkg'])
    third, local = {}, []
    for pid in seen:
        p = packages[pid]
        if p.get('source') is None:
            local.append(f"{p['name']} {p['version']} ({p.get('license') or 'licence of its repository'})")
            continue
        third[f"{p['name']} {p['version']}"] = {
            'name': p['name'],
            'version': p['version'],
            'license': p.get('license') or 'not stated',
            'holder': ', '.join(p.get('authors') or []) or f"the {p['name']} authors",
            'texts': license_texts(Path(p['manifest_path']).parent),
        }
    return third, sorted(local)


def grouped(packages: dict[str, dict]) -> list[tuple[str, list[str]]]:
    groups: dict[str, tuple[str, list[str]]] = {}
    for key in sorted(packages, key=str.lower):
        p = packages[key]
        if p['texts']:
            body = '\n\n'.join(f'--- {name} ---\n{text}' for name, text in p['texts'])
        elif 'MIT' in p['license']:
            body = MIT_TEXT.format(holder=p['holder'])
        else:
            body = f"Licence: {p['license']} (the package carries no licence file; https://spdx.org/licenses/)"
        digest = hashlib.sha256(' '.join(body.split()).encode('utf-8')).hexdigest()
        groups.setdefault(digest, (body, []))[1].append(f"{p['name']} {p['version']} ({p['license']})")
    return sorted(groups.values(), key=lambda g: g[1][0].lower())


def section(title: str, packages: dict[str, dict]) -> str:
    lines = [f'{"=" * 78}\n{title}\n{"=" * 78}\n']
    for body, users in grouped(packages):
        lines.append('Used by: ' + ', '.join(users) + '\n\n' + body + '\n\n' + '-' * 78 + '\n')
    return '\n'.join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('aircraft', type=Path)
    parser.add_argument('--node-modules', type=Path, default=Path('D:/fbw-aircraft/node_modules'))
    a = parser.parse_args()

    out = [
        'A380 for X-Plane 12 (Ported from FBW A380): third-party software licences\n',
        'The instrument pages (html_ui) bundle the npm packages listed first; the plugin, XPHFBW and the',
        'installer are built from the Rust crates listed after them. Each entry is the package\'s own licence',
        'and notice text. Chromium and its components: plugins/fbw_a380_systems/XPHFBW/CREDITS.html.',
        'Fonts under the SIL Open Font License: html_ui/Fonts/fbw-a380x/OFL-FONTS.txt.\n',
    ]
    npm = npm_packages(a.aircraft, a.node_modules)
    out.append(section(f'npm packages in the instrument pages ({len(npm)})', npm))
    for label, directory, triple, extra in CRATES:
        third, local = crates(directory, triple, extra)
        out.append(section(f'Rust crates in {label} ({len(third)})', third))
        out.append('This project\'s own crates and FlyByWire\'s (GPL): ' + ', '.join(local) + '\n')
    target = a.aircraft / 'THIRD_PARTY_LICENSES.txt'
    target.write_text('\n'.join(out), encoding='utf-8', newline='\n')
    missing = [k for k, p in npm.items() if not p['texts']]
    print(f'{target}: {target.stat().st_size // 1024} KB; npm {len(npm)} (no licence file: {missing})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
