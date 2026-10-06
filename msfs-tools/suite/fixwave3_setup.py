import os
import subprocess
from concurrent.futures import ThreadPoolExecutor

REPO = 'D:/A380/fbw-xp-worktrees/fs2020-672384b'
WT = 'D:/A380/fbw-wt'
packages = ['tests-ecam', 'tests-engine', 'tests-gear', 'tests-hyd', 'bridge-vars', 'bridge-inhibit',
            'titles-1', 'titles-2', 'titles-3', 'titles-4', 'titles-5', 'titles-6',
            'ra-authority', 'hotair-power', 'cabin-tests', 'coverage-a', 'coverage-b', 'coverage-c', 'coverage-d']
sparse = ['fbw-a380x/src/systems', 'fbw-a380x/src/wasm', 'fbw-common/src/wasm', 'fbw-common/src/systems',
          'fbw-a380x/src/base/flybywire-aircraft-a380-842/SimObjects', 'fbw-a32nx/src/wasm/systems']


def make(name):
    path = f'{WT}/fixwave3-{name}'
    if os.path.isdir(path):
        return name, 'exists'
    run = lambda *x, cwd=REPO: subprocess.run(['git', *x], cwd=cwd, capture_output=True, text=True)
    r = run('worktree', 'add', '--no-checkout', '-b', f'fixwave3/{name}', path, 'HEAD')
    if r.returncode:
        return name, r.stderr.strip()[:200]
    run('sparse-checkout', 'init', '--cone', cwd=path)
    run('sparse-checkout', 'set', *sparse, cwd=path)
    r = run('checkout', cwd=path)
    return name, 'ok' if r.returncode == 0 else r.stderr.strip()[:200]


with ThreadPoolExecutor(10) as ex:
    res = list(ex.map(make, packages))
print('worktrees:', sum(1 for _, s in res if s == 'ok'), [r for r in res if r[1] != 'ok'])
