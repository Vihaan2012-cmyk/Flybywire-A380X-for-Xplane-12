import os
import sys
import tarfile
import time

root = r'D:\A380\fbw-xp-worktrees\fs2020-672384b'
out = sys.argv[1]
subset = sys.argv[2:]
skip_top = {'.git', '.pnpm-store', 'target', 'large-files', 'bundles', 'build', 'docs', '.ace'}
skip_rel = {os.path.normpath(p) for p in ['fbw-a380x/out', 'fbw-a32nx/out', 'fbw-common/msfs-avionics-mirror/docs']}
t0 = time.time()
files = skipped = 0
with tarfile.open(out, 'w') as tar:
    tops = subset or sorted(os.listdir(root))
    for top in tops:
        if not subset and top in skip_top:
            continue
        start = os.path.join(root, top)
        if os.path.isfile(start):
            tar.add(start, arcname=top.replace('\\', '/'))
            files += 1
            continue
        for dirpath, dirnames, filenames in os.walk(start):
            rel_dir = os.path.relpath(dirpath, root)
            dirnames[:] = [d for d in dirnames if os.path.normpath(os.path.join(rel_dir, d)) not in skip_rel]
            for name in filenames + [d for d in dirnames if os.path.islink(os.path.join(dirpath, d))]:
                path = os.path.join(dirpath, name)
                try:
                    tar.add(path, arcname=os.path.relpath(path, root).replace('\\', '/'), recursive=False)
                    files += 1
                except OSError:
                    skipped += 1
            for d in list(dirnames):
                p = os.path.join(dirpath, d)
                try:
                    tar.add(p, arcname=os.path.relpath(p, root).replace('\\', '/'), recursive=False)
                except OSError:
                    dirnames.remove(d)
                    skipped += 1
print(f'{files} files, {skipped} skipped, {os.path.getsize(out) / 1e6:.0f} MB in {time.time() - t0:.0f} s')
