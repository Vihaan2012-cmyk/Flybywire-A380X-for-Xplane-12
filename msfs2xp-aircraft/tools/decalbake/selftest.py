import os, sys, tempfile
import numpy as np
from apply import rewrite
from objparse import triangles
objdir = sys.argv[1]
for n in sys.argv[2:]:
    p = os.path.join(objdir, n + '.obj')
    lines = open(p, encoding='utf-8').read().splitlines()
    out = rewrite(lines)
    q = os.path.join(tempfile.gettempdir(), n + '_rt.obj')
    open(q, 'w', encoding='utf-8', newline='\n').write('\n'.join(out) + '\n')
    _, a = triangles(p, n)
    _, b = triangles(q, n)
    ok = len(a) == len(b) and all(np.array_equal(x['pos'], y['pos']) and np.array_equal(x['uv'], y['uv']) and x['light'] == y['light'] and x['animated'] == y['animated'] for x, y in zip(a, b))
    print(n, 'identical' if ok else 'DIFFERENT', len(a), len(b))
    os.remove(q)
