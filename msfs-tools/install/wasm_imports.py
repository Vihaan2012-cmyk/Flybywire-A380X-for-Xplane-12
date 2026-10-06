import sys
def leb(b, i):
    r = s = 0
    while True:
        x = b[i]; i += 1; r |= (x & 0x7f) << s; s += 7
        if not x & 0x80: return r, i
def imports(path):
    b = open(path, 'rb').read(); i = 8; out = []
    while i < len(b):
        sid = b[i]; i += 1; size, i = leb(b, i); end = i + size
        if sid == 2:
            n, i = leb(b, i)
            for _ in range(n):
                l, i = leb(b, i); mod = b[i:i+l].decode(); i += l
                l, i = leb(b, i); name = b[i:i+l].decode(); i += l
                kind = b[i]; i += 1
                if kind == 0: _, i = leb(b, i)
                elif kind == 1: i += 1; f = b[i]; i += 1; _, i = leb(b, i); i = leb(b, i)[1] if f & 1 else i
                elif kind == 2: f = b[i]; i += 1; _, i = leb(b, i); i = leb(b, i)[1] if f & 1 else i
                elif kind == 3: i += 2
                out.append(f"{mod}.{name}")
            return out
        i = end
    return out
a, b = set(imports(sys.argv[1])), set(imports(sys.argv[2]))
print("installed:", len(a), "new:", len(b))
print("NEW imports not in installed:", sorted(b - a))
print("wasi imports in new:", sorted(x for x in b if x.startswith('wasi')))
