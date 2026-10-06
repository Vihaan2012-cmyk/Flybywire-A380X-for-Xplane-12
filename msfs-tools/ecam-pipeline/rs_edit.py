import re


def load(p):
    raw = open(p, encoding='utf-8', newline='').read()
    nl = '\r\n' if '\r\n' in raw else '\n'
    return raw.replace('\r\n', '\n'), nl


def save(p, s, nl):
    open(p, 'w', encoding='utf-8', newline='').write(s.replace('\n', nl) if nl == '\r\n' else s)


def block_end(s, i):
    depth = 0
    in_str = False
    while True:
        c = s[i]
        if in_str:
            if c == chr(92):
                i += 2
                continue
            if c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return i
        i += 1


def remove_push(s, ident):
    m = re.search(r'\n[ \t]*v\.push\(\s*proc\(\s*' + re.escape(ident) + r'\b', s)
    if not m:
        return s, False
    start = m.start()
    open_paren = s.index('(', s.index('v.push', start))
    end = block_end(s, open_paren)
    end = s.index('\n', end)
    return s[:start] + s[end:], True
