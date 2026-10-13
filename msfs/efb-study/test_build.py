"""Regression tests for build.py's CSS compat pass.

Run with: python test_build.py

Coherent GT drops a declaration outright when it cannot parse a value --
silently, with no error -- so a `clamp()`/`min()`/`max()` call that survives
the compat pass into the shipped page is a real, undetected bug. build.py's
own `main()` only prints these as "left alone"; it does not fail the build.
This file is the check that a specific one of those ("min-width:
min(250px, 100%)" on `button`, app/ui/index.html) doesn't come back.
"""
import sys

import build


def test_actual_button_rule_has_no_math_left():
    import io, os
    src = io.open(build.SRC, encoding='utf8', newline='').read()
    import re
    m = re.search(r'\bbutton\s*\{([^}]*)\}', src)
    assert m, 'no bare `button { ... }` rule found in app/ui/index.html -- check this test is still pointed at the right selector'
    body = re.sub(r'/\*.*?\*/', ' ', m.group(1), flags=re.S)  # comments may say "min(" in prose
    assert 'min(' not in body and 'max(' not in body and 'clamp(' not in body, (
        'button rule still uses a CSS math function Coherent GT cannot parse: %r' % body)


if __name__ == '__main__':
    failures = 0
    for name, fn in list(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print('ok   ' + name)
            except AssertionError as e:
                failures += 1
                print('FAIL ' + name + ': ' + str(e))
    if failures:
        sys.exit('%d test(s) failed' % failures)
    print('all tests passed')
