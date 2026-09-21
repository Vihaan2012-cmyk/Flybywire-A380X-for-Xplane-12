"""Build the EFB Study payload from the X-Plane Study app.

The pages are app/ui/index.html. Nothing in it is edited by hand: this
splits that one file into the pieces the EFB can actually load, lowers its
syntax to what the EFB's browser understands, and appends a palette.

Why a split at all. Coherent GT, the EFB's browser, refuses every way of
running code that arrives as a string: a <script> element created at runtime
is silently not executed, and the Function constructor throws. The only
mechanism that works is a real file listed in efb.html's import-script --
the same way study.js itself loads. So the app's single script block is
written out as its own file, wrapped in a function, and study.js calls it
once the markup is in the document.

Outputs into dist/:
  study-app.html          markup and styles, script block removed
  study-app.js            the app's code, transpiled, as window.__deepStudyStart
  study.js                the loader
  test/study-fixture.json
"""

import io
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..', '..'))
SRC = os.path.join(ROOT, 'app', 'ui', 'index.html')
FIXTURE = os.path.join(ROOT, 'app', 'ui', 'test', 'study-fixture.json')
OUT = os.path.join(HERE, 'dist')
FBW = os.environ.get('FBW_AIRCRAFT', 'D:\\fbw-aircraft')

BASE = '/Pages/VCockpit/Instruments/A380X/EFB/'

# Tabs the app has that the EFB should not show.
#
# The tablet already is a tablet: FlyByWire give it ground services, settings
# and a status page of their own, and theirs are the ones wired to MSFS. Ours
# would sit beside them saying different things. The last three are worse than
# redundant -- Flight Plan, Status and Support talk to the X-Plane plugin's
# HTTP port, which does not exist in MSFS at all.
#
# Hidden rather than deleted: the panels stay in the document, so any of the
# app's code that looks them up still finds them. Only the way in is removed.
HIDDEN_TABS = ('flightplan', 'ground', 'settings', 'status', 'support')

PALETTE = """
<style id="efb-palette">
  /* The X-Plane app's own colours, restated for the EFB. Appended rather
     than edited so app/ui/index.html stays a byte-for-byte source. */
  :root {
    --bg: #0d1218;
    --panel: #0d1218;
    --line: #5c6b7a;
    --line-dim: #2f3a45;
    --text: #eef4f9;
    --text-dim: #93a3b2;
    --heading: #93a3b2;
    --field: #06090d;
    --button: #1d2732;
    --button-line: #5c6b7a;
    --accent: #00c2cc;
    --ok: #3ccf6e;
    --warn: #e8a33d;
    --bad: #e05c5c;
  }
  html, body { width: 100%; height: 100%; margin: 0; overflow: hidden; }
  /* The footer belongs to the standalone app: its logo, its own branding,
     and the address of a plugin port that does not exist here. */
  footer { display: none; }

  /* Nothing here restates the app's layout. The sidebar stacking above the
     content was not a layout question at all: grid-template-columns was
     written as a clamp, this engine has no such function, and the whole
     declaration was dropped
     -- see the CSS compat pass below. The app's own rule is correct and at
     1430x1000 its media queries never fire. */
  /* Typography, restated as separate properties.

     The app writes `font: 15px/1.35 "Segoe UI", system-ui, sans-serif` on
     body and `font: <size> "Segoe UI"` on the nav buttons. `system-ui` is a
     family keyword this engine need not know, and an unknown family
     invalidates the whole shorthand -- which takes the size with it,
     because `font` sets both at once. Everything then renders at whatever
     the default is, which is how a seven-digit ATA number stopped fitting
     the 70px column it was given and ran under the name beside it.

     Split apart, a family the engine does not know costs only the family. */
  #ds-app {
    font-family: "Segoe UI", Tahoma, Verdana, Arial, sans-serif;
    font-size: 15px;
    line-height: 1.35;
  }
  .study-nav button, .study-nav button .count {
    font-family: "Segoe UI", Tahoma, Verdana, Arial, sans-serif;
    font-size: 12px;
  }

  /* The chapter count is floated right. In a 210px column most chapter
     names wrap, and a float placed after the text lands on the last line --
     so the number drops onto a line of its own underneath. On the wide
     window the app was written for the names do not wrap and the question
     never comes up. A flex row puts the number back beside the name. */
  .study-nav button {
    display: flex;
    align-items: baseline;
    justify-content: space-between;
  }
  .study-nav button .count { float: none; margin-left: 8px; flex: 0 0 auto; }

  /* Seven digits, and 70px was measured against a font that is not the one
     being used. Widen the column rather than shrink the number. */
  .fail-row { grid-template-columns: 92px minmax(0, 1fr) auto; }
  .fail-row .ata { white-space: nowrap; }
%HIDDEN_TABS%
</style>
"""

# --------------------------------------------------------------- CSS compat
#
# The EFB's browser is Coherent GT, an older Chromium than anything the app
# was written against -- older than 80, which we know because it cannot parse
# `?.`, and that is why study-app.js is transpiled. Several CSS features the
# app uses arrived at or after that point:
#
#   clamp(), min(), max()      Chromium 79
#   gap on a flex container    Chromium 84  (on a grid it is 66, so grids are fine)
#   overflow-wrap: anywhere    Chromium 80
#   inset                      Chromium 87
#
# CSS does not degrade gracefully here. A declaration whose value the engine
# cannot parse is dropped entirely and silently, so the symptom never points
# at the cause. The one that showed was
#
#   .study-layout { grid-template-columns: clamp(140px, 16vw, 210px) minmax(0, 1fr); }
#
# Dropped, the grid has no template at all, falls back to one implicit
# column, and the sidebar stacks above the content. That looked exactly like
# a small-screen media query firing, and was not.
#
# The instrument is a fixed size, so a clamp against the viewport has one
# answer, and it can be worked out here instead of at runtime.

# From the A380X's own panel.cfg:
#   [VCockpit15] pixel_size=1430,1000
#   htmlgauge00=A380X/EFB/efb.html, 0,0,1430,1000
# Nothing in efb.css scales the document, so these are CSS pixels.
VIEWPORT_W = 1430.0
VIEWPORT_H = 1000.0

LENGTH = re.compile(r'^(-?\d*\.?\d+)(px|vw|vh|vmin|vmax)$')
# The word-boundary look-behind is what keeps this off `minmax(` and
# `min-width:`: in `minmax(` the inner `max(` is preceded by `n`, and `min`
# there is followed by `m` rather than `(`.
MATH = re.compile(r'(?<![-\w])(clamp|min|max)\(')


def split_args(text):
    """Split on the commas that are not inside brackets."""
    parts, depth, start = [], 0, 0
    for i, c in enumerate(text):
        if c in '([':
            depth += 1
        elif c in ')]':
            depth -= 1
        elif c == ',' and depth == 0:
            parts.append(text[start:i])
            start = i + 1
    parts.append(text[start:])
    return parts


def to_px(text):
    """A CSS length in pixels, or None if it is not one we can resolve."""
    m = LENGTH.match(text.strip())
    if not m:
        return None
    n, unit = float(m.group(1)), m.group(2)
    return {
        'px': n,
        'vw': n * VIEWPORT_W / 100.0,
        'vh': n * VIEWPORT_H / 100.0,
        'vmin': n * min(VIEWPORT_W, VIEWPORT_H) / 100.0,
        'vmax': n * max(VIEWPORT_W, VIEWPORT_H) / 100.0,
    }[unit]


def flatten_math(css, skipped):
    """Replace clamp/min/max with the pixel value they have at this size.

    A call whose arguments are not all resolvable lengths -- a percentage,
    say, which depends on a parent this cannot see -- is left exactly as it
    was and recorded in `skipped`. Guessing at it would be worse than the
    declaration being dropped: dropped, it falls back to the initial value,
    which is the safe direction.
    """
    pos = 0
    while True:
        m = MATH.search(css, pos)
        if not m:
            return css

        depth, i = 1, m.end()
        while i < len(css) and depth:
            depth += (css[i] == '(') - (css[i] == ')')
            i += 1
        inner = css[m.end():i - 1]

        # Innermost first: rewrite this call's own arguments, then come back
        # round and evaluate the call itself now that they are plain lengths.
        if MATH.search(inner):
            css = css[:m.end()] + flatten_math(inner, skipped) + css[i - 1:]
            continue

        args = [to_px(a) for a in split_args(inner)]
        if any(a is None for a in args) or (m.group(1) == 'clamp' and len(args) != 3):
            skipped.append(css[m.start():i])
            pos = i  # leave it exactly as it was and carry on past it
            continue

        if m.group(1) == 'clamp':
            value = min(max(args[0], args[1]), args[2])
        else:
            value = min(args) if m.group(1) == 'min' else max(args)
        px = '%gpx' % round(value, 2)
        css = css[:m.start()] + px + css[i:]
        pos = m.start() + len(px)


def each_rule(css):
    """Yield (selector, body) for every rule, descending into @media."""
    i = 0
    while True:
        open_at = css.find('{', i)
        if open_at < 0:
            return
        depth, j = 1, open_at + 1
        while j < len(css) and depth:
            depth += (css[j] == '{') - (css[j] == '}')
            j += 1
        head, body = css[i:open_at].strip(), css[open_at + 1:j - 1]
        if head.startswith('@'):
            if re.match(r'@(media|supports|layer|container|document)\b', head, re.I):
                for pair in each_rule(body):
                    yield pair
        else:
            yield head, body
        i = j


GAP = re.compile(r'(?<![-\w])gap\s*:\s*([^;}]+)')


def flex_gap_fallback(css):
    """Margins standing in for `gap` on flex containers.

    Grid gap works in this engine; flex gap does not, and the app uses it in
    fourteen places -- which is why the search field, the "armed only" box
    and the "disarm all" button sit against each other with nothing between
    them. The replacement is the obvious one: a margin on every child but
    the first, along the axis the container lays out on.
    """
    out = []
    for head, body in each_rule(css):
        if not re.search(r'display\s*:\s*(inline-)?flex', body):
            continue
        m = GAP.search(body)
        if not m:
            continue
        lengths = [to_px(v) for v in m.group(1).split()]
        if not lengths or lengths[0] is None:
            continue
        direction = re.search(r'flex-direction\s*:\s*([^;}]+)', body)
        column = bool(direction) and 'column' in direction.group(1)
        row_gap = lengths[0]
        col_gap = lengths[1] if len(lengths) > 1 and lengths[1] is not None else row_gap
        along = row_gap if column else col_gap
        if along:
            side = 'margin-top' if column else 'margin-left'
            out.append('%s > * + * { %s: %gpx; }' % (head, side, round(along, 2)))
        # A wrapping row needs space between the rows as well, and no margin
        # applies only between them -- so every child carries one. The usual
        # trick is to take it back off the container's own bottom margin,
        # which is not done here: .study-toolbar sets margin-bottom itself,
        # and a generated rule quietly overriding a deliberate one is a worse
        # bug than the gap-worth of extra space this leaves behind.
        if not column and re.search(r'flex-wrap\s*:\s*wrap', body) and row_gap:
            out.append('%s > * { margin-bottom: %gpx; }' % (head, round(row_gap, 2)))
    return out


def compat(page):
    """Apply every fix above to each <style> block in the page."""
    skipped, gaps = [], []

    def fix(match):
        css = flatten_math(match.group(2), skipped)
        # `inset` and `overflow-wrap: anywhere` both post-date this engine.
        # Each gets a long-hand written beside it; the new spelling stays,
        # harmlessly ignored, so the source rule is still recognisable.
        css = re.sub(r'(?<![-\w])inset\s*:\s*0\s*;',
                     'top: 0; right: 0; bottom: 0; left: 0;', css)
        css = re.sub(r'overflow-wrap\s*:\s*anywhere\s*;',
                     'word-break: break-word; overflow-wrap: anywhere;', css)
        gaps.extend(flex_gap_fallback(css))
        return match.group(1) + css + match.group(3)

    page = re.sub(r'(<style[^>]*>)(.*?)(</style>)', fix, page, flags=re.S)

    if gaps:
        page += ('\n<style id="efb-compat">\n'
                 '  /* Flex `gap` is Chromium 84; this engine is older than 80.\n'
                 '     Generated by build.py -- see flex_gap_fallback. */\n  '
                 + '\n  '.join(gaps) + '\n</style>\n')

    return page, skipped, len(gaps)


HEADER = (
    '// Generated by build.py from app/ui/index.html. Do not edit.\n'
    '// The Study app, wrapped so study.js can start it once its markup is in\n'
    '// the document, and transpiled: the EFB\'s browser cannot parse the\n'
    '// optional chaining and nullish coalescing the app uses.\n'
)


def transpile(path):
    """Lower the app's syntax to what the EFB's browser can parse.

    Coherent GT in MSFS 2020 is an older Chromium. It does not understand
    optional chaining (`?.`) or nullish coalescing (`??`), both of which the
    app uses -- and a parse error takes the *whole file* with it. So the
    symptom is not a broken feature: it is a global that never appears and a
    panel reporting that study-app.js did not load.

    es2017 keeps async/await and template literals, which that engine does
    handle, and lowers the two it does not. esbuild comes from FlyByWire's
    own node_modules, which their instrument build already depends on.
    """
    exe = 'esbuild.CMD' if os.name == 'nt' else 'esbuild'
    esbuild = os.path.join(FBW, 'node_modules', '.bin', exe)
    if not os.path.exists(esbuild):
        sys.exit(
            'no esbuild at ' + esbuild + '\n'
            'Set FBW_AIRCRAFT to the aircraft repo. Without transpiling, the\n'
            "EFB cannot parse the app and the Study panel stays empty.")

    result = subprocess.run(
        [esbuild, path, '--target=es2017'], capture_output=True, text=True)
    if result.returncode != 0:
        sys.exit('esbuild failed:\n' + result.stderr)

    io.open(path, 'w', encoding='utf8', newline='\n').write(HEADER + result.stdout)

    # Prove the lowering actually happened rather than trusting the flag.
    lowered = io.open(path, encoding='utf8').read()
    for token, what in (('?.', 'optional chaining'), ('??', 'nullish coalescing')):
        if token in lowered:
            sys.exit('esbuild left ' + what + ' in the output; the EFB will not parse it.')


def main():
    if not os.path.exists(SRC):
        sys.exit('missing ' + SRC)

    html = io.open(SRC, encoding='utf8', newline='').read()

    match = re.search(r'<script>(.*?)</script>', html, re.S)
    if not match:
        sys.exit('no <script> block in index.html -- has it been restructured?')
    code = match.group(1)

    # Two substitutions, both exact, both because the EFB is not the app's own
    # window. The app picks between the plugin's HTTP port and its static
    # fixture from its query string; there is no query string here and no
    # plugin in MSFS, so pin it to fixture mode. Its one relative fetch has to
    # become absolute, since the EFB's document lives at a different path.
    before = code
    code = code.replace(
        "new URLSearchParams(location.search).get('fixture') === '1'", 'true')
    code = code.replace(
        "fetch('test/study-fixture.json')",
        "fetch('" + BASE + "test/study-fixture.json')")
    if code == before:
        sys.exit('neither substitution applied -- the app changed, check build.py')

    if os.path.isdir(OUT):
        shutil.rmtree(OUT)
    os.makedirs(os.path.join(OUT, 'test'))

    out_js = os.path.join(OUT, 'study-app.js')
    io.open(out_js, 'w', encoding='utf8', newline='\n').write(
        'window.__deepStudyStart = function () {\n' + code + '\n};\n')
    transpile(out_js)

    # Check the tabs are still called what they were before writing a rule
    # that would otherwise hide nothing and say nothing about it.
    for tab in HIDDEN_TABS:
        if 'data-tab="' + tab + '"' not in html:
            sys.exit('no tab named ' + tab + ' any more -- HIDDEN_TABS is stale')
    hidden = ',\n  '.join('.tab-btn[data-tab="' + t + '"]' for t in HIDDEN_TABS)
    palette = PALETTE.replace(
        '%HIDDEN_TABS%',
        '  /* FlyByWire\'s own tablet covers these, and the last three of them\n'
        '     talk to the X-Plane plugin, which is not here. */\n  '
        + hidden + ' { display: none; }')

    page = html[:match.start()] + html[match.end():] + palette
    page, skipped, gap_rules = compat(page)

    # Prove it rather than trust it. Anything left that this engine cannot
    # parse is a declaration that will be dropped in the sim with no error.
    styles = '\n'.join(re.findall(r'<style[^>]*>(.*?)</style>', page, re.S))
    remaining = []
    pos = 0
    while True:
        m = MATH.search(styles, pos)
        if not m:
            break
        depth, i = 1, m.end()
        while i < len(styles) and depth:
            depth += (styles[i] == '(') - (styles[i] == ')')
            i += 1
        remaining.append(styles[m.start():i])
        pos = i
    if sorted(remaining) != sorted(skipped):
        sys.exit('clamp/min/max left in the output that was not deliberately '
                 'skipped:\n  ' + '\n  '.join(sorted(set(remaining) - set(skipped))))
    # Bare viewport units are old enough for this engine; only the math
    # functions are not. But --gutter is worth checking on its own: it is a
    # clamp, every page's side padding is var(--gutter), and a custom
    # property that fails to parse takes all of them down at once.
    gutter = re.search(r'--gutter\s*:\s*([^;]+);', styles)
    if not gutter or not LENGTH.match(gutter.group(1).strip()):
        sys.exit('--gutter did not resolve to a plain length: '
                 + (gutter.group(1) if gutter else 'not found'))

    print('  css: flattened to %gx%g, %d flex-gap fallbacks'
          % (VIEWPORT_W, VIEWPORT_H, gap_rules))
    for call in sorted(set(skipped)):
        print('       left alone (cannot resolve here): ' + call)
    io.open(os.path.join(OUT, 'study-app.html'), 'w', encoding='utf8', newline='\n').write(page)

    shutil.copyfile(FIXTURE, os.path.join(OUT, 'test', 'study-fixture.json'))
    shutil.copyfile(os.path.join(HERE, 'study.js'), os.path.join(OUT, 'study.js'))

    for name in ('study-app.html', 'study-app.js', 'study.js', 'test/study-fixture.json'):
        path = os.path.join(OUT, name.replace('/', os.sep))
        print('  {:<24} {:>12,} bytes'.format(name, os.path.getsize(path)))


if __name__ == '__main__':
    main()
