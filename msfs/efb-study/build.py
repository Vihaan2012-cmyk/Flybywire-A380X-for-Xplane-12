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
  /* Nothing here changes how the app looks.

     There was a palette in this slot once, recolouring the pages to sit
     beside FlyByWire's EFB: a darker blue-black ground, a cyan accent, its
     own greys. It is gone. These are the same pages as the X-Plane app --
     the app's own :root, its own type, its own spacing, to the value --
     and anything that makes them look otherwise is a bug in this build
     rather than a decision.

     What is left is the three things being inside the EFB forces: the
     app's window is now this container, its footer belongs to the
     standalone app and names a plugin port that does not exist here, and
     five of its tabs are FlyByWire's job on this tablet. */
  html, body { width: 100%; height: 100%; margin: 0; overflow: hidden; }
  footer { display: none; }

  /* Room at the end of the tab strip for the overlay's CLOSE button. The
     tabs are flex: 1 1 auto, so without this they fill the strip end to end
     and the button has nowhere to sit but on top of the page. */
  .tabs { padding-right: 108px; }
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

# The screen, from the A380X's own panel.cfg:
#   [VCockpit15] pixel_size=1430,1000
#   htmlgauge00=A380X/EFB/efb.html, 0,0,1430,1000
# install.ps1 -Scale can enlarge the render target, but study.js pins the
# document back to this, so this is the screen in CSS pixels either way.
SCREEN_W = 1430.0
SCREEN_H = 1000.0

# The coordinate system the app is laid out in, which is NOT the screen.
#
# The app was written for a desktop window a metre away. The tablet is a
# small object in a 3D cockpit, and FlyByWire's own EFB pages are set about
# 1.4 times larger than ours to suit it -- their body text next to ours is
# the whole of the difference, and no amount of resolution closes it.
#
# So study.js lays the app out in a 1000px-wide box and scales that box up
# to fill the 1430px screen. Every length grows by 1.43 and not one device
# pixel is lost, because the scaling happens at draw time. These are the
# numbers the app's own lengths are resolved against, so they belong here;
# study.js reads the same width back out of APP_W.
VIEWPORT_W = 1000.0
VIEWPORT_H = 699.0

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


# `font: [<weight> ]<size>[/<line-height>] <families>`, the only shapes the
# app uses. Anything more elaborate is left alone rather than half-parsed.
FONT_SHORTHAND = re.compile(
    r'(?<![-\w])font\s*:\s*(?:(\d{3})\s+)?([\d.]+px)(?:\s*/\s*([\d.]+))?\s+([^;}]+)')


def _split_font(m):
    weight, size, line_height, families = m.groups()
    parts = []
    if weight:
        parts.append('font-weight: ' + weight)
    parts.append('font-size: ' + size)
    if line_height:
        parts.append('line-height: ' + line_height)
    parts.append('font-family: ' + families.strip())
    return '; '.join(parts)


CONDITION = re.compile(r'\(\s*(max|min)-(width|height)\s*:\s*([\d.]+)px\s*\)')


def resolve_media(css, unresolved):
    """Settle every @media here, since the screen cannot change size.

    The conditions are measured against the SCREEN, not against the box the
    app is laid out in. That distinction is the whole point: a breakpoint is
    the app asking "am I on a small screen?", and the answer is no -- it is
    on a 1430px landscape instrument. The 1000px box is a coordinate system
    chosen to make the type bigger, not a small window, and answering the
    app's question with it would collapse the sidebar onto the top of the
    page, which is where this whole thread of bugs started.

    A block whose conditions are all true is inlined; one with a false
    condition is dropped. Anything not understood is left alone and reported.
    """
    out = ''
    i = 0
    while i < len(css):
        open_at = css.find('{', i)
        if open_at < 0:
            out += css[i:]
            break
        depth, j = 1, open_at + 1
        while j < len(css) and depth:
            depth += (css[j] == '{') - (css[j] == '}')
            j += 1
        head, body = css[i:open_at].strip(), css[open_at + 1:j - 1]

        if not re.match(r'@media\b', head, re.I):
            out += css[i:open_at] + '{' + body + '}'
            i = j
            continue

        query = head[len('@media'):]
        parts = CONDITION.findall(query)
        bare = CONDITION.sub('', query)
        # Only plain `and`-joined width/height conditions, optionally
        # prefixed with `only screen`. Anything else is someone else's
        # problem and stays exactly as it was.
        if not parts or re.sub(r'\b(only|screen|and|all)\b|\s', '', bare):
            unresolved.append(head)
            out += head + '{' + resolve_media(body, unresolved) + '}'
            i = j
            continue

        keep = True
        for kind, axis, value in parts:
            have = SCREEN_W if axis == 'width' else SCREEN_H
            limit = float(value)
            if kind == 'max' and have > limit:
                keep = False
            if kind == 'min' and have < limit:
                keep = False
        if keep:
            out += resolve_media(body, unresolved)
        i = j
    return out


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
    skipped, gaps, unresolved, stripped = [], [], [], [0]

    def fix(match):
        # Comments go first. Everything below walks braces to find where a
        # rule ends, and a comment is the one place a brace can hide from
        # that -- but the immediate reason is duller: a comment sitting in
        # front of an @media becomes part of its selector text, and the
        # block stops looking like an @media at all. The prose lives in
        # app/ui/index.html and in this file; the shipped CSS is generated.
        css = re.sub(r'/\*.*?\*/', ' ', match.group(2), flags=re.S)
        css = flatten_math(css, skipped)
        css = resolve_media(css, unresolved)
        # `inset` and `overflow-wrap: anywhere` both post-date this engine.
        # Each gets a long-hand written beside it; the new spelling stays,
        # harmlessly ignored, so the source rule is still recognisable.
        css = re.sub(r'(?<![-\w])inset\s*:\s*0\s*;',
                     'top: 0; right: 0; bottom: 0; left: 0;', css)
        css = re.sub(r'overflow-wrap\s*:\s*anywhere\s*;',
                     'word-break: break-word; overflow-wrap: anywhere;', css)
        # `system-ui` is a font-family keyword, not a family, and this
        # engine need not know it. An unknown name in a `font` SHORTHAND
        # invalidates the whole declaration -- and `font` carries the size,
        # so the size goes with the family. The app names it in every font
        # stack it has. Dropping the keyword keeps the app's own sizes and
        # its own families, which is the point: nothing here is a redesign.
        dropped = css.count(', system-ui')
        css = css.replace(', system-ui', '')
        # The `font` shorthand, split into longhands.
        #
        # The app sets its nav buttons with `font: 11px "Segoe UI"`, and the
        # chapter counts inside those buttons came out half again too large
        # -- a 465 that dwarfed the chapter it belonged to. The count is a
        # floated span with no size of its own, so it should simply inherit
        # the button's. This engine does not propagate a size set through
        # the shorthand to it. Set as `font-size`, it inherits normally.
        #
        # Nothing about the values changes: the same size, weight, line
        # height and families the app asked for, said one property at a time.
        css = FONT_SHORTHAND.sub(_split_font, css)
        stripped[0] += dropped
        gaps.extend(flex_gap_fallback(css))
        return match.group(1) + css + match.group(3)

    page = re.sub(r'(<style[^>]*>)(.*?)(</style>)', fix, page, flags=re.S)

    if gaps:
        page += ('\n<style id="efb-compat">\n'
                 '  /* Flex `gap` is Chromium 84; this engine is older than 80.\n'
                 '     Generated by build.py -- see flex_gap_fallback. */\n  '
                 + '\n  '.join(gaps) + '\n</style>\n')

    return page, skipped, len(gaps), unresolved, stripped[0]


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
    page, skipped, gap_rules, unresolved, stripped = compat(page)

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

    left = re.findall(r'@media[^{]*', styles)
    if left and not unresolved:
        sys.exit('@media survived the build: ' + '; '.join(sorted(set(left))))

    print('  css: laid out at %gx%g on a %gx%g screen, %d flex-gap fallbacks, '
          '%d system-ui dropped'
          % (VIEWPORT_W, VIEWPORT_H, SCREEN_W, SCREEN_H, gap_rules, stripped))
    for call in sorted(set(skipped)):
        print('       left alone (cannot resolve here): ' + call)
    for query in sorted(set(unresolved)):
        print('       media query not understood, kept: ' + query)
    io.open(os.path.join(OUT, 'study-app.html'), 'w', encoding='utf8', newline='\n').write(page)

    shutil.copyfile(FIXTURE, os.path.join(OUT, 'test', 'study-fixture.json'))
    shutil.copyfile(os.path.join(HERE, 'study.js'), os.path.join(OUT, 'study.js'))

    for name in ('study-app.html', 'study-app.js', 'study.js', 'test/study-fixture.json'):
        path = os.path.join(OUT, name.replace('/', os.sep))
        print('  {:<24} {:>12,} bytes'.format(name, os.path.getsize(path)))


if __name__ == '__main__':
    main()
