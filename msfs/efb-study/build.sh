#!/bin/sh
# Build the EFB Study payload from the X-Plane Study app.
#
# The page is a *verbatim copy* of app/ui/index.html -- same markup, same CSS,
# same 93 render functions, same SVG synoptics. Only two things are appended:
# a palette that recolours it to the EFB, and a base tag so its relative
# fetches resolve inside the instrument directory. Nothing is rewritten, so
# the two stay in step by construction.
#
# It runs in the app's own `?fixture=1` mode, which reads a static
# study-fixture.json instead of talking to the plugin's HTTP port -- exactly
# what is needed in MSFS, where there is no plugin and no port.
set -e
cd "$(dirname "$0")"
ROOT=../..
OUT=dist

rm -rf "$OUT"
mkdir -p "$OUT/test"

cp "$ROOT/app/ui/index.html" "$OUT/study-app.html"
cp "$ROOT/app/ui/test/study-fixture.json" "$OUT/test/study-fixture.json"

# The EFB palette, appended so it wins over the app's own :root block
# without editing a single line of it.
cat >> "$OUT/study-app.html" <<'CSS'
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
  /* The app fills a desktop window; in the EFB it fills the instrument. */
  html, body { width: 100%; height: 100%; margin: 0; overflow: hidden; }
  /* The footer belongs to the standalone X-Plane app: its logo, its own
     branding and the address of a plugin port that does not exist here. */
  footer { display: none; }
</style>
CSS

printf 'built %s: %s\n' "$OUT/study-app.html" "$(wc -c < "$OUT/study-app.html") bytes"
printf '       %s: %s\n' "$OUT/test/study-fixture.json" "$(wc -c < "$OUT/test/study-fixture.json") bytes"
