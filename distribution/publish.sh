#!/bin/sh
# Publish a release, end to end:
#   distribution/publish.sh <version> ["release notes"]
#
# Packages the installed aircraft, creates the GitHub repository the first
# time (private until you make it public at release), uploads the release to
# GitHub Releases (tag v<version>, the manifest with it), deploys the release
# server (a Cloudflare Worker that serves the manifest and redirects
# downloads to GitHub), and builds the installer pointing at it. The first
# run opens a browser once to log in to Cloudflare. Run from Git Bash.
set -e
VERSION="${1:?usage: distribution/publish.sh <version> [notes]}"
NOTES="${2:-}"
REPO=Vihaan2012-cmyk/Flybywire-A380X-for-Xplane-12
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DIST="$ROOT/dist/$VERSION"

if [ -f "$DIST/manifest.json" ] && ls "$DIST"/*.zip >/dev/null 2>&1; then
  echo "== using the existing package in $DIST (delete it to repackage)"
else
  echo "== packaging $VERSION"
  python "$ROOT/tools/package_release.py" "$VERSION" --notes "$NOTES"
fi

echo "== GitHub release"
gh repo view "$REPO" >/dev/null 2>&1 || gh repo create "$REPO" --private --add-readme   --description "FlyByWire A380X for X-Plane 12 (GPL-3.0)"
if gh release view "v$VERSION" --repo "$REPO" >/dev/null 2>&1; then
  gh release upload "v$VERSION" "$DIST"/*.zip --repo "$REPO" --clobber
else
  gh release create "v$VERSION" "$DIST"/*.zip --repo "$REPO"     --title "FlyByWire A380X for X-Plane 12 $VERSION" --notes "${NOTES:-Release $VERSION}"
fi
# The manifest last: installers never see a release whose files are not there yet.
gh release upload "v$VERSION" "$DIST/manifest.json" --repo "$REPO" --clobber

echo "== release server"
cd "$ROOT/distribution/worker"
[ -d node_modules ] || npm install --no-audit --no-fund
npx wrangler whoami >/dev/null 2>&1 || npx wrangler login
URL="$(npx wrangler deploy 2>&1 | tee /dev/stderr | grep -o 'https://[^ ]*workers\.dev' | head -1)"
[ -n "$URL" ] || { echo "could not read the Worker's URL from wrangler deploy" >&2; exit 1; }

echo "== installer"
cd "$ROOT/installer"
FBW_XP_RELEASES_URL="$URL" CARGO_TARGET_DIR=/d/A380/fbw-build/target-installer \
  cargo +stable-x86_64-pc-windows-gnu build --release
cp /d/A380/fbw-build/target-installer/release/fbw-a380x-installer.exe "$DIST/FlyByWire-A380X-XP-Installer-$VERSION.exe"

echo
echo "Published $VERSION at $URL"
curl -s "$URL/manifest" | head -c 400; echo
echo "Installer: $DIST/FlyByWire-A380X-XP-Installer-$VERSION.exe"
