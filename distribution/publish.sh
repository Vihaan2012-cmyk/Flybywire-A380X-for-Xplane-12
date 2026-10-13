#!/bin/sh
# distribution/publish.sh <version> ["release notes"] [--patch-from <version>] [--prerelease] [--texture-pack 4k=<dir>] [--texture-pack 8k=<dir>]
set -e
VERSION="${1:?usage: distribution/publish.sh <version> [notes] [--patch-from <version>] [--prerelease]}"
NOTES="${2:-}"
shift; [ $# -gt 0 ] && shift
PRE=""
PATCH=""
PACKS=""
while [ $# -gt 0 ]; do
  case "$1" in
    --prerelease) PRE="--prerelease" ;;
    --patch-from) PATCH="--patch-from $2"; shift ;;
    --texture-pack) PACKS="$PACKS --texture-pack $2"; shift ;;
  esac
  shift
done
REPO=Vihaan2012-cmyk/Flybywire-A380X-for-Xplane-12
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DIST="$ROOT/dist/$VERSION"
TARGET=/d/A380/fbw-build/target

echo "== installer engine"
(cd "$ROOT/installer/engine" && CARGO_TARGET_DIR="$TARGET" RUSTFLAGS="-C target-feature=+crt-static --remap-path-prefix=$USERPROFILE=~" \
  cargo +stable-x86_64-pc-windows-msvc build --release --target x86_64-pc-windows-msvc)
BUILT="$TARGET/x86_64-pc-windows-msvc/release/a380x-xp-engine.exe"
SETUP="$(ls "$ROOT"/installer/dist/A380-for-X-Plane-12-Installer-*-x64.exe 2>/dev/null | tail -1)"

if [ -f "$DIST/manifest.json" ] && ls "$DIST"/*.zip >/dev/null 2>&1; then
  echo "== using the existing package in $DIST (delete it to repackage)"
else
  echo "== packaging $VERSION"
  python "$ROOT/tools/package_release.py" "$VERSION" --notes "$NOTES" --installer "$BUILT" $PATCH $PACKS
fi
INSTALLER=""
if [ -n "$SETUP" ]; then
  INSTALLER="$DIST/$(basename "$SETUP")"
  cp "$SETUP" "$INSTALLER"
fi

echo "== GitHub release v$VERSION"
gh repo view "$REPO" >/dev/null 2>&1 || { echo "$REPO is not reachable with gh; log in with gh auth login" >&2; exit 1; }
if gh release view "v$VERSION" --repo "$REPO" >/dev/null 2>&1; then
  gh release upload "v$VERSION" "$DIST"/*.zip "$DIST/tree.json" $INSTALLER --repo "$REPO" --clobber
else
  gh release create "v$VERSION" "$DIST"/*.zip "$DIST/tree.json" $INSTALLER --repo "$REPO" $PRE \
    --title "A380 for X-Plane 12 (Ported from FBW A380) $VERSION" --notes "${NOTES:-Release $VERSION}"
fi
gh release upload "v$VERSION" "$DIST/manifest.json" --repo "$REPO" --clobber

echo
echo "Published v$VERSION to https://github.com/$REPO/releases/tag/v$VERSION"
echo "Installer: ${INSTALLER:-none (build installer/ with npm run package:win first)}"
