#!/bin/sh
# Applies tools/js-build/patches/*.patch to D:\fbw-aircraft before FlyByWire's
# own build runs, or reverts them (docs/js-build.md's "patch file" mechanism:
# for changes too large for a single find/replace SourcePatch — new files,
# multi-file changes). FlyByWire's sources must stay unmodified between runs,
# so always pair an `apply` with a `revert` once the build this ran for is
# done.
#
# Usage (from anywhere):
#   tools/js-build/apply-patches.sh apply
#   tools/js-build/apply-patches.sh revert
#
# Every *.patch here is a `git diff` taken inside D:\fbw-aircraft (new files
# included, via `git add -N` before diffing) and applies with `git apply`
# from that repo's root.
set -e
ACTION="${1:?usage: apply-patches.sh [apply|revert]}"
FBW="${FBW_AIRCRAFT_DIR:-/d/fbw-aircraft}"
PATCH_DIR="$(cd "$(dirname "$0")/patches" && pwd)"

cd "$FBW"
for p in "$PATCH_DIR"/*.patch; do
  [ -e "$p" ] || continue
  case "$ACTION" in
    apply)
      echo "js-build: applying $(basename "$p")"
      git apply --whitespace=nowarn "$p"
      ;;
    revert)
      echo "js-build: reverting $(basename "$p")"
      git apply --whitespace=nowarn --reverse "$p"
      ;;
    *)
      echo "js-build: unknown action '$ACTION' (want apply or revert)" >&2
      exit 1
      ;;
  esac
done
