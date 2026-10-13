#!/bin/sh
# The build lives in build.py. This used to be a second, separate
# implementation of it and fell behind: it wrote a dist/ with no study.js and
# no study-app.js, left clamp() in the CSS that Coherent GT cannot parse, and
# install.ps1's "is dist/ there?" check passed anyway because study-app.html
# existed. Two builds, one of them stale and indistinguishable from the good
# one, is worse than none. Keep this as a forwarder only.
set -e
cd "$(dirname "$0")"
exec python build.py "$@"
