#!/bin/sh
BUNDLES="${1:?usage: harness.sh <html_ui dir> <out dir> [screens] [states]}"
OUT="${2:?usage: harness.sh <html_ui dir> <out dir> [screens] [states]}"
SCREENS="${3:-MFD,EWD,SD,PFD_L,PFD_R,ND_L,ND_R,FCU,ISIS,CLOCK,RTPI,BAT,RMP_1,RMP_2,RMP_3,SYSTEMS_HOST,EXTRAS_HOST}"
STATES="${4:-all}"
cd /e/fbw-int/plugin/tools/reference
rm -rf "$OUT"
REFERENCE_LOAD_TIMEOUT_MS=${REFERENCE_LOAD_TIMEOUT_MS:-30000} FBW_ROOT=E:/fbw-int/fbw-xpsdk FBW_BUNDLE_HTML_UI="$BUNDLES" node src/reference.mjs --state "$STATES" --screen "$SCREENS" --dom-every 200 --out "$OUT"
