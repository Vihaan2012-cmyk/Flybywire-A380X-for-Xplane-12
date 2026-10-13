#!/bin/sh
set -e
OUT="${1:-/e/fbw-int/xp-avionics/dist}"
cd /e/fbw-int/xp-avionics
rm -rf "$OUT"
mkdir -p "$OUT"
node_modules/.bin/esbuild src/index.ts --bundle --format=esm --target=es2020 --outfile="$OUT/index.js" --log-level=warning
node_modules/.bin/tsc -p tsconfig.json --emitDeclarationOnly --outDir "$OUT/types" > "$OUT/declarations.log" 2>&1 || echo "build: our own sources have $(grep -c 'error TS' "$OUT/declarations.log") type errors, see $OUT/declarations.log"
printf "export * from './types/index';\n" > "$OUT/index.d.ts"
printf '{\n  "name": "xp-avionics",\n  "version": "0.1.0",\n  "main": "index.js",\n  "module": "index.js",\n  "types": "index.d.ts"\n}\n' > "$OUT/package.json"
echo "build: $OUT/index.js $(wc -c < "$OUT/index.js") bytes"
