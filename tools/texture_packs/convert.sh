#!/bin/bash
set -u
EXE=/d/A380/fbw-build/target/release/msfs2xp-aircraft.exe
PKG4K='D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842'
run() {
  local name=$1 pkg=$2; shift 2
  local out="E:\fbw-texpacks\conv-$name"
  echo "== $name start $(date +%T)"
  "$EXE" "$pkg" --out "$out" --name 'A380 for X-Plane 12' --full-cabin --livery "$pkg\SimObjects\AirPlanes\_FlyByWire_A380_842-PRIDE" "$@" > "/e/fbw-texpacks/conv-$name.log" 2>&1
  echo "== $name exit $? $(date +%T)"
}
case "${1:-}" in
  local)
    run base "$PKG4K"
    run 4k "$PKG4K" --exterior-max 4096 --cabin-max 4096 --surface-max 4096 --fuselage-max 4096
    ;;
  8k)
    run 8k 'E:\fbw-texpacks\fbw-8k-package' --max-texture 8192 --interior-max 8192 --normal-max 8192 --exterior-max 8192 --cabin-max 8192 --surface-max 8192 --fuselage-max 8192
    ;;
esac
