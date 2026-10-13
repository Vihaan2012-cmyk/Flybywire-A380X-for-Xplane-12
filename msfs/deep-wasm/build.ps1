# Builds deep.wasm, the deep systems layer's MSFS module, in FlyByWire's
# Docker dev-env image (which contains the licensed MSFS SDK, so nothing is
# installed on this machine). Same route ../lvar-bench used; see its build.ps1
# for the -Native alternative and why the image is pinned by digest.
#
#   .\build.ps1            -> .\dist\deep.wasm
#
# Two things differ from lvar-bench and both matter:
#  * the whole repository is mounted, not this crate, because src/lib.rs
#    compiles ../../../src/deep/{lvar_bridge,msfs_inputs}.rs in place;
#  * the image's entrypoint cds to the mount root regardless of -w, so the
#    script cds into this crate itself. Without that, cargo builds the
#    X-Plane plugin at the repository root instead and fails on its
#    ../fbw-aircraft path dependency.

[CmdletBinding()]
param([switch]$SkipOpt)

$ErrorActionPreference = 'Stop'
$CrateRoot = $PSScriptRoot
$RepoRoot  = (Resolve-Path (Join-Path $CrateRoot '..\..')).Path
$DevEnvImage = 'ghcr.io/flybywiresim/dev-env@sha256:28b1f55c047b9ec338c3d676a82225fe135b0b1061fa7993c03b9a75b5e470cd'
$OptFlags = '-O1 --signext-lowering --enable-bulk-memory --enable-nontrapping-float-to-int'

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) { throw 'docker is not on PATH.' }
$mount = ($RepoRoot -replace '\', '/')
$opt = if ($SkipOpt) { 'cp target/wasm32-wasip1/release/deep_wasm.wasm dist/deep.wasm' }
       else { "wasm-opt $OptFlags -o dist/deep.wasm target/wasm32-wasip1/release/deep_wasm.wasm" }
$script = "set -e; cd /external/msfs/deep-wasm; mkdir -p dist; cargo build --target wasm32-wasip1 --release; $opt"

# MSYS-style path conversion would mangle the -v argument under Git Bash.
$env:MSYS_NO_PATHCONV = '1'
Write-Host "Building in $DevEnvImage ..."
$ErrorActionPreference = 'Continue'
# The plugin crate's build.rs compiles FlyByWire's C++ computers with `cc`,
# which for a wasm target needs telling which compiler and sysroot to use --
# the same flags FlyByWire's own fbw_a380/build.sh passes clang. The
# aircraft checkout is mounted where the plugin's ../fbw-aircraft path
# dependency resolves.
$aircraft = ((Resolve-Path (Join-Path $RepoRoot '..bw-aircraft')).Path -replace '\', '/')
$sysroot = '/workdir/MSFS_SDK/WASM/wasi-sysroot'
$cflags = "--sysroot=$sysroot -D__wasi__ -D_LIBCPP_HAS_NO_THREADS -mbulk-memory"
& docker run --rm -v "${mount}:/external" -v "${aircraft}:/fbw-aircraft" `
    -e CARGO_HOME=/external/msfs/deep-wasm/target/cargo_home `
    -e CC_wasm32_wasip1=clang -e CXX_wasm32_wasip1=clang++ -e AR_wasm32_wasip1=llvm-ar `
    -e "CFLAGS_wasm32_wasip1=$cflags" -e "CXXFLAGS_wasm32_wasip1=$cflags -fno-exceptions -fno-rtti" `
    $DevEnvImage bash -c $script
if ($LASTEXITCODE -ne 0) { throw "docker build failed with exit code $LASTEXITCODE" }
$ErrorActionPreference = 'Stop'

$out = Join-Path $CrateRoot 'dist\deep.wasm'
if (-not (Test-Path $out)) { throw "Build reported success but $out does not exist." }
Write-Host "Built: $out ($((Get-Item $out).Length) bytes)"
Write-Host 'Next:  .\install.ps1     (copies it into the A380X package and registers the gauge)'
