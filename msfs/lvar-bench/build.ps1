# Builds lvar-bench for MSFS.
#
#   .\build.ps1              Build in FlyByWire's Docker dev-env (recommended:
#                            the image already contains the MSFS SDK, so
#                            nothing needs installing on this machine).
#   .\build.ps1 -Native      Build with a locally installed MSFS SDK.
#
# Either way the finished module lands in .\dist\lvar-bench.wasm.
#
# The MSFS SDK is a licensed Microsoft download. It is not on crates.io, not in
# this repository and not in D:\fbw-aircraft. The Docker route exists precisely
# so the benchmark can be built without one.

[CmdletBinding()]
param(
    # Build against a local SDK instead of inside Docker.
    [switch]$Native,
    # Skip the wasm-opt pass. The module works without it; it is only smaller.
    [switch]$SkipOpt
)

$ErrorActionPreference = 'Stop'

# Windows PowerShell 5.1 turns every stderr line from a native executable into
# an ErrorRecord, which under -ErrorAction Stop kills the script on cargo's
# perfectly ordinary progress and warning output. Native commands are therefore
# run through this, which judges them by their exit code and nothing else.
function Invoke-Native {
    param([Parameter(Mandatory)][scriptblock]$Block, [Parameter(Mandatory)][string]$What)
    $ErrorActionPreference = 'Continue'
    & $Block
    if ($LASTEXITCODE -ne 0) { throw "$What failed with exit code $LASTEXITCODE" }
}

$CrateRoot = $PSScriptRoot
$DistDir   = Join-Path $CrateRoot 'dist'
$OutWasm   = Join-Path $DistDir 'lvar-bench.wasm'
$RawWasm   = Join-Path $CrateRoot 'target\wasm32-wasip1\release\lvar_bench.wasm'

# The exact image FlyByWire pin in scripts/dev-env/run.sh. Pinned by digest so
# this builds the same way in a year's time.
$DevEnvImage = 'ghcr.io/flybywiresim/dev-env@sha256:28b1f55c047b9ec338c3d676a82225fe135b0b1061fa7993c03b9a75b5e470cd'

# The wasm-opt pass FlyByWire apply to systems.wasm (package.json,
# "build-a380x:systems"). Same flags, same reasons.
$OptFlags = @('-O1', '--signext-lowering', '--enable-bulk-memory', '--enable-nontrapping-float-to-int')

New-Item -ItemType Directory -Force -Path $DistDir | Out-Null

function Resolve-MsfsSdk {
    # Same search order as msfs-rs's own msfs_sdk::calculate_msfs_sdk_path,
    # so the two cannot disagree about which SDK is in use.
    if ($env:MSFS_SDK) {
        if (Test-Path $env:MSFS_SDK) { return $env:MSFS_SDK }
        throw "MSFS_SDK is set to '$($env:MSFS_SDK)' but that directory does not exist."
    }
    if (Test-Path 'C:\MSFS SDK') { return 'C:\MSFS SDK' }
    return $null
}

if ($Native) {
    $sdk = Resolve-MsfsSdk
    if (-not $sdk) {
        Write-Host ''
        Write-Host '------------------------------------------------------------------'
        Write-Host ' MSFS SDK not found. A native build cannot proceed.'
        Write-Host ''
        Write-Host ' Looked for, in order:'
        Write-Host '   $env:MSFS_SDK   (not set)'
        Write-Host '   C:\MSFS SDK     (does not exist)'
        Write-Host ''
        Write-Host ' Either:'
        Write-Host '   - install the SDK (in MSFS: Options > General > Developers >'
        Write-Host '     "SDK Installer"; or https://docs.flightsimulator.com/), to'
        Write-Host '     C:\MSFS SDK or anywhere with $env:MSFS_SDK pointing at it,'
        Write-Host '     and make sure clang and llvm-ar are on PATH -- msfs-rs'
        Write-Host '     compiles the SDK''s nanovg.cpp during its build;'
        Write-Host '   - or drop -Native and build in Docker, which needs no SDK.'
        Write-Host '------------------------------------------------------------------'
        Write-Host ''
        exit 1
    }
    Write-Host "MSFS SDK: $sdk"

    foreach ($tool in @('cargo', 'clang', 'llvm-ar')) {
        if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
            throw "'$tool' is not on PATH. A native build needs cargo, clang and llvm-ar (the last two come with the MSFS SDK or LLVM)."
        }
    }

    Push-Location $CrateRoot
    try {
        # rust-toolchain is not pinned in this crate; FlyByWire ship with 1.96.0
        # and that is the toolchain the Docker route uses, so prefer it if present.
        $toolchainArg = @()
        $installed = (rustup toolchain list) -join "`n"
        if ($installed -match '1\.96\.0') { $toolchainArg = @('+1.96.0') }

        Invoke-Native { & cargo @toolchainArg build --target wasm32-wasip1 --release } 'cargo build'
    } finally {
        Pop-Location
    }

    if ($SkipOpt -or -not (Get-Command wasm-opt -ErrorAction SilentlyContinue)) {
        if (-not $SkipOpt) { Write-Host 'wasm-opt not on PATH; copying the unoptimised module (this is fine).' }
        Copy-Item $RawWasm $OutWasm -Force
    } else {
        Invoke-Native { & wasm-opt @OptFlags -o $OutWasm $RawWasm } 'wasm-opt'
    }
}
else {
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        throw "docker is not on PATH. Install Docker Desktop, or install the MSFS SDK and run '.\build.ps1 -Native'."
    }

    # Docker Desktop on Windows wants forward slashes in -v.
    $mount = ($CrateRoot -replace '\\', '/')

    # The image's entrypoint cds to /external, so that is where the crate must
    # be mounted. Mounting the crate (not the repo) keeps target/ inside it.
    $optCmd = if ($SkipOpt) {
        'cp target/wasm32-wasip1/release/lvar_bench.wasm dist/lvar-bench.wasm'
    } else {
        "wasm-opt $($OptFlags -join ' ') -o dist/lvar-bench.wasm target/wasm32-wasip1/release/lvar_bench.wasm"
    }

    # Two artifacts. The default one probes the filesystem; the -nofs fallback
    # compiles that probe out, so its WASI imports are exactly those the first
    # successful MSFS run already proved the host provides. If the probing build
    # fails to instantiate, the fallback goes in without a rebuild.
    $optNoFs = $optCmd -replace 'dist/lvar-bench\.wasm', 'dist/lvar-bench-nofs.wasm'

    $script = @"
set -e
cd /external
mkdir -p dist
cargo build --target wasm32-wasip1 --release
$optCmd
cargo build --target wasm32-wasip1 --release --no-default-features
$optNoFs
"@

    Write-Host "Building in $DevEnvImage (MSFS SDK inside the image at /workdir/MSFS_SDK)..."
    Invoke-Native { & docker run --rm -v "${mount}:/external" $DevEnvImage bash -c $script } 'docker build'
}

if (-not (Test-Path $OutWasm)) { throw "Build reported success but $OutWasm does not exist." }

$size = (Get-Item $OutWasm).Length
Write-Host ''
Write-Host "Built: $OutWasm ($size bytes)"
$fallback = Join-Path $DistDir 'lvar-bench-nofs.wasm'
if (Test-Path $fallback) {
    Write-Host "Fallback: $fallback ($((Get-Item $fallback).Length) bytes) -- no filesystem probe."
    Write-Host '  Install it with .\install.ps1 -NoFsProbe only if the default build fails to load.'
}
Write-Host 'Next:  .\install.ps1     (copies it into the A380X package and registers the gauge)'
