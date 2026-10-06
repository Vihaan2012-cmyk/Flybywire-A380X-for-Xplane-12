<#
.SYNOPSIS
Builds the FlyByWire A380X for X-Plane 12 and MSFS 2020 from this repository and installs it.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File build.ps1
powershell -ExecutionPolicy Bypass -File build.ps1 -Only xplane -Sasl C:\Downloads\sasl.zip
powershell -ExecutionPolicy Bypass -File build.ps1 -Prebuilt
powershell -ExecutionPolicy Bypass -File build.ps1 -NoInstall
#>
[CmdletBinding()]
param(
    [ValidateSet('all', 'msfs', 'xplane')][string]$Only = 'all',
    [string]$XPlane,
    [string]$Community,
    [string]$Sasl,
    [switch]$Prebuilt,
    [switch]$Reconvert,
    [string]$AcfTemplate,
    [switch]$NoInstall,
    [string]$CargoTargetDir
)

$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
$Root = $PSScriptRoot
$Work = Join-Path $Root 'build'
$Stage = Join-Path $Work 'stage'
$Tar = Join-Path $env:SystemRoot 'System32\tar.exe'
$Assets = Get-Content (Join-Path $Root 'assets.json') -Raw | ConvertFrom-Json
if (-not $CargoTargetDir) {
    $CargoTargetDir = if ($env:CARGO_TARGET_DIR) { $env:CARGO_TARGET_DIR } else { Join-Path $Work 'target' }
}
$DoMsfs = $Only -ne 'xplane'
$DoXPlane = $Only -ne 'msfs'
$MsfsPackage = Join-Path $Stage 'msfs\flybywire-aircraft-a380-842'
$XPlaneAircraft = Join-Path $Stage 'xplane\FlyByWire A380X'
$A380X = 'html_ui\Pages\VCockpit\Instruments\A380X'
$Panel = 'SimObjects\AirPlanes\FlyByWire_A380_842\panel'
$FbwEnv = @('-e', 'NODE_ENV=production', '-e', 'AIRCRAFT_PROJECT_PREFIX=a380x', '-e', 'AIRCRAFT_VARIANT=a380-842',
    '-e', 'VITE_BUILD=false', '-e', 'CLIENT_ID=', '-e', 'CLIENT_SECRET=')
$Started = Get-Date

function Step([string]$Text) { Write-Host "`n== $Text" -ForegroundColor Cyan }
function Fail([string]$Text) { throw "build.ps1: $Text" }

function Invoke-Native([string]$Exe, [string[]]$Arguments) {
    & $Exe @Arguments
    if ($LASTEXITCODE -ne 0) { Fail "$Exe $($Arguments -join ' ') exited with $LASTEXITCODE" }
}

function Test-Quiet([scriptblock]$Block) {
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try { & $Block *> $null; $LASTEXITCODE -eq 0 } finally { $ErrorActionPreference = $saved }
}

function Invoke-Robocopy([string]$From, [string]$To, [string[]]$Extra = @()) {
    & robocopy $From $To /E /NFL /NDL /NJH /NJS /NP /R:2 /W:2 @Extra | Out-Null
    if ($LASTEXITCODE -ge 8) { Fail "robocopy $From -> $To failed ($LASTEXITCODE)" }
    $global:LASTEXITCODE = 0
}

function New-Junction([string]$Link, [string]$Target) {
    Remove-Junction $Link
    New-Item -ItemType Junction -Path $Link -Target $Target | Out-Null
}

function Remove-Junction([string]$Link) {
    if (Test-Path $Link) { cmd /c rmdir "$Link" | Out-Null }
}

function Assert-Fresh([string]$Base, [string[]]$Files, [string]$What) {
    $stale = $Files | Where-Object {
        $f = Join-Path $Base $_
        -not (Test-Path $f) -or (Get-Item $f).LastWriteTime -lt $Started
    }
    if ($stale) { Fail ("$What left these missing or stale:`n  " + ($stale -join "`n  ")) }
}

function Assert-Tools {
    $need = [ordered]@{ python = 'Python 3: https://www.python.org/downloads/' }
    if (-not $Prebuilt) {
        $need['docker'] = 'Docker Desktop: https://www.docker.com/products/docker-desktop/'
    }
    if (($DoXPlane -and -not $Prebuilt) -or $Reconvert) {
        $need['rustup'] = 'Rust: https://rustup.rs'
        $need['g++'] = 'MinGW-w64: winget install -e --id BrechtSanders.WinLibs.POSIX.UCRT'
    }
    $missing = foreach ($k in $need.Keys) {
        if (-not (Get-Command $k -ErrorAction SilentlyContinue)) { "$k  ($($need[$k]))" }
    }
    if ($missing) { Fail ("missing tools:`n  " + ($missing -join "`n  ")) }
    if (-not $Prebuilt -and -not (Test-Quiet { docker info })) { Fail 'Docker is installed but not running; start Docker Desktop' }
    if (($DoXPlane -and -not $Prebuilt) -or $Reconvert) {
        $have = (rustup toolchain list) -join "`n"
        foreach ($tc in @('stable-x86_64-pc-windows-gnu', 'stable-x86_64-pc-windows-msvc')) {
            if ($have -notmatch [regex]::Escape($tc)) { Invoke-Native rustup @('toolchain', 'install', $tc, '--profile', 'minimal') }
        }
    }
    if ($DoXPlane -and -not $Prebuilt) {
        $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
        $vc = if (Test-Path $vswhere) { & $vswhere -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath }
        if (-not $vc) {
            Fail 'XPHFBW needs the MSVC C++ build tools: winget install Microsoft.VisualStudio.2022.BuildTools --override "--quiet --wait --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"'
        }
    }
}

function Find-XPlane {
    if ($XPlane) { return $XPlane }
    $f = Join-Path $env:LOCALAPPDATA 'x-plane_install_12.txt'
    if (Test-Path $f) {
        foreach ($line in Get-Content $f) {
            $p = $line.Trim()
            if ($p -and (Test-Path (Join-Path $p 'X-Plane.exe'))) { return $p.TrimEnd('\', '/') }
        }
    }
}

function Find-Community {
    if ($Community) { return $Community }
    foreach ($f in @((Join-Path $env:APPDATA 'Microsoft Flight Simulator\UserCfg.opt'),
            (Join-Path $env:LOCALAPPDATA 'Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalCache\UserCfg.opt'))) {
        if (Test-Path $f) {
            $m = Select-String -Path $f -Pattern '^InstalledPackagesPath\s+"(.+)"' | Select-Object -First 1
            if ($m) { return Join-Path $m.Matches[0].Groups[1].Value 'Community' }
        }
    }
}

function Save-Part($Part, [string]$Dir) {
    $file = Join-Path $Dir $Part.file
    if (Get-Command gh -ErrorAction SilentlyContinue) {
        $saved = $ErrorActionPreference
        $ErrorActionPreference = 'Continue'
        try { & gh release download $Assets.tag -R $Assets.repo -p $Part.file -D $Dir --clobber } finally { $ErrorActionPreference = $saved }
        if ($LASTEXITCODE -eq 0) { return }
    }
    Invoke-Native curl.exe @('-fL', '--retry', '3', '-o', $file, "https://github.com/$($Assets.repo)/releases/download/$($Assets.tag)/$($Part.file)")
}

function Get-Asset([string]$Kind) {
    $asset = $Assets.assets.$Kind
    if (-not $asset) { Fail "assets.json has no $Kind asset" }
    $dir = Join-Path $Work "assets\$($Assets.tag)"
    New-Item -ItemType Directory -Force $dir | Out-Null
    foreach ($part in $asset.parts) {
        $file = Join-Path $dir $part.file
        if ((Test-Path $file) -and (Get-FileHash $file -Algorithm SHA256).Hash -eq $part.sha256) { continue }
        Write-Host ("downloading {0} ({1} MB)" -f $part.file, [math]::Round($part.size / 1MB))
        Save-Part $part $dir
        if ((Get-FileHash $file -Algorithm SHA256).Hash -ne $part.sha256) { Fail "$($part.file): checksum mismatch" }
    }
    $target = Join-Path $Stage $Kind
    if (Test-Path $target) { Remove-Item -Recurse -Force $target }
    New-Item -ItemType Directory -Force $target | Out-Null
    foreach ($part in $asset.parts) {
        Write-Host "unpacking $($part.file)"
        Invoke-Native $Tar @('-xf', (Join-Path $dir $part.file), '-C', $target)
    }
}

function Invoke-FbwDocker([string]$Tree, [string]$Command, [string[]]$Options = @()) {
    $m = Select-String -Path (Join-Path $Tree 'scripts\dev-env\run.cmd') -Pattern 'ghcr\.io/flybywiresim/dev-env@sha256:[0-9a-f]+' | Select-Object -First 1
    if (-not $m) { Fail "no dev-env image in $Tree\scripts\dev-env\run.cmd" }
    Invoke-Native docker (@('run', '--rm', '-v', "${Tree}:/external") + $Options + @($m.Matches[0].Value, 'bash', '-c', $Command))
}

function Build-Msfs {
    $tree = Join-Path $Root 'fbw\msfs'
    $out = Join-Path $tree 'fbw-a380x\out\flybywire-aircraft-a380-842'
    $panelOut = 'fbw-a380x/out/flybywire-aircraft-a380-842/SimObjects/AirPlanes/FlyByWire_A380_842/panel'
    New-Item -ItemType Directory -Force $CargoTargetDir | Out-Null

    Step 'MSFS: FlyByWire dependencies'
    Invoke-FbwDocker $tree 'cd /external && pnpm i'

    Step 'MSFS: instruments, hosts and WebAssembly'
    $cmd = @(
        'cd /external',
        'npm run build-a380x:copy-base-files',
        'npm run build-a380x:locPak-translation',
        'npm run build-a380x:extras-host',
        'npm run build-a380x:systems-host',
        '(npx mach build --config fbw-a380x/mach.config.js --work-in-config-dir || echo some instruments did not build)',
        'cargo build -p a380_systems_wasm --target wasm32-wasip1 --release',
        "wasm-opt -O1 --signext-lowering --enable-bulk-memory --enable-nontrapping-float-to-int -o $panelOut/systems.wasm /wasmtarget/wasm32-wasip1/release/a380_systems_wasm.wasm",
        'npm run build-a380x:fbw',
        'scripts/build-cmake.sh',
        '(npm run build-a380x:terronnd || echo terronnd did not build, keeping the released terronnd.wasm)'
    ) -join ' && '
    Invoke-FbwDocker $tree $cmd (@('-v', "${CargoTargetDir}:/wasmtarget", '-e', 'CARGO_TARGET_DIR=/wasmtarget') + $FbwEnv)

    Assert-Fresh $out @(
        "$Panel\systems.wasm", "$Panel\fbw.wasm", "$Panel\fadec-a380x.wasm", "$Panel\extra-backend-a380x.wasm",
        "$A380X\SystemsHost\SystemsHost.js", "$A380X\ExtrasHost\index.js", "$A380X\PFD\pfd.js", "$A380X\ND\nd.js",
        "$A380X\EWD\ewd.js", "$A380X\SD\sd.js", "$A380X\SDv2\sdv2.js", "$A380X\MFD\mfd.js", "$A380X\EFB\efb.js", "$A380X\OIT\oit.js"
    ) 'the MSFS build'
    $raw = Get-ChildItem (Join-Path $out $A380X) -Recurse -Filter *.js | Select-String -Pattern 'process\.env\.(?!NODE_DEBUG)[A-Za-z_]+' -List
    if ($raw) { Fail ("raw process.env in the built MSFS JS, which blacks out the cockpit:`n  " + (($raw | ForEach-Object Path) -join "`n  ")) }

    Step 'MSFS: package'
    Get-Asset msfs
    Invoke-Robocopy $out $MsfsPackage
    Invoke-Robocopy (Join-Path $Root 'msfs-tools\efb-data') (Join-Path $MsfsPackage "$A380X\EFB")
    Invoke-Native python @((Join-Path $Root 'msfs-tools\ecam-bridge\apply.py'), $MsfsPackage)
    Invoke-Native python @((Join-Path $Root 'tools\msfs_layout.py'), $MsfsPackage)
}

function Build-XPlaneInstruments {
    $tree = Join-Path $Root 'fbw\xplane'
    Step 'X-Plane: FlyByWire dependencies'
    Invoke-FbwDocker $tree 'cd /external && pnpm i'

    Step 'X-Plane: FlyByWire instruments and hosts'
    $cmd = @(
        'cd /external',
        'npm run build-a380x:copy-base-files',
        'npm run build-a380x:extras-host',
        'npm run build-a380x:systems-host',
        '(npx mach build --config fbw-a380x/mach.config.js --work-in-config-dir || echo some instruments did not build)'
    ) -join ' && '
    Invoke-FbwDocker $tree $cmd $FbwEnv
    Assert-Fresh (Join-Path $tree 'fbw-a380x\out\flybywire-aircraft-a380-842') @(
        "$A380X\PFD\pfd.js", "$A380X\ND\nd.js", "$A380X\EWD\ewd.js", "$A380X\SDv2\sdv2.js", "$A380X\MFD\mfd.js", "$A380X\SystemsHost\SystemsHost.js"
    ) 'the X-Plane instrument build'
}

function Build-Plugin {
    $ws = Join-Path $Work 'ws'
    $plugin = Join-Path $ws 'plugin'
    $fbw = Join-Path $ws 'fbw-aircraft'
    New-Item -ItemType Directory -Force $ws | Out-Null
    New-Junction $plugin $Root
    New-Junction $fbw (Join-Path $Root 'fbw\xplane')
    $env:CARGO_TARGET_DIR = $CargoTargetDir
    try {
        Step 'X-Plane: plugin'
        Push-Location $plugin
        try { Invoke-Native cargo @('+stable-x86_64-pc-windows-gnu', 'build', '--release', '--features', 'js') } finally { Pop-Location }

        Step 'X-Plane: XPHFBW app'
        $env:CEF_PATH = Join-Path $Stage 'cef\cef'
        Push-Location (Join-Path $plugin 'app')
        try { Invoke-Native cargo @('+stable-x86_64-pc-windows-msvc', 'build', '--release', '--target', 'x86_64-pc-windows-msvc') } finally { Pop-Location }
    } finally {
        Remove-Junction $plugin
        Remove-Junction $fbw
    }
}

function Install-XPlanePieces {
    $appOut = Join-Path $CargoTargetDir 'x86_64-pc-windows-msvc\release'
    $plugDir = Join-Path $XPlaneAircraft 'plugins\fbw_a380_systems'
    $bin = Join-Path $plugDir '64'
    $x = Join-Path $plugDir 'XPHFBW'
    foreach ($d in @($bin, $x, (Join-Path $x 'locales'), (Join-Path $x 'ui'), (Join-Path $x 'js'))) { New-Item -ItemType Directory -Force $d | Out-Null }

    Step 'X-Plane: aircraft'
    $dll = Get-Item (Join-Path $CargoTargetDir 'release\fbw_a380_systems.dll')
    if ($dll.Length -lt 20MB) { Fail "fbw_a380_systems.dll is $([math]::Round($dll.Length / 1MB)) MB; a build without the js feature has dead displays" }
    Copy-Item $dll.FullName (Join-Path $bin 'win.xpl') -Force
    Copy-Item (Join-Path $CargoTargetDir 'release\fbw_a380_systems_server.exe') $bin -Force
    Copy-Item (Join-Path $appOut 'XPHFBW.exe') $x -Force
    Get-ChildItem $appOut -File | Where-Object { $_.Extension -in '.dll', '.pak', '.bin', '.dat' -and $_.Name -ne 'fbw_a380_systems.dll' } |
        Copy-Item -Destination $x -Force
    Copy-Item (Join-Path $appOut 'vk_swiftshader_icd.json') $x -Force
    Copy-Item (Join-Path $appOut 'locales\*.pak') (Join-Path $x 'locales') -Force
    Invoke-Robocopy (Join-Path $Root 'app\ui') (Join-Path $x 'ui')
    Invoke-Robocopy (Join-Path $Root 'app\js') (Join-Path $x 'js')

    $msfsPanel = Join-Path $MsfsPackage $Panel
    if (Test-Path $msfsPanel) {
        Copy-Item (Join-Path $msfsPanel 'panel.cfg'), (Join-Path $msfsPanel 'panel.xml') (Join-Path $XPlaneAircraft 'panel') -Force
    }

    $built = Join-Path $Root 'fbw\xplane\fbw-a380x\out\flybywire-aircraft-a380-842'
    $instruments = Join-Path $built $A380X
    Invoke-Robocopy (Join-Path $built 'html_ui') (Join-Path $XPlaneAircraft 'html_ui') @(
        '/XD', (Join-Path $instruments 'EFB'), (Join-Path $instruments 'OITlegacy'), (Join-Path $instruments 'popup'))
    Get-ChildItem $built -Filter *.json -File | Copy-Item -Destination $XPlaneAircraft -Force
    if (-not (Test-Path (Join-Path $XPlaneAircraft "$A380X\PFD\pfd.html"))) {
        Fail 'pfd.html is missing after the html_ui merge; every cockpit screen would be black'
    }
}

function Invoke-Reconvert([string]$XPlaneRoot) {
    if (-not (Test-Path $MsfsPackage)) { Get-Asset msfs }
    $template = $AcfTemplate
    if (-not $template -and $XPlaneRoot) { $template = Join-Path $XPlaneRoot 'Aircraft\Laminar Research\Airbus A330-300\A330.acf' }
    if (-not $template -or -not (Test-Path $template)) { Fail "-Reconvert needs X-Plane's Laminar A330.acf; pass -AcfTemplate" }

    Step 'X-Plane: converting the MSFS model'
    $env:CARGO_TARGET_DIR = $CargoTargetDir
    Push-Location (Join-Path $Root 'msfs2xp-aircraft')
    try {
        Invoke-Native cargo @('+stable-x86_64-pc-windows-gnu', 'run', '--release', '--', $MsfsPackage, '-o', (Split-Path $XPlaneAircraft),
            '--name', 'FlyByWire A380X', '--cg-z=-8', '--vmo', '340', '--mmo', '0.89', '--acf-template', $template)
    } finally { Pop-Location }
}

function Install-Sasl([string]$Dest) {
    $src = $Sasl
    if ($Sasl -like '*.zip') {
        $src = Join-Path $Work 'sasl'
        if (Test-Path $src) { Remove-Item -Recurse -Force $src }
        New-Item -ItemType Directory -Force $src | Out-Null
        Invoke-Native $Tar @('-xf', $Sasl, '-C', $src)
    }
    $xpl = Get-ChildItem $src -Recurse -Filter win.xpl | Where-Object { $_.Directory.Name -eq '64' } | Select-Object -First 1
    if (-not $xpl) { Fail "no 64\win.xpl in $Sasl; is it SASL 3 for X-Plane?" }
    Invoke-Robocopy $xpl.Directory.Parent.FullName (Join-Path $Dest 'plugins\sasl')
}

function Install-Msfs([string]$CommunityDir) {
    if (-not $CommunityDir) { Write-Warning "MSFS Community folder not found; pass -Community. The package is in $MsfsPackage"; return }
    if (Get-Process FlightSimulator -ErrorAction SilentlyContinue) { Fail 'close MSFS first' }
    $dest = Join-Path $CommunityDir 'flybywire-aircraft-a380-842'
    Step "MSFS: installing into $dest"
    Invoke-Robocopy $MsfsPackage $dest @('/MIR')
}

function Install-XPlane([string]$XPlaneRoot) {
    if (-not $XPlaneRoot) { Write-Warning "X-Plane 12 not found; pass -XPlane. The aircraft is in $XPlaneAircraft"; return }
    if (Get-Process 'X-Plane' -ErrorAction SilentlyContinue) { Fail 'close X-Plane first' }
    $dest = Join-Path $XPlaneRoot 'Aircraft\FlyByWire A380X'
    Step "X-Plane: installing into $dest"
    if ($Sasl) { Install-Sasl $dest }
    Invoke-Robocopy $XPlaneAircraft $dest
    if (-not (Test-Path (Join-Path $dest 'plugins\sasl\64\win.xpl'))) {
        Write-Warning 'SASL 3 is not in the aircraft (it is proprietary and not bundled). Download it and run again with -Sasl <zip>.'
    }
}

Assert-Tools
$xplaneRoot = Find-XPlane
$communityDir = Find-Community

if ($DoMsfs) {
    if ($Prebuilt) { Step 'MSFS: released package'; Get-Asset msfs } else { Build-Msfs }
}
if ($DoXPlane) {
    if (-not $Prebuilt) { Build-XPlaneInstruments; Get-Asset cef; Build-Plugin }
    Step 'X-Plane: released aircraft'
    Get-Asset xplane
    if ($Reconvert) { Invoke-Reconvert $xplaneRoot }
    if (-not $Prebuilt) { Install-XPlanePieces }
}
if (-not $NoInstall) {
    if ($DoMsfs) { Install-Msfs $communityDir }
    if ($DoXPlane) { Install-XPlane $xplaneRoot }
}

Step ('done in {0:hh\:mm\:ss}' -f ((Get-Date) - $Started))
if ($DoMsfs) { Write-Host "MSFS package:      $MsfsPackage" }
if ($DoXPlane) { Write-Host "X-Plane aircraft:  $XPlaneAircraft" }
