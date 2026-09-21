# Installs (or removes) the LVar benchmark module in the FlyByWire A380X package.
#
#   .\install.ps1              Install.
#   .\install.ps1 -Uninstall   Put everything back exactly as it was.
#   .\install.ps1 -PackageRoot "<path>"   Override package auto-detection.
#
# What installing does, and nothing else:
#   1. copies dist\deep-wasm.wasm next to FlyByWire's systems.wasm;
#   2. adds ONE htmlgauge line to panel.cfg, in the same [VCockpitNN] section
#      that already loads their four WASM modules;
#   3. adds ONE entry to layout.json, which MSFS requires for every file in a
#      package or it will not be loaded.
#
# panel.cfg and layout.json are backed up verbatim before either is touched, and
# -Uninstall restores the backups. No FlyByWire file is rewritten, only appended
# to, and the benchmark is meant to be uninstalled once it has answered its
# question.

[CmdletBinding()]
param(
    [switch]$Uninstall,
    [string]$PackageRoot
)

$ErrorActionPreference = 'Stop'

$CrateRoot   = $PSScriptRoot
$SourceWasm  = Join-Path $CrateRoot 'dist\deep.wasm'
# The installed name stays the same either way, so panel.cfg does not change.
$WasmName    = 'deep.wasm'
$GaugeLine   = "WasmInstrument/WasmInstrument.html?wasm_module=$WasmName&wasm_gauge=deep, 0,0,1,1"
$BackupSuffix = '.deep-backup'

function Write-Utf8NoBom([string]$Path, [string[]]$Lines) {
    $enc = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($Path, ($Lines -join "`r`n") + "`r`n", $enc)
}

function Find-PackageRoot {
    if ($PackageRoot) {
        if (-not (Test-Path $PackageRoot)) { throw "-PackageRoot '$PackageRoot' does not exist." }
        return $PackageRoot
    }

    $cfgs = @(
        "$env:LOCALAPPDATA\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalCache\UserCfg.opt",  # MSFS 2020, Store
        "$env:APPDATA\Microsoft Flight Simulator\UserCfg.opt",                                        # MSFS 2020, Steam
        "$env:LOCALAPPDATA\Packages\Microsoft.Limitless_8wekyb3d8bbwe\LocalCache\UserCfg.opt",         # MSFS 2024, Store
        "$env:APPDATA\Microsoft Flight Simulator 2024\UserCfg.opt"                                     # MSFS 2024, Steam
    )

    foreach ($cfg in $cfgs) {
        if (-not (Test-Path $cfg)) { continue }
        $line = Select-String -Path $cfg -Pattern 'InstalledPackagesPath' | Select-Object -First 1
        if (-not $line) { continue }
        if ($line.Line -notmatch '"([^"]+)"') { continue }
        $community = Join-Path $Matches[1] 'Community'
        $pkg = Join-Path $community 'flybywire-aircraft-a380-842'
        if (Test-Path $pkg) {
            Write-Host "Found A380X package via $cfg"
            return $pkg
        }
    }

    throw @"
Could not find the flybywire-aircraft-a380-842 package.
Looked for InstalledPackagesPath in:
$($cfgs -join "`n")
Pass the package folder explicitly, e.g.:
  .\install.ps1 -PackageRoot "D:\...\Community\flybywire-aircraft-a380-842"
"@
}

# The panel directory is wherever systems.wasm already lives. That covers both
# the flat SimObjects\AirPlanes\FlyByWire_A380_842\panel layout of the shipped
# fs2020 builds and the modular
# SimObjects\AirPlanes\FlyByWire_A380X\attachments\...\panel layout of the
# current source tree, without guessing which one is installed.
function Find-PanelDir([string]$pkg) {
    $hit = Get-ChildItem -Path (Join-Path $pkg 'SimObjects') -Recurse -Filter 'systems.wasm' -File -ErrorAction SilentlyContinue |
           Select-Object -First 1
    if (-not $hit) { throw "No systems.wasm found under $pkg\SimObjects -- is this really the A380X package?" }
    return $hit.DirectoryName
}

$pkg      = Find-PackageRoot
$panelDir = Find-PanelDir $pkg
$panelCfg = Join-Path $panelDir 'panel.cfg'
$layout   = Join-Path $pkg 'layout.json'
$destWasm = Join-Path $panelDir $WasmName

if (-not (Test-Path $panelCfg)) { throw "No panel.cfg in $panelDir" }
if (-not (Test-Path $layout))   { throw "No layout.json in $pkg" }

Write-Host "Package : $pkg"
Write-Host "Panel   : $panelDir"

# ---------------------------------------------------------------- uninstall --
if ($Uninstall) {
    foreach ($f in @($panelCfg, $layout)) {
        $bak = "$f$BackupSuffix"
        if (Test-Path $bak) {
            Copy-Item $bak $f -Force
            Remove-Item $bak -Force
            Write-Host "Restored $f"
        } else {
            Write-Host "No backup for $f -- left alone."
        }
    }
    if (Test-Path $destWasm) {
        Remove-Item $destWasm -Force
        Write-Host "Removed $destWasm"
    }
    Write-Host 'Uninstalled. Restart MSFS (or reload the aircraft) for it to take effect.'
    exit 0
}

# ------------------------------------------------------------------ install --
if (-not (Test-Path $SourceWasm)) {
    throw "$SourceWasm does not exist. Run .\build.ps1 first."
}

foreach ($f in @($panelCfg, $layout)) {
    $bak = "$f$BackupSuffix"
    if (-not (Test-Path $bak)) {
        Copy-Item $f $bak -Force
        Write-Host "Backed up $f -> $bak"
    } else {
        # Re-installing: start from the pristine backup so lines never double up.
        Copy-Item $bak $f -Force
        Write-Host "Reset $f from its backup before re-installing."
    }
}

Copy-Item $SourceWasm $destWasm -Force
Write-Host "Copied  $destWasm"

# --- panel.cfg: one extra htmlgauge line in the section that loads the others.
$lines = [System.IO.File]::ReadAllLines($panelCfg)
$sectionStart = -1
for ($i = 0; $i -lt $lines.Count; $i++) {
    if ($lines[$i] -match 'wasm_module=systems\.wasm') { $sectionStart = $i; break }
}
if ($sectionStart -lt 0) { throw "panel.cfg has no line loading systems.wasm; refusing to guess where to add the gauge." }

# Walk to the end of that [VCockpitNN] block, remembering the highest
# htmlgaugeNN index used and the last htmlgauge line's position.
$lastGauge = $sectionStart
$maxIdx = -1
for ($i = $sectionStart; $i -lt $lines.Count; $i++) {
    if ($lines[$i] -match '^\s*\[') { break }
    if ($lines[$i] -match '^\s*htmlgauge(\d+)\s*=') {
        $lastGauge = $i
        $idx = [int]$Matches[1]
        if ($idx -gt $maxIdx) { $maxIdx = $idx }
    }
}
$newIdx = ($maxIdx + 1).ToString('00')
$newLine = "htmlgauge$newIdx=$GaugeLine"

$out = New-Object System.Collections.Generic.List[string]
for ($i = 0; $i -lt $lines.Count; $i++) {
    $out.Add($lines[$i])
    if ($i -eq $lastGauge) { $out.Add($newLine) }
}
Write-Utf8NoBom $panelCfg $out
Write-Host "panel.cfg: added  $newLine"

# --- layout.json: one extra entry, inserted textually next to systems.wasm's so
#     the rest of the file keeps its exact formatting.
$ll = [System.IO.File]::ReadAllLines($layout)
$anchor = -1
for ($i = 0; $i -lt $ll.Count; $i++) {
    if ($ll[$i] -match '"path":\s*"([^"]*panel/systems\.wasm)"') { $anchor = $i; break }
}
if ($anchor -lt 1) { throw "layout.json has no entry for panel/systems.wasm; refusing to guess the format." }

$systemsPath = $Matches[1]
$ourPath = $systemsPath -replace 'systems\.wasm$', $WasmName
if ($ll[$anchor - 1] -notmatch '^\s*\{\s*$') { throw "layout.json is not in the expected one-key-per-line shape; not editing it." }

$item  = Get-Item $destWasm
$size  = $item.Length
$date  = $item.LastWriteTimeUtc.ToFileTimeUtc()
$indent = ($ll[$anchor - 1] -replace '\{.*$', '')

$block = @(
    "$indent{",
    "$indent  `"path`": `"$ourPath`",",
    "$indent  `"size`": $size,",
    "$indent  `"date`": $date",
    "$indent},"
)

$lout = New-Object System.Collections.Generic.List[string]
for ($i = 0; $i -lt $ll.Count; $i++) {
    if ($i -eq ($anchor - 1)) { $block | ForEach-Object { $lout.Add($_) } }
    $lout.Add($ll[$i])
}
Write-Utf8NoBom $layout $lout
Write-Host "layout.json: added entry for $ourPath ($size bytes)"

Write-Host ''
Write-Host 'Installed. Now:'
Write-Host '  1. Start MSFS and load the A380X at a gate (any airport, engines off is fine).'
Write-Host '  2. Open the console: Options > General > Developers > Developer Mode ON,'
Write-Host '     then the dev toolbar''s Console window (filter it on DEEP).'
Write-Host '  3. Wait ~35 seconds from aircraft load. The results block prints itself.'
Write-Host '  4. Or watch the DEEP_* variables in the dev toolbar''s Behaviors window.'
Write-Host ''
Write-Host 'When you are done:  .\install.ps1 -Uninstall'
