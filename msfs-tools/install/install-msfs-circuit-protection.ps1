# Installs circuit protection into the MSFS 2020 FlyByWire A380X Development
# build (commit 672384b), or restores the package exactly as it was.
#
#   .\install-msfs-circuit-protection.ps1             install everything in .\out
#   .\install-msfs-circuit-protection.ps1 -Restore    put every original file back
#   .\install-msfs-circuit-protection.ps1 -SkipEfb   everything but the EFB bundle
#   .\install-msfs-circuit-protection.ps1 -Package "<path to flybywire-aircraft-a380-842>"
#
# What it installs (each replaced file is kept once as <file>.original):
#   out\systems.wasm        -> SimObjects\AirPlanes\FlyByWire_A380_842\panel\systems.wasm
#   out\A380_COCKPIT.xml    -> SimObjects\AirPlanes\FlyByWire_A380_842\model\A380_COCKPIT.xml
#   out\EFB\*               -> html_ui\Pages\VCockpit\Instruments\A380X\EFB\
#   out\catalogue.json      -> html_ui\Pages\VCockpit\Instruments\A380X\EFB\catalogue.json (new)
# and keeps every file's entry in layout.json true (layout.json.original is
# the untouched one). MSFS must be closed.

param(
    [string]$Package = "D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842",
    [switch]$Restore,
    # Keep FlyByWire's own EFB (no Study tab). This build's EFB has no
    # Navigraph client credentials, so Navigraph sign-in does not work in it.
    [switch]$SkipEfb
)

$ErrorActionPreference = "Stop"
$Commit = "672384b9602a84d6832b8852cd6ec0abdd006f85"
$Out = Join-Path $PSScriptRoot "out"
$Layout = Join-Path $Package "layout.json"
$EfbDir = "html_ui/Pages/VCockpit/Instruments/A380X/EFB"
# Files this script adds that the package did not have, for -Restore to remove.
$AddedList = Join-Path $Package "circuit-protection-added.txt"

if (Get-Process -Name "FlightSimulator" -ErrorAction SilentlyContinue) { throw "Close MSFS first." }
if (-not (Test-Path $Layout)) { throw "No layout.json in $Package -- is this the A380X package?" }

# Development builds all read 2020.16.0; only the commit tells them apart.
$manifest = Get-Content (Join-Path $Package "manifest.json") -Raw | ConvertFrom-Json
if ($manifest.package_version -notlike "*$Commit*" -and -not $Restore) {
    throw "Package is $($manifest.package_version); this was built for 2020.16.0-672384b. The installer has moved you to another commit: rebuild from that commit (INTEGRATION.md, 'Rebuilding it yourself')."
}

function Full([string]$rel) { Join-Path $Package ($rel -replace '/', '\') }

# (relative path in the package, source file) for everything we install.
$Items = @(
    @("SimObjects/AirPlanes/FlyByWire_A380_842/panel/systems.wasm", (Join-Path $Out "systems.wasm")),
    @("SimObjects/AirPlanes/FlyByWire_A380_842/model/A380_COCKPIT.xml", (Join-Path $Out "A380_COCKPIT.xml")),
    @("$EfbDir/catalogue.json", (Join-Path $Out "catalogue.json"))
)
$EfbOut = Join-Path $Out "EFB"
if ((Test-Path $EfbOut) -and -not $SkipEfb) {
    Get-ChildItem $EfbOut -File -Recurse | ForEach-Object {
        $rel = $_.FullName.Substring($EfbOut.Length + 1) -replace '\\', '/'
        $Items += , @("$EfbDir/$rel", $_.FullName)
    }
}

$text = [System.IO.File]::ReadAllText($Layout)

function Set-LayoutEntry([string]$rel) {
    $file = Get-Item (Full $rel)
    $size = $file.Length
    $date = $file.LastWriteTimeUtc.ToFileTimeUtc()
    $pattern = '("path":\s*"' + [regex]::Escape($rel) + '",\s*"size":\s*)\d+(,\s*"date":\s*)\d+'
    if ($script:text -match $pattern) {
        $script:text = [regex]::Replace($script:text, $pattern, "`${1}$size`${2}$date")
    } else {
        # A file the package did not list: add its entry before the closing bracket.
        $entry = ",`n    {`n      `"path`": `"$rel`",`n      `"size`": $size,`n      `"date`": $date`n    }`n  ]"
        $script:text = [regex]::Replace($script:text, '\s*\]\s*\}\s*$', "$entry`n}")
    }
}

if ($Restore) {
    # Every file this script (or the earlier install-systems-wasm.ps1) replaced
    # has its original beside it; put each back, whatever .\out holds now.
    $dirs = @("SimObjects/AirPlanes/FlyByWire_A380_842/panel", "SimObjects/AirPlanes/FlyByWire_A380_842/model", $EfbDir)
    foreach ($dir in $dirs) {
        if (-not (Test-Path (Full $dir))) { continue }
        Get-ChildItem (Full $dir) -Filter "*.original" -File -Recurse | ForEach-Object {
            $target = $_.FullName.Substring(0, $_.FullName.Length - ".original".Length)
            Copy-Item $_.FullName $target -Force
            Remove-Item $_.FullName
            Write-Host "restored $target"
        }
    }
    if (Test-Path $AddedList) {
        foreach ($rel in Get-Content $AddedList) {
            $target = Full $rel
            if (Test-Path $target) { Remove-Item $target; Write-Host "removed $rel" }
        }
        Remove-Item $AddedList
    }
    if (Test-Path "$Layout.original") {
        Copy-Item "$Layout.original" $Layout -Force
        Remove-Item "$Layout.original"
        Write-Host "restored layout.json"
    }
    return
}

foreach ($item in $Items) {
    if (-not (Test-Path $item[1])) { throw "Missing build output $($item[1])" }
}
if (-not (Test-Path "$Layout.original")) { Copy-Item $Layout "$Layout.original" }

$added = @()
if (Test-Path $AddedList) { $added = @(Get-Content $AddedList) }
foreach ($item in $Items) {
    $rel = $item[0]
    $target = Full $rel
    if (Test-Path $target) {
        if (-not (Test-Path "$target.original") -and -not ($added -contains $rel)) { Copy-Item $target "$target.original" }
    } else {
        New-Item -ItemType Directory -Force (Split-Path $target) | Out-Null
        if (-not ($added -contains $rel)) { $added += $rel }
    }
    Copy-Item $item[1] $target -Force
    Set-LayoutEntry $rel
    Write-Host "installed $rel"
}
if ($added.Count -gt 0) { Set-Content -Path $AddedList -Value $added -Encoding ASCII }
[System.IO.File]::WriteAllText($Layout, $text)
# The layout.json must still parse.
$null = [System.IO.File]::ReadAllText($Layout) | ConvertFrom-Json
Write-Host "layout.json updated ($($Items.Count) entries). Start MSFS: the first load recompiles the module."
