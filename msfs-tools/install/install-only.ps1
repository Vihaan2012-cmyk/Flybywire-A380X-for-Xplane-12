param(
    [Parameter(Mandatory = $true)][string[]]$Items,
    [string]$Package = "D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842",
    [string]$OutDir = (Join-Path $PSScriptRoot "out"),
    [switch]$Ecam,
    [switch]$WhatIf
)

$ErrorActionPreference = "Stop"
if ($Items | Where-Object { $_ -match '^(SystemsHost|EWD|SDv2|MFD|PFD)/' }) { $Ecam = $true }
if ($Ecam) {
    foreach ($b in @("EWD/ewd.js", "SDv2/sdv2.js", "SystemsHost/SystemsHost.js")) {
        if ($Items -notcontains $b -and (Test-Path (Join-Path $OutDir $b))) { $Items += $b }
    }
}
if (Get-Process -Name "FlightSimulator" -ErrorAction SilentlyContinue) { throw "Close MSFS first (FlightSimulator.exe is running)." }
$Layout = Join-Path $Package "layout.json"
if (-not (Test-Path $Layout)) { throw "No layout.json in $Package" }
$BackupRoot = "E:/fbw-backup/{0}-install-only" -f (Get-Date -Format "yyyy-MM-dd-HHmm")

function Dest-Of([string]$item) {
    if ($item -like "*.wasm") { return "SimObjects/AirPlanes/FlyByWire_A380_842/panel/$item" }
    return "html_ui/Pages/VCockpit/Instruments/A380X/" + ($item -replace '\\', '/')
}

$plan = foreach ($item in $Items) {
    $src = Join-Path $OutDir ($item -replace '/', '\')
    if (-not (Test-Path $src)) { throw "No build output $src" }
    $rel = Dest-Of $item
    [PSCustomObject]@{ Item = $item; Source = $src; Rel = $rel; Full = (Join-Path $Package ($rel -replace '/', '\')) }
}

foreach ($p in $plan) { Write-Host ("  {0,-26} -> {1}" -f $p.Item, $p.Rel) }
if ($WhatIf) { Write-Host "[dry run] nothing changed"; return }

New-Item -ItemType Directory -Force $BackupRoot | Out-Null
Copy-Item $Layout (Join-Path $BackupRoot "layout.json") -Force
foreach ($p in $plan) {
    if (Test-Path $p.Full) {
        $b = Join-Path $BackupRoot $p.Rel
        New-Item -ItemType Directory -Force (Split-Path $b) | Out-Null
        Copy-Item $p.Full $b -Force
    }
    if (Test-Path "$($p.Full).pre-ecam") {
        Move-Item "$($p.Full).pre-ecam" "$(Join-Path $BackupRoot $p.Rel).pre-ecam" -Force
    }
    New-Item -ItemType Directory -Force (Split-Path $p.Full) | Out-Null
    Copy-Item $p.Source $p.Full -Force
    Write-Host "  installed $($p.Rel)"
}

$text = [System.IO.File]::ReadAllText($Layout)
foreach ($p in $plan) {
    $file = Get-Item $p.Full
    $pattern = '("path":\s*"' + [regex]::Escape($p.Rel) + '",\s*"size":\s*)\d+(,\s*"date":\s*)\d+'
    if ($text -match $pattern) {
        $text = [regex]::Replace($text, $pattern, "`${1}$($file.Length)`${2}$($file.LastWriteTimeUtc.ToFileTimeUtc())")
    } else {
        $entry = ",`n    {`n      `"path`": `"$($p.Rel)`",`n      `"size`": $($file.Length),`n      `"date`": $($file.LastWriteTimeUtc.ToFileTimeUtc())`n    }`n  ]"
        $text = [regex]::Replace($text, '\s*\]\s*\}\s*$', "$entry`n}")
        Write-Host "  layout.json: new entry $($p.Rel)"
    }
}
[System.IO.File]::WriteAllText($Layout, $text)
$null = [System.IO.File]::ReadAllText($Layout) | ConvertFrom-Json
Write-Host "layout.json updated; backups in $BackupRoot"

if ($Ecam) {
    & python "D:/A380/fbw-build/wasm-fs2020/ecam-msfs/apply.py" $Package
    if ($LASTEXITCODE -ne 0) { throw "ecam apply.py failed (exit $LASTEXITCODE)" }
}
