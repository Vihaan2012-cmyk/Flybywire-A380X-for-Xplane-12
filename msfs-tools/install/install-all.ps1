param(
    [string]$Package = "D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842",
    [string]$OutDir = (Join-Path $PSScriptRoot "out"),
    [switch]$WhatIf,
    [switch]$SkipEcam
)

$ErrorActionPreference = "Stop"
$Commit = "672384b9602a84d6832b8852cd6ec0abdd006f85"
$BackupRoot = "E:/fbw-backup/{0}-msfs-authority-install" -f (Get-Date -Format "yyyy-MM-dd")
$Layout = Join-Path $Package "layout.json"
$Manifest = Join-Path $Package "manifest.json"

$EcamApply = "D:/A380/fbw-build/wasm-fs2020/ecam-msfs/apply.py"
$EcamPatches = "D:/A380/fbw-build/wasm-fs2020/ecam-msfs/patches-msfs.json"

function Say($msg) { Write-Host $msg }
function SayDry($msg) { Write-Host "[dry run] $msg" -ForegroundColor Yellow }

if (Get-Process -Name "FlightSimulator" -ErrorAction SilentlyContinue) {
    throw "Close MSFS first (FlightSimulator.exe is running)."
}
if (-not (Test-Path $Manifest)) { throw "No manifest.json in $Package -- is this the A380X package?" }
if (-not (Test-Path $Layout)) { throw "No layout.json in $Package" }

$manifestObj = Get-Content $Manifest -Raw | ConvertFrom-Json
if ($manifestObj.package_version -notlike "*$Commit*") {
    throw "Package is $($manifestObj.package_version); this installer is for 2020.16.0-672384b. Rebuild from that commit first (see INTEGRATION.md)."
}
Say "Package version OK: $($manifestObj.package_version)"

function Full([string]$rel) { Join-Path $Package ($rel -replace '/', '\') }

$Artefacts = @(
    @{ Name = "systems.wasm";            Source = (Join-Path $OutDir "systems.wasm");            Dest = "SimObjects/AirPlanes/FlyByWire_A380_842/panel/systems.wasm" }
    @{ Name = "fbw.wasm";                Source = (Join-Path $OutDir "fbw.wasm");                Dest = "SimObjects/AirPlanes/FlyByWire_A380_842/panel/fbw.wasm" }
    @{ Name = "ewd.js";  Source = (Join-Path $OutDir "EWD/ewd.js");  Dest = "html_ui/Pages/VCockpit/Instruments/A380X/EWD/ewd.js" }
    @{ Name = "ewd.css"; Source = (Join-Path $OutDir "EWD/ewd.css"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/EWD/ewd.css" }
    @{ Name = "ewd.html"; Source = (Join-Path $OutDir "EWD/ewd.html"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/EWD/ewd.html" }
    @{ Name = "sd.js";   Source = (Join-Path $OutDir "SD/sd.js");   Dest = "html_ui/Pages/VCockpit/Instruments/A380X/SD/sd.js" }
    @{ Name = "sd.css";  Source = (Join-Path $OutDir "SD/sd.css");  Dest = "html_ui/Pages/VCockpit/Instruments/A380X/SD/sd.css" }
    @{ Name = "sd.html"; Source = (Join-Path $OutDir "SD/sd.html"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/SD/sd.html" }
    @{ Name = "efb.js";  Source = (Join-Path $OutDir "EFB/efb.js");  Dest = "html_ui/Pages/VCockpit/Instruments/A380X/EFB/efb.js" }
    @{ Name = "efb.css"; Source = (Join-Path $OutDir "EFB/efb.css"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/EFB/efb.css" }
    @{ Name = "pfd.js"; Source = (Join-Path $OutDir "PFD/pfd.js"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/PFD/pfd.js" }
    @{ Name = "pfd.css"; Source = (Join-Path $OutDir "PFD/pfd.css"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/PFD/pfd.css" }
    @{ Name = "pfd.html"; Source = (Join-Path $OutDir "PFD/pfd.html"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/PFD/pfd.html" }
    @{ Name = "nd.js"; Source = (Join-Path $OutDir "ND/nd.js"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/ND/nd.js" }
    @{ Name = "nd.css"; Source = (Join-Path $OutDir "ND/nd.css"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/ND/nd.css" }
    @{ Name = "nd.html"; Source = (Join-Path $OutDir "ND/nd.html"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/ND/nd.html" }
    @{ Name = "sdv2.js"; Source = (Join-Path $OutDir "SDv2/sdv2.js"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/SDv2/sdv2.js" }
    @{ Name = "sdv2.css"; Source = (Join-Path $OutDir "SDv2/sdv2.css"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/SDv2/sdv2.css" }
    @{ Name = "sdv2.html"; Source = (Join-Path $OutDir "SDv2/sdv2.html"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/SDv2/sdv2.html" }
    @{ Name = "mfd.js"; Source = (Join-Path $OutDir "MFD/mfd.js"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/MFD/mfd.js" }
    @{ Name = "mfd.css"; Source = (Join-Path $OutDir "MFD/mfd.css"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/MFD/mfd.css" }
    @{ Name = "mfd.html"; Source = (Join-Path $OutDir "MFD/mfd.html"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/MFD/mfd.html" }
    @{ Name = "oit.js"; Source = (Join-Path $OutDir "OIT/oit.js"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/OIT/oit.js" }
    @{ Name = "oit.css"; Source = (Join-Path $OutDir "OIT/oit.css"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/OIT/oit.css" }
    @{ Name = "oit.html"; Source = (Join-Path $OutDir "OIT/oit.html"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/OIT/oit.html" }
    @{ Name = "catalogue.json"; Source = (Join-Path $OutDir "EFB/catalogue.json"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/EFB/catalogue.json" }
    @{ Name = "consequences.json"; Source = (Join-Path $OutDir "EFB/consequences.json"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/EFB/consequences.json" }
    @{ Name = "study-pages.json"; Source = (Join-Path $OutDir "EFB/study-pages.json"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/EFB/study-pages.json" }
    @{ Name = "SystemsHost.js"; Source = (Join-Path $OutDir "SystemsHost/SystemsHost.js"); Dest = "html_ui/Pages/VCockpit/Instruments/A380X/SystemsHost/SystemsHost.js" }
)

$layoutText = Get-Content $Layout -Raw

function Resolve-LayoutEntry([string]$rel) {
    $pattern = '"path":\s*"' + [regex]::Escape($rel) + '",\s*"size":\s*(\d+),\s*"date":\s*(\d+)'
    if ($script:layoutText -match $pattern) {
        return @{ Found = $true; Size = $Matches[1]; Date = $Matches[2] }
    }
    return @{ Found = $false }
}

Say ""
Say "=== artefact -> destination (verified against layout.json) ==="
$resolved = @()
foreach ($a in $Artefacts) {
    $entry = Resolve-LayoutEntry $a.Dest
    $destFull = Full $a.Dest
    $srcExists = Test-Path $a.Source
    $destExists = Test-Path $destFull
    $status = if ($entry.Found) { "in layout.json (size=$($entry.Size))" } else { "NOT in layout.json -- will be added as a new entry" }
    $srcStatus = if ($srcExists) { "source present" } else { "source MISSING (build not run yet)" }
    Say ("  {0,-24} -> {1,-70} [{2}; {3}]" -f $a.Name, $a.Dest, $status, $srcStatus)
    $resolved += [PSCustomObject]@{ Name = $a.Name; Source = $a.Source; Dest = $a.Dest; DestFull = $destFull; SrcExists = $srcExists; DestExists = $destExists; LayoutFound = $entry.Found }
}

if (-not $WhatIf) {
    foreach ($r in $resolved) {
        if (-not $r.SrcExists) { throw "Missing build output $($r.Source) -- run build-all.sh first." }
    }
}

Say ""
Say "=== backup ==="
if ($WhatIf) {
    SayDry "would create $BackupRoot"
} else {
    New-Item -ItemType Directory -Force $BackupRoot | Out-Null
}
foreach ($r in $resolved) {
    if (-not $r.DestExists) {
        Say "  $($r.Dest): no existing file to back up (new)"
        continue
    }
    $backupPath = Join-Path $BackupRoot $r.Dest
    if ($WhatIf) {
        SayDry "would back up $($r.Dest) -> $backupPath"
    } else {
        New-Item -ItemType Directory -Force (Split-Path $backupPath) | Out-Null
        Copy-Item $r.DestFull $backupPath -Force
        Say "  backed up $($r.Dest)"
    }
}
if (-not $WhatIf) {
    Copy-Item $Layout (Join-Path $BackupRoot "layout.json") -Force
    Say "  backed up layout.json"
} else {
    SayDry "would back up layout.json -> $BackupRoot/layout.json"
}

Say ""
Say "=== copy ==="
foreach ($r in $resolved) {
    if ($WhatIf) {
        SayDry "would copy $($r.Source) -> $($r.DestFull)"
        continue
    }
    New-Item -ItemType Directory -Force (Split-Path $r.DestFull) | Out-Null
    Copy-Item $r.Source $r.DestFull -Force
    Say "  installed $($r.Dest)"
}

Say ""
Say "=== ECAM JS patch (A10, deep/ecam bridge) ==="
if ($SkipEcam) {
    Say "  -SkipEcam: skipped"
} elseif (-not (Test-Path $EcamApply) -or -not (Test-Path $EcamPatches)) {
    Say "  no patch script/data found at $EcamApply -- A10 may no longer need an install-time patch; nothing to do"
} else {
    if ($WhatIf) {
        $patches = Get-Content $EcamPatches -Raw | ConvertFrom-Json
        $msfsPatches = $patches | Where-Object { $_.msfs -and ($_.status -eq "ok" -or $_.status -eq "fixed") }
        $files = $msfsPatches | ForEach-Object { $_.path } | Sort-Object -Unique
        SayDry "would run: python `"$EcamApply`" `"$Package`""
        SayDry ("would apply {0} patch(es) across {1} file(s):" -f $msfsPatches.Count, $files.Count)
        foreach ($f in $files) { SayDry "  html_ui/$($f.TrimStart('/'))" }
    } else {
        Say "  running apply.py..."
        & python $EcamApply $Package
        if ($LASTEXITCODE -ne 0) { throw "ecam apply.py failed (exit $LASTEXITCODE)" }
    }
}

Say ""
Say "=== layout.json update ==="
if ($WhatIf) {
    foreach ($r in $resolved) {
        SayDry "would set layout.json entry for $($r.Dest) to the installed file's size/date"
    }
} else {
    $text = [System.IO.File]::ReadAllText($Layout)
    foreach ($r in $resolved) {
        $file = Get-Item $r.DestFull
        $size = $file.Length
        $date = $file.LastWriteTimeUtc.ToFileTimeUtc()
        $pattern = '("path":\s*"' + [regex]::Escape($r.Dest) + '",\s*"size":\s*)\d+(,\s*"date":\s*)\d+'
        if ($text -match $pattern) {
            $text = [regex]::Replace($text, $pattern, "`${1}$size`${2}$date")
            Say "  $($r.Dest): size=$size date=$date"
        } else {
            $entry = ",`n    {`n      `"path`": `"$($r.Dest)`",`n      `"size`": $size,`n      `"date`": $date`n    }`n  ]"
            $text = [regex]::Replace($text, '\s*\]\s*\}\s*$', "$entry`n}")
            Say "  $($r.Dest): added new entry, size=$size date=$date"
        }
    }
    [System.IO.File]::WriteAllText($Layout, $text)
    $null = [System.IO.File]::ReadAllText($Layout) | ConvertFrom-Json   # must still parse
    Say "  layout.json written and re-validated as JSON"
}

Say ""
Say "=== summary ==="
if ($WhatIf) {
    Say ("DRY RUN: {0} artefact(s) would be installed into {1}" -f $resolved.Count, $Package)
    Say "No files were changed. Run without -WhatIf to install for real."
} else {
    Say ("Installed {0} artefact(s) into {1}" -f $resolved.Count, $Package)
    Say "Backups: $BackupRoot"
    Say "Start MSFS: the first load after a WASM change recompiles the module (slower)."
}
