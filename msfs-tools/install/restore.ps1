param(
    [Parameter(Mandatory = $true)][string]$BackupDir,
    [string]$Package = "D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842",
    [switch]$WhatIf,
    [switch]$RestoreEcamToo
)

$ErrorActionPreference = "Stop"

function Say($msg) { Write-Host $msg }
function SayDry($msg) { Write-Host "[dry run] $msg" -ForegroundColor Yellow }

if (-not (Test-Path $BackupDir)) { throw "No such backup folder: $BackupDir" }
if (Get-Process -Name "FlightSimulator" -ErrorAction SilentlyContinue) {
    throw "Close MSFS first (FlightSimulator.exe is running)."
}
if (-not (Test-Path (Join-Path $Package "manifest.json"))) {
    throw "No manifest.json in $Package -- is this the A380X package?"
}

$files = Get-ChildItem $BackupDir -File -Recurse
if ($files.Count -eq 0) { throw "$BackupDir has no files to restore." }

Say "=== restoring from $BackupDir ==="
foreach ($f in $files) {
    $rel = $f.FullName.Substring($BackupDir.Length).TrimStart('\', '/')
    $dest = Join-Path $Package $rel
    if ($WhatIf) {
        SayDry "would restore $rel"
    } else {
        New-Item -ItemType Directory -Force (Split-Path $dest) | Out-Null
        Copy-Item $f.FullName $dest -Force
        Say "  restored $rel"
    }
}

if ($RestoreEcamToo) {
    $ecamApply = "D:/A380/fbw-build/wasm-fs2020/ecam-msfs/apply.py"
    if (Test-Path $ecamApply) {
        if ($WhatIf) {
            SayDry "would run: python `"$ecamApply`" `"$Package`" --restore"
        } else {
            Say ""
            Say "=== restoring ECAM JS patch ==="
            & python $ecamApply $Package --restore
            if ($LASTEXITCODE -ne 0) { throw "ecam apply.py --restore failed (exit $LASTEXITCODE)" }
        }
    } else {
        Say "no $ecamApply -- nothing to restore there"
    }
}

Say ""
if ($WhatIf) {
    Say ("DRY RUN: {0} file(s) would be restored from {1}" -f $files.Count, $BackupDir)
} else {
    Say ("Restored {0} file(s) from {1}" -f $files.Count, $BackupDir)
}
