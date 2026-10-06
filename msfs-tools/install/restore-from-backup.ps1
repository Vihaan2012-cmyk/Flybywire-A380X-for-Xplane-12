param(
    [Parameter(Mandatory = $true)][string]$Backup,
    [string]$Only = ""
)

$ErrorActionPreference = "Stop"
if (Get-Process -Name FlightSimulator -ErrorAction SilentlyContinue) { throw "Close MSFS first." }
$pkg = "D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842"
$saved = "E:\fbw-backup\{0}-before-restore" -f (Get-Date -Format "yyyy-MM-dd-HHmmss")
$files = Get-ChildItem $Backup -Recurse -File | Where-Object { $_.Name -ne "layout.json" }
$layoutPath = Join-Path $pkg "layout.json"
New-Item -ItemType Directory -Force $saved | Out-Null
Copy-Item $layoutPath (Join-Path $saved "layout.json") -Force
$text = [System.IO.File]::ReadAllText($layoutPath)
foreach ($f in $files) {
    $rel = $f.FullName.Substring($Backup.TrimEnd('\').Length + 1) -replace '\\', '/'
    if ($Only -and -not $rel.StartsWith($Only)) { continue }
    $dest = Join-Path $pkg ($rel -replace '/', '\')
    if (Test-Path $dest) {
        $keep = Join-Path $saved ($rel -replace '/', '\')
        New-Item -ItemType Directory -Force (Split-Path $keep) | Out-Null
        Copy-Item $dest $keep -Force
    }
    New-Item -ItemType Directory -Force (Split-Path $dest) | Out-Null
    Copy-Item $f.FullName $dest -Force
    $item = Get-Item $dest
    $pattern = '("path":\s*"' + [regex]::Escape($rel) + '",\s*"size":\s*)\d+(,\s*"date":\s*)\d+'
    if ($text -match $pattern) {
        $text = [regex]::Replace($text, $pattern, "`${1}$($item.Length)`${2}$($item.LastWriteTimeUtc.ToFileTimeUtc())")
    }
    "restored $rel"
}
[System.IO.File]::WriteAllText($layoutPath, $text)
$null = [System.IO.File]::ReadAllText($layoutPath) | ConvertFrom-Json
"done; what was installed is saved in $saved"
