param([Parameter(Mandatory = $true)][string]$Source)

$ErrorActionPreference = "Stop"
if (Get-Process -Name FlightSimulator -ErrorAction SilentlyContinue) { throw "Close MSFS first." }
$pkg = "D:\Microsoft Flight Simulator 2020\Microsoft Flight Simulator 2020 Packages\Community\flybywire-aircraft-a380-842"
$rel = "SimObjects/AirPlanes/FlyByWire_A380_842/panel/systems.wasm"
$dest = Join-Path $pkg ($rel -replace '/', '\')
$backup = "E:\fbw-backup\{0}-systems-wasm-swap" -f (Get-Date -Format "yyyy-MM-dd-HHmmss")
New-Item -ItemType Directory -Force $backup | Out-Null
Copy-Item $dest (Join-Path $backup "systems.wasm") -Force
Copy-Item (Join-Path $pkg "layout.json") (Join-Path $backup "layout.json") -Force
Copy-Item $Source $dest -Force
$layoutPath = Join-Path $pkg "layout.json"
$text = [System.IO.File]::ReadAllText($layoutPath)
$item = Get-Item $dest
$pattern = '("path":\s*"' + [regex]::Escape($rel) + '",\s*"size":\s*)\d+(,\s*"date":\s*)\d+'
if ($text -notmatch $pattern) { throw "systems.wasm is not in layout.json" }
$text = [regex]::Replace($text, $pattern, "`${1}$($item.Length)`${2}$($item.LastWriteTimeUtc.ToFileTimeUtc())")
[System.IO.File]::WriteAllText($layoutPath, $text)
$null = [System.IO.File]::ReadAllText($layoutPath) | ConvertFrom-Json
"installed $Source ($($item.Length) bytes); previous one backed up in $backup"
