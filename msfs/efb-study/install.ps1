# Install the Study overlay into an MSFS A380X package.
#
# Purely additive: FlyByWire's efb.js is never touched. Two files of ours are
# added and exactly one import-script line is appended to efb.html. Their
# layout.json is updated, because MSFS refuses to serve a file that is not
# listed there.
#
# Re-run this after any FlyByWire update: the update rewrites efb.html and
# layout.json, so the overlay's registration is lost even though efb.js
# needing no change is the whole point of the design.
#
#   .\install.ps1
#   .\install.ps1 -Package "D:\...\flybywire-aircraft-a380-842"
#   .\install.ps1 -Remove

param(
    [string]$Package,
    [switch]$Remove
)

$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path

if (-not $Package) {
    # UserCfg.opt records where MSFS keeps its packages.
    $cfg = "$env:LOCALAPPDATA\Packages\Microsoft.FlightSimulator_8wekyb3d8bbwe\LocalCache\UserCfg.opt"
    if (Test-Path $cfg) {
        $line = Select-String -Path $cfg -Pattern '^InstalledPackagesPath\s+"(.+)"' | Select-Object -First 1
        if ($line) {
            $root = $line.Matches[0].Groups[1].Value
            $candidate = Join-Path $root 'Community\flybywire-aircraft-a380-842'
            if (Test-Path $candidate) { $Package = $candidate }
        }
    }
}
if (-not $Package -or -not (Test-Path $Package)) {
    throw "Could not find the A380X package. Pass -Package with its path."
}

$efbDir = Join-Path $Package 'html_ui\Pages\VCockpit\Instruments\A380X\EFB'
if (-not (Test-Path (Join-Path $efbDir 'efb.html'))) {
    throw "Not an A380X package (no efb.html): $Package"
}

$layoutPath = Join-Path $Package 'layout.json'
$htmlPath = Join-Path $efbDir 'efb.html'
$importLines = @(
  '<script type="text/html" import-script="/Pages/VCockpit/Instruments/A380X/EFB/study-app.js" import-async="false"></script>',
  '<script type="text/html" import-script="/Pages/VCockpit/Instruments/A380X/EFB/study.js" import-async="false"></script>'
)
$ours = @('study.js', 'study-app.js', 'study-app.html', 'test/study-fixture.json')

function Set-LayoutEntries {
    param([string[]]$Names, [switch]$Delete)

    $layout = Get-Content $layoutPath -Raw | ConvertFrom-Json
    $content = [System.Collections.ArrayList]::new()
    foreach ($entry in $layout.content) { [void]$content.Add($entry) }

    # efb.html always needs its recorded size refreshed: we just changed it.
    $paths = @($Names) + @('efb.html')

    foreach ($name in $paths) {
        $rel = "html_ui/Pages/VCockpit/Instruments/A380X/EFB/$name"
        $existing = $content | Where-Object { $_.path -eq $rel }

        if ($Delete -and $name -ne 'efb.html') {
            foreach ($e in @($existing)) { $content.Remove($e) }
            continue
        }

        $file = Join-Path $Package ($rel -replace '/', '\')
        if (-not (Test-Path $file)) { continue }
        $info = Get-Item $file
        # MSFS stores Windows FILETIME: 100ns ticks since 1601-01-01 UTC.
        $date = $info.LastWriteTimeUtc.ToFileTimeUtc()

        if ($existing) {
            $existing[0].size = $info.Length
            $existing[0].date = $date
        } else {
            [void]$content.Add([PSCustomObject]@{ path = $rel; size = $info.Length; date = $date })
        }
    }

    $layout.content = @($content | Sort-Object path)
    # MSFS's layout.json parser rejects a UTF-8 BOM, and PowerShell 5.1's
    # Set-Content -Encoding UTF8 writes one. A BOM here makes the whole
    # package invalid and the aircraft disappears from the picker, with no
    # error anywhere that says so. Write the bytes ourselves instead.
    $json = $layout | ConvertTo-Json -Depth 10
    [System.IO.File]::WriteAllText($layoutPath, $json, (New-Object System.Text.UTF8Encoding $false))
}

if ($Remove) {
    foreach ($name in $ours) {
        $p = Join-Path $efbDir $name
        if (Test-Path $p) { Remove-Item $p -Force }
    }
    (Get-Content $htmlPath) | Where-Object { $_ -notmatch 'study\.js' } | ForEach-Object { $_ } | Out-String | ForEach-Object { [System.IO.File]::WriteAllText($htmlPath, $_, (New-Object System.Text.UTF8Encoding $false)) }
    Set-LayoutEntries -Names $ours -Delete
    Write-Host "Removed. efb.js was never touched."
    exit 0
}

$dist = Join-Path $here 'dist'
if (-not (Test-Path (Join-Path $dist 'study-app.html'))) { throw "Run build.sh first: dist/ is missing." }
New-Item -ItemType Directory -Force -Path (Join-Path $efbDir 'test') | Out-Null
Copy-Item (Join-Path $dist 'study.js') (Join-Path $efbDir 'study.js') -Force
Copy-Item (Join-Path $dist 'study-app.html') (Join-Path $efbDir 'study-app.html') -Force
Copy-Item (Join-Path $dist 'study-app.js') (Join-Path $efbDir 'study-app.js') -Force
Copy-Item (Join-Path $dist 'test\study-fixture.json') (Join-Path $efbDir 'test\study-fixture.json') -Force
Write-Host "Copied study.js, study-app.html, test/study-fixture.json"


foreach ($line in $importLines) {
    $marker = if ($line -match 'study-app\.js') { 'study-app.js' } else { '/study.js' }
    if (-not (Select-String -Path $htmlPath -Pattern ([regex]::Escape($marker)) -Quiet)) {
        [System.IO.File]::AppendAllText($htmlPath, $line + "`r`n", (New-Object System.Text.UTF8Encoding $false))
        Write-Host "efb.html: added $marker"
    }
}

Set-LayoutEntries -Names $ours
Write-Host "layout.json updated"
Write-Host ""
Write-Host "Done. efb.js untouched. Reload the aircraft and use the STUDY button."
