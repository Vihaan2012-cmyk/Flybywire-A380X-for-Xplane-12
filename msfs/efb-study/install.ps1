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
$importLine = '<script type="text/html" import-script="/Pages/VCockpit/Instruments/A380X/EFB/study.js" import-async="false"></script>'
$ours = @('study.js', 'catalogue.json')

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
    $layout | ConvertTo-Json -Depth 10 | Set-Content $layoutPath -Encoding UTF8
}

if ($Remove) {
    foreach ($name in $ours) {
        $p = Join-Path $efbDir $name
        if (Test-Path $p) { Remove-Item $p -Force }
    }
    (Get-Content $htmlPath) | Where-Object { $_ -notmatch 'study\.js' } | Set-Content $htmlPath -Encoding UTF8
    Set-LayoutEntries -Names $ours -Delete
    Write-Host "Removed. efb.js was never touched."
    exit 0
}

Copy-Item (Join-Path $here 'study.js') (Join-Path $efbDir 'study.js') -Force
Write-Host "Copied study.js"

$catalogue = Join-Path $here 'catalogue.json'
if (Test-Path $catalogue) {
    Copy-Item $catalogue (Join-Path $efbDir 'catalogue.json') -Force
    Write-Host "Copied catalogue.json"
} elseif (-not (Test-Path (Join-Path $efbDir 'catalogue.json'))) {
    Write-Warning "No catalogue.json here and none installed. Generate it with:"
    Write-Warning "  CATALOGUE_OUT=msfs/efb-study/catalogue.json cargo test --lib -- --ignored --exact study::catalogue::tests::dump_catalogue"
}

if (-not (Select-String -Path $htmlPath -Pattern 'study\.js' -Quiet)) {
    Add-Content $htmlPath $importLine -Encoding UTF8
    Write-Host "efb.html: import line added"
} else {
    Write-Host "efb.html: import line already present"
}

Set-LayoutEntries -Names $ours
Write-Host "layout.json updated"
Write-Host ""
Write-Host "Done. efb.js untouched. Reload the aircraft and use the STUDY button."
