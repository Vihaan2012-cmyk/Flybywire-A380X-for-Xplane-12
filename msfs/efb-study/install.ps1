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

#   .\install.ps1 -Scale 2        sharper tablet (see below)
#   .\install.ps1 -Scale 1        put the tablet back
#
# -Scale is the one thing here that edits a FlyByWire file: panel.cfg, to
# enlarge the EFB's render target. The original is copied to
# panel.cfg.deepstudy-backup first and -Remove puts it back.

param(
    [string]$Package,
    [switch]$Remove,
    [ValidateRange(1, 4)]
    [int]$Scale = 0
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

$panelPath = Join-Path $Package 'SimObjects\AirPlanes\FlyByWire_A380_842\panel\panel.cfg'
$panelBackup = "$panelPath.deepstudy-backup"
$panelRel = 'SimObjects/AirPlanes/FlyByWire_A380_842/panel/panel.cfg'

function Set-LayoutEntries {
    param([string[]]$Names, [switch]$Delete, [string[]]$Extra = @())

    $layout = Get-Content $layoutPath -Raw | ConvertFrom-Json
    $content = [System.Collections.ArrayList]::new()
    foreach ($entry in $layout.content) { [void]$content.Add($entry) }

    # efb.html always needs its recorded size refreshed: we just changed it.
    $paths = @($Names) + @('efb.html')

    foreach ($name in ($paths + $Extra)) {
        # $Extra entries are already package-relative; ours are EFB-relative.
        $rel = if ($Extra -contains $name) { $name }
               else { "html_ui/Pages/VCockpit/Instruments/A380X/EFB/$name" }
        $existing = $content | Where-Object { $_.path -eq $rel }

        if ($Delete -and $name -ne 'efb.html' -and ($Extra -notcontains $name)) {
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

function Set-EfbScale {
    # Enlarge the EFB's render target by $Factor.
    #
    # [VCockpit15] holds three numbers. size_mm is how big the panel is in
    # the cockpit and must not change -- it is the physical screen. What
    # changes is pixel_size, the render target, and the rectangle the gauge
    # is drawn into, which has to match it or the page ends up in one corner.
    #
    # study.js scales the document back down to 1430x1000 so nothing is laid
    # out differently; the drawing is simply finer. Without that half of this
    # is worse than useless -- FlyByWire's pages would come out half size.
    param([int]$Factor)

    if (-not (Test-Path $panelPath)) { throw "No panel.cfg at $panelPath" }
    if (-not (Test-Path $panelBackup)) { Copy-Item $panelPath $panelBackup }

    # Always from the backup, so -Scale is an absolute setting rather than
    # something that compounds each time it is run.
    $text = [System.IO.File]::ReadAllText($panelBackup)
    $w = 1430 * $Factor
    $h = 1000 * $Factor

    $before = $text
    $text = $text -replace '(?m)^pixel_size=1430,1000\s*$', "pixel_size=$w,$h"
    $text = $text -replace '(?m)^(htmlgauge00=A380X/EFB/efb\.html,\s*0,0,)1430,1000\s*$', "`${1}$w,$h"
    if ($Factor -ne 1 -and $text -eq $before) {
        throw "panel.cfg does not have the [VCockpit15] lines this expected -- has FlyByWire changed it?"
    }

    # No BOM: the original has none and MSFS is particular about these files.
    [System.IO.File]::WriteAllText($panelPath, $text, (New-Object System.Text.UTF8Encoding $false))
    Write-Host "panel.cfg: EFB render target now ${w}x${h}"
}

if ($Remove) {
    foreach ($name in $ours) {
        $p = Join-Path $efbDir $name
        if (Test-Path $p) { Remove-Item $p -Force }
    }
    # Both lines this script appends: study-app.js and study.js. The old
    # pattern 'study\.js' did not match 'study-app.js', which left that
    # import behind on removal.
    (Get-Content $htmlPath) | Where-Object { $_ -notmatch 'study-app\.js|/study\.js' } | ForEach-Object { $_ } | Out-String | ForEach-Object { [System.IO.File]::WriteAllText($htmlPath, $_, (New-Object System.Text.UTF8Encoding $false)) }
    if (Test-Path $panelBackup) {
        Copy-Item $panelBackup $panelPath -Force
        Remove-Item $panelBackup -Force
        Write-Host "panel.cfg restored from backup"
    }
    Set-LayoutEntries -Names $ours -Delete -Extra @($panelRel)
    Write-Host "Removed. efb.js was never touched."
    exit 0
}

if ($Scale -gt 0) { Set-EfbScale -Factor $Scale }

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

Set-LayoutEntries -Names $ours -Extra @($panelRel)
Write-Host "layout.json updated"
Write-Host ""
Write-Host "Done. efb.js untouched. Reload the aircraft and use the STUDY button."
