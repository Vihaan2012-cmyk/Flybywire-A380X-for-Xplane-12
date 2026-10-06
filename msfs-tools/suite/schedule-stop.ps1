$stopAt = Get-Date '2026-10-03 08:15'
while ((Get-Date) -lt $stopAt) { Start-Sleep 30 }
New-Item -ItemType File -Force 'E:\test1results\STOP' | Out-Null
Add-Content 'E:\test1results\run.log' "[$(Get-Date -Format HH:mm:ss)] scheduled STOP (10 h 45 min budget)"
$killAt = Get-Date '2026-10-03 08:45'
while ((Get-Date) -lt $killAt) { Start-Sleep 30 }
docker ps -q --filter "ancestor=ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c" | ForEach-Object { docker kill $_ | Out-Null }
Add-Content 'E:\test1results\run.log' "[$(Get-Date -Format HH:mm:ss)] replay time budget reached"
