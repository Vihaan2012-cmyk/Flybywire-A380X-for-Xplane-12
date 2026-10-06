while ($true) {
  $p = Get-Process campaign -ErrorAction SilentlyContinue
  if (-not $p) { Start-Sleep 30; if (-not (Get-Process campaign -ErrorAction SilentlyContinue)) { break } else { continue } }
  $a = $p.TotalProcessorTime.TotalSeconds; $t0 = Get-Date
  Start-Sleep 60
  try { $p.Refresh(); $cores = ($p.TotalProcessorTime.TotalSeconds - $a) / ((Get-Date) - $t0).TotalSeconds
    $line = '{0:HH:mm} {1,5:N1} cores busy, private {2:N1} GB' -f (Get-Date), $cores, ($p.PrivateMemorySize64 / 1GB)
    Add-Content 'E:\test1results\cpu.log' $line } catch { }
}
