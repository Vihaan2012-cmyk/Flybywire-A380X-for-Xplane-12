LOCK=/e/fbw-int/xpsdk-lock
acquire_lock() {
  waited=0
  until mkdir "$LOCK" 2>/dev/null; do
    if [ -n "$(find "$LOCK" -maxdepth 0 -mmin +15 2>/dev/null)" ]; then rm -rf "$LOCK"; fi
    sleep 3
    waited=$((waited + 3))
  done
  echo "$1 $(date +%H:%M:%S)" > "$LOCK/owner"
  [ $waited -gt 0 ] && echo "lock: waited ${waited}s"
  return 0
}
release_lock() {
  rm -rf "$LOCK"
}
