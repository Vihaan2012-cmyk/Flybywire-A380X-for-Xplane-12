import json, shutil, sys, time

ECAM = 'D:/A380/fbw-build/wasm-fs2020/ecam-msfs/'
live = json.load(open('D:/A380/msfs-a380/wiring/ecam-patches-live.json', encoding='utf-8'))
old = json.load(open(ECAM + 'patches-msfs.json', encoding='utf-8'))
shutil.copy(ECAM + 'patches-msfs.json', ECAM + 'patches-msfs.json.bak-' + time.strftime('%Y%m%d-%H%M%S'))

done = 0
for p in live:
    matches = [o for o in old if o['path'] == p['path'] and o['reason'] == p['reason']]
    if len(matches) != 1:
        sys.exit(f"{p['path']}: {len(matches)} existing entries carry this reason; refusing to guess")
    o = matches[0]
    if not p['replace'].startswith(p['find']):
        sys.exit(f"{p['path']}: the live replace does not start with its own anchor")
    o['replace'] = o['find'] + p['replace'][len(p['find']):]
    done += 1
json.dump(old, open(ECAM + 'patches-msfs.json', 'w', encoding='utf-8'), indent=1)
print(f'refreshed {done} deep ECAM bridge patches; {len(old) - done} other patches kept')
