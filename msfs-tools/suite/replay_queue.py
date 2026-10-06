import datetime
import glob
import json
import os
import shutil
import subprocess
import sys
import time

OUT = 'E:/test1results'
IMAGE = 'ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c'
TEST = 'fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/__tests__/FwsReplay.test.ts'
END = datetime.datetime.strptime(sys.argv[1], '%Y-%m-%d %H:%M')
PARALLEL = int(sys.argv[2])
CHUNK = int(sys.argv[3])


def log(msg):
    with open(f'{OUT}/run.log', 'a', encoding='utf-8') as f:
        f.write(f'[{datetime.datetime.now():%H:%M:%S}] {msg}\n')


def status(stage, detail):
    with open(f'{OUT}/replay-status.json', 'w', encoding='utf-8') as f:
        json.dump({'stage': stage, 'detail': detail, 'time': f'{datetime.datetime.now():%Y-%m-%d %H:%M:%S}'}, f)


def outputs(tag):
    labels = set()
    for p in glob.glob(f'{OUT}/fws-{tag}/**/fws-*.jsonl', recursive=True):
        with open(p, encoding='utf-8', errors='ignore') as f:
            for line in f:
                try:
                    labels.add(json.loads(line)['label'])
                except (ValueError, KeyError):
                    pass
    return labels


def docker(*args):
    return subprocess.run(['docker', *args], capture_output=True, text=True, timeout=120)


lines = {}
for tag in ('runway', 'apron'):
    with open(f'{OUT}/fws-{tag}/runs.jsonl', encoding='utf-8', errors='ignore') as f:
        lines[tag] = [(json.loads(l)['label'], l) for l in f if l.strip()]
total = sum(len(v) for v in lines.values())
done = {tag: outputs(tag) for tag in lines}
queue = []
for tag, singles in (('runway', True), ('apron', True), ('runway', False), ('apron', False)):
    todo = [l for (label, l) in lines[tag] if (' + ' not in label) == singles and label not in done[tag]]
    for i in range(0, len(todo), CHUNK):
        queue.append((tag, todo[i:i + CHUNK], 0))
log(f'replay queue: {len(queue)} chunks of {CHUNK}, {PARALLEL} at a time, {sum(len(d) for d in done.values())} of {total} already replayed, until {END:%H:%M}')

running = {}
seq = 0
while (queue or running) and datetime.datetime.now() < END:
    alive = set(docker('ps', '--filter', 'name=fwsq-', '--format', '{{.Names}}').stdout.split())
    for name in list(running):
        if name not in alive:
            tag, chunk, attempt, cdir = running.pop(name)
            got = outputs_in = set()
            for p in glob.glob(f'{cdir}/fws-*.jsonl'):
                with open(p, encoding='utf-8', errors='ignore') as f:
                    for line in f:
                        try:
                            got.add(json.loads(line)['label'])
                        except (ValueError, KeyError):
                            pass
            rest = [l for l in chunk if json.loads(l)['label'] not in got]
            if rest and attempt < 2:
                queue.insert(0, (tag, rest[1:] if attempt == 1 else rest, attempt + 1))
                log(f'{name}: {len(got)} of {len(chunk)} replayed, requeued {len(rest) - (1 if attempt == 1 else 0)}')
    while queue and len(running) < PARALLEL:
        tag, chunk, attempt = queue.pop(0)
        if not chunk:
            continue
        seq += 1
        name = f'fwsq-{seq}'
        cdir = f'{OUT}/fws-{tag}/q/{name}'
        os.makedirs(cdir, exist_ok=True)
        shutil.copyfile(f'{OUT}/fws-{tag}/base.json', f'{cdir}/base.json')
        with open(f'{cdir}/runs.jsonl', 'w', encoding='utf-8') as f:
            f.writelines(l if l.endswith('\n') else l + '\n' for l in chunk)
        r = docker('run', '-d', '--rm', '--name', name, '-v', 'fbw-replay-src:/external', '-v', f'{cdir}:/replay', '-e', 'FWS_REPLAY_DIR=/replay',
                   '-e', 'FWS_SHARD=0/1', '-e', 'CLIENT_ID=', '-e', 'CLIENT_SECRET=', IMAGE, 'bash', '-c', f'cd /external && node node_modules/vitest/vitest.mjs run {TEST}')
        if r.returncode != 0:
            log(f'{name} failed to start: {r.stderr.strip()[:200]}')
            queue.insert(0, (tag, chunk, attempt))
            time.sleep(5)
            break
        running[name] = (tag, chunk, attempt, cdir)
    n = sum(len(outputs(t)) for t in lines)
    status('ecam replay', f'{n} of {total} runs replayed ({len(running)} containers, {len(queue)} chunks queued)')
    time.sleep(5)

for name in list(running):
    docker('kill', name)
log('replay queue stopped' + (' at the time budget' if queue or running else ', every run replayed'))
time.sleep(5)
status('merging', 'ECAM.md')
subprocess.run(['python', 'D:/A380/msfs-a380/suite/merge_ecam.py', OUT], stdout=open(f'{OUT}/run.log', 'a'), stderr=subprocess.STDOUT)
n = sum(len(outputs(t)) for t in lines)
status('done', f'{n} of {total} runs replayed; see SUMMARY.md and ECAM.md')
log(f'replay all done: {n} of {total} runs replayed')
