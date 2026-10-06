import datetime
import glob
import json
import os
import shutil
import subprocess
import sys
import time

OUT = os.environ.get('CAMPAIGN_DIR', 'E:/test1results')
REPO = 'D:/A380/fbw-xp-worktrees/fs2020-672384b'
IMAGE = 'ghcr.io/flybywiresim/dev-env@sha256:314818673efe81469039e998b18f00d14e1fe2236b85f88f6c42004beef8ea7c'
TEST = 'fbw-a380x/src/systems/systems-host/CpiomC/FlightWarningSystem/__tests__/FwsReplay.test.ts'
END = datetime.datetime.strptime(sys.argv[1], '%Y-%m-%d %H:%M')
DOCKER_SLOTS = int(sys.argv[2])
NATIVE_SLOTS = int(sys.argv[3])
DOCKER_CHUNK = 80
NATIVE_CHUNK = 160


def log(msg):
    with open(f'{OUT}/run.log', 'a', encoding='utf-8') as f:
        f.write(f'[{datetime.datetime.now():%H:%M:%S}] {msg}\n')


def status(stage, detail):
    with open(f'{OUT}/replay-status.json', 'w', encoding='utf-8') as f:
        json.dump({'stage': stage, 'detail': detail, 'time': f'{datetime.datetime.now():%Y-%m-%d %H:%M:%S}'}, f)


def labels_in(paths):
    got = set()
    for p in paths:
        try:
            with open(p, encoding='utf-8', errors='ignore') as f:
                for line in f:
                    try:
                        got.add(json.loads(line)['label'])
                    except (ValueError, KeyError):
                        pass
        except OSError:
            pass
    return got


def done_labels(tag):
    return labels_in(glob.glob(f'{OUT}/fws-{tag}/**/fws-*.jsonl', recursive=True))


def docker(*args):
    try:
        return subprocess.run(['docker', *args], capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(args, 1, '', 'timeout')


lines = {}
for tag in ('runway', 'apron'):
    with open(f'{OUT}/fws-{tag}/runs.jsonl', encoding='utf-8', errors='ignore') as f:
        lines[tag] = [(json.loads(l)['label'], l if l.endswith('\n') else l + '\n') for l in f if l.strip()]
phases = {p for p in os.environ.get('REPLAY_PHASES', '').split(',') if p}
if phases:
    keep = set()
    with open(f'{OUT}/pairs.jsonl', encoding='utf-8', errors='ignore') as f:
        for l in f:
            try:
                v = json.loads(l)
            except ValueError:
                continue
            if v.get('phase') in phases:
                keep.add(v.get('label'))
    for tag in lines:
        lines[tag] = [(label, l) for (label, l) in lines[tag] if ' + ' not in label or label in keep]
total = sum(len(v) for v in lines.values())

running = {}
alive = set(docker('ps', '--filter', 'name=fwsq-', '--format', '{{.Names}}').stdout.split())
for name in alive:
    for tag in lines:
        cdir = f'{OUT}/fws-{tag}/q/{name}'
        if os.path.isdir(cdir):
            with open(f'{cdir}/runs.jsonl', encoding='utf-8') as f:
                running[name] = ('docker', tag, f.readlines(), 0, cdir, None)
claimed = {tag: set() for tag in lines}
for kind, tag, chunk, _, _, _ in running.values():
    claimed[tag].update(json.loads(l)['label'] for l in chunk)
done = {tag: done_labels(tag) for tag in lines}
queue = []
for tag, singles in (('runway', True), ('apron', True), ('runway', False), ('apron', False)):
    todo = [l for (label, l) in lines[tag] if (' + ' not in label) == singles and label not in done[tag] and label not in claimed[tag]]
    for i in range(0, len(todo), DOCKER_CHUNK):
        queue.append((tag, todo[i:i + DOCKER_CHUNK], 0))
log(f'replay queue 2: adopted {len(running)} containers, {len(queue)} chunks queued, {DOCKER_SLOTS} docker + {NATIVE_SLOTS} native slots, until {END:%H:%M}')

seq = int(time.time()) % 100000
while (queue or running) and datetime.datetime.now() < END:
    alive = set(docker('ps', '--filter', 'name=fwsq-', '--filter', 'name=fwsr-', '--format', '{{.Names}}').stdout.split())
    for name in list(running):
        kind, tag, chunk, attempt, cdir, proc = running[name]
        finished = (name not in alive) if kind == 'docker' else (proc.poll() is not None)
        if not finished:
            continue
        running.pop(name)
        got = labels_in(glob.glob(f'{cdir}/fws-*.jsonl'))
        rest = [l for l in chunk if json.loads(l)['label'] not in got]
        if rest and attempt < 2:
            if attempt == 1:
                rest = rest[1:]
            queue.insert(0, (tag, rest, attempt + 1))
            log(f'{name}: {len(got)} of {len(chunk)} replayed, requeued {len(rest)}')
    n_docker = sum(1 for v in running.values() if v[0] == 'docker')
    n_native = sum(1 for v in running.values() if v[0] == 'native')
    while queue and (n_docker < DOCKER_SLOTS or n_native < NATIVE_SLOTS):
        kind = 'docker' if n_docker < DOCKER_SLOTS else 'native'
        tag, chunk, attempt = queue.pop(0)
        if kind == 'native':
            while len(chunk) < NATIVE_CHUNK and queue and queue[0][0] == tag:
                chunk = chunk + queue.pop(0)[1]
        if not chunk:
            continue
        seq += 1
        name = f'fwsr-{seq}' if kind == 'docker' else f'fwsn-{seq}'
        cdir = f'{OUT}/fws-{tag}/q/{name}'
        os.makedirs(cdir, exist_ok=True)
        shutil.copyfile(f'{OUT}/fws-{tag}/base.json', f'{cdir}/base.json')
        with open(f'{cdir}/runs.jsonl', 'w', encoding='utf-8') as f:
            f.writelines(chunk)
        if kind == 'docker':
            r = docker('run', '-d', '--rm', '--name', name, '-v', 'fbw-replay-src:/external', '-v', f'{cdir}:/replay', '-e', 'FWS_REPLAY_DIR=/replay',
                       '-e', 'FWS_SHARD=0/1', '-e', 'CLIENT_ID=', '-e', 'CLIENT_SECRET=', IMAGE, 'bash', '-c', f'cd /external && node node_modules/vitest/vitest.mjs run {TEST}')
            if r.returncode != 0:
                log(f'{name} failed to start: {r.stderr.strip()[:200]}')
                queue.insert(0, (tag, chunk, attempt))
                break
            running[name] = ('docker', tag, chunk, attempt, cdir, None)
            n_docker += 1
        else:
            env = dict(os.environ, FWS_REPLAY_DIR=cdir, FWS_SHARD='0/1', CLIENT_ID='', CLIENT_SECRET='')
            proc = subprocess.Popen(['node', 'node_modules/vitest/vitest.mjs', 'run', TEST], cwd=REPO, env=env,
                                    stdout=open(f'{cdir}/log.txt', 'w'), stderr=subprocess.STDOUT, creationflags=subprocess.BELOW_NORMAL_PRIORITY_CLASS)
            running[name] = ('native', tag, chunk, attempt, cdir, proc)
            n_native += 1
    n = sum(len(done_labels(t)) for t in lines)
    status('ecam replay', f'{n} of {total} runs replayed ({n_docker} containers + {n_native} native, {len(queue)} chunks queued)')
    time.sleep(5)

for name, (kind, tag, chunk, attempt, cdir, proc) in list(running.items()):
    if kind == 'docker':
        docker('kill', name)
    else:
        subprocess.run(['taskkill', '/T', '/F', '/PID', str(proc.pid)], capture_output=True)
log('replay queue 2 stopped' + (' at the time budget' if queue or running else ', every run replayed'))
time.sleep(5)
status('merging', 'ECAM.md')
subprocess.run(['python', 'D:/A380/msfs-a380/suite/merge_ecam.py', OUT], stdout=open(f'{OUT}/run.log', 'a'), stderr=subprocess.STDOUT)
n = sum(len(done_labels(t)) for t in lines)
status('done', f'{n} of {total} runs replayed; see SUMMARY.md and ECAM.md')
log(f'replay all done: {n} of {total} runs replayed')
