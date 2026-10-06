import glob
import re

REPO = 'D:/A380/fbw-xp-worktrees/fs2020-672384b'
esc = re.compile('(?:' + chr(92) * 2 + 'x1b|' + chr(27) + ")[<']?[0-9]*m")
TITLES = {}
for f in glob.glob(REPO + '/fbw-a380x/src/systems/instruments/src/MsfsAvionicsCommon/EcamMessages/AbnormalSensed/*.ts'):
    s = open(f, encoding='utf-8').read()
    for m in re.finditer(r"(\d{9}): \{\s*title:\s*(?:'([^']*)'|\"([^\"]*)\")", s):
        TITLES[m.group(1)] = re.sub(r'\s+', ' ', esc.sub('', m.group(2) or m.group(3)).replace(chr(92) + "'m", '').replace(chr(27) + "'m", '')).strip()
BY_TITLE = {}
for i, t in TITLES.items():
    BY_TITLE.setdefault(t.upper(), []).append(i)
