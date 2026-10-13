import re
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PACK_DIR = 'A380_Sp(RR)(P)(C)/fmod/'
BANKS = ['Master Bank.bank', 'RREngines.bank', 'Ground.bank', 'Weather.bank']


def bank_renames() -> list[tuple[str, str]]:
    source = (ROOT / 'src' / 'sound_pack.rs').read_text(encoding='utf-8')
    block = source.split('pub const BANK_PARAMETERS', 1)[1].split('];', 1)[0]
    out = []
    for original, base in re.findall(r'\("([^"]*)",\s*"([^"]*)"\)', block):
        bracket = '[' if base.endswith('[') else ''
        stem = base[:-1] if bracket else base
        out.append((original, stem + '_' * max(0, len(original) - len(stem) - len(bracket)) + bracket))
    return out


def patch_bank(data: bytes, renames: list[tuple[str, str]]) -> tuple[bytes, dict[str, int]]:
    counts = {}
    for original, renamed in renames:
        a, b = original.encode('latin-1'), renamed.encode('latin-1')
        assert len(a) == len(b), (original, renamed)
        counts[original] = data.count(a)
        data = data.replace(a, b)
    return data, counts


def events(guids: str) -> set[str]:
    return {m.group(1) for m in re.finditer(r'event:(\S+)', guids)}


def trim_snd(snd: str, guids: str) -> tuple[str, list[str]]:
    known = events(guids)
    eol = '\r\n' if '\r\n' in snd else '\n'
    lines = snd.split(eol)
    out, block, dropped = [], None, []
    for line in lines:
        key = line.strip().split(None, 1)
        if block is None and key and key[0] == 'BEGIN_SOUND_ATTACHMENT':
            block = [line]
            continue
        if block is not None:
            block.append(line)
            if key and key[0] == 'END_SOUND_ATTACHMENT':
                names = [l.strip().split(None, 1)[1].strip() for l in block if l.strip().startswith('EVENT_NAME')]
                missing = [n for n in names if n not in known]
                if missing:
                    dropped.extend(missing)
                else:
                    out.extend(block)
                block = None
            continue
        out.append(line)
    return eol.join(out), dropped


def read_pack(zip_path: Path) -> dict[str, bytes]:
    with zipfile.ZipFile(zip_path) as z:
        return {name: z.read(PACK_DIR + name) for name in BANKS + ['GUIDs.txt', 'A388.snd']}
