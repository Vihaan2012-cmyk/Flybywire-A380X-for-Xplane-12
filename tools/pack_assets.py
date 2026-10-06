"""Pack the parts of the aircraft that git cannot hold into release assets.

    python tools/pack_assets.py <kind> <folder> <out dir> [--tag TAG]

kind is msfs (the installed MSFS package), xplane (the installed X-Plane
aircraft) or cef (the CEF runtime XPHFBW builds against). Each part is a
complete zip under PART_LIMIT, and assets.json at the repository root records
every part's size and SHA-256 for build.ps1.
"""

import argparse
import fnmatch
import hashlib
import json
import os
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / "assets.json"
REPO = "Vihaan2012-cmyk/Flybywire-A380X-for-Xplane-12"
PART_LIMIT = 1500 * 1024 * 1024
STORED = {".png", ".jpg", ".jpeg", ".ogg", ".wem", ".bnk", ".pak", ".7z", ".zip", ".gz"}

SASL_OURS = {
    "plugins/sasl/data/modules/main.lua",
    "plugins/sasl/data/modules/configuration/configuration.ini",
}

KINDS = {
    "msfs": {
        "root": "flybywire-aircraft-a380-842",
        "exclude": ["*.pre-ecam", "*.original", "*.bak*", "layout.json.*", "circuit-protection-added.txt"],
    },
    "xplane": {
        "root": "FlyByWire A380X",
        "exclude": ["FlyByWire A380X lighting test*", "*_prefs.txt", "Output/*", "*.bak*", "*.log"],
    },
    "cef": {
        "root": "cef",
        "exclude": [],
    },
}


def excluded(kind: str, rel: str) -> bool:
    if kind == "xplane" and rel.startswith("plugins/sasl/") and rel not in SASL_OURS:
        return True
    name = rel.rsplit("/", 1)[-1]
    return any(fnmatch.fnmatch(rel, p) or fnmatch.fnmatch(name, p) for p in KINDS[kind]["exclude"])


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def files(kind: str, src: Path):
    for dirpath, dirs, names in os.walk(src):
        dirs.sort()
        for name in sorted(names):
            path = Path(dirpath) / name
            rel = path.relative_to(src).as_posix()
            if not excluded(kind, rel):
                yield path, rel


def pack(kind: str, src: Path, out: Path):
    root = KINDS[kind]["root"]
    out.mkdir(parents=True, exist_ok=True)
    parts, batch, size = [], [], 0
    for path, rel in files(kind, src):
        n = path.stat().st_size
        if batch and size + n > PART_LIMIT:
            parts.append(batch)
            batch, size = [], 0
        batch.append((path, rel))
        size += n
    if batch:
        parts.append(batch)

    written = []
    for i, batch in enumerate(parts, 1):
        name = f"{kind}.part{i:02d}.zip"
        target = out / name
        with zipfile.ZipFile(target, "w", allowZip64=True) as z:
            for path, rel in batch:
                method = zipfile.ZIP_STORED if path.suffix.lower() in STORED else zipfile.ZIP_DEFLATED
                z.write(path, f"{root}/{rel}", compress_type=method, compresslevel=6)
        written.append({"file": name, "size": target.stat().st_size, "sha256": sha256(target), "files": len(batch)})
        print(f"{name}: {len(batch)} files, {target.stat().st_size / 1048576:.0f} MB")
    return {"root": root, "parts": written}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=sorted(KINDS))
    ap.add_argument("src", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--tag")
    a = ap.parse_args()

    manifest = json.loads(MANIFEST.read_text(encoding="utf-8")) if MANIFEST.exists() else {"repo": REPO, "assets": {}}
    if a.tag:
        manifest["tag"] = a.tag
    manifest["assets"][a.kind] = pack(a.kind, a.src, a.out)
    MANIFEST.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
