"""Build a release for the installer: the archive, its install steps, and the
manifest the release server (distribution/worker) serves.

    python tools/package_release.py <version> [--aircraft <installed aircraft>] [--notes "..."]

Run after `tools/install.sh` has built and installed everything into the
aircraft folder. The archive holds the whole converted aircraft (the
plugin, XPHFBW, FlyByWire's instruments, the converted model, liveries and
panel) and install.json, the steps the installer runs.

Never in it: SASL's own files (proprietary; the installer installs SASL from
the user's own archive, and only our generated module goes in the release)
and X-Plane's per-user preferences for the aircraft.

Output: dist/<version>/ with the archive and manifest.json. Publish both to
the R2 bucket (see distribution/README.md).
"""

import argparse
import hashlib
import json
import os
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_AIRCRAFT = Path(r"D:\Steam Games\steamapps\common\X-Plane 12\Aircraft\FlyByWire A380X")
AIRCRAFT_IN_XPLANE = "Aircraft/FlyByWire A380X"

# What the installer runs, in order: SASL from the user's own archive first
# (proprietary, never in the release), then the whole aircraft over it (which
# brings the generated SASL module into plugins/sasl/data/modules).
INSTALL = {
    "aircraft": AIRCRAFT_IN_XPLANE,
    "steps": [
        {"kind": "sasl", "to": "plugins/sasl"},
        {"kind": "copy", "from": "aircraft", "to": ""},
    ],
}

# The only files under plugins/sasl that are ours: the module and project
# configuration the converter generates (msfs2xp-aircraft sasl.rs). Everything
# else there is SASL's own (binaries, readmes, example configuration, widget
# fonts, its runtime state).
OURS_IN_SASL = {
    "plugins/sasl/data/modules/main.lua",
    "plugins/sasl/data/modules/configuration/configuration.ini",
}


# Never in the release: SASL's own files and X-Plane's per-user preferences
# for the aircraft.
def excluded(rel: str) -> bool:
    rel = rel.replace("\\", "/")
    if rel.startswith("plugins/sasl/") and rel not in OURS_IN_SASL:
        return True
    return rel.endswith("_prefs.txt")


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# Each archive part stays under what `wrangler r2 object put` uploads (300
# MiB), so publishing needs nothing but wrangler. Every part is a complete
# zip of its own files; the installer unpacks them all into one folder.
PART_LIMIT = 250 * 1024 * 1024


def files_to_ship(src: Path):
    for dirpath, _, files in os.walk(src):
        for name in sorted(files):
            path = Path(dirpath) / name
            rel = path.relative_to(src).as_posix()
            if not excluded(rel):
                yield path, f"aircraft/{rel}"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("version")
    ap.add_argument("--aircraft", type=Path, default=DEFAULT_AIRCRAFT)
    ap.add_argument("--notes", default="")
    a = ap.parse_args()

    out_dir = ROOT / "dist" / a.version
    out_dir.mkdir(parents=True, exist_ok=True)
    for old in out_dir.glob("*.zip"):
        old.unlink()

    parts: list[Path] = []
    z = None
    size = 0
    n = 0

    def new_part():
        nonlocal z, size
        if z is not None:
            z.close()
        path = out_dir / f"fbw-a380x-xp-{a.version}-part{len(parts) + 1}.zip"
        parts.append(path)
        z = zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED, compresslevel=6)
        size = 0

    new_part()
    # The small, essential files in the first part.
    for lic in ("LICENSE", "COPYING"):
        if (ROOT / lic).exists():
            z.write(ROOT / lic, lic)
    z.writestr("NOTICE.txt", (
        "FlyByWire A380X for X-Plane 12. Free software: GNU GPL v3 (see LICENSE).\n"
        "Built on FlyByWire Simulations' A380X (GPL-3.0 code; original 3D assets CC BY-NC 4.0).\n"
        "Not affiliated with or endorsed by Microsoft, Laminar Research or FlyByWire Simulations.\n"
        "Microsoft Flight Simulator (c) Microsoft Corporation; FlyByWire's aircraft was created under Microsoft's\n"
        "Game Content Usage Rules. Liveries remain their authors'. SASL is not included: the installer\n"
        "installs it from your own copy.\n"
    ))
    z.writestr("install.json", json.dumps(INSTALL, indent=1))
    for path, arc in files_to_ship(a.aircraft):
        # Uncompressed size is a safe upper bound on what the file adds.
        file_size = path.stat().st_size
        if size > 0 and size + file_size > PART_LIMIT:
            new_part()
        z.write(path, arc)
        size += file_size
        n += 1
    z.close()

    manifest = {
        "version": a.version,
        "notes": a.notes,
        "files": [{"name": p.name, "size": p.stat().st_size, "sha256": sha256(p)} for p in parts],
    }
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=1))
    total = sum(p.stat().st_size for p in parts)
    print(f"{len(parts)} parts, {total / 1e6:.1f} MB, {n} aircraft files in {out_dir}")


if __name__ == "__main__":
    main()
