"""Build a release for the installer: the archive, its install steps, and the
manifest the release server (distribution/worker) serves.

    python tools/package_release.py <version> [--aircraft <installed aircraft>] [--notes "..."] [--patch-from <version>]

A base release is the installed aircraft exactly as it is. A patch release
(--patch-from) holds only the files that changed or appeared since that
version, plus the list of files that went away; its manifest carries the
whole chain from the base, so the installer can build any install from it.

Run after `tools/install.sh` has built and installed everything into the
aircraft folder. The archive holds the whole converted aircraft (the
plugin, XPHFBW, FlyByWire's instruments, the converted model, liveries and
panel) and install.json, the steps the installer runs.

Never in it: SASL's own files (proprietary; the installer downloads SASL
from 1-sim, and only our generated module goes in the release), Laminar's
.acf and airfoils (the installer builds the .acf from the user's own A330 and
the release carries only *.acfpatch), the FCU font (downloaded from
FlyByWire's repository), and FTSiM+'s A380RR_Sp sound pack's banks and .snd
(installed from FTSiM+'s own demo download; the release carries our .sndpatch).

Output: dist/<version>/ with the archive parts, converter.zip and
manifest.json, published to GitHub Releases by distribution/publish.sh.
"""

import argparse
import importlib.util
import subprocess
import hashlib
import json
import os
import re
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_AIRCRAFT = Path(r"D:\Steam Games\steamapps\common\X-Plane 12\Aircraft\A380 for X-Plane 12")
AIRCRAFT_IN_XPLANE = "Aircraft/A380 for X-Plane 12"
REPO = "Vihaan2012-cmyk/Flybywire-A380X-for-Xplane-12"

# What the installer runs, in order: an updater from before the chain hands
# the whole job to the installer in the archive (the installer itself skips
# that step); SASL from the user's own archive (proprietary, never in the
# release); then the whole aircraft over it (which brings the generated SASL
# module into plugins/sasl/data/modules).
INSTALLER_IN_ARCHIVE = "FlyByWire-A380X-XP-Installer.exe"
TAKEOVER = {"kind": "run", "program": INSTALLER_IN_ARCHIVE, "args": ["--takeover", "{xplane}"]}
DEFAULT_INSTALLER = Path(r"D:\A380\fbw-build\target\x86_64-pc-windows-msvc\release\a380x-xp-engine.exe")
LAMINAR_A330 = "Aircraft/Laminar Research/Airbus A330-300"
AIRFOILS = ["Airbus Root (high subsonic).afl", "Airbus Mid (high subsonic).afl", "Airbus Tip (high subsonic).afl"]
SHIPPED_ACFS = ["A380.acf", "A380 Clean Cockpit.acf"]
FCU_FONT = "html_ui/Fonts/fbw-a380x/A380X_FCU.ttf"
FCU_FONT_URL = (
    "https://raw.githubusercontent.com/flybywiresim/aircraft/607e0612d63e81ac7a4942cdb1b6f181c73cab60/"
    "fbw-a380x/src/base/flybywire-aircraft-a380-842/html_ui/Fonts/fbw-a380x/A380X_FCU.ttf"
)
FCU_FONT_SHA256 = "fab287f184145206289a04085758e1a6f19c8330b89649ba7fc59f072c049490"
SASL_URL = "https://1-sim.com/files/SASL-v3.23.3.zip"
SASL_SHA256 = "f48d71adde3244e091c561ac6f9892b1dead3d98508d779ffb538224a50cceec"
SOUND_PACK_URL = "https://mega.nz/file/a4IUHByK#0a5HymbXbkhJYQfphs9SYyTBduyRgXJI36C3UiRtlJQ"
SND = "fmod/A380.snd"
SND_ALSO = ["fmod/A380 Clean Cockpit.snd"]
DEFAULT_SOUND_PACK = Path.home() / "Downloads" / "A380_Sp(RR)(P)(C).zip"
FROM_XPLANE = [
    {"kind": "from_xplane", "from": f"{LAMINAR_A330}/airfoils", "files": AIRFOILS, "to": "airfoils"},
    *[{"kind": "build_acf", "template": f"{LAMINAR_A330}/A330.acf", "patch": acf + "patch", "to": acf} for acf in SHIPPED_ACFS],
    {"kind": "download", "url": FCU_FONT_URL, "sha256": FCU_FONT_SHA256, "to": FCU_FONT},
]
INSTALL = {
    "aircraft": AIRCRAFT_IN_XPLANE,
    "steps": [
        TAKEOVER,
        {"kind": "sasl", "to": "plugins/sasl", "url": SASL_URL, "sha256": SASL_SHA256},
        {"kind": "copy", "from": "aircraft", "to": ""},
        *FROM_XPLANE,
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


# Never in the release: SASL's own files, and the two files the installer
# itself writes into every install.
INSTALLER_OWNED = {"fbw_xp_installed_version.txt", "FlyByWire A380X Updater.exe"}

# Never in the release either: the converter's own notes, dev pages and test
# fixtures, source maps, logs, backups, and the user's own X-Plane state.
DEV_FILES = {"README.txt", "cockpit_bindings.txt", "plugins/fbw_a380_systems/XPHFBW/ui/harness.html"}
DEV_DIRS = ("Output/", "html_ui/Pages/VCockpit/Instruments/A380X/IFE/theme/", "html_ui/Pages/VCockpit/Instruments/A380X/EFB/", "plugins/fbw_a380_systems/XPHFBW/ui/test/", "plugins/fbw_a380_systems/XPHFBW/js/test/")
BACKUP = re.compile(r"\.(before|with|bak|orig|old)\b|\.(map|log|tmp|pdb)$|~$|_prefs\.txt$", re.I)

PATCH_INSTALL = {"aircraft": AIRCRAFT_IN_XPLANE, "steps": [TAKEOVER, {"kind": "copy", "from": "aircraft", "to": ""}, *FROM_XPLANE]}


def excluded(rel: str) -> bool:
    rel = rel.replace("\\", "/")
    if rel.startswith("plugins/sasl/") and rel not in OURS_IN_SASL:
        return True
    name = rel.rsplit("/", 1)[-1]
    return (
        rel in INSTALLER_OWNED
        or rel in DEV_FILES
        or rel.startswith(DEV_DIRS)
        or BACKUP.search(name) is not None
        or ".acf" in name.lower()
        or (rel.startswith("airfoils/") and name in AIRFOILS)
        or rel == FCU_FONT
        or (rel.startswith("fmod/") and (rel.endswith(".bank") or rel.endswith(".snd") or rel == "fmod/GUIDs.txt"))
        or (rel.startswith("plugins/fbw_a380_systems/sound/") and (rel.lower().endswith(".pck") or "/voices/" in rel))
    )


def tool(name: str):
    spec = importlib.util.spec_from_file_location(name, ROOT / "tools" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generated(aircraft: Path, sound_pack: Path, work: Path) -> tuple[dict[str, bytes], dict]:
    if sha256(aircraft / FCU_FONT) != FCU_FONT_SHA256:
        raise SystemExit(f"{FCU_FONT} no longer matches FlyByWire's copy at {FCU_FONT_URL}; update the URL and hash")
    acf_patch, spr, text_patch = tool("acf_patch"), tool("sound_pack_release"), tool("text_patch")
    template = aircraft.parent.parent / LAMINAR_A330 / "A330.acf"
    out = {f"{acf}patch": acf_patch.make(aircraft / acf, template).encode("latin-1") for acf in SHIPPED_ACFS}
    pack = spr.read_pack(sound_pack)
    trimmed, _ = spr.trim_snd((aircraft / SND).read_bytes().decode("latin-1"), pack["GUIDs.txt"].decode("latin-1"))
    work.mkdir(parents=True, exist_ok=True)
    (work / "release.snd").write_bytes(trimmed.encode("latin-1"))
    (work / "A388.snd").write_bytes(pack["A388.snd"])
    out[f"{SND}patch"] = text_patch.make(work / "release.snd", work / "A388.snd").encode("latin-1")
    step = {
        "kind": "sound_pack",
        "url": SOUND_PACK_URL,
        "zip_sha256": sha256(sound_pack),
        "to": "fmod",
        "sha256": {name: hashlib.sha256(data).hexdigest() for name, data in pack.items()},
        "banks": spr.BANKS,
        "copy": ["GUIDs.txt"],
        "renames": spr.bank_renames(),
        "snd_template": "A388.snd",
        "snd_patch": f"{SND}patch",
        "snd_to": SND,
        "snd_also": SND_ALSO,
    }
    return out, step


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


def tree_of(src: Path, extra: dict[str, bytes]) -> dict:
    tree = {arc[len("aircraft/"):]: {"size": path.stat().st_size, "sha256": sha256(path)} for path, arc in files_to_ship(src)}
    tree.update({rel: {"size": len(data), "sha256": hashlib.sha256(data).hexdigest()} for rel, data in extra.items()})
    return tree


def previous(version: str) -> tuple[dict, dict]:
    d = ROOT / "dist" / version
    if not (d / "tree.json").exists() or not (d / "manifest.json").exists():
        d.mkdir(parents=True, exist_ok=True)
        subprocess.run(["gh", "release", "download", f"v{version}", "--repo", REPO, "-p", "tree.json", "-p", "manifest.json", "-D", str(d), "--clobber"], check=True)
    return json.loads((d / "tree.json").read_text()), json.loads((d / "manifest.json").read_text())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("version")
    ap.add_argument("--aircraft", type=Path, default=DEFAULT_AIRCRAFT)
    ap.add_argument("--notes", default="")
    ap.add_argument("--converter", type=Path, default=Path.home() / "Downloads" / "msfs2xp-aircraft")
    ap.add_argument("--patch-from")
    ap.add_argument("--installer", type=Path, default=DEFAULT_INSTALLER)
    ap.add_argument("--texture-pack", action="append", default=[], metavar="SIZE=DIR")
    ap.add_argument("--sound-pack", type=Path, default=DEFAULT_SOUND_PACK)
    a = ap.parse_args()
    if not a.installer.is_file():
        raise SystemExit(f"{a.installer} not found: build the installer first (distribution/publish.sh does)")

    out_dir = ROOT / "dist" / a.version
    out_dir.mkdir(parents=True, exist_ok=True)
    for old in out_dir.glob("*.zip"):
        old.unlink()

    extra, sound_step = generated(a.aircraft, a.sound_pack, out_dir / "sound-pack")
    tree = tree_of(a.aircraft, extra)
    if a.patch_from:
        prev_tree, prev_manifest = previous(a.patch_from)
        ship = {rel for rel, f in tree.items() if prev_tree.get(rel, {}).get("sha256") != f["sha256"]}
        deleted = sorted(set(prev_tree) - set(tree))
        if not ship and not deleted:
            raise SystemExit(f"nothing changed since {a.patch_from}")
        kind, install = "patch", dict(PATCH_INSTALL)
        if f"{SND}patch" in ship:
            install["steps"] = install["steps"][:2] + [sound_step] + install["steps"][2:]
        chain = prev_manifest.get("chain") or [{"version": prev_manifest["version"], "kind": "full", "files": [f for f in prev_manifest["files"] if f.get("component") != "converter"], "deleted": []}]
    else:
        ship, deleted, kind, install, chain = set(tree), [], "full", dict(INSTALL), []
        install["steps"] = install["steps"][:3] + [sound_step] + install["steps"][3:]

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
    for lic in ("LICENSE", "COPYING"):
        if (ROOT / lic).exists():
            z.write(ROOT / lic, lic)
    z.writestr("NOTICE.txt", (
        "A380 for X-Plane 12 (Ported from FBW A380). Free software: GNU GPL v3 (see LICENSE).\n"
        "Built on FlyByWire Simulations' A380X (GPL-3.0 code; original 3D assets CC BY-NC 4.0).\n"
        "Not affiliated with or endorsed by Microsoft, Laminar Research or FlyByWire Simulations.\n"
        "Contains no Microsoft Flight Simulator content. Liveries remain their authors'.\n"
        "Not included, downloaded during installation from their own sites: SASL (1-sim), FTSiM+'s\n"
        "A380RR_Sp sound pack (Peter Hager demo) and FlyByWire's FCU font. The .acf and wing airfoils are built from your own\n"
        "X-Plane 12's Airbus A330-300.\n"
        "Every component's license and notice: THIRD_PARTY_NOTICES.txt in the aircraft folder.\n"
    ))
    z.writestr("install.json", json.dumps(install, indent=1))
    z.write(a.installer, INSTALLER_IN_ARCHIVE)
    for rel, data in extra.items():
        if rel in ship:
            z.writestr(f"aircraft/{rel}", data)
            n += 1
    for path, arc in files_to_ship(a.aircraft):
        if arc[len("aircraft/"):] not in ship:
            continue
        file_size = path.stat().st_size
        if size > 0 and size + file_size > PART_LIMIT:
            new_part()
        z.write(path, arc)
        size += file_size
        n += 1
    z.close()

    aircraft_files = [{"name": p.name, "size": p.stat().st_size, "sha256": sha256(p), "component": "aircraft"} for p in parts]
    files = list(aircraft_files)
    exe = a.converter / "msfs2xp-aircraft.exe"
    if exe.is_file():
        conv = out_dir / "converter.zip"
        with zipfile.ZipFile(conv, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as cz:
            for f in sorted(a.converter.iterdir()):
                if f.is_file():
                    cz.write(f, f.name)
        files.append({"name": conv.name, "size": conv.stat().st_size, "sha256": sha256(conv), "component": "converter"})
        print(f"converter: {conv.stat().st_size / 1e6:.1f} MB from {a.converter}")
    else:
        print(f"converter: none ({exe} not found)")
    for spec in a.texture_pack:
        label, folder = spec.split("=", 1)
        folder = Path(folder)
        pack_parts: list[Path] = []
        pz, psize = None, 0
        for path in sorted(p for p in folder.rglob("*") if p.is_file()):
            file_size = path.stat().st_size
            if pz is None or (psize > 0 and psize + file_size > PART_LIMIT):
                if pz is not None:
                    pz.close()
                pack_parts.append(out_dir / f"fbw-a380x-xp-{a.version}-textures-{label}-part{len(pack_parts) + 1}.zip")
                pz, psize = zipfile.ZipFile(pack_parts[-1], "w", zipfile.ZIP_DEFLATED, compresslevel=6), 0
            pz.write(path, path.relative_to(folder).as_posix())
            psize += file_size
        if pz is not None:
            pz.close()
        files += [{"name": p.name, "size": p.stat().st_size, "sha256": sha256(p), "component": f"textures-{label}"} for p in pack_parts]
        print(f"textures-{label}: {len(pack_parts)} parts, {sum(p.stat().st_size for p in pack_parts) / 1e6:.1f} MB from {folder}")
    chain.append({"version": a.version, "kind": kind, "files": aircraft_files, "deleted": deleted})
    manifest = {"version": a.version, "kind": kind, "notes": a.notes, "files": files, "deleted": deleted, "chain": chain}
    if a.patch_from:
        manifest["from"] = a.patch_from
    (out_dir / "tree.json").write_text(json.dumps(tree, indent=0, sort_keys=True))
    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=1))
    total = sum(p.stat().st_size for p in parts)
    print(f"{kind} {a.version}: {len(parts)} parts, {total / 1e6:.1f} MB, {n} of {len(tree)} aircraft files, {len(deleted)} deleted, chain of {len(chain)}")


if __name__ == "__main__":
    main()
