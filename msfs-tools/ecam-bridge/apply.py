#!/usr/bin/env python3
"""Applies (or restores) patches-msfs.json's ECAM-bridge SourcePatches
against an installed MSFS FlyByWire A380X package's JS.

    python apply.py <package_dir>              apply every msfs:true, status
                                                 ok/fixed patch
    python apply.py <package_dir> --restore     put every <file>.pre-ecam
                                                 back and remove them

Every touched file is backed up once, next to itself, as `<file>.pre-ecam`
(never overwritten by a later run -- if it already exists, that run's
"before" state is left alone and only the live file is touched again). A
patch whose `find` does not occur in the current file text exactly once is
refused and reported, never applied partially; if the patch's `replace`
text is already present, that file is treated as already patched for that
patch (skipped, not an error) -- so re-running this script after a
successful run is a safe no-op.

`layout.json`'s `content` array is updated in place for every file actually
rewritten (byte length, and `LastWriteTimeUtc` as a Windows FILETIME), the
same convention `D:/A380/fbw-build/wasm-fs2020/install-msfs-circuit-
protection.ps1`'s `Set-LayoutEntry` uses, so the sim does not refuse the
modified file as unlisted. `--restore` restores `layout.json` from
`layout.json.pre-ecam` too.

Never touches anything outside `<package_dir>`; only reads/writes files
this script itself backed up or is about to.
"""
import argparse
import json
import re
import sys
from pathlib import Path

FILETIME_EPOCH_OFFSET = 116444736000000000  # 1601-01-01 -> 1970-01-01, in 100ns ticks
HUNDRED_NS_PER_SEC = 10_000_000


def to_filetime(mtime_ns: int) -> int:
    """Windows FILETIME (100ns ticks since 1601-01-01 UTC) for a
    `os.stat().st_mtime_ns` value, matching .NET's `DateTime.
    ToFileTimeUtc()` (what `Set-LayoutEntry` in the PowerShell installer
    uses)."""
    return mtime_ns // 100 + FILETIME_EPOCH_OFFSET


def load_patches(patches_path: Path):
    data = json.loads(patches_path.read_text(encoding="utf-8"))
    return [p for p in data if p.get("msfs") and p.get("status") in ("ok", "fixed")]


def target_file(package_dir: Path, patch_path: str) -> Path:
    return package_dir / "html_ui" / patch_path.lstrip("/")


def layout_rel_path(package_dir: Path, file_path: Path) -> str:
    return file_path.relative_to(package_dir).as_posix()


def set_layout_entry(layout_text: str, rel: str, size: int, date: int) -> str:
    pattern = re.compile(
        r'("path":\s*"' + re.escape(rel) + r'",\s*"size":\s*)\d+(,\s*"date":\s*)\d+'
    )
    new_text, n = pattern.subn(rf"\g<1>{size}\g<2>{date}", layout_text, count=1)
    if n == 1:
        return new_text
    # A file layout.json did not already list: add an entry to `content`.
    entry = f',\n    {{\n      "path": "{rel}",\n      "size": {size},\n      "date": {date}\n    }}\n  ]'
    new_text, n = re.subn(r"\s*\]\s*\}\s*$", entry + "\n}", layout_text, count=1)
    if n != 1:
        raise RuntimeError(f"could not find layout.json's closing content array to add {rel}")
    return new_text


def apply_patches_to_text(text: str, patches: list, file_label: str, log) -> tuple[str, int, int]:
    applied = 0
    skipped = 0
    for p in patches:
        find = p["find"]
        replace = p["replace"]
        # Two different patch shapes need two different "already applied"
        # tests, or each misfires on the other:
        #
        # "Extend" patches (the five deep-ECAM-bridge ones): `replace` is
        # `find` with code appended after it (source_patch.rs's own
        # `every_replace_still_contains_the_original_find_text_verbatim`
        # test requires this, so a diff stays reviewable). `find` therefore
        # still matches exactly once even after the patch has already run --
        # checking `find`'s count alone would re-apply it on every run and
        # silently duplicate the merged code each time. These must be
        # checked by whether the full (large, specific) `replace` text is
        # already present.
        #
        # "Replace" patches (everything else): `replace` is genuinely
        # different text -- except when it is a short, generic *substring*
        # of `find` itself (e.g. one patch's `find` is `Math.min(cond ? 900
        # : 850, Math.round(egt))` and its `replace` is just
        # `Math.round(egt)`, verbatim inside `find`). There, checking
        # whether `replace` is "already present" first would misfire even
        # on a pristine, unpatched file, since `find` -- which must still be
        # there for the patch to be a candidate at all -- already contains
        # it. These must be checked by `find`'s count first, exactly as
        # `source_patch::apply` itself does; only fall back to a
        # `replace`-presence check when `find` is gone.
        is_extend = replace.startswith(find) and replace != find
        if is_extend:
            if text.count(replace) >= 1:
                skipped += 1
                log(f"  already applied [{p['index']}] (replace text already present) -- skipped")
                continue
            count = text.count(find)
        else:
            count = text.count(find)
            if count == 0 and replace and text.count(replace) >= 1:
                skipped += 1
                log(f"  already applied [{p['index']}] (replace text already present) -- skipped")
                continue
        if count == 1:
            text = text.replace(find, replace, 1)
            applied += 1
            log(f"  applied [{p['index']}] {p['reason'][:90]}")
        else:
            skipped += 1
            log(f"  REFUSED [{p['index']}] find occurs {count} times in {file_label}, not once -- left unchanged")
    return text, applied, skipped


def do_install(package_dir: Path, patches_path: Path, layout_path: Path) -> int:
    patches = load_patches(patches_path)
    by_file: dict[Path, list] = {}
    for p in patches:
        by_file.setdefault(target_file(package_dir, p["path"]), []).append(p)

    if not layout_path.is_file():
        print(f"error: no layout.json at {layout_path}", file=sys.stderr)
        return 2
    layout_backup = layout_path.with_name(layout_path.name + ".pre-ecam")
    # newline="" everywhere below: these files use bare \n line endings
    # (esbuild's output, and this package's layout.json); text mode without
    # it applies Windows' universal-newline translation and would rewrite
    # every \n as \r\n on write, silently bloating and mangling every file
    # this script touches.
    layout_text = open(layout_path, encoding="utf-8", newline="").read()
    layout_changed = False

    total_applied = 0
    total_skipped = 0
    total_refused = 0
    any_file_error = False

    for file_path, file_patches in sorted(by_file.items(), key=lambda kv: str(kv[0])):
        if not file_path.is_file():
            print(f"error: {file_path} not found in package -- skipping its {len(file_patches)} patch(es)", file=sys.stderr)
            any_file_error = True
            continue
        print(f"{file_path}")
        backup_path = file_path.with_name(file_path.name + ".pre-ecam")
        if not backup_path.is_file():
            # Byte-for-byte, not through text mode, so the backup is
            # guaranteed identical to what shipped, whatever its encoding
            # or line endings.
            backup_path.write_bytes(file_path.read_bytes())
            print("  backed up to " + backup_path.name)

        original_text = open(file_path, encoding="utf-8", newline="").read()
        new_text, applied, skipped = apply_patches_to_text(
            original_text, file_patches, file_path.name, print
        )
        total_applied += applied
        total_skipped += skipped

        if new_text != original_text:
            open(file_path, "w", encoding="utf-8", newline="").write(new_text)
            stat = file_path.stat()
            rel = layout_rel_path(package_dir, file_path)
            layout_text = set_layout_entry(layout_text, rel, stat.st_size, to_filetime(stat.st_mtime_ns))
            layout_changed = True

    if layout_changed:
        if not layout_backup.is_file():
            layout_backup.write_bytes(layout_path.read_bytes())
        open(layout_path, "w", encoding="utf-8", newline="").write(layout_text)
        json.loads(layout_path.read_text(encoding="utf-8"))  # must still parse
        print(f"layout.json updated")

    print(f"\n{total_applied} applied, {total_skipped} already-applied/refused across {len(by_file)} file(s)")
    return 1 if any_file_error else 0


def do_restore(package_dir: Path, layout_path: Path) -> int:
    restored = 0
    for backup in package_dir.rglob("*.pre-ecam"):
        target = backup.with_name(backup.name[: -len(".pre-ecam")])
        target.write_bytes(backup.read_bytes())
        backup.unlink()
        print(f"restored {target}")
        restored += 1
    layout_backup = layout_path.with_name(layout_path.name + ".pre-ecam")
    if layout_backup.is_file():
        layout_path.write_bytes(layout_backup.read_bytes())
        layout_backup.unlink()
        print("restored layout.json")
    print(f"\n{restored} file(s) restored")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("package_dir", type=Path)
    ap.add_argument("--restore", action="store_true")
    ap.add_argument("--patches", type=Path, default=Path(__file__).parent / "patches-msfs.json")
    args = ap.parse_args()

    package_dir = args.package_dir.resolve()
    layout_path = package_dir / "layout.json"

    if args.restore:
        return do_restore(package_dir, layout_path)
    return do_install(package_dir, args.patches, layout_path)


if __name__ == "__main__":
    sys.exit(main())
