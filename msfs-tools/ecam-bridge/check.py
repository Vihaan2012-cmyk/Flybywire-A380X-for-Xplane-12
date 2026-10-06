#!/usr/bin/env python3
"""Checks every SourcePatch in patches.json against an MSFS FlyByWire A380X
install: for each patch, does its `find` text occur in the mapped file
exactly once (applies), zero times (missing) or more than once (ambiguous)?

Read-only: never writes into the package. Usage:

    python check.py <package_dir> [--patches patches.json] [--report check-report.md]

<package_dir> is the installed package root, e.g.
"D:/Microsoft Flight Simulator 2020/Microsoft Flight Simulator 2020 Packages/
Community/flybywire-aircraft-a380-842" -- the directory that itself contains
html_ui/.
"""
import argparse
import json
import sys
from pathlib import Path


def map_path(patch_path: str, package_dir: Path) -> Path:
    # patch['path'] is VFS-relative, e.g.
    # "/Pages/VCockpit/Instruments/A380X/SystemsHost/SystemsHost.js"
    rel = patch_path.lstrip("/")
    return package_dir / "html_ui" / rel


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("package_dir", type=Path)
    ap.add_argument("--patches", type=Path, default=Path(__file__).parent / "patches.json")
    ap.add_argument("--report", type=Path, default=Path(__file__).parent / "check-report.md")
    args = ap.parse_args()

    patches = json.loads(args.patches.read_text(encoding="utf-8"))

    file_cache: dict[Path, str] = {}
    results = []
    for i, p in enumerate(patches):
        target = map_path(p["path"], args.package_dir)
        if target not in file_cache:
            if target.is_file():
                file_cache[target] = target.read_text(encoding="utf-8", errors="strict")
            else:
                file_cache[target] = None
        text = file_cache[target]
        if text is None:
            status = "file-not-found"
            count = 0
        else:
            count = text.count(p["find"])
            if count == 1:
                status = "applies"
            elif count == 0:
                status = "missing"
            else:
                status = "ambiguous"
        results.append(
            {
                "index": i,
                "path": p["path"],
                "source": p["source"],
                "reason": p["reason"],
                "target_file": str(target),
                "status": status,
                "count": count,
            }
        )

    counts = {}
    for r in results:
        counts[r["status"]] = counts.get(r["status"], 0) + 1

    lines = []
    lines.append("# MSFS ECAM patch check report")
    lines.append("")
    lines.append(f"Package: `{args.package_dir}`")
    lines.append("")
    lines.append(f"Total patches checked: {len(results)}")
    for status in ("applies", "missing", "ambiguous", "file-not-found"):
        lines.append(f"- {status}: {counts.get(status, 0)}")
    lines.append("")

    for status in ("applies", "missing", "ambiguous", "file-not-found"):
        group = [r for r in results if r["status"] == status]
        if not group:
            continue
        lines.append(f"## {status} ({len(group)})")
        lines.append("")
        for r in group:
            lines.append(f"- [{r['index']}] `{r['path']}` (count={r['count']}, source={r['source']})")
            lines.append(f"  - reason: {r['reason']}")
        lines.append("")

    args.report.write_text("\n".join(lines), encoding="utf-8")

    # Also dump the raw machine-readable result alongside the report.
    (args.report.parent / "check-report.json").write_text(json.dumps(results, indent=2), encoding="utf-8")

    print(f"{len(results)} patches checked: " + ", ".join(f"{k}={v}" for k, v in counts.items()))
    print(f"report written to {args.report}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
