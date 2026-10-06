"""Rewrite an MSFS package's layout.json and manifest.json total_package_size.

    python tools/msfs_layout.py <package folder>
"""

import json
import os
import sys
from pathlib import Path

FILETIME_EPOCH = 116444736000000000
SKIP = {"layout.json", "manifest.json"}


def main():
    pkg = Path(sys.argv[1])
    content, total = [], 0
    for dirpath, dirs, names in os.walk(pkg):
        dirs.sort()
        for name in sorted(names):
            path = Path(dirpath) / name
            rel = path.relative_to(pkg).as_posix()
            if rel in SKIP or name.endswith(".pre-ecam"):
                continue
            st = path.stat()
            content.append({"path": rel, "size": st.st_size, "date": FILETIME_EPOCH + int(st.st_mtime * 10_000_000)})
            total += st.st_size

    layout = pkg / "layout.json"
    layout.write_text(json.dumps({"content": content}, indent=2) + "\n", encoding="utf-8")
    manifest_path = pkg / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["total_package_size"] = f"{total + layout.stat().st_size:020d}"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"layout.json: {len(content)} files, {total / 1048576:.0f} MB")


if __name__ == "__main__":
    main()
