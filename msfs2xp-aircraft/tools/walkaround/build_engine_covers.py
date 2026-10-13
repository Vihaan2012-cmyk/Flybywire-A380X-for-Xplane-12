"""OBJ8 object for the engine inlet covers, from engine_cover_blender.py's export.

    python build_engine_covers.py <engine_covers.obj> <out a380_engine_covers.obj>

Each cover and its streamer show while fbw/walkaround/eng_inlet_cover_N is
1, like every other walkaround item, with the same toggle command.
"""
import sys
from collections import OrderedDict


def read_obj(path):
    v, vt, vn = [], [], []
    groups = OrderedDict()
    cur = None
    for line in open(path, encoding="utf-8"):
        p = line.split()
        if not p:
            continue
        if p[0] == "o":
            cur = p[1]
            groups[cur] = []
        elif p[0] == "v":
            v.append(tuple(map(float, p[1:4])))
        elif p[0] == "vt":
            vt.append(tuple(map(float, p[1:3])))
        elif p[0] == "vn":
            vn.append(tuple(map(float, p[1:4])))
        elif p[0] == "f":
            corners = [tuple(int(x) - 1 for x in c.split("/")) for c in p[1:]]
            for i in range(1, len(corners) - 1):
                groups[cur].append((corners[0], corners[i], corners[i + 1]))
    return v, vt, vn, groups


def main(src, out):
    v, vt, vn, groups = read_obj(src)
    items = OrderedDict()
    for name, tris in groups.items():
        item = name.replace("_streamer", "")
        items.setdefault(item, []).extend(tris)
    verts, index, lookup, ranges = [], [], {}, []
    for item, tris in items.items():
        start = len(index)
        for tri in tris:
            for c in (tri[0], tri[2], tri[1]):
                if c not in lookup:
                    p, t, n = v[c[0]], vt[c[1]], vn[c[2]]
                    lookup[c] = len(verts)
                    verts.append((p, n, t))
                index.append(lookup[c])
        ranges.append((item, start, len(index) - start))
    lines = ["I", "800", "OBJ", "", "TEXTURE a380_engine_cover.png", "TEXTURE_NORMAL a380_engine_cover_NML.png", "ATTR_no_cull", "ATTR_shadow", "", f"POINT_COUNTS {len(verts)} 0 0 {len(index)}", ""]
    for p, n, t in verts:
        lines.append(f"VT {p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {n[0]:.4f} {n[1]:.4f} {n[2]:.4f} {t[0]:.5f} {t[1]:.5f}")
    lines.append("")
    i = 0
    while i + 10 <= len(index):
        lines.append("IDX10 " + " ".join(map(str, index[i:i + 10])))
        i += 10
    while i < len(index):
        lines.append(f"IDX {index[i]}")
        i += 1
    lines.append("")
    for item, start, count in ranges:
        n = item.rsplit("_", 1)[-1]
        lines += [
            "ANIM_begin",
            f"ANIM_hide -0.5 0.5 fbw/walkaround/{item}",
            f"ATTR_manip_command hand fbw/walkaround/{item}_toggle Remove engine {n} inlet cover",
            f"TRIS {start} {count}",
            "ATTR_manip_none",
            "ANIM_end",
        ]
    open(out, "w", newline="\n").write("\n".join(lines) + "\n")
    print(f"{out}: {len(verts)} vertices, {len(index) // 3} triangles, {len(ranges)} covers")


if __name__ == "__main__":
    main(*sys.argv[1:3])
