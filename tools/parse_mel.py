"""Parse an Airbus A380 MEL (pdftotext -layout output) into JSON.

The MEL itself is Airbus/operator copyright and is never bundled: this reads
the user's own copy and writes the structured result next to it, for the
plugin to load at runtime.

    pdftotext -layout a380-mel.pdf a380-mel.txt
    python tools/parse_mel.py a380-mel.txt a380-mel.json

Layout parsed (MEL ITEMS pages):

    49-10-01                                   APU          <- item
    2 49-10-01A                                             <- sub-item
          Repair interval   Nbr installed   Nbr required   Placard
                 C                1               0          Yes
    (o) May be inoperative provided that ...                <- conditions
                              Reference(s)
    (o) Refer to OpsProc 49-10-01A ...                      <- references

and MEL OPERATIONAL PROCEDURES pages, where one or more sub-item ids head a
procedure body. Anything that looks like a sub-item but cannot be read is
listed under "unparsed" rather than dropped.
"""

import json
import re
import sys

FOOTER = re.compile(r"^\s*KAL A380 FLEET\b")
FURNITURE = [
    re.compile(r"^\s*MEL\s.*\d{2} [A-Z]{3} \d{2}\s*$"),
    re.compile(r"^\s*MEL\s*$"),
    re.compile(r"^\s*A380\s+MEL (ITEMS|OPERATIONAL PROCEDURES|MAINTENANCE PROCEDURES)\s*$"),
    re.compile(r"^\s*MINIMUM EQUIPMENT LIST\b"),
    re.compile(r"^\s*\d{2} - [A-Z].*$"),
    re.compile(r"^\s*\d{2}-\d{2} - \S.*$"),
    re.compile(r"^\s*Continued (on|from) the (following|previous) page\s*$"),
    re.compile(r"^\s*Intentionally left blank\s*$"),
    re.compile(r"^\s*Ident\.:"),
    re.compile(r"^\s*\d+\s*$"),
]

ITEM = re.compile(r"^(\d{2}-\d{2}-\d{2})\s{3,}(\S.*?)\s*$")
SUB = re.compile(r"^\s*\d+\s+(\d{2}-\d{2}-\d{2}[A-Z]{1,2})\s*$")
OPS_HEAD = re.compile(r"^(\d{2}-\d{2}-\d{2}[A-Z]{1,2})(?:\s{3,}(\S.*?))?\s*$")

# The MEL's own preamble (PRE-RI): calendar days per category; A has none.
INTERVAL_DAYS = {"A": None, "B": 3, "C": 10, "D": 120}


def pages(lines):
    page = []
    for line in lines:
        page.append(line)
        if FOOTER.match(line):
            yield page
            page = []
    if page:
        yield page


def kind_of(page):
    head = " ".join(page[:6])
    if "MEL OPERATIONAL PROCEDURES" in head:
        return "ops"
    if "MEL ITEMS" in head and "PREAMBLE" not in head:
        return "items"
    return None


APPLICABLE = re.compile(r"^(\s*Applicable to:\s*\S+)")


def content(page):
    out = []
    for line in page:
        if FOOTER.match(line) or any(f.match(line) for f in FURNITURE):
            continue
        # pdftotext sometimes lays a table's value row onto this line: keep
        # whatever follows, at its own columns.
        m = APPLICABLE.match(line)
        if m:
            line = " " * len(m.group(1)) + line[len(m.group(1)):]
            if not line.strip():
                continue
        out.append(line.rstrip())
    return out


LABELS = [("category", "Repair interval"), ("installed", "Nbr installed"), ("required", "Nbr required"), ("placard", "Placard")]


def header_columns(line):
    """Each present label's centre column."""
    cols = {}
    for key, label in LABELS:
        i = line.find(label)
        if i >= 0:
            cols[key] = i + len(label) / 2
    return cols


def read_values(line, cols):
    """Tokens of a value row, each given to the header label nearest it."""
    out = {}
    for m in re.finditer(r"\S+(?: \S+)?", line):
        centre = (m.start() + m.end()) / 2
        key = min(cols, key=lambda k: abs(cols[k] - centre))
        out[key] = m.group(0)
    return out


def parse_items(lines, unparsed):
    items = {}
    item = sub = None
    mode = None
    pending_values = False
    for line in lines:
        m = ITEM.match(line)
        if m:
            item = items.setdefault(m.group(1), {"ata": m.group(1), "title": m.group(2), "subitems": []})
            sub = None
            continue
        m = SUB.match(line)
        if m and item is not None:
            sub = {"id": m.group(1), "category": None, "installed": None, "required": None, "placard": None,
                   "repair_interval_days": None, "conditions": [], "references": []}
            item["subitems"].append(sub)
            mode = None
            continue
        if sub is None:
            continue
        if "Repair interval" in line:
            pending_values = header_columns(line)
            continue
        if pending_values and line.strip():
            values = read_values(line, pending_values)
            pending_values = False
            if values.get("category") in INTERVAL_DAYS:
                for key in ("category", "installed", "required", "placard"):
                    sub[key] = values.get(key)
                sub["repair_interval_days"] = INTERVAL_DAYS[values["category"]]
            else:
                unparsed.append({"id": sub["id"], "line": line.strip()})
            mode = "conditions"
            continue
        if line.strip() == "Reference(s)":
            mode = "references"
            continue
        if not line.strip():
            continue
        if mode == "conditions":
            sub["conditions"].append(line.strip())
        elif mode == "references":
            sub["references"].append(line.strip())
    for it in items.values():
        for s in it["subitems"]:
            text = " ".join(s["conditions"])
            s["conditions"] = text
            s["ops_procedure_required"] = "(o)" in text
            s["maintenance_procedure_required"] = "(m)" in text
            s["amm"] = sorted(set(re.findall(r"AMM (\d{2}-\d{2}-\d{2}-\d{3}-\d{3})", " ".join(s["references"]))))
    return items


def parse_ops(lines):
    procs = {}
    ids, body = [], []

    def flush():
        if ids:
            text = "\n".join(l for l in body).strip()
            for i in ids:
                procs[i] = text

    heading = True
    for line in lines:
        m = OPS_HEAD.match(line)
        if m and (heading or m.group(2)):
            if m.group(2):
                flush()
                ids, body = [m.group(1)], []
            else:
                ids.append(m.group(1))
            heading = True
            continue
        heading = False
        if ids:
            body.append(line.strip())
    flush()
    return procs


def main(src, dst):
    with open(src, encoding="utf-8", errors="replace") as f:
        lines = f.read().splitlines()
    item_lines, ops_lines = [], []
    for page in pages(lines):
        k = kind_of(page)
        if k == "items":
            item_lines += content(page)
        elif k == "ops":
            ops_lines += content(page)
    unparsed = []
    items = parse_items(item_lines, unparsed)
    ops = parse_ops(ops_lines)
    subs = [s for it in items.values() for s in it["subitems"]]
    for s in subs:
        s["ops_procedure"] = ops.get(s["id"])
    missing_ops = [s["id"] for s in subs if s["ops_procedure_required"] and not s["ops_procedure"]]
    out = {
        "source": "A380 MEL (user's local copy; not bundled)",
        "repair_interval_days": INTERVAL_DAYS,
        "items": list(items.values()),
        "unparsed": unparsed,
        "ops_procedure_missing": missing_ops,
    }
    with open(dst, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
    cats = {}
    for s in subs:
        cats[s["category"]] = cats.get(s["category"], 0) + 1
    print(f"items {len(items)}, sub-items {len(subs)}, categories {cats}")
    print(f"ops procedures {len(ops)}, sub-items needing one but none found {len(missing_ops)}")
    print(f"unparsed value lines {len(unparsed)}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
