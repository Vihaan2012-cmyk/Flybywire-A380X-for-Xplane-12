#!/usr/bin/env python3
import os
import re
import sys
from collections import defaultdict

CHECKOUT = "D:/A380/fbw-xp-worktrees/fs2020-672384b"
DEEP_SRC = f"{CHECKOUT}/fbw-a380x/src/wasm/systems/deep_systems/src"
HOST_DIR = f"{CHECKOUT}/fbw-a380x/src/wasm/systems/a380_systems/src/deep_systems"
LIVE_RS = f"{DEEP_SRC}/deep/live.rs"
TRUTH_RS = f"{HOST_DIR}/truth.rs"
REPORTS = "D:/A380/msfs-a380/reports"
GAPS_DIR = f"{REPORTS}/gaps"

CONSUMER_ROOTS = [
    f"{CHECKOUT}/fbw-a380x/src/wasm/systems/a380_systems/src",
    f"{CHECKOUT}/fbw-common/src/wasm/systems/systems/src",
    f"{CHECKOUT}/fbw-a380x/src/wasm/fbw_a380",
    f"{CHECKOUT}/fbw-a380x/src/wasm/fadec_a380x",
    f"{CHECKOUT}/fbw-a380x/src/systems/instruments/src",
    f"{CHECKOUT}/fbw-a380x/src/systems/systems-host",
    f"{CHECKOUT}/fbw-common/src/systems/instruments/src",
]

PUBLISHER_ROOTS = [f"{DEEP_SRC}/deep"]

RUST_EXT = {".rs"}
CPP_EXT = {".cpp", ".h", ".hpp", ".cc"}
TS_EXT = {".ts", ".tsx", ".js", ".jsx"}
ALL_EXT = RUST_EXT | CPP_EXT | TS_EXT


def iter_files(roots, exts=ALL_EXT):
    for root in roots:
        if not os.path.isdir(root):
            continue
        for dirpath, _dirnames, filenames in os.walk(root):
            if "node_modules" in dirpath or "/target" in dirpath.replace("\\", "/"):
                continue
            for fn in filenames:
                if os.path.splitext(fn)[1].lower() in exts:
                    yield os.path.join(dirpath, fn)


def read(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def extract_struct_block(text, struct_name):
    m = re.search(r"pub struct %s\s*\{" % re.escape(struct_name), text)
    if not m:
        return ""
    start = m.end()
    depth = 1
    i = start
    while i < len(text) and depth > 0:
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
        i += 1
    return text[start:i - 1]


FIELD_RE = re.compile(r"^\s*pub\s+([a-z_][a-z0-9_]*)\s*:\s*([^,]+?),?\s*$", re.MULTILINE)


def strip_doc_comments(block):
    lines = []
    in_block_comment = False
    for line in block.splitlines():
        s = line.strip()
        if in_block_comment:
            if "*/" in s:
                in_block_comment = False
            continue
        if s.startswith("/*"):
            if "*/" not in s:
                in_block_comment = True
            continue
        if s.startswith("///") or s.startswith("//!") or s.startswith("//"):
            continue
        if s.startswith("#["):
            continue
        lines.append(line)
    return "\n".join(lines)


def parse_truth_fields(live_rs_text):
    block = extract_struct_block(live_rs_text, "Truth")
    block = strip_doc_comments(block)
    fields = []
    for m in FIELD_RE.finditer(block):
        name, ty = m.group(1), m.group(2).strip()
        fields.append((name, ty))
    return fields


def extract_truth_literal(truth_rs_text):
    matches = list(re.finditer(r"\bTruth\s*\{", truth_rs_text))
    if not matches:
        return ""
    m = matches[-1]
    start = m.end()
    depth = 1
    i = start
    while i < len(truth_rs_text) and depth > 0:
        if truth_rs_text[i] == "{":
            depth += 1
        elif truth_rs_text[i] == "}":
            depth -= 1
        i += 1
    return truth_rs_text[start:i - 1]


LITERAL_FIELD_RE = re.compile(r"(?:^|[,{]\s*)([a-z_][a-z0-9_]*)\s*(?::|(?=[,}]))", re.MULTILINE)


def parse_truth_literal_fields(literal_text):
    has_default_spread = bool(re.search(r"\.\.\s*default\b", literal_text))
    literal_text = "\n".join(re.sub(r"//.*$", "", ln) for ln in literal_text.splitlines())
    names = set()
    depth = 0
    entry_start = 0
    entries = []
    for i, ch in enumerate(literal_text):
        if ch in "{([":
            depth += 1
        elif ch in "})]":
            depth -= 1
        elif ch == "," and depth == 0:
            entries.append(literal_text[entry_start:i])
            entry_start = i + 1
    entries.append(literal_text[entry_start:])
    for entry in entries:
        e = entry.strip()
        if not e or e.startswith(".."):
            continue
        m = re.match(r"^([a-z_][a-z0-9_]*)\s*(?::|$)", e)
        if m:
            names.add(m.group(1))
    return names, has_default_spread


def find_post_literal_assignments(mod_rs_text):
    names = {}
    for m in re.finditer(r"\btruth\.([a-z_][a-z0-9_]*)\s*(?:\[[^\]]*\])?\s*=\s*([^;]+);", mod_rs_text):
        names[m.group(1)] = m.group(2).strip()
    return names


def classify_inputs():
    live_text = read(LIVE_RS)
    truth_text = read(TRUTH_RS)
    mod_text = read(f"{HOST_DIR}/mod.rs")
    fields = parse_truth_fields(live_text)
    literal = extract_truth_literal(truth_text)
    set_names, has_default = parse_truth_literal_fields(literal)
    post_literal = find_post_literal_assignments(mod_text)

    rows = []
    for name, ty in fields:
        if name == "published":
            rows.append((name, ty, "n/a", "the previous frame's published outputs, handed back by Deep::tick"))
            continue
        if name in set_names:
            rows.append((name, ty, "real/derived", "set explicitly in truth.rs's Truth{...} literal"))
        elif name in post_literal:
            rows.append((name, ty, "real/derived",
                         f"set after the literal, in mod.rs's update(): `truth.{name} = {post_literal[name]}`"))
        else:
            rows.append((name, ty, "GAP: default", "not set in the literal, and no `truth.<field> = ...` found in mod.rs's update() -> falls through to Truth::default() via ..default"))
    return rows, has_default


OUT_CALL_RE = re.compile(r'out\(\s*"([A-Z][A-Z0-9_]*)"')
OUT_CALL_FMT_RE = re.compile(r'out\(\s*&?format!\(\s*"([A-Z][A-Z0-9_]*?)(\{[^"]*)?"')
FORMAT_STEM_RE = re.compile(r'format!\(\s*"([A-Z][A-Z0-9_]*?)(?:\{[^"]*\})?"')
GENERIC_STRING_RE = re.compile(r'"(DEEP_[A-Z0-9_]+|[A-Z][A-Z0-9_]{3,})"')


def collect_published_stems():
    stems = set()
    literal_names = set()
    host_mod = read(f"{HOST_DIR}/mod.rs")
    for m in re.finditer(r'get_identifier\(\s*"(DEEP_[A-Z0-9_]+)"', host_mod):
        stems.add(m.group(1))
        literal_names.add(m.group(1))
    for m in re.finditer(r'get_identifier\(\s*format!\(\s*"(DEEP_[A-Z0-9_]*?)\{', host_mod):
        stems.add(m.group(1).rstrip("_"))
    for path in iter_files(PUBLISHER_ROOTS, {".rs"}):
        text = read(path)
        if "out(" not in text and "format!" not in text:
            continue
        for m in OUT_CALL_RE.finditer(text):
            if m.group(1).startswith("A32NX_"):
                continue
            literal_names.add(m.group(1))
            stems.add(m.group(1))
        for m in FORMAT_STEM_RE.finditer(text):
            stem = m.group(1).rstrip("_")
            if len(stem) > 3 and not stem.startswith("A32NX_"):
                stems.add(stem)
    return stems, literal_names


def collect_registry_alert_names():
    names = set()
    for path in iter_files(PUBLISHER_ROOTS, {".rs"}):
        if not path.endswith("registry.rs"):
            continue
        text = read(path)
        for m in re.finditer(r'"(DEEP_[A-Z0-9_]+)"', text):
            names.add(m.group(1))
    return names


RUST_GET_ID_RE = re.compile(r'get_identifier\(\s*"([A-Za-z0-9_]+)"')
CPP_LVAR_RE = re.compile(r'"(?:L:)?A32NX_([A-Z0-9_]+)"')
TS_LVAR_RE = re.compile(r'L:A32NX_([A-Z0-9_]+)')
TRUTH_PUBLISHED_GET_RE = re.compile(r'\.published\.get(?:_or)?\(\s*"([A-Z0-9_]+)"')


def collect_consumers():
    rust_ids = set()
    cpp_ts_names = set()
    deep_internal = set()

    for path in iter_files(CONSUMER_ROOTS, RUST_EXT):
        text = read(path)
        for m in RUST_GET_ID_RE.finditer(text):
            rust_ids.add(m.group(1))
    for path in iter_files(CONSUMER_ROOTS, CPP_EXT):
        text = read(path)
        for m in CPP_LVAR_RE.finditer(text):
            cpp_ts_names.add(m.group(1))
    for path in iter_files(CONSUMER_ROOTS, TS_EXT):
        text = read(path)
        for m in TS_LVAR_RE.finditer(text):
            cpp_ts_names.add(m.group(1))
    for path in iter_files(PUBLISHER_ROOTS, {".rs"}):
        text = read(path)
        for m in TRUTH_PUBLISHED_GET_RE.finditer(text):
            deep_internal.add(m.group(1))

    return rust_ids, cpp_ts_names, deep_internal


def stem_of(name):
    return re.sub(r"[_0-9]+$", "", name)


FAULT_HINT = re.compile(r"(FAULT|ALARM|OVERLOAD|TRIP|WARN|AVAIL|DISAGREE|DEGRADED|LOSS|LOST|FAILED|FAILURE|JAMMED|STUCK|LEAK|OVERHEAT|OVERSPEED|SHORT|OPEN\b|CLOSED\b|NOT_)")
DIAG_HINT = re.compile(r"(WEAR|HOURS|CURRENT_A|TEMP|PRESSURE|VOLTAGE|SPEED|POSITION|FRAC|RATE|COUNT|PCT|_C$|_K$|_PA$|_W$|_V$)")


def likely_kind(name):
    if FAULT_HINT.search(name):
        return "health/loss -- check display"
    if DIAG_HINT.search(name):
        return "diagnostic -- ok if unread"
    return "?"


def name_is_consumed(name, rust_ids, cpp_ts_names, deep_internal):
    candidates = rust_ids | cpp_ts_names | deep_internal
    if name in candidates:
        return True
    st = stem_of(name)
    for c in candidates:
        if c == name or c.startswith(name) or name.startswith(c):
            return True
        if stem_of(c) == st and st:
            return True
    return False


HAND_VERIFIED_FINDINGS = {
    "study-pages": [
        "`DEEP_DERIVED_FBW_FAILURE_<id>` (49 ids: 25 electrical + 16 hydraulics "
        "+ 8 pneumatic, `docs/deep/authority.md`): published every frame by the "
        "owning area, confirmed **not** read anywhere under "
        "`fbw-common/src/systems/instruments/src` or "
        "`fbw-a380x/src/systems/instruments/src` (grepped by hand, zero hits). "
        "The host only consumes it as a Rust value "
        "(`a380_systems/src/deep_systems/mod.rs:493`, `self.deep.derived_failures()`) "
        "to drive FlyByWire's own failure system -- the crew never sees *why* a "
        "level-2 verdict fired, only FlyByWire's resulting page (ELEC/HYD/BLEED). "
        "`authority.md` says this is meant to be Study-visible with the component "
        "and reason. Real gap: wire a Study panel row per coupling. NOTE: "
        "`authority.md` (as written) only documents electrical/hydraulic/pneumatic "
        "(49 couplings) -- by 21:25 this session, `fire_ice`, `gear_structure` and "
        "`sensors` also implement `derived_failures` (fire-suppression 26_0xx, "
        "12 gear-structure couplings, radio-altimeter/transceiver 34_0xx), which "
        "the design doc predates. Same gap, larger scope than the doc states; "
        "route to A4 (fire-gear-apu) and A8 (avionics) to confirm the design doc "
        "should be updated, not just the Study page.",
    ],
    "avionics": [
        "`DEEP_ENG_<n>_OIL_QTY_SENSED_FRAC` / `_OIL_TEMP_SENSED_C` "
        "(`deep/sensors/live_discrete.rs:1004,1010`): flagged dangling because "
        "the publisher stores the `format!(...)`-built name in a struct field "
        "(`var: format!(...)`) and publishes it later as `out(&x.var, ...)`, a "
        "pattern this script's regexes don't follow. Confirmed real and "
        "consumed by hand: `SD/Pages/Engine/elements/EngineColumn.tsx:44-46` "
        "reads both through a template-literal `useSimVar`. Not an action item.",
    ],
    "ecam": [
        "`DEEP_SMOKE_FWD_CARGO_A_ALARM` / `DEEP_SMOKE_CARGO_BULK_ALARM`: this "
        "script's regexes cannot find the publisher (the name is built in a way "
        "neither `out(\"LITERAL\"` nor `format!(\"STEM_{}\"` catches), but it IS "
        "published and IS consumed -- confirmed by hand: read by "
        "`SD/Pages/Cond/elements/CargoTemperatures.tsx` and asserted by "
        "`deep_systems/src/deep/sensors/live_discrete.rs`'s own tests. Listed here "
        "only so nobody re-flags it as a dangling read; not an action item.",
    ],
}

AREA_OWNER_FILES = {
    "electrical": ["deep/electrical"],
    "hydraulics": ["deep/hydraulics"],
    "pneumatic": ["deep/pneumatic_ducts", "deep/thermal_zones"],
    "fire-gear-apu": ["deep/fire_ice", "deep/gear_structure", "deep/apu", "deep/oxygen"],
    "engines": ["deep/engine_accessories"],
    "flight-controls": ["deep/flight_controls", "deep/autoflight"],
    "fuel": ["deep/fuel"],
    "avionics": ["deep/sensors", "deep/avionics_network", "deep/communications", "deep/cabin", "deep/environment"],
    "breakers": ["deep/breakers", "deep/wiring"],
    "ecam": ["deep/ecam"],
}

TRUTH_FIELD_OWNER = {
    "engine_": "engines",
    "apu_": "fire-gear-apu",
    "ac_bus": "electrical",
    "dc_bus": "electrical",
    "prim_": "flight-controls",
    "sec_": "flight-controls",
    "flap_lever": "flight-controls",
    "capt_sidestick": "flight-controls",
    "rudder_pedal": "flight-controls",
    "body_rate": "flight-controls",
    "hydraulic_pressure": "hydraulics",
    "tyre_pressure": "fire-gear-apu",
    "door_open": "avionics",
    "gpu_plugged_in": "electrical",
    "controls": "avionics",
    "commanded_surfaces": "flight-controls",
    "leg_": "fire-gear-apu",
    "cabin_": "pneumatic",
    "fdac_": "pneumatic",
    "ocsm_": "pneumatic",
    "fuel_tank_quantity": "fuel",
    "landing_elevation": "avionics",
    "athr_": "flight-controls",
    "ap1_active": "flight-controls",
    "ap2_active": "flight-controls",
    "fmgc_flight_phase": "flight-controls",
    "ir": "avionics",
    "att_hdg_switching_knob": "avionics",
    "pack_flow_insufficient": "pneumatic",
    "environment": "avionics",
    "sun_elevation": "avionics",
}


def owner_for_field(name):
    for prefix, owner in TRUTH_FIELD_OWNER.items():
        if name.startswith(prefix) or name == prefix:
            return owner
    return "avionics"


def owner_for_published(name, dir_hits):
    for owner, dirs in AREA_OWNER_FILES.items():
        for d in dirs:
            if d in dir_hits.get(name, set()):
                return owner
    return "unknown"


def main():
    os.makedirs(GAPS_DIR, exist_ok=True)

    input_rows, has_default_spread = classify_inputs()
    n_real = sum(1 for r in input_rows if r[2] == "real/derived")
    n_gap = sum(1 for r in input_rows if r[2].startswith("GAP"))

    stems, literal_names = collect_published_stems()
    registry_names = collect_registry_alert_names()
    all_published = stems | registry_names
    rust_ids, cpp_ts_names, deep_internal = collect_consumers()

    consumed = set()
    unconsumed = set()
    for name in sorted(all_published):
        if name_is_consumed(name, rust_ids, cpp_ts_names, deep_internal):
            consumed.add(name)
        else:
            unconsumed.add(name)

    all_consumer_deep = {n for n in (rust_ids | cpp_ts_names) if n.startswith("DEEP_")}
    dangling = set()
    for name in sorted(all_consumer_deep):
        st = stem_of(name)
        if not any(p == name or p.startswith(st) or st.startswith(stem_of(p)) for p in all_published):
            dangling.add(name)

    with open(f"{REPORTS}/wiring-matrix.md", "w", encoding="utf-8") as f:
        f.write("# Wiring matrix (A18)\n\n")
        f.write("Static analysis, regenerated by `tools/wiring_audit.py`. ")
        f.write("Output-side matching is by name/stem and is best-effort for ")
        f.write("dynamically-built (`format!`) names -- see the tool's own docstring.\n\n")
        f.write("## Totals\n\n")
        f.write(f"- Truth inputs: {len(input_rows)} fields, **{n_real} real/derived**, **{n_gap} default (gap)**\n")
        f.write(f"- Published names/stems found: {len(all_published)} ({len(literal_names)} literal `out(\"...\")`, "
                f"{len(stems) - len(literal_names)} `format!` stems, {len(registry_names)} registry-only)\n")
        f.write(f"- Consumed: {len(consumed)}, **Unconsumed (gap): {len(unconsumed)}**\n")
        f.write(f"- Dangling `DEEP_*` reads (consumer cites a name nothing publishes): **{len(dangling)}**\n\n")

        f.write("## Gaps first: Truth inputs still at `Truth::default()`\n\n")
        f.write("| field | type | note |\n|---|---|---|\n")
        for name, ty, status, note in input_rows:
            if status.startswith("GAP"):
                f.write(f"| `{name}` | `{ty}` | {note} |\n")
        f.write("\n")

        f.write("## Gaps first: published names with no found consumer\n\n")
        f.write("(best-effort stem match; a name here may still be read through a pattern this script "
                "doesn't parse, e.g. a name assembled from two `format!` calls -- treat as a lead, not gospel)\n\n")
        for name in sorted(unconsumed):
            f.write(f"- `{name}` ({likely_kind(name)})\n")
        f.write("\n")

        f.write("## Dangling DEEP_* reads (consumer names nothing publishes)\n\n")
        for name in sorted(dangling):
            loc = "(location not found)"
            for path in iter_files(CONSUMER_ROOTS, ALL_EXT):
                text = read(path)
                idx = text.find(name)
                if idx != -1:
                    loc = f"{path}:{text.count(chr(10), 0, idx) + 1}"
                    break
            f.write(f"- `{name}` -- {loc}\n")
        f.write("\n")

        f.write("## Full Truth input table\n\n")
        f.write("| field | type | status |\n|---|---|---|\n")
        for name, ty, status, note in input_rows:
            f.write(f"| `{name}` | `{ty}` | {status} |\n")
        f.write("\n")

        f.write("## Full published-name table\n\n")
        f.write("| name/stem | consumed? |\n|---|---|\n")
        for name in sorted(all_published):
            f.write(f"| `{name}` | {'yes' if name in consumed else 'NO'} |\n")

    gap_by_area = defaultdict(list)
    for name, ty, status, note in input_rows:
        if status.startswith("GAP"):
            gap_by_area[owner_for_field(name)].append(("input", name, ty, note))

    dir_hits = defaultdict(set)
    for path in iter_files(PUBLISHER_ROOTS, {".rs"}):
        rel = os.path.relpath(path, DEEP_SRC).replace("\\", "/")
        area_dir = "/".join(rel.split("/")[:2])
        text = read(path)
        found = set(m.group(1) for m in OUT_CALL_RE.finditer(text))
        found |= set(m.group(1).rstrip("_") for m in FORMAT_STEM_RE.finditer(text))
        found |= set(m.group(1) for m in re.finditer(r'"(DEEP_[A-Z0-9_]+)"', text))
        for n in found:
            dir_hits[n].add(area_dir)

    def owner_by_prefix(name):
        best_owner, best_len = "unrouted", 0
        for pub_name, dirs in dir_hits.items():
            if name.startswith(pub_name) or pub_name.startswith(name):
                for owner, area_dirs in AREA_OWNER_FILES.items():
                    if dirs & set(area_dirs) and len(pub_name) > best_len:
                        best_owner, best_len = owner, len(pub_name)
        return best_owner

    for name in sorted(unconsumed):
        owner = owner_for_published(name, dir_hits)
        if owner == "unknown":
            owner = owner_by_prefix(name)
        gap_by_area[owner].append(("output", name, likely_kind(name), "no consumer found (best-effort match)"))

    def find_first_line(name, roots, exts):
        pat = re.compile(re.escape(name))
        for path in iter_files(roots, exts):
            text = read(path)
            idx = text.find(name)
            if idx != -1:
                line = text.count("\n", 0, idx) + 1
                return f"{path}:{line}"
        return "(location not found)"

    dangling_locations = {}
    for name in sorted(dangling):
        owner = owner_for_published(stem_of(name), dir_hits)
        if owner == "unknown":
            owner = owner_by_prefix(stem_of(name))
        if owner in ("unknown", "unrouted"):
            owner = "ecam"
        loc = find_first_line(name, CONSUMER_ROOTS, ALL_EXT)
        dangling_locations[name] = loc
        gap_by_area[owner].append(("dangling-read", name, "",
                                    f"consumer reads this DEEP_* name; no publisher matched it. First seen: {loc}"))

    area_files = ["electrical", "hydraulics", "pneumatic", "fire-gear-apu", "engines",
                  "flight-controls", "fuel", "avionics", "breakers", "ecam", "sd-pages",
                  "study-pages", "unrouted"]
    for area in area_files:
        items = gap_by_area.get(area, [])
        with open(f"{GAPS_DIR}/{area}.md", "w", encoding="utf-8") as f:
            f.write(f"# Gaps for {area} (from A18's wiring audit)\n\n")
            f.write(f"Regenerated by `tools/wiring_audit.py`. {len(items)} candidate gap(s).\n\n")
            inputs = [i for i in items if i[0] == "input"]
            outputs = [i for i in items if i[0] == "output"]
            dangling_items = [i for i in items if i[0] == "dangling-read"]
            if inputs:
                f.write("## Truth inputs still defaulted\n\n")
                for _, name, ty, note in inputs:
                    f.write(f"- `Truth::{name}` (`{ty}`): {note}. File: `{TRUTH_RS}`\n")
                f.write("\n")
            if outputs:
                f.write("## Published values with no found consumer\n\n")
                f.write("`kind` is a naming heuristic only (never trust it over reading the code): "
                        "**health/loss** names look like an annunciation FlyByWire or a display should "
                        "show if real (a gap if truly unread); **diagnostic** names look like a raw "
                        "measurement, which is fine to leave Study-only or even unread; **?** didn't "
                        "match either pattern.\n\n")
                for _, name, kind, note in outputs:
                    f.write(f"- `{name}` ({kind}): {note}\n")
                f.write("\n")
            if dangling_items:
                f.write("## Dangling reads (nothing publishes this name)\n\n")
                for _, name, _, note in dangling_items:
                    f.write(f"- `{name}`: {note}\n")
                f.write("\n")
            if not items:
                f.write("No gaps found by this pass.\n")
            hand = HAND_VERIFIED_FINDINGS.get(area)
            if hand:
                f.write("\n## Checked by hand (not just regex-matched)\n\n")
                for note in hand:
                    f.write(f"- {note}\n")

    print(f"inputs: {n_real} real, {n_gap} gap")
    print(f"published: {len(all_published)}, consumed: {len(consumed)}, unconsumed: {len(unconsumed)}")
    print(f"dangling: {len(dangling)}")
    for name, loc in dangling_locations.items():
        print(f"  DANGLING {name} -- {loc}")
    unrouted_count = len(gap_by_area.get("unrouted", []))
    if unrouted_count:
        print(f"unrouted gap items (see gaps/unrouted.md): {unrouted_count}")


if __name__ == "__main__":
    sys.exit(main())
