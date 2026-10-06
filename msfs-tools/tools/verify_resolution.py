#!/usr/bin/env python3
import json
import re
from pathlib import Path

EARLIER_PASS = Path("D:/A380/fbw-build/wasm-fs2020/ecam-msfs")
HERE = Path(__file__).parent

FBW_OWNED_TRIGGER_VARS = {
    "OVHD_APU_MASTER_SW_PB_IS_ON",
    "OVHD_APU_BLEED_PB_IS_ON",
    "OVHD_APU_START_PB_IS_ON",
    "OVHD_ELEC_APU_GEN_1_PB_IS_ON",
    "OVHD_ELEC_APU_GEN_2_PB_IS_ON",
}

RAW_MSFS_SIMVARS = {
    "AIRSPEED_KT",
}


def lvar_key(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9]", "_", name).upper()


def resolved(raw_name: str) -> str:
    key = lvar_key(raw_name)
    if key.startswith("A32NX_"):
        key = key[len("A32NX_"):]
    return "A32NX_" + key


def load_published() -> set[str]:
    out = set()
    with open(EARLIER_PASS / "host-published.txt", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            raw = line[len("A32NX_"):] if line.startswith("A32NX_") else line
            out.add(resolved(raw))
    return out


def is_raw_simvar(name: str) -> bool:
    return " " in name


def main() -> int:
    published = load_published()
    owned = {resolved(n) for n in FBW_OWNED_TRIGGER_VARS}
    alerts = json.loads((EARLIER_PASS / "alerts.json").read_text(encoding="utf-8"))

    total_vars = 0
    resolved_vars = 0
    fbw_owned_hits = 0
    raw_simvar_hits = 0
    dangling = []

    for a in alerts:
        missing = []
        for v in a.get("vars", []):
            raw = v[2:] if v[:2].lower() == "l:" else v
            total_vars += 1
            if is_raw_simvar(raw) or raw in RAW_MSFS_SIMVARS:
                raw_simvar_hits += 1
                resolved_vars += 1
                continue
            r = resolved(raw)
            if r in published:
                resolved_vars += 1
            elif r in owned:
                fbw_owned_hits += 1
                resolved_vars += 1
            else:
                missing.append(raw)
        if missing:
            dangling.append((a["key"], a["ata"], a["title"], missing))

    lines = []
    lines.append("# ECAM deep-alert trigger-variable resolution (static, from the earlier pass's dumps)")
    lines.append("")
    lines.append(f"Alerts: {len(alerts)}")
    lines.append(f"Trigger variable references: {total_vars}")
    lines.append(f"  resolved (deep-published): {resolved_vars - fbw_owned_hits - raw_simvar_hits}")
    lines.append(f"  resolved (FlyByWire-owned lvar, {len(FBW_OWNED_TRIGGER_VARS)} known names): {fbw_owned_hits}")
    lines.append(f"  resolved (raw MSFS simvar, read unprefixed): {raw_simvar_hits}")
    lines.append(f"  UNRESOLVED (real gap or dangling name): {total_vars - resolved_vars}")
    lines.append(f"Alerts with at least one unresolved trigger variable: {len(dangling)} / {len(alerts)}")
    lines.append("")
    lines.append("## Unresolved (needs a fix or a FBW_OWNED_TRIGGER_VARS/RAW_MSFS_SIMVARS entry)")
    lines.append("")
    for key, ata, title, missing in dangling:
        lines.append(f"- ATA{ata} `{key}` ({title}): {', '.join(missing)}")

    (HERE / "resolution-report.md").write_text("\n".join(lines), encoding="utf-8")
    (HERE / "resolution-report.json").write_text(
        json.dumps(
            {
                "alerts": len(alerts),
                "total_var_refs": total_vars,
                "resolved": resolved_vars,
                "unresolved": total_vars - resolved_vars,
                "dangling": [{"key": k, "ata": a, "title": t, "missing": m} for k, a, t, m in dangling],
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"{total_vars} trigger variable refs, {resolved_vars} resolved, {total_vars - resolved_vars} unresolved")
    print(f"report written to {HERE / 'resolution-report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
