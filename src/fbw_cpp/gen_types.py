"""Generates exact Rust mirrors of FlyByWire's Simulink bus structs.

Reads the generated `*_types.h` headers of FlyByWire's A380 PRIM, SEC and FCU
models (fbw-a380x/src/wasm/fbw_a380/src/model) and writes:

  src/fbw_types.rs                 #[repr(C)] structs, field for field, with
                                   compile-time size/align/offset asserts
  src/fbw_cpp/fbw_types_layout.h   the same numbers as C++ static_asserts,
                                   included by src/fbw_cpp/prim_shim.cpp

The layout numbers are worked out here from the x86-64 C rules (every field at
the next multiple of its alignment, the struct padded to its largest
alignment) and then checked by both compilers, so a disagreement between
this script, rustc and g++ is a build error rather than a silent misread.

Structs FlyByWire defines in more than one header must have identical fields
everywhere, or this fails. Structs already mirrored by src/fbw_controllers.rs
(the FADEC buses) are checked against the FADEC header and re-used from there.

Usage: python src/fbw_cpp/gen_types.py [path/to/fbw_a380/src/model]
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CRATE = os.path.dirname(os.path.dirname(HERE))
MODEL = (
    sys.argv[1]
    if len(sys.argv) > 1
    else os.path.join(CRATE, "..", "fbw-aircraft", "fbw-a380x", "src", "wasm", "fbw_a380", "src", "model")
)

HEADERS = [
    "A380PrimComputerGeneralLogic_types.h",
    "A380SecComputer_types.h",
    "A380FcuComputer_types.h",
]
FADEC_HEADER = "A380FadecComputer_types.h"

# The structs the shim hands across, and everything they contain, each taken
# from its own model's header. (Some structs the roots do not reach, such as
# base_elac_adr_computation_data, do differ between the models' headers.)
PRIM, SEC, FCU = HEADERS
ROOTS = {
    "prim_inputs": PRIM,
    "base_prim_discrete_outputs": PRIM,
    "base_prim_analog_outputs": PRIM,
    "base_prim_out_bus": PRIM,
    "ap_raw_laws_flare": PRIM,
    "sec_inputs": SEC,
    "base_sec_out_bus": SEC,
    "base_sec_discrete_outputs": SEC,
    "base_sec_analog_outputs": SEC,
    "fcu_inputs": FCU,
    "base_fcu_bus": FCU,
    "base_fcu_discrete_outputs": FCU,
}

# Already mirrored in src/fbw_controllers.rs (name there).
FROM_CONTROLLERS = {
    "base_arinc_429": "BaseArinc429",
    "base_prim_fctl_out_bus": "BasePrimFctlOutBus",
    "base_prim_fe_out_bus": "BasePrimFeOutBus",
    "base_prim_fg_out_bus": "BasePrimFgOutBus",
    "base_prim_out_bus": "BasePrimOutBus",
    "base_eec": "BaseEec",
}

PRIMITIVES = {
    # C type: (size, align, Rust type, comment)
    "real_T": (8, 8, "f64", None),
    "real32_T": (4, 4, "f32", None),
    "uint32_T": (4, 4, "u32", None),
    "int32_T": (4, 4, "i32", None),
    "boolean_T": (1, 1, "u8", "boolean_T (unsigned char)"),
    "int8_T": (1, 1, "i8", None),
    "uint8_T": (1, 1, "u8", None),
}


def read(name):
    with open(os.path.join(MODEL, name), encoding="utf-8") as f:
        return f.read().replace("\r", "")


def parse(text):
    structs = {}
    for m in re.finditer(r"struct (\w+)\s*\{(.*?)\};", text, re.S):
        fields = []
        for line in m.group(2).strip().splitlines():
            line = line.strip()
            if not line:
                continue
            fm = re.fullmatch(r"(\w+)\s+(\w+);", line)
            if not fm:
                raise SystemExit(f"unexpected field in {m.group(1)}: {line!r}")
            fields.append((fm.group(1), fm.group(2)))
        structs[m.group(1)] = fields
    enums = {}
    for m in re.finditer(r"enum class (\w+)\s*:\s*(\w+)", text):
        enums[m.group(1)] = m.group(2)
    return structs, enums


def camel(name):
    return "".join(part[:1].upper() + part[1:] for part in name.split("_"))


def main():
    parsed = {h: parse(read(h)) for h in HEADERS}
    structs, enums, origin = {}, {}, {}

    def take(name, header):
        hs, he = parsed[header]
        if name in structs:
            if structs[name] != hs[name]:
                raise SystemExit(f"{name} differs between {origin[name]} and {header}")
            return
        structs[name] = hs[name]
        origin[name] = header
        for ty, _ in hs[name]:
            if ty in hs:
                take(ty, header)
            elif ty in he:
                if ty in enums and enums[ty] != he[ty]:
                    raise SystemExit(f"enum {ty} differs between headers")
                enums[ty] = he[ty]

    for root in ROOTS:
        take(root, ROOTS[root])
    fadec, _ = parse(read(FADEC_HEADER))
    for name in FROM_CONTROLLERS:
        if fadec.get(name) != structs.get(name):
            raise SystemExit(f"{name} in {FADEC_HEADER} differs from the PRIM headers")

    layout = {}

    def lay(name):
        if name in layout:
            return layout[name]
        offset, align, offsets = 0, 1, []
        for ty, field in structs[name]:
            if ty in PRIMITIVES:
                size, a = PRIMITIVES[ty][:2]
            elif ty in enums:
                size, a = PRIMITIVES[enums[ty]][:2]
            elif ty in structs:
                size, a = lay(ty)[:2]
            else:
                raise SystemExit(f"unknown type {ty} in {name}")
            offset = (offset + a - 1) // a * a
            offsets.append((field, offset))
            offset += size
            align = max(align, a)
        size = (offset + align - 1) // align * align
        layout[name] = (size, align, offsets)
        return layout[name]

    order = []

    def visit(name):
        if name in order:
            return
        for ty, _ in structs[name]:
            if ty in structs:
                visit(ty)
        order.append(name)

    for root in ROOTS:
        visit(root)
        lay(root)

    rust = []
    rust.append("//! Exact mirrors of FlyByWire's A380 PRIM, SEC and FCU bus structs.")
    rust.append("//!")
    rust.append("//! GENERATED by src/fbw_cpp/gen_types.py from the Simulink headers")
    rust.append("//! A380PrimComputerGeneralLogic_types.h, A380SecComputer_types.h and")
    rust.append("//! A380FcuComputer_types.h. Do not edit; re-run the script instead.")
    rust.append("//!")
    rust.append("//! `boolean_T` is `u8`; Simulink `enum class ... : int32_T` fields are `i32`")
    rust.append("//! (the enum's name is in the field comment). Every size, alignment and")
    rust.append("//! field offset is asserted here and, with the same numbers, in")
    rust.append("//! src/fbw_cpp/fbw_types_layout.h against g++'s layout.")
    rust.append("")
    rust.append("#![allow(dead_code, non_snake_case, clippy::upper_case_acronyms)]")
    rust.append("")
    rust.append("use std::mem::{align_of, offset_of, size_of};")
    rust.append("")
    reused = sorted(set(FROM_CONTROLLERS.values()))
    rust.append("pub use crate::fbw_controllers::{" + ", ".join(reused) + "};")
    rust.append("")
    for name in order:
        if name in FROM_CONTROLLERS:
            continue
        rust.append(f"/// `{name}` in {origin[name]}.")
        rust.append("#[repr(C)]")
        rust.append("#[derive(Clone, Copy, Debug, Default)]")
        rust.append(f"pub struct {camel(name)} {{")
        for ty, field in structs[name]:
            if ty in PRIMITIVES:
                rty, comment = PRIMITIVES[ty][2], PRIMITIVES[ty][3]
            elif ty in enums:
                rty, comment = PRIMITIVES[enums[ty]][2], f"{ty} (enum class : {enums[ty]})"
            else:
                rty, comment = FROM_CONTROLLERS.get(ty, camel(ty)), None
            line = f"    pub {field}: {rty},"
            if comment:
                line += f"  // {comment}"
            rust.append(line)
        rust.append("}")
        rust.append("")

    rust.append("// Layout contract with src/fbw_cpp/fbw_types_layout.h (same numbers).")
    rust.append("const _: () = {")
    cpp = []
    cpp.append("// GENERATED by src/fbw_cpp/gen_types.py. Do not edit.")
    cpp.append("// Layout contract with src/fbw_types.rs: the same sizes, alignments and")
    cpp.append("// offsets are asserted there.")
    cpp.append("#pragma once")
    cpp.append("#include <cstddef>")
    cpp.append("")
    for name in order:
        size, align, offsets = layout[name]
        rname = FROM_CONTROLLERS.get(name, camel(name))
        rust.append(f"    assert!(size_of::<{rname}>() == {size} && align_of::<{rname}>() == {align});")
        cpp.append(f"static_assert(sizeof({name}) == {size} && alignof({name}) == {align}, \"{name}\");")
        for field, off in offsets:
            rust.append(f"    assert!(offset_of!({rname}, {field}) == {off});")
            cpp.append(f"static_assert(offsetof({name}, {field}) == {off}, \"{name}.{field}\");")
    rust.append("};")
    rust.append("")

    with open(os.path.join(CRATE, "src", "fbw_types.rs"), "w", newline="\n", encoding="utf-8") as f:
        f.write("\n".join(rust))
    with open(os.path.join(HERE, "fbw_types_layout.h"), "w", newline="\n", encoding="utf-8") as f:
        f.write("\n".join(cpp) + "\n")
    print(f"{len(order)} structs; prim_inputs is {layout['prim_inputs'][0]} bytes")


main()
