//! Compiles FlyByWire's generated A380 computers (their Simulink C++, used
//! unchanged from the FlyByWire checkout next to this crate) and the small C
//! ABI shims in `src/fbw_cpp` into one static library:
//!
//! - the FADEC computer (`A380FadecComputer`), with `shim.cpp`;
//! - the three PRIMs, three SECs and two FCUs, run through FlyByWire's own
//!   wrappers `prim/Prim.cpp`, `sec/Sec.cpp` and `fcu/Fcu.cpp` (self tests,
//!   power monitoring, the order the PRIM's four model parts step in), with
//!   `prim_shim.cpp`.
//!
//! The wrappers are copied byte for byte into OUT_DIR so that Prim.cpp's
//! `#include "../interface/SimConnectInterface.h"` finds the stand-in in
//! `src/fbw_cpp/stubs` rather than FlyByWire's MSFS SimConnect header; their
//! other relative includes (`../model/...`, `../utils/...`, `../Arinc429.h`)
//! fall through to FlyByWire's tree through the include paths, as in their
//! own build (fbw_a380/build.sh, which adds `-I src/prim`, `-I src/sec`, ...).

use std::path::{Path, PathBuf};

fn main() {
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let out = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let fbw = manifest.join("../fbw-aircraft/fbw-a380x/src/wasm/fbw_a380/src");
    let model = fbw.join("model");
    let shim = manifest.join("src/fbw_cpp");

    // The wrappers, copied unchanged, beside the SimConnectInterface stand-in.
    let wrappers = out.join("fbw_a380");
    for (dir, files) in [
        ("prim", ["Prim.cpp", "Prim.h"]),
        ("sec", ["Sec.cpp", "Sec.h"]),
        ("fcu", ["Fcu.cpp", "Fcu.h"]),
    ] {
        std::fs::create_dir_all(wrappers.join(dir)).unwrap();
        for file in files {
            copy(&fbw.join(dir).join(file), &wrappers.join(dir).join(file));
        }
    }
    std::fs::create_dir_all(wrappers.join("interface")).unwrap();
    copy(
        &shim.join("stubs/interface/SimConnectInterface.h"),
        &wrappers.join("interface/SimConnectInterface.h"),
    );

    let fadec_sources = [
        model.join("A380FadecComputer.cpp"),
        model.join("A380FadecComputer_data.cpp"),
        shim.join("shim.cpp"),
    ];
    // As listed in fbw_a380/build.sh for the PRIM, SEC and FCU and what they
    // call.
    let mut computer_sources: Vec<PathBuf> = [
        "A380PrimComputerGeneralLogic_data",
        "A380PrimComputerGeneralLogic",
        "A380PrimComputerFctl_data",
        "A380PrimComputerFctl",
        "A380PrimComputerFe_data",
        "A380PrimComputerFe",
        "A380FgOuterLoops",
        "A380PrimComputerFg_data",
        "A380PrimComputerFg",
        "A380SecComputer_data",
        "A380SecComputer",
        "A380PitchNormalLaw",
        "A380PitchAlternateLaw",
        "A380PitchDirectLaw",
        "A380LateralNormalLaw",
        "A380LateralDirectLaw",
        "A380FcuComputer_data",
        "A380FcuComputer",
        "intrp3d_l_pw",
        "look1_binlxpw",
        "look1_binlcpw",
        "look1_iflf_binlxpw",
        "look2_binlxpw",
        "look2_iflf_binlxpw",
        "plook_binx",
        "binsearch_u32d",
        "rt_modd",
        "rt_remd",
    ]
    .iter()
    .map(|name| model.join(format!("{name}.cpp")))
    .collect();
    computer_sources.extend([
        wrappers.join("prim/Prim.cpp"),
        wrappers.join("sec/Sec.cpp"),
        wrappers.join("fcu/Fcu.cpp"),
        fbw.join("utils/PulseNode.cpp"),
        fbw.join("utils/SRFlipFLop.cpp"),
        shim.join("prim_shim.cpp"),
    ]);

    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-changed={}", shim.display());
    println!("cargo:rerun-if-changed={}", model.display());
    for dir in ["prim", "sec", "fcu", "utils"] {
        println!("cargo:rerun-if-changed={}", fbw.join(dir).display());
    }

    cc::Build::new()
        .cpp(true)
        .std("c++17")
        .files(&fadec_sources)
        .files(&computer_sources)
        // The copied wrappers first, then FlyByWire's tree as their own build
        // has it.
        .include(&wrappers)
        .include(&model)
        .include(fbw.join("prim"))
        .include(fbw.join("sec"))
        .include(fbw.join("fcu"))
        .include(fbw.join("utils"))
        .include(&shim)
        // As FlyByWire builds it: no exceptions. No RTTI is needed either.
        .flag_if_supported("-fno-exceptions")
        .flag_if_supported("-fno-rtti")
        // Keep multiply-add pairs as written, so results do not depend on
        // whether the compiler fuses them.
        .flag_if_supported("-ffp-contract=off")
        // Keep sin and cos as the separate libm calls the code makes, rather
        // than fused into one `sincos` call.
        .flag_if_supported("-fno-builtin-sin")
        .flag_if_supported("-fno-builtin-cos")
        // The computers are placed into zeroed memory to start from the same
        // zeros as FlyByWire's global instance; keep that zeroing.
        .flag_if_supported("-fno-lifetime-dse")
        .opt_level(2)
        .warnings(false)
        // The models only need <cmath>; nothing from libstdc++ is linked.
        .cpp_link_stdlib(None)
        // The library is linked by `#[link]` on the extern blocks in
        // src/fbw_controllers.rs and src/fbw_computers.rs, not by a
        // `rustc-link-lib` line here: Cargo hands build-script link libraries
        // only to the library target when there is one, and the integration
        // tests compile those modules themselves. The search path does reach
        // every target.
        .cargo_metadata(false)
        .compile("fbw_controllers");
    println!("cargo:rustc-link-search=native={}", out.display());
}

fn copy(from: &Path, to: &Path) {
    let bytes = std::fs::read(from).unwrap_or_else(|e| panic!("{}: {e}", from.display()));
    // Only rewrite when changed, so the C++ is not rebuilt for nothing.
    if std::fs::read(to).ok().as_deref() != Some(&bytes[..]) {
        std::fs::write(to, bytes).unwrap();
    }
}
