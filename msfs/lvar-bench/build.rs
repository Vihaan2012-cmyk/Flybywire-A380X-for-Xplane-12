//! Emits the MSFS-specific link arguments for the `wasm32-wasip1` build.
//!
//! FlyByWire keep these in `.cargo/config.toml` with the SDK path hardcoded to
//! the one inside their Docker image (`/workdir/MSFS_SDK`). `rustflags` cannot
//! interpolate environment variables, so a hardcoded path means the crate only
//! builds in one place. Emitting the same flags from here instead lets the SDK
//! be found wherever it actually is, and lets a missing SDK say so in words
//! rather than as a linker error about a missing `libclang_rt.builtins-wasm32.a`.
//!
//! Caveat, stated plainly: the `msfs` crate's *own* build script also needs the
//! SDK, and dependencies build before we do, so on a machine with no SDK the
//! failure you see will be msfs-rs's terse
//! `Could not locate MSFS SDK. Make sure you have it installed or try setting
//! the MSFS_SDK env var.` and not the message below. `build.ps1` / `build.sh`
//! check before invoking cargo so the useful message comes out first.

use std::path::Path;

/// Where the SDK is looked for, in order. Matches msfs-rs's own
/// `msfs_sdk::calculate_msfs_sdk_path` so the two agree about what they found.
const SDK_CANDIDATES: [&str; 2] = ["/workdir/MSFS_SDK", r"C:\MSFS SDK"];

fn main() {
    println!("cargo:rerun-if-env-changed=MSFS_SDK");

    let target = std::env::var("TARGET").unwrap_or_default();
    if !target.starts_with("wasm32") {
        // A host build (`cargo check` on the workstation, say) links nothing
        // MSFS-specific. msfs-rs will still want the SDK for bindgen.
        return;
    }

    let sdk = resolve_sdk();
    println!("cargo:warning=lvar-bench: using MSFS SDK at {sdk}");

    let sysroot_lib = format!("{sdk}/WASM/wasi-sysroot/lib/wasm32-wasi");

    // Mirrors D:\fbw-aircraft\.cargo\config.toml, flag for flag, including the
    // split `-l` / `c` and `-L` / <path> pairs.
    let link_args: Vec<String> = vec![
        "-l".into(),
        "c".into(),
        format!("{sysroot_lib}/libclang_rt.builtins-wasm32.a"),
        "-L".into(),
        sysroot_lib.clone(),
        "--export-table".into(),
        "--allow-undefined".into(),
        "--export-dynamic".into(),
        "--export=__wasm_call_ctors".into(),
        "--export=malloc".into(),
        "--export=free".into(),
        "--export=mark_decommit_pages".into(),
        "--export=mallinfo".into(),
        "--export=mchunkit_begin".into(),
        "--export=mchunkit_next".into(),
        "--export=get_pages_state".into(),
    ];

    for arg in link_args {
        println!("cargo:rustc-link-arg={arg}");
    }
}

fn resolve_sdk() -> String {
    if let Ok(sdk) = std::env::var("MSFS_SDK") {
        if Path::new(&sdk).exists() {
            return sdk;
        }
        panic!(
            "MSFS_SDK is set to {sdk:?} but no such directory exists.\n\
             Point MSFS_SDK at the root of an installed MSFS SDK (the folder\n\
             containing WASM\\wasi-sysroot), or unset it to fall back to the\n\
             default locations."
        );
    }

    for candidate in SDK_CANDIDATES {
        if Path::new(candidate).exists() {
            return candidate.to_string();
        }
    }

    panic!(
        "\n\
         ------------------------------------------------------------------\n\
         lvar-bench cannot build: the MSFS SDK was not found.\n\
         \n\
         Looked for, in order:\n\
           $MSFS_SDK          (environment variable, not set)\n\
           {}\n\
           {}\n\
         \n\
         The SDK is a licensed Microsoft download and is not on crates.io,\n\
         not in this repository, and not in D:\\fbw-aircraft. Two ways out:\n\
         \n\
           1. Build in FlyByWire's Docker image, which already contains the\n\
              SDK at /workdir/MSFS_SDK. This needs no local SDK at all:\n\
                  msfs\\lvar-bench\\build.ps1 -Docker\n\
         \n\
           2. Install the SDK natively. It is offered in MSFS itself under\n\
              Options -> General -> Developers -> 'SDK Installer', or from\n\
              https://docs.flightsimulator.com/. Install it to C:\\MSFS SDK,\n\
              or install it elsewhere and set MSFS_SDK to that path.\n\
              A native build also needs clang and llvm-ar on PATH, because\n\
              msfs-rs compiles the SDK's nanovg.cpp.\n\
         ------------------------------------------------------------------\n"
        , SDK_CANDIDATES[0], SDK_CANDIDATES[1]
    );
}
