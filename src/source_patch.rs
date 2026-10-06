//! A text-level fix applied to FlyByWire's own compiled JS as it loads
//! (`docs/js-build.md`), shared by both engines that can serve that JS: the
//! plugin's in-process QuickJS `Cockpit` (`src/js/msfs/mod.rs`, behind
//! `feature = "js"`) and XPHFBW's separate `coui://` handler
//! (`app/src/scheme.rs`), a different binary (`XPHFBW.exe`) that links this
//! crate WITHOUT that feature. `SourcePatch` and `apply` live here,
//! unconditionally compiled and with no dependency on `rquickjs`/`oxc`, so
//! both can reach them -- before this module existed, every `SourcePatch`
//! in the crate (`ecam_patches.rs`, `wxr/mod.rs`, `oans/plugin.rs`,
//! `deep/ecam/patches.rs`) was defined behind `feature = "js"` purely
//! because this struct was declared inside the `js`-feature module, so
//! XPHFBW could not even name the type, let alone apply the patches -- and
//! XPHFBW, not the QuickJS `Cockpit`, is what the user actually sees almost
//! all the time (`docs/deep/debug_screen_clicks.md`).

/// A change to a script as it loads: in the file at `path` (under
/// html_ui), the text `find`, which must occur exactly once, becomes
/// `replace`. A patch that does not match is reported and the file runs
/// unchanged.
#[derive(Clone, Debug)]
pub struct SourcePatch {
    pub path: String,
    pub find: String,
    pub replace: String,
    /// Why, for the log.
    pub reason: String,
}

/// Applies every patch in `patches` whose `path` case-insensitively equals
/// `clean` (the VFS-relative path both callers already resolve their own
/// way -- `js/msfs/mod.rs`'s `clean`, `app/src/scheme.rs`'s `path`) to
/// `text`, in the order given. A patch applies only when its `find` occurs
/// in `text` exactly once; `log(true, ...)` reports a successful patch,
/// `log(false, ...)` an anchor that matched zero or more-than-one times
/// (the file is left unchanged for that one patch either way) -- callers
/// turn that bool into whatever their own log takes (`js/msfs/mod.rs`:
/// `LogLevel::Info`/`LogLevel::Error`; `app/src/scheme.rs`:
/// `crate::logging::log`, which has no level).
pub fn apply(patches: &[SourcePatch], clean: &str, mut text: String, mut log: impl FnMut(bool, &str)) -> String {
    for patch in patches.iter().filter(|p| p.path.eq_ignore_ascii_case(clean)) {
        let found = text.matches(patch.find.as_str()).count();
        if found == 1 {
            text = text.replacen(patch.find.as_str(), &patch.replace, 1);
            log(true, &format!("{}: {}", patch.path, patch.reason));
        } else {
            log(
                false,
                &format!("{}: the patch ({}) matches {found} times, not once; the file runs unchanged", patch.path, patch.reason),
            );
        }
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;

    fn patch(find: &str, replace: &str) -> SourcePatch {
        SourcePatch { path: "/a.js".to_string(), find: find.to_string(), replace: replace.to_string(), reason: "test".to_string() }
    }

    #[test]
    fn a_patch_matching_once_is_applied_and_reported_true() {
        let mut ok = None;
        let out = apply(&[patch("x", "y")], "/a.js", "ax".to_string(), |o, _| ok = Some(o));
        assert_eq!(out, "ay");
        assert_eq!(ok, Some(true));
    }

    #[test]
    fn a_patch_matching_zero_or_many_times_is_left_unchanged_and_reported_false() {
        let mut ok = None;
        let out = apply(&[patch("x", "y")], "/a.js", "no match here".to_string(), |o, _| ok = Some(o));
        assert_eq!(out, "no match here");
        assert_eq!(ok, Some(false));

        let mut ok2 = None;
        let out2 = apply(&[patch("x", "y")], "/a.js", "x and x again".to_string(), |o, _| ok2 = Some(o));
        assert_eq!(out2, "x and x again");
        assert_eq!(ok2, Some(false));
    }

    #[test]
    fn a_patch_for_a_different_path_does_not_apply() {
        let out = apply(&[patch("x", "y")], "/b.js", "ax".to_string(), |_, _| panic!("must not log"));
        assert_eq!(out, "ax");
    }
}
