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
        let (spans, exact) = locate(&text, &patch.find);
        if spans.len() == 1 {
            let replace = if exact { patch.replace.clone() } else { kit_names(&patch.replace) };
            text.replace_range(spans[0].clone(), &replace);
            log(true, &format!("{}: {}", patch.path, patch.reason));
        } else {
            log(
                false,
                &format!("{}: the patch ({}) matches {} times, not once; the file runs unchanged", patch.path, patch.reason, spans.len()),
            );
        }
    }
    text
}

pub fn count(text: &str, find: &str) -> usize {
    locate(text, find).0.len()
}

pub fn locate(text: &str, find: &str) -> (Vec<std::ops::Range<usize>>, bool) {
    let exact: Vec<_> = text.match_indices(find).map(|(i, m)| i..i + m.len()).collect();
    if !exact.is_empty() {
        return (exact, true);
    }
    let pattern = compile(&short_numbers(&kit_names(&drop_arrow_parens(find))));
    (find_all(text, &pattern), false)
}

#[derive(Debug, PartialEq)]
enum Part {
    Lit(String),
    Ws,
    OptSemi,
}

fn push_lit(parts: &mut Vec<Part>, s: &str) {
    if s.is_empty() {
        return;
    }
    if let Some(Part::Lit(l)) = parts.last_mut() {
        l.push_str(s);
    } else {
        parts.push(Part::Lit(s.to_string()));
    }
}

fn push_ws(parts: &mut Vec<Part>) {
    if !matches!(parts.last(), Some(Part::Ws) | None) {
        parts.push(Part::Ws);
    }
}

fn skip_gap(c: &[char], mut i: usize) -> usize {
    loop {
        while i < c.len() && c[i].is_whitespace() {
            i += 1;
        }
        if i + 1 < c.len() && c[i] == '/' && c[i + 1] == '/' {
            while i < c.len() && c[i] != '\n' {
                i += 1;
            }
        } else if i + 1 < c.len() && c[i] == '/' && c[i + 1] == '*' {
            i += 2;
            while i + 1 < c.len() && !(c[i] == '*' && c[i + 1] == '/') {
                i += 1;
            }
            i = (i + 2).min(c.len());
        } else {
            return i;
        }
    }
}

fn compile(find: &str) -> Vec<Part> {
    let c: Vec<char> = find.chars().collect();
    let mut parts = Vec::new();
    code(&c, 0, &mut parts, false);
    while matches!(parts.last(), Some(Part::Ws)) {
        parts.pop();
    }
    if let Some(Part::Lit(l)) = parts.last_mut() {
        if l.ends_with(';') {
            l.pop();
            if l.is_empty() {
                parts.pop();
            }
            parts.push(Part::OptSemi);
        }
    }
    parts
}

fn code(c: &[char], mut i: usize, parts: &mut Vec<Part>, in_template: bool) -> usize {
    let mut depth = 0usize;
    while i < c.len() {
        let ch = c[i];
        let gap = skip_gap(c, i);
        if gap > i {
            push_ws(parts);
            i = gap;
            continue;
        }
        match ch {
            '\'' | '"' => {
                let start = i;
                i += 1;
                while i < c.len() && c[i] != ch {
                    if c[i] == '\\' {
                        i += 1;
                    }
                    i += 1;
                }
                i = (i + 1).min(c.len());
                push_lit(parts, &c[start..i].iter().collect::<String>());
            }
            '`' => {
                push_lit(parts, "`");
                i += 1;
                while i < c.len() && c[i] != '`' {
                    if c[i] == '\\' && i + 1 < c.len() {
                        push_lit(parts, &c[i..i + 2].iter().collect::<String>());
                        i += 2;
                    } else if c[i] == '$' && i + 1 < c.len() && c[i + 1] == '{' {
                        push_lit(parts, "${");
                        i = code(c, i + 2, parts, true);
                        push_lit(parts, "}");
                    } else {
                        push_lit(parts, &c[i].to_string());
                        i += 1;
                    }
                }
                push_lit(parts, "`");
                i = (i + 1).min(c.len());
            }
            ';' if c.get(skip_gap(c, i + 1)) == Some(&'}') => {
                parts.push(Part::OptSemi);
                i += 1;
            }
            '{' => {
                depth += 1;
                push_lit(parts, "{");
                i += 1;
            }
            '}' if in_template && depth == 0 => return i + 1,
            '}' => {
                depth = depth.saturating_sub(1);
                push_lit(parts, "}");
                i += 1;
            }
            _ => {
                push_lit(parts, &ch.to_string());
                i += 1;
            }
        }
    }
    i
}

fn match_at(text: &[u8], mut at: usize, parts: &[Part]) -> Option<usize> {
    for part in parts {
        match part {
            Part::Lit(l) => {
                if !text[at..].starts_with(l.as_bytes()) {
                    return None;
                }
                at += l.len();
            }
            Part::Ws => {
                while at < text.len() && text[at].is_ascii_whitespace() {
                    at += 1;
                }
            }
            Part::OptSemi => {
                if text.get(at) == Some(&b';') {
                    at += 1;
                }
            }
        }
    }
    Some(at)
}

fn find_all(text: &str, parts: &[Part]) -> Vec<std::ops::Range<usize>> {
    let Some(Part::Lit(first)) = parts.first() else { return Vec::new() };
    let bytes = text.as_bytes();
    let mut out: Vec<std::ops::Range<usize>> = Vec::new();
    for (i, _) in text.match_indices(first.as_str()) {
        if out.last().is_some_and(|r| i < r.end) {
            continue;
        }
        if let Some(end) = match_at(bytes, i, parts) {
            out.push(i..end);
        }
    }
    out
}

fn is_ident(b: u8) -> bool {
    b.is_ascii_alphanumeric() || b == b'_' || b == b'$'
}

fn drop_arrow_parens(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = String::with_capacity(s.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'(' && (i == 0 || !is_ident(b[i - 1])) {
            let mut j = i + 1;
            while j < b.len() && b[j] == b' ' {
                j += 1;
            }
            let id = j;
            while j < b.len() && is_ident(b[j]) {
                j += 1;
            }
            let id_end = j;
            while j < b.len() && b[j] == b' ' {
                j += 1;
            }
            if id_end > id && !b[id].is_ascii_digit() && j < b.len() && b[j] == b')' {
                let mut k = j + 1;
                while k < b.len() && b[k] == b' ' {
                    k += 1;
                }
                if s[k..].starts_with("=>") {
                    out.push_str(&s[id..id_end]);
                    i = j + 1;
                    continue;
                }
            }
        }
        let ch = s[i..].chars().next().unwrap_or(' ');
        out.push(ch);
        i += ch.len_utf8();
    }
    out
}

fn short_numbers(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = String::with_capacity(s.len());
    let mut i = 0;
    let mut quote: Option<u8> = None;
    while i < b.len() {
        let c = b[i];
        if let Some(q) = quote {
            if c == b'\\' && i + 1 < b.len() {
                out.push_str(&s[i..i + 2]);
                i += 2;
                continue;
            }
            if c == q {
                quote = None;
            }
        } else if c == b'"' || c == b'\'' {
            quote = Some(c);
        } else if c.is_ascii_digit() && (i == 0 || !(is_ident(b[i - 1]) || b[i - 1] == b'.')) {
            let mut j = i;
            while j < b.len() && (b[j].is_ascii_digit() || b[j] == b'.') {
                j += 1;
            }
            if j < b.len() && is_ident(b[j]) {
                out.push_str(&s[i..j]);
                i = j;
                continue;
            }
            let lit = &s[i..j];
            if let Some(frac) = lit.strip_prefix("0.").filter(|f| !f.is_empty() && f.bytes().all(|d| d.is_ascii_digit())) {
                out.push('.');
                out.push_str(frac);
            } else if lit.bytes().all(|d| d.is_ascii_digit()) && lit.len() > 1 {
                let zeros = lit.len() - lit.trim_end_matches('0').len();
                let head = lit.trim_end_matches('0');
                let exp = format!("{head}e{zeros}");
                if zeros > 0 && !head.is_empty() && exp.len() < lit.len() {
                    out.push_str(&exp);
                } else {
                    out.push_str(lit);
                }
            } else {
                out.push_str(lit);
            }
            i = j;
            continue;
        }
        let ch = s[i..].chars().next().unwrap_or(' ');
        out.push(ch);
        i += ch.len_utf8();
    }
    out
}

struct KitNames {
    exports: Vec<(String, String)>,
    members: Vec<(String, String)>,
}

fn names() -> &'static KitNames {
    static NAMES: std::sync::OnceLock<KitNames> = std::sync::OnceLock::new();
    NAMES.get_or_init(|| {
        let v: serde_json::Value = serde_json::from_str(include_str!("xp_kit_names.json")).unwrap_or_default();
        let pairs = |k: &str| -> Vec<(String, String)> {
            v.get(k)
                .and_then(|m| m.as_object())
                .map(|m| m.iter().filter_map(|(a, b)| Some((a.clone(), b.as_str()?.to_string()))).collect())
                .unwrap_or_default()
        };
        KitNames { exports: pairs("exports"), members: pairs("members") }
    })
}

pub fn kit_names(s: &str) -> String {
    const FACTORIES: [(&str, &str); 4] = [("Subject", "cell"), ("MappedSubject", "derive"), ("ConsumerSubject", "feed"), ("ArraySubject", "list")];
    let n = names();
    let b = s.as_bytes();
    let mut out = String::with_capacity(s.len());
    let mut i = 0;
    while i < b.len() {
        if is_ident(b[i]) && (i == 0 || !is_ident(b[i - 1])) {
            let mut j = i;
            while j < b.len() && is_ident(b[j]) {
                j += 1;
            }
            let word = &s[i..j];
            let after_dot = i > 0 && b[i - 1] == b'.';
            if let Some((_, f)) = FACTORIES.iter().find(|(k, _)| *k == word).filter(|_| s[j..].starts_with(".create(")) {
                out.push_str(f);
                out.push('(');
                i = j + ".create(".len();
                continue;
            }
            if word == "pub" && after_dot && matches!(s[j..].trim_start().chars().next(), Some('(' | '<')) {
                out.push_str("send");
            } else if let Some((_, m)) = n.members.iter().find(|(k, _)| k != "pub" && k == word) {
                out.push_str(m);
            } else if let Some((_, e)) = n.exports.iter().find(|(k, _)| k == word).filter(|_| !after_dot) {
                out.push_str(e);
            } else {
                out.push_str(word);
            }
            i = j;
            continue;
        }
        let ch = s[i..].chars().next().unwrap_or(' ');
        out.push(ch);
        i += ch.len_utf8();
    }
    out
}

/// `amdb.ts`'s two Navigraph calls in `nd.js`, sent through `fetch` instead of axios, so the
/// plugin answers them (`oans::plugin`, and XPHFBW's runtime routes the URL to the plugin).
pub fn amdb_fetch_patches() -> Vec<SourcePatch> {
    const OLD_LINE: &str = "const response = await navigraphRequest.get(`https://amdb.api.navigraph.com/v1/${query}`);";
    const NEW_LINE: &str = "const response = { data: await (await fetch(`https://amdb.api.navigraph.com/v1/${query}`)).json() };";
    const NDJS: &str = "/Pages/VCockpit/Instruments/A380X/ND/nd.js";
    let patch = |find: &str, why: &str| SourcePatch {
        path: NDJS.to_string(),
        find: find.to_string(),
        replace: find.replacen(OLD_LINE, NEW_LINE, 1),
        reason: format!(
            "{why}: navigraphRequest needs XMLHttpRequest, which nothing here defines (axios's \
             getDefaultAdapter only tries `typeof XMLHttpRequest !== \"undefined\"`), so it is \
             sent through fetch() instead, which src/oans/plugin.rs answers locally"
        ),
    };
    vec![
        // amdb.ts:8-17 (searchAmdbAirports).
        patch(
            "    let query = \"search\";\n    query += `?q=${queryString}`;\n    navigraphAuth;\n    const response = await navigraphRequest.get(`https://amdb.api.navigraph.com/v1/${query}`);",
            "amdb.ts searchAmdbAirports",
        ),
        // amdb.ts:19-39 (getAmdbData).
        patch(
            "    query += `&include=${includeString}`;\n    navigraphAuth;\n    const response = await navigraphRequest.get(`https://amdb.api.navigraph.com/v1/${query}`);",
            "amdb.ts getAmdbData",
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn patch(find: &str, replace: &str) -> SourcePatch {
        SourcePatch { path: "/a.js".to_string(), find: find.to_string(), replace: replace.to_string(), reason: "test".to_string() }
    }

    #[test]
    fn every_static_patch_matches_the_installed_pages_once() {
        let Some(root) = std::env::var_os("FBW_HTML_UI") else { return };
        let bad: Vec<String> = crate::static_source_patches()
            .into_iter()
            .filter_map(|p| {
                let text = std::fs::read_to_string(std::path::Path::new(&root).join(p.path.trim_start_matches('/'))).unwrap_or_default();
                let n = count(&text, &p.find);
                (n != 1).then(|| format!("{} x{n}: {}", p.path, p.reason))
            })
            .collect();
        assert!(bad.is_empty(), "{}", bad.join("\n"));
    }

    #[test]
    fn writes_every_patched_page() {
        let (Some(root), Some(out)) = (std::env::var_os("FBW_HTML_UI"), std::env::var_os("FBW_PATCHED_OUT")) else { return };
        let patches = crate::static_source_patches();
        let mut paths: Vec<String> = patches.iter().map(|p| p.path.clone()).collect();
        paths.sort();
        paths.dedup();
        for path in paths {
            let rel = path.trim_start_matches('/');
            let text = std::fs::read_to_string(std::path::Path::new(&root).join(rel)).unwrap_or_default();
            let idents = |s: &str| -> std::collections::BTreeSet<String> {
                s.split(|c: char| !(c.is_ascii_alphanumeric() || c == '_' || c == '$')).filter(|w| w.chars().next().is_some_and(|c| !c.is_ascii_digit())).map(str::to_string).collect()
            };
            let known = idents(&text);
            for p in patches.iter().filter(|p| p.path == path) {
                let missing: Vec<String> = idents(&kit_names(&p.replace)).difference(&known).cloned().collect();
                if !missing.is_empty() {
                    println!("{path}: {}: new names {missing:?}", p.reason.chars().take(60).collect::<String>());
                }
            }
            let mut failed = Vec::new();
            let patched = apply(&patches, &path, text, |ok, msg| if !ok { failed.push(msg.to_string()) });
            assert!(failed.is_empty(), "{}", failed.join("\n"));
            let dest = std::path::Path::new(&out).join(rel);
            std::fs::create_dir_all(dest.parent().unwrap()).unwrap();
            std::fs::write(dest, patched).unwrap();
        }
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
    fn a_patch_written_against_readable_output_matches_the_minified_kit_build() {
        let find = "      this.wxr1Failed = Subject.create(true);\n      // radar\n      this.wxr2Failed = Subject.create(true);\n    }";
        let bundle = "x;this.wxr1Failed=cell(true);this.wxr2Failed=cell(true)}init(){}";
        assert_eq!(count(bundle, find), 1);
        let out = apply(&[patch(find, "this.wxr1Failed = Subject.create(false); this.wxr2Failed = Subject.create(false); }")], "/a.js", bundle.to_string(), |o, _| assert!(o));
        assert_eq!(out, "x;this.wxr1Failed = cell(false); this.wxr2Failed = cell(false); }init(){}");
    }

    #[test]
    fn arrow_parens_template_expressions_and_members_follow_the_kit_build() {
        let find = "bus.getSubscriber().on(`x_${ a + 1 }`).handle((v) => this.pub(\"y\", v));";
        let bundle = "bus.reader().on(`x_${a+1}`).handle(v=>this.send(\"y\",v))}";
        assert_eq!(count(bundle, find), 1);
        assert_eq!(count("bus.reader().on(`x_${a+2}`)", find), 0);
    }

    #[test]
    fn numbers_are_matched_in_their_minified_spelling() {
        assert_eq!(count("[null,null,x.get()<.35]},y=1e3", "[null, null, x.get() < 0.35]"), 1);
        assert_eq!(count("y=1e3;z=100", "y = 1000;"), 1);
        assert_eq!(count("y=1e3;z=100", "z = 100"), 1);
        assert_eq!(count("s=\"0.35\"", "s = \"0.35\""), 1);
    }

    #[test]
    fn a_patch_for_a_different_path_does_not_apply() {
        let out = apply(&[patch("x", "y")], "/b.js", "ax".to_string(), |_, _| panic!("must not log"));
        assert_eq!(out, "ax");
    }
}
