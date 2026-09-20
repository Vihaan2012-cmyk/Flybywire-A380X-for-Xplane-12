//! View numbering shared by the plugin and the XPHFBW app
//! (docs/briefs/xphfbw-js-bridge.md, "View numbering"): the index of a
//! panel.cfg `[VCockpitNN]` section among the sections that have an active
//! `htmlgauge00` and are not the EFB/OIT/popup views `src/js/msfs/mod.rs`
//! `Cockpit::new` leaves out. `[VCockpitNN]` sections in the plugin's own
//! QuickJS `Cockpit` and the sessions this module's `view_list` describes are
//! two independent engines (only one draws a given screen at a time,
//! xphfbw-js-bridge.md rule 7), so their internal indices need not match;
//! what must match is that the plugin and the app agree with each other,
//! which is why both call this one function.
//!
//! Free of any dependency gated behind the `js` feature (`mod js`, which
//! `panel.cfg` parsing otherwise lives in, needs `rquickjs`/`oxc`): the
//! XPHFBW app links this crate without that feature (app/Cargo.toml), so
//! this module (and the rest of `xphfbw_bridge.rs`) must build without it.

/// Gauge URLs left out of the panel.cfg-driven views (js::msfs::
/// EXCLUDED_GAUGES, which a `--features js` test below cross-checks this
/// against): the OITs' *legacy* page, the popup, and WASM gauges, which are
/// native modules rather than pages a browser can load, and the EFB, whose
/// gauge XPHFBW does not draw at all (this port never rendered it — even
/// were it excluded here). [`EFB_SCREEN`] gets its own hand-made view
/// instead of one from this list (app/src/views.rs's `spawn_efb_view`): the
/// app's own study/settings UI, not FlyByWire's.
///
/// `A380X/OIT/` is *not* excluded (docs/oit.md): panel.cfg's two OIT
/// sections list `A380X/OIT/oit.html` first and `A380X/OITlegacy/` second,
/// so each OIT view is the current OIS page and the legacy one stays out,
/// exactly as the MFD view takes `A380X/MFD/mfd.html` over the terronnd
/// WASM gauge that precedes it.
pub const EXCLUDED_GAUGES: &[&str] = &["A380X/EFB/", "A380X/OITlegacy/", "A380X/popup/", "WasmInstrument/"];

/// The EFB screen's device id (the converted cockpit mesh's material,
/// `SCREEN_EFB`; docs/screens.md, display/screens.rs's `SCREENS`), and the
/// CSS pixel size XPHFBW's off-screen browser for it is created at
/// (panel.cfg's own `$SCREEN_EFB` gauge is 1430x1000; matched here so the
/// converter's UV mapping for that mesh still lines up with what is drawn on
/// it, even though the browser navigates to the app's own page rather than
/// FlyByWire's `A380X/EFB/efb.html`).
pub const EFB_SCREEN: &str = "SCREEN_EFB";
pub const EFB_WIDTH: u32 = 1430;
pub const EFB_HEIGHT: u32 = 1000;

/// One `[VCockpitNN]` section with at least one gauge left after
/// `EXCLUDED_GAUGES` is filtered out: one XPHFBW off-screen browser view
/// (xphfbw-app.md's "one off-screen CEF browser per instrument view").
#[derive(Clone, Debug, PartialEq)]
pub struct ViewDef {
    /// The index the bridge protocol uses for this view (xphfbw_bridge.rs's
    /// `Uplink::Call.view`/`Loaded.view`, `Session.downlinks[view]`).
    pub index: u32,
    /// The panel.cfg section name (e.g. `"VCockpit01"`).
    pub section: String,
    /// The first surviving `htmlgaugeNN`'s URL, relative to
    /// `/Pages/VCockpit/Instruments/` (what the browser navigates to).
    pub gauge_url: String,
    /// The page's pixel size (panel.cfg's `pixel_size`), which is also the
    /// screen's own size when the view has one (display/screens.rs's
    /// `ScreenDef::width/height`).
    pub width: u32,
    pub height: u32,
    /// The panel.cfg `texture` without its leading `$` (display/screens.rs's
    /// `ScreenDef::id`); empty for a screenless view (`NO_TEXTURE`, or no
    /// `texture` line at all — SystemsHost, ExtrasHost).
    pub screen: String,
}

struct Section {
    name: String,
    texture: Option<String>,
    display: (u32, u32),
    gauge_urls: Vec<String>,
}

/// panel.cfg's `[VCockpitNN]` sections, each with the `htmlgaugeNN` urls it
/// lists, in order. A line commented with a leading `;` (or blank) is
/// skipped, as is anything after a `//` comment; a section that is not a
/// `VCockpit` one (or the file ending) closes the section before it, mirrors
/// `js::msfs::parse_panel_cfg`.
fn parse_sections(text: &str) -> Vec<Section> {
    let mut sections: Vec<Section> = Vec::new();
    for raw in text.lines() {
        let line = raw.split("//").next().unwrap_or("").trim();
        if line.starts_with(';') || line.is_empty() {
            continue;
        }
        if let Some(name) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
            if name.to_ascii_lowercase().starts_with("vcockpit") {
                sections.push(Section { name: name.to_string(), texture: None, display: (0, 0), gauge_urls: Vec::new() });
            } else {
                // A section that is not a view ends the one before it.
                sections.push(Section { name: String::new(), texture: None, display: (0, 0), gauge_urls: Vec::new() });
            }
            continue;
        }
        let Some(section) = sections.last_mut() else { continue };
        if section.name.is_empty() {
            continue;
        }
        let Some((key, value)) = line.split_once('=') else { continue };
        let key = key.trim().to_ascii_lowercase();
        let value = value.split(';').next().unwrap_or("").trim();
        match key.as_str() {
            "pixel_size" => {
                let mut it = value.split(',').map(|n| n.trim().parse::<f64>().unwrap_or(0.) as u32);
                section.display = (it.next().unwrap_or(0), it.next().unwrap_or(0));
            }
            "texture" => {
                let t = value.trim_start_matches('$');
                section.texture = (!t.eq_ignore_ascii_case("NO_TEXTURE") && !t.is_empty()).then(|| t.to_string());
            }
            k if k.starts_with("htmlgauge") => {
                if let Some(url) = value.split(',').next().map(str::trim).filter(|u| !u.is_empty()) {
                    section.gauge_urls.push(url.to_string());
                }
            }
            _ => {}
        }
    }
    sections.retain(|s| !s.name.is_empty());
    sections
}

/// The views panel.cfg describes, numbered as xphfbw-js-bridge.md's "View
/// numbering" says: in section order, only sections with a surviving gauge,
/// index 0 for the first one.
pub fn view_list(panel_cfg: &str) -> Vec<ViewDef> {
    let mut out = Vec::new();
    for section in parse_sections(panel_cfg) {
        let Some(gauge_url) = section
            .gauge_urls
            .iter()
            .find(|url| !EXCLUDED_GAUGES.iter().any(|e| url.to_ascii_lowercase().starts_with(&e.to_ascii_lowercase())))
            .cloned()
        else {
            continue;
        };
        out.push(ViewDef {
            index: out.len() as u32,
            section: section.name,
            gauge_url,
            width: section.display.0,
            height: section.display.1,
            screen: section.texture.unwrap_or_default(),
        });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const PANEL_CFG: &str = r#"
[VCockpit01]
size_mm=768,1024
pixel_size=768,1024
texture=$SCREEN_DU_PFDL
htmlgauge00=A380X/PFD/pfd.html, 0,0,768,1024

[VCockpit02]
;size_mm=768,1024
pixel_size=768,1024
texture=NO_TEXTURE
htmlgauge00=A380X/SystemsHost/index.html, 0,0,1,1

[VCockpit03]
pixel_size=1024,1024
texture=$SCREEN_EFB
htmlgauge00=A380X/EFB/efb.html, 0,0,1024,1024

; a commented-out view, entirely skipped
[VCockpit04]
pixel_size=512,512
;htmlgauge00=A380X/OIT/oit.html, 0,0,512,512

[VCockpit05]
pixel_size=1646,1024
texture=$SCREEN_DU_MFD
htmlgauge00=WasmInstrument/wasminstrument.html?wasm_gauge=terronnd, 622,0,768,1024
htmlgauge01=A380X/MFD/mfd.html, 0,0,1646,1024

[window_titles]
Window00=Main Panel
"#;

    #[test]
    fn views_are_numbered_only_over_sections_with_a_surviving_gauge() {
        let views = view_list(PANEL_CFG);
        // VCockpit03 (EFB) and VCockpit04 (no active htmlgauge00) are left
        // out entirely, so VCockpit05 is index 2, not 4.
        assert_eq!(views.len(), 3);
        assert_eq!(views[0], ViewDef { index: 0, section: "VCockpit01".into(), gauge_url: "A380X/PFD/pfd.html".into(), width: 768, height: 1024, screen: "SCREEN_DU_PFDL".into() });
        assert_eq!(views[1], ViewDef { index: 1, section: "VCockpit02".into(), gauge_url: "A380X/SystemsHost/index.html".into(), width: 768, height: 1024, screen: String::new() });
        // The first surviving (non-native) gauge is the MFD's html one, not
        // the excluded WasmInstrument terronnd gauge that precedes it.
        assert_eq!(views[2], ViewDef { index: 2, section: "VCockpit05".into(), gauge_url: "A380X/MFD/mfd.html".into(), width: 1646, height: 1024, screen: "SCREEN_DU_MFD".into() });
    }

    /// panel.cfg's two OIT sections, verbatim (VCockpit19/20): each lists
    /// the current OIS page first and the superseded legacy one second, so
    /// the view is the current page and the legacy one never runs.
    #[test]
    fn each_oit_view_is_the_current_page_not_the_legacy_one() {
        const OIT_CFG: &str = r#"
[VCockpit19]
size_mm=1333,1000
pixel_size=1333,1000
texture=$SCREEN_OIT_LEFT
htmlgauge00=A380X/OIT/oit.html?Index=1, 0,0,1333,1000
htmlgauge01=A380X/OITlegacy/oitlegacy.html, 0,0,1333,1000

[VCockpit20]
size_mm=1333,1000
pixel_size=1333,1000
texture=$SCREEN_OIT_RIGHT
htmlgauge00=A380X/OIT/oit.html?Index=2, 0,0,1333,1000
htmlgauge01=A380X/OITlegacy/oitlegacy.html, 0,0,1333,1000
"#;
        let views = view_list(OIT_CFG);
        assert_eq!(views.len(), 2);
        assert_eq!(views[0].gauge_url, "A380X/OIT/oit.html?Index=1");
        assert_eq!(views[0].screen, OIT_LEFT_SCREEN);
        assert_eq!((views[0].width, views[0].height), (1333, 1000));
        assert_eq!(views[1].gauge_url, "A380X/OIT/oit.html?Index=2");
        assert_eq!(views[1].screen, OIT_RIGHT_SCREEN);
        // Both are screens the plugin makes a device for, in SCREEN_ORDER.
        for v in &views {
            assert!(SCREEN_ORDER.contains(&v.screen.as_str()), "{} is not in SCREEN_ORDER", v.screen);
        }
    }

    #[test]
    fn an_empty_panel_cfg_has_no_views() {
        assert_eq!(view_list(""), Vec::new());
        assert_eq!(view_list("[VCockpit01]\npixel_size=1,1\n"), Vec::new());
    }

    /// Cross-checked against `js::msfs`'s own parsing and exclusion, so the
    /// two independent implementations agree on which sections count and in
    /// what order, without this module depending on the `js` feature.
    #[cfg(feature = "js")]
    #[test]
    fn agrees_with_js_msfs_cockpit_new_s_own_skipping() {
        use crate::js::msfs::{parse_panel_cfg, EXCLUDED_GAUGES as JS_EXCLUDED};
        assert_eq!(EXCLUDED_GAUGES, JS_EXCLUDED);
        let panels = parse_panel_cfg(PANEL_CFG);
        let expected: Vec<String> = panels
            .into_iter()
            .filter(|p| p.gauges.iter().any(|g| !JS_EXCLUDED.iter().any(|e| g.url.to_ascii_lowercase().starts_with(&e.to_ascii_lowercase()))))
            .map(|p| p.name)
            .collect();
        let got: Vec<String> = view_list(PANEL_CFG).into_iter().map(|v| v.section).collect();
        assert_eq!(got, expected);
    }
}

/// The cockpit screens in the order both sides number them: the plugin's
/// `display::screens::SCREENS` (checked by its tests) and the `screen` index
/// of `Input` records XPHFBW routes to a view's browser.
pub const SCREEN_ORDER: &[&str] = &[
    "SCREEN_DU_PFDL",
    "SCREEN_DU_NDL",
    "SCREEN_DU_PFDR",
    "SCREEN_DU_NDR",
    "SCREEN_DU_EWD",
    "SCREEN_DU_SD",
    "SCREEN_DU_MFD",
    "FCU",
    "SCREEN_ISIS_1",
    "Clock",
    "RTPI",
    "BAT",
    "SCREEN_DU_RMP_1",
    "SCREEN_DU_RMP_2",
    "SCREEN_DU_RMP_3",
    EFB_SCREEN,
    OIT_LEFT_SCREEN,
    OIT_RIGHT_SCREEN,
];

/// The two OIT screens' device ids (panel.cfg's `$SCREEN_OIT_LEFT` and
/// `$SCREEN_OIT_RIGHT`, VCockpit19/20), appended to [`SCREEN_ORDER`] so no
/// screen already in it changes index.
pub const OIT_LEFT_SCREEN: &str = "SCREEN_OIT_LEFT";
pub const OIT_RIGHT_SCREEN: &str = "SCREEN_OIT_RIGHT";
