//! Glass cockpit screens: the meshes MSFS's HTML gauges draw on.
//!
//! panel.cfg names each gauge's texture (`texture=$SCREEN_DU_PFDL`), and the
//! cockpit model's screen meshes use a material of that name. In X-Plane
//! the systems plugin draws those screens as avionics devices, which a mesh
//! shows with `ATTR_cockpit_device <id> <bus> <lighting channel> <auto
//! adjust>`, its UVs mapping the device's screen. The device id is the
//! texture name without its `$`, as the plugin registers it.

use std::collections::BTreeMap;

/// A screen panel.cfg gives a gauge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Screen {
    /// The device id: panel.cfg's texture name without `$`.
    pub id: String,
    pub width: u32,
    pub height: u32,
    /// The gauges drawn on it, as panel.cfg lists them (`A380X/PFD/pfd.html?Index=1`).
    pub gauges: Vec<String>,
}

/// `ATTR_cockpit_device` arguments after the id: bus bitfield, lighting
/// channel, auto adjust. The plugin decides whether a screen is powered and
/// how bright it is (FlyByWire's display units dim by their CDS
/// potentiometers and blank unpowered), so the device takes no X-Plane bus
/// and no daylight boost; its brightness callback ignores the channel.
pub const DEVICE_ARGS: &str = "0 0 0";

/// Screens the real A380 crew actually touches: the EFB tablets, the two
/// OITs and the MFD keyboard/trackball unit. Everything else here (PFD, ND,
/// EWD, SD, ISIS, clock, RMPs, FCU) is read by the systems plugin and
/// operated through physical buttons/knobs elsewhere in the cockpit, never
/// tapped on the glass itself, so it gets no `ATTR_manip_device`: one fewer
/// manipulator per screen is one fewer thing that can go wrong, and the OBJ8
/// spec's own requirement ("the manipulator must be the same shape, and
/// UV-mapped the same way as the screen of the cockpit device") only needs
/// meeting where a tap is real input.
pub const TOUCH_SCREENS: &[&str] = &["SCREEN_EFB", "SCREEN_OIT_LEFT", "SCREEN_OIT_RIGHT", "SCREEN_DU_MFD"];

/// The screens in a panel.cfg, by upper-case device id, leaving out those whose every
/// gauge is in a skipped instrument folder (`EFB` skips `A380X/EFB/...`).
pub fn parse_panel(text: &str, skip: &[String]) -> BTreeMap<String, Screen> {
    let mut screens = BTreeMap::new();
    let mut current: Option<Screen> = None;
    let finish = |s: Option<Screen>, screens: &mut BTreeMap<String, Screen>| {
        if let Some(s) = s {
            let skipped = |g: &String| {
                let path = g.split(['?', ',']).next().unwrap_or("");
                path.split('/').nth(1).is_some_and(|folder| skip.iter().any(|k| k.eq_ignore_ascii_case(folder)))
            };
            if !s.id.is_empty() && !s.gauges.is_empty() && !s.gauges.iter().all(skipped) {
                screens.insert(s.id.to_ascii_uppercase(), s);
            }
        }
    };
    for raw in text.lines() {
        let line = raw.split(';').next().unwrap_or("").trim();
        if line.starts_with('[') {
            finish(current.take(), &mut screens);
            if line[1..].to_ascii_uppercase().starts_with("VCOCKPIT") {
                current = Some(Screen { id: String::new(), width: 0, height: 0, gauges: Vec::new() });
            }
            continue;
        }
        let (Some(s), Some((key, value))) = (current.as_mut(), line.split_once('=')) else { continue };
        let (key, value) = (key.trim().to_ascii_lowercase(), value.trim());
        if key == "texture" {
            if !value.eq_ignore_ascii_case("NO_TEXTURE") {
                s.id = value.trim_start_matches('$').to_string();
            }
        } else if key == "pixel_size" {
            let mut it = value.split(',').map(|n| n.trim().parse().unwrap_or(0));
            s.width = it.next().unwrap_or(0);
            s.height = it.next().unwrap_or(0);
        } else if key.starts_with("htmlgauge") {
            let url = value.split(',').next().unwrap_or("").trim();
            if !url.is_empty() {
                s.gauges.push(url.to_string());
            }
        }
    }
    finish(current.take(), &mut screens);
    screens
}

/// The screen a material draws, by its name (`$SCREEN_DU_PFDL` or `SCREEN_ISIS_1`).
/// MSFS matches texture names without case: panel.cfg's `$Clock` is the
/// model's `$CLOCK`.
pub fn screen_of<'a>(screens: &'a BTreeMap<String, Screen>, material: &str) -> Option<&'a Screen> {
    screens.get(&material.trim_start_matches('$').to_ascii_uppercase())
}

#[cfg(test)]
mod tests {
    use super::*;

    const PANEL: &str = "\
[VCockpit01]
size_mm=1646,1024
pixel_size=1646,1024
texture=$SCREEN_DU_MFD
htmlgauge00=A380X/MFD/mfd.html?duID=2, 0,0,1646,1024

[VCockpit02]
pixel_size=768,1024
texture=NO_TEXTURE

[VCockpit07]
pixel_size=768,1024
texture=$SCREEN_DU_NDL
htmlgauge00=WasmInstrument/WasmInstrument.html?wasm_module=terronnd.wasm&wasm_gauge=terronnd,0,0,768,1024,L
htmlgauge01=A380X/ND/nd.html?Index=1&duID=1, 0,0,768,1024

[VCockpit11]
pixel_size=512,512
texture=SCREEN_ISIS_2
;htmlgauge00=A380X/ISISlegacy/isislegacy.html, 0,0,512,512

[VCockpit15]
pixel_size=1430,1000
texture=$SCREEN_EFB
htmlgauge00=A380X/EFB/efb.html, 0,0,1430,1000

[VPainting01]
texture=$RegistrationNumber
";

    #[test]
    fn screens_come_from_the_gauges_panel_cfg_draws() {
        let s = parse_panel(PANEL, &["EFB".to_string()]);
        assert_eq!(s.keys().collect::<Vec<_>>(), ["SCREEN_DU_MFD", "SCREEN_DU_NDL"]);
        assert_eq!((s["SCREEN_DU_MFD"].width, s["SCREEN_DU_MFD"].height), (1646, 1024));
        assert_eq!(s["SCREEN_DU_NDL"].gauges.len(), 2);
        assert!(screen_of(&s, "$SCREEN_DU_NDL").is_some());
        assert_eq!(screen_of(&s, "$screen_du_mfd").map(|x| x.id.as_str()), Some("SCREEN_DU_MFD"));
        assert!(screen_of(&s, "SCREEN_ISIS_2").is_none(), "its only gauge is commented out");
    }
}
