//! The A380X's screens, from FlyByWire's panel.cfg
//! (SimObjects/AirPlanes/FlyByWire_A380_842/panel/panel.cfg), each shown on
//! the X-Plane cockpit device of the same name.
//!
//! A screen is what the instruments draw on, named by its panel.cfg texture
//! without the `$`, in panel.cfg's case. The converter marks each screen
//! mesh with `ATTR_cockpit_device <id>`, so the device id is the screen id.
//! SCREEN_DU_MFD is one texture on two meshes: FlyByWire draws both MFDs
//! into it side by side with a 110 px gap (panel.cfg, the comment under
//! `htmlgauge00=A380X/MFD/mfd.html`), and each side dims with its own knob,
//! so a screen has one dimming region per mesh.
//!
//! Left out: the OITs (not used), and SCREEN_ISIS_2, whose gauge FlyByWire
//! has commented out. The EFB (`SCREEN_EFB`) is included, but XPHFBW does not
//! draw FlyByWire's own (unrendered) EFB gauge on it; the app's own study/
//! settings UI is shown there instead (xphfbw_bridge_views.rs's
//! `EFB_SCREEN`, app/src/views.rs's `spawn_efb_view`), since this port does
//! not render FlyByWire's real EFB.

/// Part of a screen dimmed as one, as FlyByWire's model dims one emissive
/// mesh: `ASOBO_GT_Component_Emissive_Gauge` multiplies the mesh's emissive
/// by its `POTENTIOMETER` and by its `EMISSIVE_CODE`, which here is whether
/// any of the listed buses is powered.
#[derive(Debug)]
pub struct Dimming {
    /// x, y, width, height in CSS pixels.
    pub region: [u32; 4],
    /// The `LIGHT POTENTIOMETER` index.
    pub potentiometer: u32,
    /// Variables (without `L:`), any of which powers the region.
    pub buses: &'static [&'static str],
}

#[derive(Debug)]
pub struct ScreenDef {
    /// The panel.cfg texture name without `$`: also the X-Plane device id.
    pub id: &'static str,
    /// What X-Plane's UI calls the device.
    pub title: &'static str,
    /// CSS pixels, as panel.cfg sizes the gauge; also the device's texels.
    pub width: u32,
    pub height: u32,
    /// Empty for screens whose emissive is constant and which dim in their
    /// own drawing (A380_COCKPIT.xml, `Standby_Indicator`: "we control the
    /// individual emissives with css opacity").
    pub dimming: &'static [Dimming],
}

const ESS: &str = "A32NX_ELEC_DC_ESS_BUS_IS_POWERED";
const DC1: &str = "A32NX_ELEC_DC_1_BUS_IS_POWERED";
const DC2: &str = "A32NX_ELEC_DC_2_BUS_IS_POWERED";

const fn full(w: u32, h: u32, potentiometer: u32, buses: &'static [&'static str]) -> Dimming {
    Dimming { region: [0, 0, w, h], potentiometer, buses }
}

// Potentiometers and buses: A380_COCKPIT.xml, component `Screens` (the CDS
// units and the FCU), and model/behaviour/rmp.xml (RMP_SCREEN_POT,
// RMP_MAIN_ELEC).
pub static SCREENS: &[ScreenDef] = &[
    ScreenDef { id: "SCREEN_DU_PFDL", title: "A380X PFD (captain)", width: 768, height: 1024, dimming: &[full(768, 1024, 88, &[ESS])] },
    ScreenDef { id: "SCREEN_DU_NDL", title: "A380X ND (captain)", width: 768, height: 1024, dimming: &[full(768, 1024, 89, &[ESS, DC1])] },
    ScreenDef { id: "SCREEN_DU_PFDR", title: "A380X PFD (first officer)", width: 768, height: 1024, dimming: &[full(768, 1024, 90, &[DC2])] },
    ScreenDef { id: "SCREEN_DU_NDR", title: "A380X ND (first officer)", width: 768, height: 1024, dimming: &[full(768, 1024, 91, &[DC1, DC2])] },
    ScreenDef { id: "SCREEN_DU_EWD", title: "A380X EWD", width: 768, height: 1024, dimming: &[full(768, 1024, 92, &[ESS])] },
    ScreenDef { id: "SCREEN_DU_SD", title: "A380X SD", width: 768, height: 1024, dimming: &[full(768, 1024, 93, &[DC2])] },
    ScreenDef {
        id: "SCREEN_DU_MFD",
        title: "A380X MFDs",
        width: 1646,
        height: 1024,
        dimming: &[
            Dimming { region: [0, 0, 768, 1024], potentiometer: 98, buses: &[ESS, DC1] },
            Dimming { region: [878, 0, 768, 1024], potentiometer: 99, buses: &[DC1, DC2] },
        ],
    },
    ScreenDef { id: "FCU", title: "A380X FCU", width: 2560, height: 1280, dimming: &[full(2560, 1280, 87, &[ESS, DC2])] },
    ScreenDef { id: "SCREEN_ISIS_1", title: "A380X ISIS", width: 512, height: 512, dimming: &[] },
    ScreenDef { id: "Clock", title: "A380X clock", width: 256, height: 256, dimming: &[] },
    ScreenDef { id: "RTPI", title: "A380X rudder trim", width: 338, height: 128, dimming: &[] },
    ScreenDef { id: "BAT", title: "A380X battery voltage", width: 256, height: 128, dimming: &[] },
    ScreenDef { id: "SCREEN_DU_RMP_1", title: "A380X RMP 1", width: 1664, height: 1024, dimming: &[full(1664, 1024, 80, &[ESS])] },
    ScreenDef { id: "SCREEN_DU_RMP_2", title: "A380X RMP 2", width: 1664, height: 1024, dimming: &[full(1664, 1024, 81, &[ESS])] },
    ScreenDef { id: "SCREEN_DU_RMP_3", title: "A380X RMP 3", width: 1664, height: 1024, dimming: &[full(1664, 1024, 82, &[DC1])] },
    // The EFB tablet: no dimming region, like the other screens that dim in
    // their own drawing (SCREEN_ISIS_1, Clock, RTPI, BAT). Unlike those, this
    // is deliberate rather than incidental: a real EFB is a portable unit
    // with its own battery (it works removed from its cradle), so it is not
    // wired to any aircraft bus or CDS potentiometer here; the app renders
    // its own brightness setting into the page instead (app/ui/index.html's
    // "EFB brightness" slider, xphfbw.efbBrightness).
    ScreenDef { id: "SCREEN_EFB", title: "A380X EFB (XPHFBW)", width: 1430, height: 1000, dimming: &[] },
];

/// A screen by the id a stream names it with: case and a leading `$` do
/// not matter, since panel.cfg itself writes `$Clock` where the model says
/// `$CLOCK`.
pub fn find(id: &str) -> Option<usize> {
    let key = super::text::screen_key(id);
    SCREENS.iter().position(|s| s.id.eq_ignore_ascii_case(&key))
}

/// A region's brightness from its potentiometer, as the instruments read it
/// (`percent over 100`, CdsDisplayUnit.tsx), and whether a bus powers it.
/// A potentiometer nothing has made yet reads 0, as it does for the
/// instruments, which then go to standby.
pub fn brightness(potentiometer: Option<f64>, powered: bool) -> f32 {
    match potentiometer {
        Some(v) if v.is_finite() && powered => v.clamp(0., 1.) as f32,
        _ => 0.,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn screens_are_in_the_order_xphfbw_numbers_them() {
        let ids: Vec<&str> = SCREENS.iter().map(|s| s.id).collect();
        assert_eq!(ids, crate::xphfbw_bridge_views::SCREEN_ORDER);
    }

    #[test]
    fn screens_are_the_converters_device_ids() {
        assert_eq!(find("$SCREEN_DU_PFDL"), Some(0));
        assert_eq!(find("$CLOCK").map(|i| SCREENS[i].id), Some("Clock"));
        assert_eq!(find("SCREEN_EFB").map(|i| SCREENS[i].id), Some("SCREEN_EFB"));
        // Exactly the ids the converter writes (docs/team.md), now that the
        // EFB is no longer in the converter's default `--skip-screens`.
        let mut ids: Vec<&str> = SCREENS.iter().map(|s| s.id).collect();
        ids.sort();
        let want = [
            "BAT", "Clock", "FCU", "RTPI", "SCREEN_DU_EWD", "SCREEN_DU_MFD", "SCREEN_DU_NDL", "SCREEN_DU_NDR", "SCREEN_DU_PFDL",
            "SCREEN_DU_PFDR", "SCREEN_DU_RMP_1", "SCREEN_DU_RMP_2", "SCREEN_DU_RMP_3", "SCREEN_DU_SD", "SCREEN_EFB", "SCREEN_ISIS_1",
        ];
        assert_eq!(ids, want);
        for s in SCREENS {
            for d in s.dimming {
                let [x, y, w, h] = d.region;
                assert!(x + w <= s.width && y + h <= s.height, "{}", s.id);
            }
        }
        assert_eq!(brightness(Some(0.5), true), 0.5);
        assert_eq!(brightness(Some(0.5), false), 0.);
        assert_eq!(brightness(None, true), 0.);
    }
}
