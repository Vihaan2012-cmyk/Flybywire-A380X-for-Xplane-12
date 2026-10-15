use crate::source_patch::SourcePatch;
use std::sync::Mutex;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Du {
    CaptPfd,
    CaptNd,
    CaptMfd,
    FoPfd,
    FoNd,
    FoMfd,
    Ewd,
    Sd,
}

pub const ALL: [Du; 8] = [Du::CaptPfd, Du::CaptNd, Du::CaptMfd, Du::FoPfd, Du::FoNd, Du::FoMfd, Du::Ewd, Du::Sd];

pub type Route = [Option<Du>; 8];

pub const NORMAL: Route = [Some(Du::CaptPfd), Some(Du::CaptNd), Some(Du::CaptMfd), Some(Du::FoPfd), Some(Du::FoNd), Some(Du::FoMfd), Some(Du::Ewd), Some(Du::Sd)];

impl Du {
    pub fn id(self) -> usize {
        self as usize
    }
}

const ESS: &str = "A32NX_ELEC_DC_ESS_BUS_IS_POWERED";
const DC1: &str = "A32NX_ELEC_DC_1_BUS_IS_POWERED";
const DC2: &str = "A32NX_ELEC_DC_2_BUS_IS_POWERED";

const BUSES: [&[&str]; 8] = [&[ESS], &[ESS, DC1], &[ESS, DC1], &[DC2], &[DC1, DC2], &[DC1, DC2], &[ESS], &[DC2]];

const BREAKERS: [Option<&str>; 8] = [
    Some("A32NX_BKR_capt-pfd-du_OPEN"),
    Some("A32NX_BKR_capt-nd-du_OPEN"),
    None,
    Some("A32NX_BKR_fo-pfd-du_OPEN"),
    Some("A32NX_BKR_fo-nd-du_OPEN"),
    None,
    Some("A32NX_BKR_capt-ewd-du_OPEN"),
    None,
];

pub const XFR_PB: [&str; 2] = ["A380X_EFIS_L_PFD_ND_XFR_PB", "A380X_EFIS_R_PFD_ND_XFR_PB"];
pub const RECONF_PB: [&str; 2] = ["A380X_EFIS_L_DU_RECONF_PB", "A380X_EFIS_R_DU_RECONF_PB"];

pub fn host_var(format: Du) -> String {
    format!("A380X_CDS_HOST_{}", format.id())
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Inputs {
    pub failed: [bool; 8],
    pub xfr: [bool; 2],
    pub reconf: [bool; 2],
}

#[derive(Clone, Debug, Default)]
pub struct Reconf {
    xfr: [bool; 2],
    manual: [Option<Du>; 8],
    held: [bool; 4],
}

fn side_units(side: usize) -> (Du, Du, Du) {
    if side == 0 {
        (Du::CaptPfd, Du::CaptNd, Du::CaptMfd)
    } else {
        (Du::FoPfd, Du::FoNd, Du::FoMfd)
    }
}

fn automatic(xfr: &[bool; 2], failed: &[bool; 8]) -> Route {
    let mut r: Route = [None; 8];
    for side in 0..2 {
        let (pfd, nd, mfd) = side_units(side);
        let (a, b) = if xfr[side] { (nd, pfd) } else { (pfd, nd) };
        if !failed[a.id()] {
            r[a.id()] = Some(pfd);
            r[b.id()] = (!failed[b.id()]).then_some(nd);
        } else if !failed[b.id()] {
            r[b.id()] = Some(pfd);
        }
        r[mfd.id()] = (!failed[mfd.id()]).then_some(mfd);
    }
    if !failed[Du::Ewd.id()] {
        r[Du::Ewd.id()] = Some(Du::Ewd);
        r[Du::Sd.id()] = (!failed[Du::Sd.id()]).then_some(Du::Sd);
    } else if !failed[Du::Sd.id()] {
        r[Du::Sd.id()] = Some(Du::Ewd);
    }
    r
}

impl Reconf {
    pub fn step(&mut self, i: &Inputs) -> Route {
        for side in 0..2 {
            if i.xfr[side] && !self.held[side] {
                self.xfr[side] = !self.xfr[side];
            }
        }
        let mut route = self.route(&i.failed);
        for side in 0..2 {
            if i.reconf[side] && !self.held[2 + side] {
                self.press_reconf(side, &i.failed, &route);
                route = self.route(&i.failed);
            }
        }
        self.held = [i.xfr[0], i.xfr[1], i.reconf[0], i.reconf[1]];
        route
    }

    fn route(&mut self, failed: &[bool; 8]) -> Route {
        if !failed.iter().any(|f| *f) {
            self.manual = [None; 8];
        }
        let mut r = automatic(&self.xfr, failed);
        for du in ALL {
            let Some(f) = self.manual[du.id()] else { continue };
            let elsewhere = ALL.iter().any(|&o| o != du && r[o.id()] == Some(f));
            if failed[du.id()] || elsewhere {
                self.manual[du.id()] = None;
            } else {
                r[du.id()] = Some(f);
            }
        }
        r
    }

    fn press_reconf(&mut self, side: usize, failed: &[bool; 8], route: &Route) {
        let (pfd, nd, mfd) = side_units(side);
        if ![pfd, nd, mfd, Du::Ewd, Du::Sd].iter().any(|d| failed[d.id()]) {
            return;
        }
        let ok = |d: Du| !failed[d.id()];
        let target = if side == 0 {
            [mfd, nd, pfd].into_iter().find(|&d| ok(d))
        } else if ok(pfd) && ok(nd) {
            Some(nd)
        } else {
            [mfd, pfd].into_iter().find(|&d| ok(d))
        };
        let Some(t) = target else { return };
        let capable: &[Du] = if t == mfd {
            &[Du::Ewd, Du::Sd, pfd, nd]
        } else if t == nd {
            &[mfd, Du::Ewd, Du::Sd]
        } else {
            &[nd, mfd]
        };
        let current = self.manual[t.id()];
        let lost = |f: Du| !ALL.iter().any(|&d| route[d.id()] == Some(f));
        let options: Vec<Option<Du>> =
            std::iter::once(None).chain(capable.iter().copied().filter(|&f| lost(f) || current == Some(f)).map(Some)).collect();
        let at = options.iter().position(|o| *o == current).unwrap_or(0);
        self.manual[t.id()] = options[(at + 1) % options.len()];
    }
}

pub fn hosts(route: &Route) -> [f64; 8] {
    let mut out = [0.; 8];
    for d in ALL {
        if let Some(f) = route[d.id()] {
            out[f.id()] = (d.id() + 1) as f64;
        }
    }
    out
}

static STATE: Mutex<Option<Reconf>> = Mutex::new(None);
static ROUTE: Mutex<Route> = Mutex::new(NORMAL);

pub fn update(mut read: impl FnMut(&str) -> Option<f64>) -> Route {
    let mut on = |name: &str| read(name).is_some_and(|v| v > 0.5);
    let mut i = Inputs::default();
    for d in ALL {
        let powered = BUSES[d.id()].iter().any(|b| on(b));
        let breaker = BREAKERS[d.id()].is_some_and(|b| on(b));
        i.failed[d.id()] = !powered || breaker;
    }
    for side in 0..2 {
        i.xfr[side] = on(XFR_PB[side]);
        i.reconf[side] = on(RECONF_PB[side]);
    }
    let route = STATE.lock().map(|mut s| s.get_or_insert_with(Reconf::default).step(&i)).unwrap_or(NORMAL);
    if let Ok(mut r) = ROUTE.lock() {
        *r = route;
    }
    route
}

pub fn route() -> Route {
    ROUTE.lock().map(|r| *r).unwrap_or(NORMAL)
}

const CDS_PAGES: [&str; 4] = [
    "/Pages/VCockpit/Instruments/A380X/PFD/pfd.js",
    "/Pages/VCockpit/Instruments/A380X/ND/nd.js",
    "/Pages/VCockpit/Instruments/A380X/EWD/ewd.js",
    "/Pages/VCockpit/Instruments/A380X/MFD/mfd.js",
];

const CDS_FIND: &str = "    update() {
      const potentiometer = SimVar.GetSimVarValue(
        `LIGHT POTENTIOMETER:${DisplayUnitToPotentiometer[this.props.displayUnitId]}`,
        \"percent over 100\"
      );
      const poweredByBus1 = SimVar.GetSimVarValue(
        `L:A32NX_ELEC_${DisplayUnitToDCBus[this.props.displayUnitId][0]}_BUS_IS_POWERED`,
        \"Bool\"
      );
      const poweredByBus2 = SimVar.GetSimVarValue(
        `L:A32NX_ELEC_${DisplayUnitToDCBus[this.props.displayUnitId][1]}_BUS_IS_POWERED`,
        \"Bool\"
      );";

const CDS_REPLACE: &str = "    update() {
      const cdsHost = (SimVar.GetSimVarValue(`L:A380X_CDS_HOST_${this.props.displayUnitId}`, \"number\") || this.props.displayUnitId + 1) - 1;
      const potentiometer = SimVar.GetSimVarValue(
        `LIGHT POTENTIOMETER:${DisplayUnitToPotentiometer[cdsHost]}`,
        \"percent over 100\"
      );
      const poweredByBus1 = SimVar.GetSimVarValue(
        `L:A32NX_ELEC_${DisplayUnitToDCBus[cdsHost][0]}_BUS_IS_POWERED`,
        \"Bool\"
      );
      const poweredByBus2 = SimVar.GetSimVarValue(
        `L:A32NX_ELEC_${DisplayUnitToDCBus[cdsHost][1]}_BUS_IS_POWERED`,
        \"Bool\"
      );";

const SD_PAGE: &str = "/Pages/VCockpit/Instruments/A380X/SD/sd.js";

const SD_FIND: &str = "      const [potentiometer] = useSimVar(
        `LIGHT POTENTIOMETER:${DisplayUnitToPotentiometer[displayUnitId]}`,
        \"percent over 100\",
        200
      );
      const [electricityState0] = useSimVar(
        `L:A32NX_ELEC_${DisplayUnitToDCBus[displayUnitId][0]}_BUS_IS_POWERED`,
        \"bool\",
        200
      );
      const [electricityState1] = useSimVar(
        `L:A32NX_ELEC_${DisplayUnitToDCBus[displayUnitId][1]}_BUS_IS_POWERED`,
        \"bool\",
        200
      );";

const SD_REPLACE: &str = "      const [cdsHostRaw] = useSimVar(`L:A380X_CDS_HOST_${displayUnitId}`, \"number\", 200);
      const cdsHost = (cdsHostRaw || displayUnitId + 1) - 1;
      const [potentiometer] = useSimVar(
        `LIGHT POTENTIOMETER:${DisplayUnitToPotentiometer[cdsHost]}`,
        \"percent over 100\",
        200
      );
      const [electricityState0] = useSimVar(
        `L:A32NX_ELEC_${DisplayUnitToDCBus[cdsHost][0]}_BUS_IS_POWERED`,
        \"bool\",
        200
      );
      const [electricityState1] = useSimVar(
        `L:A32NX_ELEC_${DisplayUnitToDCBus[cdsHost][1]}_BUS_IS_POWERED`,
        \"bool\",
        200
      );";

pub fn source_patches() -> Vec<SourcePatch> {
    let reason = "display unit power and brightness follow the unit showing the page (PFD/ND XFR, DU RECONF)".to_string();
    let mut out: Vec<SourcePatch> = CDS_PAGES
        .iter()
        .map(|p| SourcePatch { path: p.to_string(), find: CDS_FIND.to_string(), replace: CDS_REPLACE.to_string(), reason: reason.clone() })
        .collect();
    out.push(SourcePatch { path: SD_PAGE.to_string(), find: SD_FIND.to_string(), replace: SD_REPLACE.to_string(), reason });
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn failed(units: &[Du]) -> [bool; 8] {
        let mut f = [false; 8];
        for u in units {
            f[u.id()] = true;
        }
        f
    }

    fn press(r: &mut Reconf, i: Inputs, set: impl Fn(&mut Inputs)) -> Route {
        let mut down = i;
        set(&mut down);
        r.step(&down);
        r.step(&i)
    }

    #[test]
    fn every_unit_shows_its_own_page_normally() {
        let mut r = Reconf::default();
        assert_eq!(r.step(&Inputs::default()), NORMAL);
    }

    #[test]
    fn the_pfd_nd_pb_swaps_the_onside_pages_and_back() {
        let mut r = Reconf::default();
        let i = Inputs::default();
        let swapped = press(&mut r, i, |i| i.xfr[0] = true);
        assert_eq!(swapped[Du::CaptPfd.id()], Some(Du::CaptNd));
        assert_eq!(swapped[Du::CaptNd.id()], Some(Du::CaptPfd));
        assert_eq!(swapped[Du::FoPfd.id()], Some(Du::FoPfd));
        assert_eq!(press(&mut r, i, |i| i.xfr[0] = true), NORMAL);
    }

    #[test]
    fn holding_the_pfd_nd_pb_swaps_once() {
        let mut r = Reconf::default();
        let held = Inputs { xfr: [false, true], ..Default::default() };
        for _ in 0..5 {
            r.step(&held);
        }
        assert_eq!(r.step(&held)[Du::FoPfd.id()], Some(Du::FoNd));
    }

    #[test]
    fn a_failed_pfd_unit_moves_the_pfd_to_the_nd_unit() {
        let mut r = Reconf::default();
        let route = r.step(&Inputs { failed: failed(&[Du::CaptPfd]), ..Default::default() });
        assert_eq!(route[Du::CaptNd.id()], Some(Du::CaptPfd));
        assert_eq!(route[Du::CaptPfd.id()], None);
    }

    #[test]
    fn a_failed_ewd_unit_moves_the_ewd_to_the_sd_unit() {
        let mut r = Reconf::default();
        let route = r.step(&Inputs { failed: failed(&[Du::Ewd]), ..Default::default() });
        assert_eq!(route[Du::Sd.id()], Some(Du::Ewd));
    }

    #[test]
    fn reconf_does_nothing_without_a_failure() {
        let mut r = Reconf::default();
        assert_eq!(press(&mut r, Inputs::default(), |i| i.reconf[0] = true), NORMAL);
    }

    #[test]
    fn captain_reconf_puts_the_lost_sd_on_the_captain_mfd_then_back() {
        let mut r = Reconf::default();
        let i = Inputs { failed: failed(&[Du::Sd]), ..Default::default() };
        assert_eq!(r.step(&i)[Du::CaptMfd.id()], Some(Du::CaptMfd));
        assert_eq!(press(&mut r, i, |i| i.reconf[0] = true)[Du::CaptMfd.id()], Some(Du::Sd));
        assert_eq!(press(&mut r, i, |i| i.reconf[0] = true)[Du::CaptMfd.id()], Some(Du::CaptMfd));
    }

    #[test]
    fn captain_reconf_cycles_through_every_lost_page() {
        let mut r = Reconf::default();
        let i = Inputs { failed: failed(&[Du::Ewd, Du::Sd]), ..Default::default() };
        let shown: Vec<_> = (0..3).map(|_| press(&mut r, i, |i| i.reconf[0] = true)[Du::CaptMfd.id()]).collect();
        assert_eq!(shown, vec![Some(Du::Ewd), Some(Du::Sd), Some(Du::CaptMfd)]);
    }

    #[test]
    fn a_page_cannot_be_shown_on_both_sides() {
        let mut r = Reconf::default();
        let i = Inputs { failed: failed(&[Du::Sd]), ..Default::default() };
        press(&mut r, i, |i| i.reconf[0] = true);
        let route = press(&mut r, i, |i| i.reconf[1] = true);
        assert_eq!(route[Du::CaptMfd.id()], Some(Du::Sd));
        assert_eq!(route.iter().filter(|f| **f == Some(Du::Sd)).count(), 1);
    }

    #[test]
    fn first_officer_reconf_uses_the_nd_unit_while_pfd_and_nd_work() {
        let mut r = Reconf::default();
        let i = Inputs { failed: failed(&[Du::FoMfd]), ..Default::default() };
        assert_eq!(press(&mut r, i, |i| i.reconf[1] = true)[Du::FoNd.id()], Some(Du::FoMfd));
    }

    #[test]
    fn captain_reconf_uses_the_nd_unit_when_the_mfd_unit_failed() {
        let mut r = Reconf::default();
        let i = Inputs { failed: failed(&[Du::CaptMfd]), ..Default::default() };
        assert_eq!(press(&mut r, i, |i| i.reconf[0] = true)[Du::CaptNd.id()], Some(Du::CaptMfd));
    }

    #[test]
    fn a_recovered_unit_clears_the_manual_reconfiguration() {
        let mut r = Reconf::default();
        let i = Inputs { failed: failed(&[Du::Sd]), ..Default::default() };
        press(&mut r, i, |i| i.reconf[0] = true);
        assert_eq!(r.step(&Inputs::default()), NORMAL);
    }

    #[test]
    fn hosts_name_the_unit_showing_each_page() {
        let mut route = NORMAL;
        route[Du::CaptPfd.id()] = Some(Du::CaptNd);
        route[Du::CaptNd.id()] = Some(Du::CaptPfd);
        let h = hosts(&route);
        assert_eq!(h[Du::CaptPfd.id()], (Du::CaptNd.id() + 1) as f64);
        assert_eq!(h[Du::CaptNd.id()], (Du::CaptPfd.id() + 1) as f64);
        assert_eq!(h[Du::Ewd.id()], (Du::Ewd.id() + 1) as f64);
    }

    #[test]
    fn the_cds_patches_match_the_installed_pages_once() {
        let Some(root) = std::env::var_os("FBW_HTML_UI") else { return };
        for p in source_patches() {
            let text = std::fs::read_to_string(std::path::Path::new(&root).join(p.path.trim_start_matches('/'))).unwrap();
            assert_eq!(crate::source_patch::count(&text, &p.find), 1, "{}", p.path);
        }
    }
}
