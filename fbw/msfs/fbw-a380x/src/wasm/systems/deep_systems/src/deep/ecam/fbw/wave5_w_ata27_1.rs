use super::{phase, proc, sd_page, FbwProc};
use crate::deep::api::{any, var, Cond, Level};

fn network_alive() -> Cond {
    any(vec![
        var("ELEC_AC_1_BUS_IS_POWERED").on(),
        var("ELEC_AC_2_BUS_IS_POWERED").on(),
        var("ELEC_AC_3_BUS_IS_POWERED").on(),
        var("ELEC_AC_4_BUS_IS_POWERED").on(),
    ])
}

pub fn wire(v: &mut Vec<FbwProc>) {
    v.push(
        proc(
            272_800_016,
            "F/CTL FLAPS LOCKED",
            Level::Caution,
            sd_page::FCTL,
            var("FCTL_FLAP_OVERSPEED_DAMAGE").gt(0.02),
            "a high-lift drive line's torque limiter has been sustained over its design threshold by pure airload (VFE exceeded with the device extended), leaving PERMANENT damage -- deep::flight_controls::high_lift's own aero-torque-vs-limiter-threshold model, not an injected fault; never fires within VFE on a healthy aircraft; FCOM PRO-ABN-ECAM p.5114: the wing-tip brakes lock the flaps to avoid surface runaway, overspeed or asymmetry",
        )
        .confirm(1.0)
        .inhibit(phase::TAKEOFF_AND_LANDING)
        .items(0, Vec::new()),
    );
    v.push(
        proc(
            272_800_024,
            "F/CTL SLATS LOCKED",
            Level::Caution,
            sd_page::FCTL,
            any(vec![
                var("FCTL_SLAT_OVERSPEED_DAMAGE").gt(0.02),
                var("FCTL_DROOP_OVERSPEED_DAMAGE").gt(0.02),
            ]),
            "as 272800016 but for the slat/droop-nose drive line -- the droop nose devices share the slat system's own wingtip brake on the real aircraft, so a droop overspeed is reported through the same alert; FCOM PRO-ABN-ECAM p.5134: the wing-tip brakes lock the slats to avoid surface runaway, overspeed or asymmetry",
        )
        .confirm(1.0)
        .inhibit(phase::TAKEOFF_AND_LANDING)
        .items(0, Vec::new()),
    );
}

pub fn procs() -> Vec<FbwProc> {
    let mut v = Vec::new();
    wire(&mut v);
    v
}
