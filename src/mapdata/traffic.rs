//! Air traffic for FlyByWire's TCAS: MSFS's `GET_AIR_TRAFFIC` Coherent call
//! answered from X-Plane's TCAS targets.
//!
//! FlyByWire's A380X TCAS (`systems-host/Misc/tcas/components/
//! LegacyTcasComputer.ts:480`) and its VDL datalink (`fbw-common/src/
//! systems/datalink/router/src/vhf/VDL.ts:71`) call `GET_AIR_TRAFFIC` and
//! read each entry as `JS_NPCPlane` (`tcas/lib/TcasConstants.ts:105`):
//! `name`, `uId`, `lat`, `lon`, `alt` in metres (they multiply by 3.281),
//! `isOnGround` and `heading` in degrees. The TCAS computes vertical speed,
//! ground speed and closure itself from successive answers.
//!
//! The plugin reads X-Plane's `sim/cockpit2/tcas/targets/...` datarefs each
//! frame (`plugin.rs`) and keeps the targets here; the call is answered from
//! that copy on the scripts' thread, which is the same thread.

use std::sync::Mutex;

/// One of X-Plane's TCAS targets.
#[derive(Clone, Debug, PartialEq)]
pub struct Target {
    /// `sim/cockpit2/tcas/targets/modeS_id`: the airframe's 24-bit address.
    pub mode_s_id: u32,
    /// `sim/cockpit2/tcas/targets/flight_id`.
    pub flight_id: String,
    pub latitude: f64,
    pub longitude: f64,
    /// Metres above mean sea level.
    pub elevation_m: f64,
    /// True heading, degrees.
    pub heading: f64,
    pub on_ground: bool,
}

static TARGETS: Mutex<Vec<Target>> = Mutex::new(Vec::new());

/// Replace the current targets.
pub fn set(targets: Vec<Target>) {
    *TARGETS.lock().unwrap_or_else(|e| e.into_inner()) = targets;
}

/// The Coherent calls answered here.
pub const CALLS: [&str; 1] = ["GET_AIR_TRAFFIC"];

/// Answer a Coherent call if it is one of `CALLS`: the JSON the call's
/// promise resolves to.
pub fn call(name: &str, _args_json: &str) -> Option<Result<String, String>> {
    if !CALLS.contains(&name) {
        return None;
    }
    let targets = TARGETS.lock().unwrap_or_else(|e| e.into_inner());
    Some(Ok(to_json(&targets)))
}

/// `JS_NPCPlane[]`.
pub fn to_json(targets: &[Target]) -> String {
    let list: Vec<serde_json::Value> = targets
        .iter()
        .map(|t| {
            serde_json::json!({
                "name": t.flight_id,
                "uId": t.mode_s_id,
                "lat": t.latitude,
                "lon": t.longitude,
                "alt": t.elevation_m,
                "isOnGround": t.on_ground,
                "heading": t.heading,
            })
        })
        .collect();
    serde_json::Value::Array(list).to_string()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn traffic_is_answered_as_msfs_npc_planes() {
        set(vec![Target {
            mode_s_id: 0xABCDEF,
            flight_id: "DLH4AB".into(),
            latitude: 47.5,
            longitude: 11.25,
            elevation_m: 3048.,
            heading: 271.5,
            on_ground: false,
        }]);
        let json = call("GET_AIR_TRAFFIC", "[]").unwrap().unwrap();
        let v: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(v[0]["uId"], 0xABCDEF);
        assert_eq!(v[0]["alt"], 3048.);
        assert_eq!(v[0]["name"], "DLH4AB");
        assert_eq!(v[0]["isOnGround"], false);
        assert!(call("LOAD_AIRPORT", "[]").is_none());
    }
}
