use crate::deep::api::*;

pub fn register(r: &mut Registry) {
    r.component(ComponentDef {
        id: "18_int.weather_truth_feed".into(),
        area: Area::Integration,
        ata: 34,
        name: "Real X-Plane weather truth feed (XPLMGetWeatherAtLocation + sim/weather/aircraft/*)".into(),
        params: vec![ParamDef {
            name: "weather_api_available".into(),
            meaning: "1.0 the host's weather source answered this tick (deep::weather::WeatherSource::has_weather_api()) .. 0.0 unavailable (pre-12 SDK target on X-Plane, or outside its regional data) -- every icing/lightning/hail/turbulence input derived from it falls back to a documented dry/calm default while this is 0, never a fabricated reading".into(),
            healthy: 1.0,
        }],
        failures: vec![],
    });

    r.component(ComponentDef {
        id: "18_int.airframe_ice_state".into(),
        area: Area::Integration,
        ata: 30,
        name: "Aggregate airframe ice aerodynamic penalty, as applied to X-Plane's flight model".into(),
        params: vec![
            ParamDef { name: "cl_max_loss_frac".into(), meaning: "0 none .. 1 total, from fire_ice::icing::IcingOutputs summed across tracked surfaces".into(), healthy: 0.0 },
            ParamDef { name: "cd_increase_frac".into(), meaning: "extra profile-drag coefficient fraction actually injected via sim/flightmodel/forces/faxil_plug_acf this tick".into(), healthy: 0.0 },
        ],
        failures: vec![],
    });

    for leg in ["nose", "left_wing", "right_wing", "left_body", "right_body"] {
        r.component(ComponentDef {
            id: format!("18_int.gear_xp_relay.{leg}"),
            area: Area::Integration,
            ata: 32,
            name: format!("{leg} gear leg -> X-Plane deploy_ratio/drag relay"),
            params: vec![ParamDef {
                name: "relay_active".into(),
                meaning: "1.0 gear_structure::LegOutput.collapsed is being relayed as a forced-retracted sim/aircraft/parts/acf_gear_deploy element (W215, E:/fbw-debug/fixes/W215.md: not the read-only sim/flightmodel2/gear/deploy_ratio this used to name) .. 0.0 leg intact, X-Plane's own gear physics unmodified. The 'plus ground-drag force injection' this string used to claim is not wired (fixes/W124.md's own SUMMARY: a collapsed leg's own drag force is deliberately left out, no static_load_share_n is published to compute it from)".into(),
                healthy: 0.0,
            }],
            failures: vec![],
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_registered_item_validates_clean() {
        let mut r = Registry::default();
        register(&mut r);
        let errors = r.validate_area();
        assert!(errors.is_empty(), "{errors:?}");
        assert_eq!(r.components.len(), 7, "weather feed + ice state + 5 gear legs");
        assert!(r.failures.is_empty(), "Integration originates no new injectable failures (module doc)");
        assert!(r.alerts.is_empty(), "Integration raises no ECAM alerts of its own (module doc)");
    }

    #[test]
    fn every_component_id_is_unique_and_carries_the_integration_area() {
        let mut r = Registry::default();
        register(&mut r);
        for c in &r.components {
            assert_eq!(c.area, Area::Integration);
        }
        let mut ids: Vec<&str> = r.components.iter().map(|c| c.id.as_str()).collect();
        ids.sort();
        ids.dedup();
        assert_eq!(ids.len(), r.components.len(), "no duplicate ids");
    }
}
