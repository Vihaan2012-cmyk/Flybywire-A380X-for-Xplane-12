//! Where the deep layer's frame time goes, area by area.
//!
//! `cargo test --release --lib -- --ignored --nocapture deep::cost_profile`
//!
//! Steps every area exactly as `Deep::tick` does (tick all, collect derived
//! failures, publish into the next frame's `PublishedFrame`) over a few of
//! the failure audit's flight profiles, timing each area's `tick` and
//! `publish` separately, and prints them worst first.

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use crate::deep::integration::failure_audit::profiles;
    use crate::deep::live::{Area, Faults, PublishedFrame};

    fn areas() -> Vec<Box<dyn Area>> {
        vec![
            crate::deep::apu::live::live_system(),
            crate::deep::autoflight::live::live_system(),
            crate::deep::avionics_network::live::live_system(),
            crate::deep::breakers::live::live_system(),
            crate::deep::cabin::live::live_system(),
            crate::deep::communications::live::live_system(),
            crate::deep::electrical::live::live_system(),
            crate::deep::engine_accessories::live::live_system(),
            crate::deep::environment::live::live_system(),
            crate::deep::fire_ice::live::live_system(),
            crate::deep::flight_controls::live::live_system(),
            crate::deep::fuel::live::live_system(),
            crate::deep::gear_structure::live::live_system(),
            crate::deep::hydraulics::live::live_system(),
            crate::deep::oxygen::live::live_system(),
            crate::deep::pneumatic_ducts::live::live_system(),
            crate::deep::sensors::live::live_system(),
            crate::deep::thermal_zones::live::live_system(),
            crate::deep::wiring::live::live_system(),
        ]
    }

    #[test]
    #[ignore]
    fn deep_cost_per_area() {
        const WARM: usize = 60;
        const FRAMES: usize = 600;
        for profile in profiles().into_iter().filter(|p| ["cruise", "ground_apu", "takeoff_roll"].contains(&p.name)) {
            let mut areas = areas();
            let truth0 = (profile.truth)();
            let faults = Faults::default();
            let mut last = PublishedFrame::default();
            let mut tick = vec![0f64; areas.len()];
            let mut publish = vec![0f64; areas.len()];
            let mut derived_total = 0f64;
            let mut published_count = 0usize;
            for frame in 0..WARM + FRAMES {
                let measure = frame >= WARM;
                let mut truth = truth0.clone();
                truth.published = std::mem::take(&mut last);
                for (i, area) in areas.iter_mut().enumerate() {
                    let t = Instant::now();
                    area.tick(&truth, &faults);
                    if measure {
                        tick[i] += t.elapsed().as_secs_f64();
                    }
                }
                let t = Instant::now();
                let mut n = 0usize;
                for area in &areas {
                    area.derived_failures(&mut |d| n += (d.magnitude > 0.) as usize);
                }
                if measure {
                    derived_total += t.elapsed().as_secs_f64();
                }
                let mut published = std::mem::take(&mut truth.published);
                published.begin();
                for (i, area) in areas.iter().enumerate() {
                    let t = Instant::now();
                    area.publish(&mut |k, v| published.set(k, v));
                    if measure {
                        publish[i] += t.elapsed().as_secs_f64();
                    }
                }
                published.finish();
                published_count = published.len();
                last = published;
            }
            let per = |s: f64| s * 1e3 / FRAMES as f64;
            let mut rows: Vec<(f64, f64, &str)> = areas.iter().enumerate().map(|(i, a)| (per(tick[i]), per(publish[i]), a.name())).collect();
            rows.sort_by(|a, b| (b.0 + b.1).total_cmp(&(a.0 + a.1)));
            let total: f64 = rows.iter().map(|r| r.0 + r.1).sum::<f64>() + per(derived_total);
            println!("\n{}: {total:.3} ms per frame ({published_count} published values), derived {:.3}", profile.name, per(derived_total));
            for (t, p, name) in rows {
                println!("  {:>7.3} tick {:>7.3} publish  {name}", t, p);
            }
        }
    }
}
