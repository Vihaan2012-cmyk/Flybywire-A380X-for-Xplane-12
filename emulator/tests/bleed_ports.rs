//! The engine's customer bleed ports, end to end: the physical engine's IP8
//! and HP6 pressures feed FlyByWire's bleed system, whose HP valve follows
//! EASA.E.012 section 10's switch-over (IP8 above 206.8 kPa, HP6 below),
//! and the engine takes its bleed from whichever port that leaves open.
//! The cockpit TGT is the EEC's trimmed value (Note 16).

use fbw_a380_emulator::{presets, Emulator};

fn engine(e: &mut Emulator, name: &str, n: usize) -> f64 {
    e.get_var(&format!("{name}:{n}"))
}

#[test]
fn the_hp_valve_follows_the_ip_port_pressure_from_idle_to_take_off() {
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(|| {
            let mut e = presets::engines_running();
            e.run(0.1, 300);
            for n in 1..=4 {
                let ip8 = engine(&mut e, "ENGINE_IP_PORT_PRESSURE_PA", n);
                assert!(ip8 < 206_800.0, "engine {n} idle IP8 {ip8:.0} Pa");
                assert_eq!(e.get_var(&format!("PNEU_ENG_{n}_HP_VALVE_OPEN")), 1.0, "engine {n}: HP6 feeds the bleed at idle");
                let (shown, measured) = (engine(&mut e, "ENGINE_EGT", n), engine(&mut e, "ENGINE_EGT_UNTRIMMED", n));
                assert!((measured - shown - 89.0).abs() < 1e-6, "engine {n}: idle trim {:.1}", measured - shown);
            }
            for n in 1..=4 {
                e.set_thrust_lever(n, 1.0);
            }
            e.run(0.1, 300);
            for n in 1..=4 {
                let ip8 = engine(&mut e, "ENGINE_IP_PORT_PRESSURE_PA", n);
                assert!(ip8 > 206_800.0, "engine {n} take-off IP8 {ip8:.0} Pa");
                assert_eq!(e.get_var(&format!("PNEU_ENG_{n}_HP_VALVE_OPEN")), 0.0, "engine {n}: IP8 feeds the bleed at take-off");
                let (shown, measured) = (engine(&mut e, "ENGINE_EGT", n), engine(&mut e, "ENGINE_EGT_UNTRIMMED", n));
                assert!((measured - shown - 56.0).abs() < 1e-6, "engine {n}: take-off trim {:.1}", measured - shown);
                assert!(engine(&mut e, "ENGINE_BLEED_EXTRACTION_KG_S", n) <= engine(&mut e, "ENGINE_BLEED_LIMIT_KG_S", n));
            }
        })
        .unwrap()
        .join()
        .unwrap();
}
