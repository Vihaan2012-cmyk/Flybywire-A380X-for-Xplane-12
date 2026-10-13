//! FlyByWire's A380X systems in their own process, for the X-Plane plugin
//! (src/remote). Started by the plugin as
//! `fbw_a380_systems_server <connection tag> <plugin process id>`.

fn main() {
    let mut args = std::env::args().skip(1);
    let Some(tag) = args.next() else {
        eprintln!("usage: fbw_a380_systems_server <connection tag> [plugin process id]");
        std::process::exit(2);
    };
    let parent = args.next().and_then(|p| p.parse::<u32>().ok());
    // Building the aircraft moves large structures through deep frames.
    let worker = std::thread::Builder::new()
        .name("fbw systems".into())
        .stack_size(256 << 20)
        .spawn(move || fbw_a380_systems::serve_systems(&tag, parent));
    let code = match worker.map(|w| w.join()) {
        Ok(Ok(Ok(()))) => 0,
        Ok(Ok(Err(e))) => {
            eprintln!("fbw_a380_systems_server: {e}");
            1
        }
        _ => 1,
    };
    std::process::exit(code);
}
