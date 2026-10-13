//! Every simulator variable the deep areas publish, as the MSFS host names
//! them (`A32NX_` + `lvar_key`), one per line.
fn main() {
    let deep = deep_systems::DeepSystems::new();
    for name in deep.published_names() {
        println!("A32NX_{}", deep_systems::lvar_key(&name));
    }
}
