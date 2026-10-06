fn main() {
    let deep = deep_systems::DeepSystems::new();
    for name in deep.published_names() {
        println!("A32NX_{}", deep_systems::lvar_key(&name));
    }
}
