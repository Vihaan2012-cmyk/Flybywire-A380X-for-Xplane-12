fn quote(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn main() {
    let r = deep_systems::deep::registry();
    let failures: Vec<String> = r
        .failures
        .iter()
        .map(|f| format!("{{\"id\":{},\"ata\":{},\"name\":{},\"component\":{},\"effect\":{}}}", f.id, f.ata, quote(&f.name), quote(&f.component), quote(&f.effect)))
        .collect();
    let components: Vec<String> = r
        .components
        .iter()
        .map(|c| {
            let params: Vec<String> = c
                .params
                .iter()
                .map(|p| format!("{{\"name\":{},\"meaning\":{},\"healthyValue\":{}}}", quote(&p.name), quote(&p.meaning), p.healthy))
                .collect();
            let ids: Vec<String> = c.failures.iter().map(|i| i.to_string()).collect();
            format!("{{\"id\":{},\"ata\":{},\"name\":{},\"parameters\":[{}],\"failures\":[{}]}}", quote(&c.id), c.ata, quote(&c.name), params.join(","), ids.join(","))
        })
        .collect();
    let path = std::env::args().nth(1).unwrap_or_else(|| "E:/registry-export.json".to_string());
    std::fs::write(&path, format!("{{\"failures\":[{}],\"components\":[{}]}}", failures.join(","), components.join(","))).expect("write registry export");
    println!("{} failures, {} components -> {path}", r.failures.len(), r.components.len());
}
