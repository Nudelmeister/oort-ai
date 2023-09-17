const MODULES: &[(&str, &str)] = &[
    ("missile", "src/missile.rs"),
    ("message", "src/message.rs"),
    ("radar", "src/radar.rs"),
    ("track", "src/track.rs"),
    //("message", "src/message.rs"),
];

fn main() {
    let mut source = std::fs::read_to_string("src/lib.rs").unwrap();
    for &module in MODULES {
        include_module(module, &mut source);
    }

    std::fs::write("out.rs", source).unwrap();
}

fn include_module((mod_name, mod_path): (&str, &str), source: &mut String) {
    *source = source.replacen(&format!("mod {mod_name};"), "", 1);

    let mod_src = std::fs::read_to_string(mod_path).unwrap();

    source.push_str("mod ");
    source.push_str(mod_name);
    source.push_str("{\n");
    source.push_str(&mod_src);
    source.push_str("}\n");
}
