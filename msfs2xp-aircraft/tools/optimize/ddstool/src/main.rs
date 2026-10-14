use msfs2xp::texture::{compress_rgba_to_dds, decode_rgba8, load_dds};

fn main() {
    let a: Vec<String> = std::env::args().collect();
    match a[1].as_str() {
        "decode" => {
            let img = load_dds(&std::fs::read(&a[2]).unwrap()).unwrap();
            let rgba = decode_rgba8(&img, 0).unwrap();
            std::fs::write(&a[3], rgba).unwrap();
            println!("{} {} {:?} {}", img.width, img.height, img.format, img.mips.len());
        }
        "encode" => {
            let rgba = std::fs::read(&a[2]).unwrap();
            let (w, h): (u32, u32) = (a[3].parse().unwrap(), a[4].parse().unwrap());
            let out = compress_rgba_to_dds(rgba, w, h).unwrap();
            std::fs::write(&a[5], &out.bytes).unwrap();
            println!("{} bytes", out.bytes.len());
        }
        _ => panic!("decode|encode"),
    }
}
