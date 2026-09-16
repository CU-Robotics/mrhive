use std::env;
use std::path::PathBuf;

fn main() {
    // link mrcal
    println!("cargo:rustc-link-lib=mrcal");

    // generate raw bindings
    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("unable to generate mrcal bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("couldn't write raw bindings");
}
