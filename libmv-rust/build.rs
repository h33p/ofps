extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=libmv-c.h");
    println!("cargo:rustc-link-search=native=/usr/local/lib");
    println!("cargo:rustc-link-lib=multiview");

    let src = ["libmv-c.cpp"];

    for i in &src {
        println!("cargo:rerun-if-changed={}", i);
    }

    let mut builder = cc::Build::new();

    let build = builder.cpp(true).files(src.iter());

    build.compile("libmv-c");

    let bindings = bindgen::Builder::default()
        .header("libmv-c.h")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .derive_default(true)
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
