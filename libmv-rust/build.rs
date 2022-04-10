extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-changed=libmv-c.h");
    let manifest = env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search=native={manifest}/libmv/bin-opt/lib/");
    println!("cargo:rustc-link-lib=multiview");

    let src = ["libmv-c.cpp"];

    for i in &src {
        println!("cargo:rerun-if-changed={}", i);
    }

    let mut builder = cc::Build::new();

    // Disable warnings coming from eigen
    let build = builder
        .cpp(true)
        .files(src.iter())
        .include("libmv/src/")
        .include("libmv/src/third_party/eigen")
        .flag_if_supported("-Wno-deprecated-declarations")
        .flag_if_supported("-Wno-ignored-attributes")
        .flag_if_supported("-Wno-int-in-bool-context")
        .flag_if_supported("-Wno-deprecated-copy")
        .flag_if_supported("-Wno-misleading-indentation");

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
