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

    println!("cargo:warning=Compiling libmv");

    println!("{}", std::str::from_utf8(&std::process::Command::new("bash")
            .arg("-c").arg("cd libmv; CC=clang CXX=clang++ make")
            .output()
            .expect("Failed to build kernel libmv!").stdout).unwrap());

    let mut builder = cc::Build::new();

    let build = builder
        .cpp(true)
        .files(src.iter())
        .include("libmv/src/");

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
