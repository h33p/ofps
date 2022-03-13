use std::fs::File;
use std::io::Write;
use std::{env, path};

use rustc_version::version_meta;

fn main() {
    let mut path = path::PathBuf::from(env::var_os("OUT_DIR").unwrap());
    path.push("version.rs");
    let mut f = File::create(&path).unwrap();

    let version = version_meta()
        .expect("Failed to read rustc version.")
        .short_version_string;

    write!(
        f,
        "
            /// `rustc` version string.
            pub const RUSTC_VERSION: &str = \"{version}\";
            ",
    )
    .unwrap();
}
