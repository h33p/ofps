# Optical Flow Processing Stack

OFPS is a generic optical flow processing library, and OFPS Suite is an accopanying app demonstating its functionality.

![](docs/report/panorama.jpg)

## Running OFPS Suite

1. Install the latest stable Rust toolchain (version 1.60.0) through https://rustup.rs/.

2. Install dependencies (see dedicated subsection).

3. Build default plugins with `cargo build --release`

4. Optionally, build libmv estimator (more involved, see its subsection).

5. Run OFPS suite with `cargo run --release --bin ofps-suite`

### Installing dependencies

Ubuntu/Debian:

```
sudo apt-get install atk1.0 libgtk-3-dev ffmpeg libavutil-dev libavcodec-dev libavformat-dev libavfilter-dev libavdevice-dev libopencv-dev libclang-dev clang libxcb-shape0-dev libxcb-xfixes0-dev
```

Fedora:

```
sudo dnf -y install https://download1.rpmfusion.org/free/fedora/rpmfusion-free-release-$(rpm -E %fedora).noarch.rpm
sudo dnf install gtk3-devel clang clang-devel opencv-devel ffmpeg-devel
```

Windows/macOS:

Good luck :)

### Building libmv estimator

First, source the environment at the root of the repo

```
source env
```

Then, install extra dependencies:

Ubuntu/Debian:

```
sudo apt-get install cmake libceres-dev libjpeg-dev
```

Fedora:

```
sudo dnf install cmake ceres-solver-devel libjpeg-turbo-devel
```

Go to libmv-rust/libmv directory. Run `make`. Not everything will compile. That is okay - we only need `libmultiview.so` and its dependencies.

Go back to root of the repo, run `cargo build --release --workspace`.

### Troubleshooting

Set log level to see errors better:

```
export RUST_LOG=<trace,debug,info,warn,error>
```

If it is a graphics issue, try forcing OpenGL backend:

```
export WGPU_BACKEND=gles
```

## Documentation

Assuming the workspace compiles, following steps 1-3 of OFPS Suite section, run `cargo doc --open`.

## Unit tests

Assuming the workspace compiles, run `cargo test`.
