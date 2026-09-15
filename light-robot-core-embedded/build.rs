use flate2::write::GzEncoder;
use flate2::Compression;
use std::fs::{create_dir_all, read_dir, remove_dir_all, File};
use std::io::{copy, BufReader};
use std::path::{Path, PathBuf};
use std::process::Command;
use trunk_build_time::cmd::build;
use trunk_build_time::config;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    for path in [
        "../light-robot-core-frontend/Cargo.toml",
        "../light-robot-core-frontend/index.html",
        "../light-robot-core-frontend/index.scss",
        "../light-robot-core-frontend/resources",
        "../light-robot-core-frontend/src",
        "../light-robot-core-api/src",
        "../light-robot-core-flight-control/src",
        "../light-robot-core-simulation/src",
    ] {
        println!("cargo:rerun-if-changed={path}");
    }

    let cfg = config::ConfigOptsBuild {
        release: true,
        target: Some(PathBuf::from("../light-robot-core-frontend/index.html")),
        filehash: Some(false),
        ..Default::default()
    };
    // This build script itself targets ESP32-S3 and therefore runs under the
    // Xtensa `esp` toolchain. Trunk's child Cargo process builds wasm32, which
    // must instead use the host stable toolchain; an inherited
    // RUSTUP_TOOLCHAIN=esp otherwise overrides the frontend's local toolchain.
    std::env::set_var("RUSTUP_TOOLCHAIN", "stable");
    let stable_cargo = Command::new("rustup")
        .args(["which", "cargo", "--toolchain", "stable"])
        .output()
        .expect("rustup must be available to build the WebAssembly frontend");
    assert!(
        stable_cargo.status.success(),
        "stable Rust toolchain is unavailable"
    );
    let stable_cargo = PathBuf::from(String::from_utf8(stable_cargo.stdout).unwrap().trim());
    let stable_rustc = Command::new("rustup")
        .args(["which", "rustc", "--toolchain", "stable"])
        .output()
        .expect("rustup must be available to build the WebAssembly frontend");
    assert!(
        stable_rustc.status.success(),
        "stable Rust toolchain is unavailable"
    );
    std::env::set_var(
        "RUSTC",
        String::from_utf8(stable_rustc.stdout).unwrap().trim(),
    );
    // An outer `cargo clippy` sets wrappers for the ESP target. They must not
    // wrap Trunk's nested stable/wasm build.
    for variable in ["RUSTC_WRAPPER", "RUSTC_WORKSPACE_WRAPPER", "CLIPPY_ARGS"] {
        std::env::remove_var(variable);
    }
    let stable_bin = stable_cargo.parent().unwrap();
    let mut frontend_path = std::ffi::OsString::from(stable_bin.as_os_str());
    frontend_path.push(":");
    frontend_path.push(std::env::var_os("PATH").unwrap_or_default());
    std::env::set_var("PATH", frontend_path);
    build::Build { build: cfg }.run(None).await.unwrap();

    let source = Path::new("../light-robot-core-frontend/dist");
    let destination = Path::new("../light-robot-core-frontend/dist-gz");
    let _ = remove_dir_all(destination);
    gzip_directory(source, destination)?;

    embuild::espidf::sysenv::output();
    Ok(())
}

fn gzip_directory(source: &Path, destination: &Path) -> std::io::Result<()> {
    create_dir_all(destination)?;
    for entry in read_dir(source)? {
        let entry = entry?;
        let source_path = entry.path();
        let destination_path = destination.join(entry.file_name());
        if source_path.is_dir() {
            gzip_directory(&source_path, &destination_path)?;
            continue;
        }

        let mut input = BufReader::new(File::open(source_path)?);
        let output = File::create(destination_path)?;
        let mut encoder = GzEncoder::new(output, Compression::default());
        copy(&mut input, &mut encoder)?;
        encoder.finish()?;
    }
    Ok(())
}
