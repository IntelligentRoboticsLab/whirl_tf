//! Writes the type stub for the `whirl_tf._native` extension module.

fn main() -> pyo3_stub_gen::Result<()> {
    let manifest_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    _native::stub_info()?.generate()?;

    // pyo3-stub-gen represents a dotted extension module as a package in a
    // mixed layout. The extension itself is a single `_native` module, so
    // keep the stub next to the compiled extension as `_native.pyi`.
    let generated = manifest_dir.join("whirl_tf/_native/__init__.pyi");
    let target = manifest_dir.join("whirl_tf/_native.pyi");
    std::fs::rename(&generated, &target)?;
    std::fs::remove_dir(generated.parent().expect("generated stub has a parent"))?;
    Ok(())
}
