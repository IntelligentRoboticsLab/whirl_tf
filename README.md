# whirl-tf

[![License](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/IntelligentRoboticsLab/whirl_tf#license)
[![Crates.io](https://img.shields.io/crates/v/whirl_tf.svg)](https://crates.io/crates/whirl_tf)
[![Docs](https://docs.rs/whirl_tf/badge.svg)](https://docs.rs/whirl_tf/latest/whirl_tf/)
[![PyPI](https://img.shields.io/pypi/v/whirl-tf.svg)](https://pypi.org/project/whirl-tf/)

`whirl-tf` provides timestamped coordinate-frame transforms for robotics. The
core is implemented in Rust and exposed as both the `whirl_tf` crate and a
Python package built with PyO3 and maturin.

The transform buffer supports static and dynamic edges, interpolation between
dynamic samples, explicit-timestamp lookup, matrix conversion, frame-name
validation, and a small set of canonical robot frame constants. It does not
extrapolate outside the available dynamic history.

## Installation

Install the Python package from PyPI:

```bash
pip install whirl-tf
```

Or add the Rust crate:

```bash
cargo add whirl_tf
```

Python 3.11 and newer are supported. Wheels are published for manylinux
x86-64, manylinux AArch64, and macOS Apple silicon.

## Python example

```python
from whirl_tf import TransformBuffer

identity = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]

buffer = TransformBuffer(history_seconds=10.0)
buffer.insert_transform("field", "odom", identity, stamp_ns=0, is_static=True)
T_odom_to_field = buffer.lookup("odom", "field", stamp_ns=1_000_000_000)
```

`lookup(from_frame, to_frame, stamp_ns)` returns a 4 x 4 matrix that maps
coordinates expressed in `from_frame` into `to_frame`.

ROS-style Python `TransformStamped` and `TFMessage` objects can be inserted
with `insert_transform_stamped` and `insert_transform_message`; these methods
use attribute access and therefore do not require a ROS Python dependency.

## Rust example

```rust
use std::time::Duration;

use nalgebra::Isometry3;
use whirl_tf::TransformBuffer;

let mut buffer = TransformBuffer::new(Duration::from_secs(10));
buffer.insert_isometry("field", "odom", &Isometry3::identity(), 0, true)?;
let odom_to_field = buffer.lookup_ns("odom", "field", 1_000_000_000)?;

# Ok::<(), whirl_tf::TransformBufferError>(())
```

The crate's default `ros` feature also exposes minimal ROS 2 transform message
types and conversion helpers. Disable default features when only the
ROS-free timestamp and matrix API is needed:

```toml
whirl_tf = { version = "0.1", default-features = false }
```

## Development

The repository contains a Rust core crate and a separate PyO3 binding crate.
[Pixi](https://pixi.sh/) provides the development tasks:

```bash
pixi run rs-check
pixi run python-stubs
pixi run -e py py-lint
pixi run -e py py-build-wheel
```

The generated wheel is written to `whirl_tf_py/dist/`.

## License

Licensed under either the [Apache License, Version 2.0](LICENSE-APACHE) or the
[MIT License](LICENSE-MIT), at your option.
