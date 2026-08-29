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
validation, and transform-aware message filtering. It does not extrapolate
outside the available dynamic history.

## Installation

Install the Python package from PyPI:

```bash
pip install whirl-tf
```

Or add the Rust crate:

```bash
cargo add whirl_tf
```

Python 3.10 and newer are supported. Wheels are published for manylinux
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

## Filtering stamped messages

`MessageFilter` holds a stamped message until its transform is available. It
reads `message.header.frame_id` and `message.header.stamp` by attribute, so it
works with ROS messages without making whirl-tf depend on ROS:

```python
from whirl_tf import MessageFilter, TransformBuffer

buffer = TransformBuffer(history_seconds=10.0)

def handle_scan(scan):
    matrix = buffer.lookup(
        scan.header.frame_id,
        "field",
        scan.header.stamp.sec * 1_000_000_000 + scan.header.stamp.nanosec,
    )
    # Process the scan with matrix here.

scan_filter = MessageFilter(buffer, "field", queue_size=100, callback=handle_scan)

# These can be used directly as subscription callbacks. Their execution order
# no longer matters: every successful transform insertion wakes the filter.
scan_subscription_callback = scan_filter.add
tf_subscription_callback = buffer.insert_transform_message
```

Waiting messages are delivered in FIFO order, including when a later message
becomes transformable first. A nonzero queue size bounds memory and discards
the oldest waiting message on overflow; `register_failure_callback` reports
that message and a `FilterFailureReason`. A queue size of zero is unbounded.

Use `set_target_frames` when every message must be transformable into multiple
frames. `set_tolerance_ns` additionally requires transform coverage at the
message timestamp plus the given interval, matching tf2's tolerance behavior.

## Rust example

```rust
use std::time::Duration;

use nalgebra::Isometry3;
use whirl_tf::{MessageFilter, TransformBuffer};

let mut buffer = TransformBuffer::new(Duration::from_secs(10));
buffer.insert_isometry("field", "odom", &Isometry3::identity(), 0, true)?;
let odom_to_field = buffer.lookup_ns("odom", "field", 1_000_000_000)?;

let mut filter = MessageFilter::new("field", 100)?;
filter.add("scan", "odom", 1_000_000_000);
let ready_scans = filter.drain_ready(&buffer);
assert_eq!(ready_scans, ["scan"]);

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
