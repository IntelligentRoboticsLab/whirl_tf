use std::time::Duration;

use ::whirl_tf::{
    TransformBuffer as RustTransformBuffer, frames, isometry_from_components, isometry_to_matrix4,
    matrix4_to_isometry,
};
use nalgebra::{Isometry3, Matrix4};
use pyo3::exceptions::{PyException, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyAny, PyModule};
use pyo3_stub_gen::create_exception;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pyfunction, gen_stub_pymethods};

create_exception!(whirl_tf._native, TransformError, PyException);
create_exception!(whirl_tf._native, FrameError, PyException);

/// Frames exposed as module-level constants. Declared once so the runtime values
/// and their stub declarations cannot drift apart.
macro_rules! frame_constants {
    ($(($name:literal, $value:expr)),* $(,)?) => {
        $(pyo3_stub_gen::module_variable!("whirl_tf._native", $name, String);)*

        fn add_frame_constants(module: &Bound<'_, PyModule>) -> PyResult<()> {
            $(module.add($name, $value)?;)*
            Ok(())
        }
    };
}

frame_constants!(
    ("FIELD", frames::FIELD),
    ("ODOM", frames::ODOM),
    ("BASE_LINK", frames::BASE_LINK),
    ("BASE_FOOTPRINT", frames::BASE_FOOTPRINT),
    ("TORSO", frames::TORSO),
    ("GAZE", frames::GAZE),
    ("LEFT_CAMERA_LINK", frames::LEFT_CAMERA_LINK),
    (
        "LEFT_CAMERA_OPTICAL_FRAME",
        frames::LEFT_CAMERA_OPTICAL_FRAME
    ),
    ("LEFT_SOLE", frames::LEFT_SOLE),
    ("RIGHT_SOLE", frames::RIGHT_SOLE),
);

pyo3_stub_gen::module_variable!("whirl_tf._native", "ALL", Vec<String>);

/// Gathers the stub declarations submitted by the macros above; used by the
/// `stub_gen` binary.
pub fn stub_info() -> pyo3_stub_gen::Result<pyo3_stub_gen::StubInfo> {
    let manifest_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    pyo3_stub_gen::StubInfo::from_pyproject_toml(manifest_dir.join("pyproject.toml"))
}

#[gen_stub_pyclass]
#[pyclass(name = "TransformBuffer")]
struct TransformBuffer {
    inner: RustTransformBuffer,
}

#[gen_stub_pymethods]
#[pymethods]
impl TransformBuffer {
    #[new]
    fn new(history_seconds: f64) -> PyResult<Self> {
        if !history_seconds.is_finite() || history_seconds < 0.0 {
            return Err(PyValueError::new_err(
                "history_seconds must be finite and non-negative",
            ));
        }
        Ok(Self {
            inner: RustTransformBuffer::new(Duration::from_secs_f64(history_seconds)),
        })
    }

    #[pyo3(signature = (parent, child, matrix, stamp_ns, is_static=false))]
    fn insert_transform(
        &mut self,
        parent: &str,
        child: &str,
        matrix: Vec<Vec<f64>>,
        stamp_ns: u128,
        is_static: bool,
    ) -> PyResult<()> {
        let matrix = nested_matrix(matrix)?;
        let isometry = matrix4_to_isometry(&matrix).map_err(transform_error)?;
        validate_nanoseconds(stamp_ns)?;
        self.inner
            .insert_isometry(parent, child, &isometry, stamp_ns, is_static)
            .map_err(transform_error)
    }

    #[pyo3(signature = (message, is_static=false))]
    fn insert_transform_stamped(
        &mut self,
        message: &Bound<'_, PyAny>,
        is_static: bool,
    ) -> PyResult<()> {
        let transform = transform_stamped_from_python(message)?;
        self.inner
            .insert_isometry(
                &transform.parent,
                &transform.child,
                &transform.isometry,
                transform.stamp_ns,
                is_static,
            )
            .map_err(transform_error)
    }

    #[pyo3(signature = (message, is_static=false))]
    fn insert_transform_message(
        &mut self,
        message: &Bound<'_, PyAny>,
        is_static: bool,
    ) -> PyResult<()> {
        let transforms = message.getattr("transforms")?;
        for item in transforms.try_iter()? {
            let transform = transform_stamped_from_python(&item?)?;
            self.inner
                .insert_isometry(
                    &transform.parent,
                    &transform.child,
                    &transform.isometry,
                    transform.stamp_ns,
                    is_static,
                )
                .map_err(transform_error)?;
        }
        Ok(())
    }

    fn lookup(&self, from_frame: &str, to_frame: &str, stamp_ns: u128) -> PyResult<Vec<Vec<f64>>> {
        validate_nanoseconds(stamp_ns)?;
        let isometry = self
            .inner
            .lookup_ns(from_frame, to_frame, stamp_ns)
            .map_err(transform_error)?;
        Ok(matrix_to_nested(&isometry_to_matrix4(&isometry)))
    }

    fn can_transform(&self, from_frame: &str, to_frame: &str, stamp_ns: u128) -> bool {
        validate_nanoseconds(stamp_ns).is_ok()
            && self.inner.can_transform_ns(from_frame, to_frame, stamp_ns)
    }

    fn clear_dynamic(&mut self) {
        self.inner.clear_dynamic();
    }
}

#[gen_stub_pyfunction]
#[pyfunction]
fn require_frame(actual: &str, expected: &str, interface: &str) -> PyResult<()> {
    ::whirl_tf::require_frame(actual, expected, interface)
        .map_err(|error| FrameError::new_err(error.to_string()))
}

#[pymodule(name = "_native")]
fn whirl_tf(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<TransformBuffer>()?;
    module.add_function(wrap_pyfunction!(require_frame, module)?)?;
    module.add("TransformError", module.py().get_type::<TransformError>())?;
    module.add("FrameError", module.py().get_type::<FrameError>())?;
    add_frame_constants(module)?;
    module.add("ALL", frames::ALL)?;
    Ok(())
}

fn nested_matrix(rows: Vec<Vec<f64>>) -> PyResult<Matrix4<f64>> {
    if rows.len() != 4 || rows.iter().any(|row| row.len() != 4) {
        return Err(PyValueError::new_err("matrix must have shape 4x4"));
    }
    let values = rows.into_iter().flatten().collect::<Vec<_>>();
    Ok(Matrix4::from_row_slice(&values))
}

fn matrix_to_nested(matrix: &Matrix4<f64>) -> Vec<Vec<f64>> {
    (0..4)
        .map(|row| (0..4).map(|column| matrix[(row, column)]).collect())
        .collect()
}

fn validate_nanoseconds(nanoseconds: u128) -> PyResult<()> {
    let seconds = nanoseconds / 1_000_000_000;
    i32::try_from(seconds)
        .map_err(|_| PyValueError::new_err("stamp_ns exceeds the ROS Time range"))?;
    Ok(())
}

struct ParsedTransform {
    parent: String,
    child: String,
    isometry: Isometry3<f64>,
    stamp_ns: u128,
}

fn transform_stamped_from_python(message: &Bound<'_, PyAny>) -> PyResult<ParsedTransform> {
    let header = message.getattr("header")?;
    let stamp = header.getattr("stamp")?;
    let transform = message.getattr("transform")?;
    let translation = transform.getattr("translation")?;
    let rotation = transform.getattr("rotation")?;
    let sec: i32 = stamp.getattr("sec")?.extract()?;
    let nanosec: u32 = stamp.getattr("nanosec")?.extract()?;
    if sec < 0 {
        return Err(PyValueError::new_err(
            "timestamp seconds must be non-negative",
        ));
    }
    if nanosec >= 1_000_000_000 {
        return Err(PyValueError::new_err(
            "timestamp nanoseconds must be less than one second",
        ));
    }
    let stamp_ns =
        u128::try_from(sec).expect("non-negative i32") * 1_000_000_000 + u128::from(nanosec);
    let isometry = isometry_from_components(
        [
            translation.getattr("x")?.extract()?,
            translation.getattr("y")?.extract()?,
            translation.getattr("z")?.extract()?,
        ],
        [
            rotation.getattr("x")?.extract()?,
            rotation.getattr("y")?.extract()?,
            rotation.getattr("z")?.extract()?,
            rotation.getattr("w")?.extract()?,
        ],
    )
    .map_err(transform_error)?;
    Ok(ParsedTransform {
        parent: header.getattr("frame_id")?.extract()?,
        child: message.getattr("child_frame_id")?.extract()?,
        isometry,
        stamp_ns,
    })
}

fn transform_error(error: impl std::fmt::Display) -> PyErr {
    TransformError::new_err(error.to_string())
}
