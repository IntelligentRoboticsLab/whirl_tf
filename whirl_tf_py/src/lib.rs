use std::sync::{Arc, Mutex, RwLock, Weak};
use std::time::Duration;

use ::whirl_tf::{
    FilterFailureReason as RustFilterFailureReason, MessageFilter as RustMessageFilter,
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
    inner: RwLock<RustTransformBuffer>,
    filters: Mutex<Vec<Weak<Mutex<MessageFilterState>>>>,
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
            inner: RwLock::new(RustTransformBuffer::new(Duration::from_secs_f64(
                history_seconds,
            ))),
            filters: Mutex::new(Vec::new()),
        })
    }

    #[pyo3(signature = (parent, child, matrix, stamp_ns, is_static=false))]
    fn insert_transform(
        &self,
        py: Python<'_>,
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
            .write()
            .expect("transform buffer lock was poisoned")
            .insert_isometry(parent, child, &isometry, stamp_ns, is_static)
            .map_err(transform_error)?;
        self.notify_filters(py)
    }

    #[pyo3(signature = (message, is_static=false))]
    fn insert_transform_stamped(
        &self,
        py: Python<'_>,
        message: &Bound<'_, PyAny>,
        is_static: bool,
    ) -> PyResult<()> {
        let transform = transform_stamped_from_python(message)?;
        self.inner
            .write()
            .expect("transform buffer lock was poisoned")
            .insert_isometry(
                &transform.parent,
                &transform.child,
                &transform.isometry,
                transform.stamp_ns,
                is_static,
            )
            .map_err(transform_error)?;
        self.notify_filters(py)
    }

    #[pyo3(signature = (message, is_static=false))]
    fn insert_transform_message(
        &self,
        py: Python<'_>,
        message: &Bound<'_, PyAny>,
        is_static: bool,
    ) -> PyResult<()> {
        let transforms = message.getattr("transforms")?;
        let transforms = transforms
            .try_iter()?
            .map(|item| transform_stamped_from_python(&item?))
            .collect::<PyResult<Vec<_>>>()?;
        let insert_result = {
            let mut buffer = self
                .inner
                .write()
                .expect("transform buffer lock was poisoned");
            transforms.into_iter().try_for_each(|transform| {
                buffer
                    .insert_isometry(
                        &transform.parent,
                        &transform.child,
                        &transform.isometry,
                        transform.stamp_ns,
                        is_static,
                    )
                    .map_err(transform_error)
            })
        };
        let notify_result = self.notify_filters(py);
        insert_result?;
        notify_result
    }

    fn lookup(&self, from_frame: &str, to_frame: &str, stamp_ns: u128) -> PyResult<Vec<Vec<f64>>> {
        validate_nanoseconds(stamp_ns)?;
        let isometry = self
            .inner
            .read()
            .expect("transform buffer lock was poisoned")
            .lookup_ns(from_frame, to_frame, stamp_ns)
            .map_err(transform_error)?;
        Ok(matrix_to_nested(&isometry_to_matrix4(&isometry)))
    }

    fn can_transform(&self, from_frame: &str, to_frame: &str, stamp_ns: u128) -> bool {
        validate_nanoseconds(stamp_ns).is_ok()
            && self
                .inner
                .read()
                .expect("transform buffer lock was poisoned")
                .can_transform_ns(from_frame, to_frame, stamp_ns)
    }

    fn clear_dynamic(&self) {
        self.inner
            .write()
            .expect("transform buffer lock was poisoned")
            .clear_dynamic();
    }
}

impl TransformBuffer {
    fn register_filter(&self, filter: &Arc<Mutex<MessageFilterState>>) {
        let mut filters = self.filters.lock().expect("filter list lock was poisoned");
        filters.retain(|filter| filter.strong_count() != 0);
        filters.push(Arc::downgrade(filter));
    }

    fn notify_filters(&self, py: Python<'_>) -> PyResult<()> {
        let filters = {
            let mut weak_filters = self.filters.lock().expect("filter list lock was poisoned");
            let filters = weak_filters
                .iter()
                .filter_map(Weak::upgrade)
                .collect::<Vec<_>>();
            weak_filters.retain(|filter| filter.strong_count() != 0);
            filters
        };
        for filter in filters {
            signal_dispatch(py, dispatch_filter(py, &filter, self, Vec::new()))?;
        }
        Ok(())
    }
}

/// Why a message was discarded before its transform became available.
#[gen_stub_pyclass]
#[pyclass(frozen, skip_from_py_object)]
#[derive(Clone, Debug, Eq, PartialEq)]
struct FilterFailureReason {
    inner: RustFilterFailureReason,
}

impl From<RustFilterFailureReason> for FilterFailureReason {
    fn from(reason: RustFilterFailureReason) -> Self {
        Self { inner: reason }
    }
}

#[gen_stub_pymethods]
#[pymethods]
impl FilterFailureReason {
    #[classattr]
    const EMPTY_FRAME_ID: FilterFailureReason = FilterFailureReason {
        inner: RustFilterFailureReason::EmptyFrameId,
    };

    #[classattr]
    const INVALID_FRAME_ID: FilterFailureReason = FilterFailureReason {
        inner: RustFilterFailureReason::InvalidFrameId,
    };

    #[classattr]
    const QUEUE_FULL: FilterFailureReason = FilterFailureReason {
        inner: RustFilterFailureReason::QueueFull,
    };

    fn __repr__(&self) -> &'static str {
        match self.inner {
            RustFilterFailureReason::EmptyFrameId => "FilterFailureReason.EMPTY_FRAME_ID",
            RustFilterFailureReason::InvalidFrameId => "FilterFailureReason.INVALID_FRAME_ID",
            RustFilterFailureReason::QueueFull => "FilterFailureReason.QUEUE_FULL",
        }
    }

    fn __str__(&self) -> &'static str {
        match self.inner {
            RustFilterFailureReason::EmptyFrameId => "empty frame id",
            RustFilterFailureReason::InvalidFrameId => "invalid frame id",
            RustFilterFailureReason::QueueFull => "queue full",
        }
    }

    fn __eq__(&self, other: &Self) -> bool {
        self == other
    }
}

struct MessageFilterState {
    inner: RustMessageFilter<Py<PyAny>>,
    callbacks: Vec<Py<PyAny>>,
    failure_callbacks: Vec<Py<PyAny>>,
}

/// Queues stamped Python messages until whirl-tf can transform them.
#[gen_stub_pyclass]
#[pyclass(name = "MessageFilter")]
struct MessageFilter {
    state: Arc<Mutex<MessageFilterState>>,
    buffer: Py<TransformBuffer>,
}

#[gen_stub_pymethods]
#[pymethods]
impl MessageFilter {
    #[new]
    #[pyo3(signature = (buffer, target_frame, queue_size, callback=None, tolerance_ns=0))]
    fn new(
        py: Python<'_>,
        buffer: Py<TransformBuffer>,
        target_frame: &str,
        queue_size: usize,
        callback: Option<Py<PyAny>>,
        tolerance_ns: u128,
    ) -> PyResult<Self> {
        validate_nanoseconds(tolerance_ns)?;
        if let Some(callback) = callback.as_ref() {
            require_callable(py, callback)?;
        }
        let mut inner =
            RustMessageFilter::new(target_frame, queue_size).map_err(transform_error)?;
        inner.set_tolerance_ns(tolerance_ns);
        let state = Arc::new(Mutex::new(MessageFilterState {
            inner,
            callbacks: callback.into_iter().collect(),
            failure_callbacks: Vec::new(),
        }));
        buffer.bind(py).borrow().register_filter(&state);
        Ok(Self { state, buffer })
    }

    fn add(&self, py: Python<'_>, message: Py<PyAny>) -> PyResult<()> {
        let (source_frame, stamp_ns) = stamped_header_from_python(message.bind(py))?;
        let dropped = {
            let mut state = self.state.lock().expect("message filter lock was poisoned");
            state.inner.add(message, &source_frame, stamp_ns)
        };
        let buffer = self.buffer.bind(py).borrow();
        signal_dispatch(py, dispatch_filter(py, &self.state, &buffer, dropped))
    }

    fn register_callback(&self, py: Python<'_>, callback: Py<PyAny>) -> PyResult<()> {
        require_callable(py, &callback)?;
        self.state
            .lock()
            .expect("message filter lock was poisoned")
            .callbacks
            .push(callback);
        Ok(())
    }

    fn register_failure_callback(&self, py: Python<'_>, callback: Py<PyAny>) -> PyResult<()> {
        require_callable(py, &callback)?;
        self.state
            .lock()
            .expect("message filter lock was poisoned")
            .failure_callbacks
            .push(callback);
        Ok(())
    }

    fn set_target_frame(&self, py: Python<'_>, target_frame: &str) -> PyResult<()> {
        self.state
            .lock()
            .expect("message filter lock was poisoned")
            .inner
            .set_target_frame(target_frame)
            .map_err(transform_error)?;
        self.dispatch(py)
    }

    fn set_target_frames(&self, py: Python<'_>, target_frames: Vec<String>) -> PyResult<()> {
        self.state
            .lock()
            .expect("message filter lock was poisoned")
            .inner
            .set_target_frames(target_frames)
            .map_err(transform_error)?;
        self.dispatch(py)
    }

    fn set_tolerance_ns(&self, py: Python<'_>, tolerance_ns: u128) -> PyResult<()> {
        validate_nanoseconds(tolerance_ns)?;
        self.state
            .lock()
            .expect("message filter lock was poisoned")
            .inner
            .set_tolerance_ns(tolerance_ns);
        self.dispatch(py)
    }

    fn set_queue_size(&self, py: Python<'_>, queue_size: usize) -> PyResult<()> {
        let dropped = self
            .state
            .lock()
            .expect("message filter lock was poisoned")
            .inner
            .set_queue_size(queue_size);
        let buffer = self.buffer.bind(py).borrow();
        signal_dispatch(py, dispatch_filter(py, &self.state, &buffer, dropped))
    }

    fn clear(&self) {
        self.state
            .lock()
            .expect("message filter lock was poisoned")
            .inner
            .clear();
    }

    #[getter]
    fn pending_count(&self) -> usize {
        self.state
            .lock()
            .expect("message filter lock was poisoned")
            .inner
            .pending_count()
    }

    #[getter]
    fn queue_size(&self) -> usize {
        self.state
            .lock()
            .expect("message filter lock was poisoned")
            .inner
            .queue_size()
    }

    #[getter]
    fn tolerance_ns(&self) -> u128 {
        self.state
            .lock()
            .expect("message filter lock was poisoned")
            .inner
            .tolerance_ns()
    }

    #[getter]
    fn target_frames(&self) -> Vec<String> {
        self.state
            .lock()
            .expect("message filter lock was poisoned")
            .inner
            .target_frames()
            .to_vec()
    }

    fn dispatch(&self, py: Python<'_>) -> PyResult<()> {
        let buffer = self.buffer.bind(py).borrow();
        signal_dispatch(py, dispatch_filter(py, &self.state, &buffer, Vec::new()))
    }
}

struct FilterDispatch {
    ready: Vec<Py<PyAny>>,
    dropped: Vec<(Py<PyAny>, FilterFailureReason)>,
    callbacks: Vec<Py<PyAny>>,
    failure_callbacks: Vec<Py<PyAny>>,
}

fn dispatch_filter(
    py: Python<'_>,
    state: &Arc<Mutex<MessageFilterState>>,
    buffer: &TransformBuffer,
    dropped: Vec<::whirl_tf::DroppedMessage<Py<PyAny>>>,
) -> FilterDispatch {
    let mut state = state.lock().expect("message filter lock was poisoned");
    let ready = state.inner.drain_ready(
        &buffer
            .inner
            .read()
            .expect("transform buffer lock was poisoned"),
    );
    FilterDispatch {
        ready,
        dropped: dropped
            .into_iter()
            .map(|dropped| (dropped.message, dropped.reason.into()))
            .collect(),
        callbacks: state
            .callbacks
            .iter()
            .map(|callback| callback.clone_ref(py))
            .collect(),
        failure_callbacks: state
            .failure_callbacks
            .iter()
            .map(|callback| callback.clone_ref(py))
            .collect(),
    }
}

fn signal_dispatch(py: Python<'_>, dispatch: FilterDispatch) -> PyResult<()> {
    for (message, reason) in dispatch.dropped {
        let reason = Py::new(py, reason)?;
        for callback in &dispatch.failure_callbacks {
            callback.call1(py, (message.bind(py), reason.bind(py)))?;
        }
    }
    for message in dispatch.ready {
        for callback in &dispatch.callbacks {
            callback.call1(py, (message.bind(py),))?;
        }
    }
    Ok(())
}

fn require_callable(py: Python<'_>, callback: &Py<PyAny>) -> PyResult<()> {
    if callback.bind(py).is_callable() {
        Ok(())
    } else {
        Err(PyValueError::new_err("callback must be callable"))
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
    module.add_class::<MessageFilter>()?;
    module.add_class::<FilterFailureReason>()?;
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

fn stamped_header_from_python(message: &Bound<'_, PyAny>) -> PyResult<(String, u128)> {
    let header = message.getattr("header")?;
    let stamp_ns = stamp_from_python(&header.getattr("stamp")?)?;
    Ok((header.getattr("frame_id")?.extract()?, stamp_ns))
}

fn stamp_from_python(stamp: &Bound<'_, PyAny>) -> PyResult<u128> {
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
    Ok(u128::try_from(sec).expect("non-negative i32") * 1_000_000_000 + u128::from(nanosec))
}

struct ParsedTransform {
    parent: String,
    child: String,
    isometry: Isometry3<f64>,
    stamp_ns: u128,
}

fn transform_stamped_from_python(message: &Bound<'_, PyAny>) -> PyResult<ParsedTransform> {
    let header = message.getattr("header")?;
    let transform = message.getattr("transform")?;
    let translation = transform.getattr("translation")?;
    let rotation = transform.getattr("rotation")?;
    let stamp_ns = stamp_from_python(&header.getattr("stamp")?)?;
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
