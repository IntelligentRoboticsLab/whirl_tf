use std::collections::HashSet;
use std::time::Duration;

#[cfg(feature = "ros")]
use crate::ros_msgs::builtin_interfaces::Time;
#[cfg(feature = "ros")]
use crate::ros_msgs::geometry_msgs::{
    Quaternion as QuaternionMsg, Transform, TransformStamped, Vector3 as Vector3Msg,
};
#[cfg(feature = "ros")]
use crate::ros_msgs::std_msgs::Header;
#[cfg(feature = "ros")]
use crate::ros_msgs::tf2_msgs::TFMessage;
use nalgebra::{Isometry3, Matrix3, Matrix4, Quaternion, Rotation3, Translation3, UnitQuaternion};
use thiserror::Error;
use transforms::Registry;
use transforms::geometry::{
    Quaternion as TransformQuaternion, Transform as BufferedTransform, Vector3 as TransformVector3,
};
use transforms::time::Timestamp;

#[cfg(feature = "ros")]
const NANOSECONDS_PER_SECOND: u128 = 1_000_000_000;
#[cfg(feature = "ros")]
const NANOSECONDS_PER_SECOND_U32: u32 = 1_000_000_000;
const MATRIX_TOLERANCE: f64 = 1e-6;

#[derive(Debug, Error)]
pub enum TransformBufferError {
    #[error("invalid frame {frame:?}: {reason}")]
    InvalidFrame { frame: String, reason: &'static str },
    #[error("invalid ROS timestamp {sec}s {nanosec}ns: {reason}")]
    InvalidTimestamp {
        sec: i32,
        nanosec: u32,
        reason: &'static str,
    },
    #[error("dynamic transform {parent:?} -> {child:?} has the reserved zero timestamp")]
    ZeroDynamicTimestamp { parent: String, child: String },
    #[error("failed to insert transform {parent:?} -> {child:?}: {source}")]
    Insert {
        parent: String,
        child: String,
        #[source]
        source: Box<transforms::errors::BufferError>,
    },
    #[error("failed to look up transform from {from:?} to {to:?}: {source}")]
    Lookup {
        from: String,
        to: String,
        #[source]
        source: Box<transforms::errors::TransformError>,
    },
    #[error("transform contains a non-finite component")]
    NonFinite,
    #[error("transform quaternion has norm {norm}, expected 1")]
    NonUnitQuaternion { norm: f64 },
    #[error("matrix is not a rigid homogeneous transform")]
    NonRigidMatrix,
}

#[derive(Debug)]
pub struct TransformBuffer {
    registry: Registry<Timestamp>,
    dynamic_children: HashSet<String>,
}

impl TransformBuffer {
    #[must_use]
    pub fn new(history: Duration) -> Self {
        Self {
            registry: Registry::with_max_age(history),
            dynamic_children: HashSet::new(),
        }
    }

    #[cfg(feature = "ros")]
    pub fn insert_message(
        &mut self,
        message: &TFMessage,
        is_static: bool,
    ) -> Result<(), TransformBufferError> {
        for transform in &message.transforms {
            self.insert(transform, is_static)?;
        }
        Ok(())
    }

    #[cfg(feature = "ros")]
    pub fn insert(
        &mut self,
        message: &TransformStamped,
        is_static: bool,
    ) -> Result<(), TransformBufferError> {
        let isometry = transform_stamped_to_isometry(message)?;
        let stamp_ns = nanoseconds_from_ros(message.header.stamp)?;
        self.insert_isometry(
            &message.header.frame_id,
            &message.child_frame_id,
            &isometry,
            stamp_ns,
            is_static,
        )
    }

    pub fn insert_isometry(
        &mut self,
        parent: &str,
        child: &str,
        isometry: &Isometry3<f64>,
        stamp_ns: u128,
        is_static: bool,
    ) -> Result<(), TransformBufferError> {
        let parent = normalize_frame(parent)?;
        let child = normalize_frame(child)?;
        if parent == child {
            return Err(TransformBufferError::InvalidFrame {
                frame: child,
                reason: "a frame cannot be its own parent",
            });
        }
        validate_isometry(isometry)?;
        let timestamp = if is_static {
            Timestamp::zero()
        } else if stamp_ns == 0 {
            return Err(TransformBufferError::ZeroDynamicTimestamp { parent, child });
        } else {
            Timestamp::from_nanos(stamp_ns)
        };
        let quaternion = isometry.rotation.quaternion();
        let transform = BufferedTransform {
            translation: TransformVector3::new(
                isometry.translation.x,
                isometry.translation.y,
                isometry.translation.z,
            ),
            rotation: TransformQuaternion::new(
                quaternion.w,
                quaternion.i,
                quaternion.j,
                quaternion.k,
            ),
            timestamp,
            parent: parent.clone(),
            child: child.clone(),
        };
        self.registry
            .add_transform(transform)
            .map_err(|source| TransformBufferError::Insert {
                parent: parent.clone(),
                child: child.clone(),
                source: Box::new(source),
            })?;
        if !is_static {
            self.dynamic_children.insert(child);
        }
        Ok(())
    }

    #[cfg(feature = "ros")]
    pub fn lookup(
        &self,
        from: &str,
        to: &str,
        stamp: Time,
    ) -> Result<Isometry3<f64>, TransformBufferError> {
        self.lookup_ns(from, to, nanoseconds_from_ros(stamp)?)
    }

    pub fn lookup_ns(
        &self,
        from: &str,
        to: &str,
        stamp_ns: u128,
    ) -> Result<Isometry3<f64>, TransformBufferError> {
        let from = normalize_frame(from)?;
        let to = normalize_frame(to)?;
        let transform = self
            .registry
            .get_transform(&to, &from, Timestamp::from_nanos(stamp_ns))
            .map_err(|source| TransformBufferError::Lookup {
                from: from.clone(),
                to: to.clone(),
                source: Box::new(source),
            })?;
        buffered_transform_to_isometry(&transform)
    }

    #[cfg(feature = "ros")]
    #[must_use]
    pub fn can_transform(&self, from: &str, to: &str, stamp: Time) -> bool {
        self.lookup(from, to, stamp).is_ok()
    }

    #[must_use]
    pub fn can_transform_ns(&self, from: &str, to: &str, stamp_ns: u128) -> bool {
        self.lookup_ns(from, to, stamp_ns).is_ok()
    }

    pub fn clear_dynamic(&mut self) {
        for child in self.dynamic_children.drain() {
            self.registry.remove_frame(&child);
        }
    }
}

#[cfg(feature = "ros")]
pub fn transform_stamped_to_isometry(
    message: &TransformStamped,
) -> Result<Isometry3<f64>, TransformBufferError> {
    transform_to_isometry(&message.transform)
}

#[cfg(feature = "ros")]
pub fn isometry_to_transform_stamped(
    isometry: &Isometry3<f64>,
    parent: &str,
    child: &str,
    stamp: Time,
) -> Result<TransformStamped, TransformBufferError> {
    let parent = normalize_frame(parent)?;
    let child = normalize_frame(child)?;
    if parent == child {
        return Err(TransformBufferError::InvalidFrame {
            frame: child,
            reason: "a frame cannot be its own parent",
        });
    }
    let quaternion = isometry.rotation.quaternion();
    Ok(TransformStamped {
        header: Header {
            stamp,
            frame_id: parent,
        },
        child_frame_id: child,
        transform: Transform {
            translation: Vector3Msg {
                x: isometry.translation.x,
                y: isometry.translation.y,
                z: isometry.translation.z,
            },
            rotation: QuaternionMsg {
                x: quaternion.i,
                y: quaternion.j,
                z: quaternion.k,
                w: quaternion.w,
            },
        },
    })
}

#[must_use]
pub fn isometry_to_matrix4(isometry: &Isometry3<f64>) -> Matrix4<f64> {
    isometry.to_homogeneous()
}

pub fn matrix4_to_isometry(matrix: &Matrix4<f64>) -> Result<Isometry3<f64>, TransformBufferError> {
    if !matrix.iter().all(|value| value.is_finite()) {
        return Err(TransformBufferError::NonFinite);
    }
    if matrix[(3, 0)].abs() > MATRIX_TOLERANCE
        || matrix[(3, 1)].abs() > MATRIX_TOLERANCE
        || matrix[(3, 2)].abs() > MATRIX_TOLERANCE
        || (matrix[(3, 3)] - 1.0).abs() > MATRIX_TOLERANCE
    {
        return Err(TransformBufferError::NonRigidMatrix);
    }
    let rotation_matrix: Matrix3<f64> = matrix.fixed_view::<3, 3>(0, 0).into_owned();
    let orthogonality = rotation_matrix.transpose() * rotation_matrix - Matrix3::identity();
    if orthogonality.norm() > MATRIX_TOLERANCE
        || (rotation_matrix.determinant() - 1.0).abs() > MATRIX_TOLERANCE
    {
        return Err(TransformBufferError::NonRigidMatrix);
    }
    let rotation = Rotation3::from_matrix_unchecked(rotation_matrix);
    Ok(Isometry3::from_parts(
        Translation3::new(matrix[(0, 3)], matrix[(1, 3)], matrix[(2, 3)]),
        UnitQuaternion::from_rotation_matrix(&rotation),
    ))
}

#[cfg(feature = "ros")]
pub fn matrix4_to_transform_stamped(
    matrix: &Matrix4<f64>,
    parent: &str,
    child: &str,
    stamp: Time,
) -> Result<TransformStamped, TransformBufferError> {
    isometry_to_transform_stamped(&matrix4_to_isometry(matrix)?, parent, child, stamp)
}

#[cfg(feature = "ros")]
fn transform_to_isometry(transform: &Transform) -> Result<Isometry3<f64>, TransformBufferError> {
    isometry_from_components(
        [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ],
        [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ],
    )
}

pub fn isometry_from_components(
    translation: [f64; 3],
    rotation_xyzw: [f64; 4],
) -> Result<Isometry3<f64>, TransformBufferError> {
    let values = [
        translation[0],
        translation[1],
        translation[2],
        rotation_xyzw[0],
        rotation_xyzw[1],
        rotation_xyzw[2],
        rotation_xyzw[3],
    ];
    if !values.iter().all(|value| value.is_finite()) {
        return Err(TransformBufferError::NonFinite);
    }
    let quaternion = Quaternion::new(
        rotation_xyzw[3],
        rotation_xyzw[0],
        rotation_xyzw[1],
        rotation_xyzw[2],
    );
    let norm = quaternion.norm();
    if (norm - 1.0).abs() > transforms::geometry::Transform::<Timestamp>::UNIT_NORM_TOLERANCE {
        return Err(TransformBufferError::NonUnitQuaternion { norm });
    }
    Ok(Isometry3::from_parts(
        Translation3::new(translation[0], translation[1], translation[2]),
        UnitQuaternion::new_normalize(quaternion),
    ))
}

fn buffered_transform_to_isometry(
    transform: &BufferedTransform<Timestamp>,
) -> Result<Isometry3<f64>, TransformBufferError> {
    isometry_from_components(
        [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ],
        [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ],
    )
}

fn validate_isometry(isometry: &Isometry3<f64>) -> Result<(), TransformBufferError> {
    let quaternion = isometry.rotation.quaternion();
    let values = [
        isometry.translation.x,
        isometry.translation.y,
        isometry.translation.z,
        quaternion.i,
        quaternion.j,
        quaternion.k,
        quaternion.w,
    ];
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(TransformBufferError::NonFinite)
    }
}

pub(crate) fn normalize_frame(frame: &str) -> Result<String, TransformBufferError> {
    let normalized = frame.trim_start_matches('/');
    if normalized.is_empty() {
        return Err(TransformBufferError::InvalidFrame {
            frame: frame.to_owned(),
            reason: "frame name must not be empty",
        });
    }
    if normalized.contains('/') {
        return Err(TransformBufferError::InvalidFrame {
            frame: frame.to_owned(),
            reason: "frame name must be unqualified",
        });
    }
    Ok(normalized.to_owned())
}

#[cfg(feature = "ros")]
fn nanoseconds_from_ros(stamp: Time) -> Result<u128, TransformBufferError> {
    if stamp.sec < 0 {
        return Err(TransformBufferError::InvalidTimestamp {
            sec: stamp.sec,
            nanosec: stamp.nanosec,
            reason: "seconds must be non-negative",
        });
    }
    if stamp.nanosec >= NANOSECONDS_PER_SECOND_U32 {
        return Err(TransformBufferError::InvalidTimestamp {
            sec: stamp.sec,
            nanosec: stamp.nanosec,
            reason: "nanoseconds must be less than one second",
        });
    }
    let seconds =
        u128::try_from(stamp.sec).map_err(|_| TransformBufferError::InvalidTimestamp {
            sec: stamp.sec,
            nanosec: stamp.nanosec,
            reason: "seconds must be non-negative",
        })?;
    Ok(seconds * NANOSECONDS_PER_SECOND + u128::from(stamp.nanosec))
}

#[cfg(all(test, feature = "ros"))]
mod tests {
    use super::*;
    use nalgebra::{UnitQuaternion, Vector3};

    fn stamp(sec: i32, nanosec: u32) -> Time {
        Time { sec, nanosec }
    }

    fn transform(parent: &str, child: &str, sec: i32, x: f64) -> TransformStamped {
        TransformStamped {
            header: Header {
                stamp: stamp(sec, 0),
                frame_id: parent.to_owned(),
            },
            child_frame_id: child.to_owned(),
            transform: Transform {
                translation: Vector3Msg { x, y: 0.0, z: 0.0 },
                rotation: QuaternionMsg {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    }

    #[test]
    fn lookup_uses_from_to_to_direction_and_inverse() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(10));
        buffer
            .insert(&transform("field", "odom", 0, 1.0), true)
            .unwrap();
        buffer
            .insert(&transform("odom", "base_link", 1, 2.0), false)
            .unwrap();

        let base_to_field = buffer.lookup("base_link", "field", stamp(1, 0)).unwrap();
        let field_to_base = buffer.lookup("field", "base_link", stamp(1, 0)).unwrap();
        assert!((base_to_field.translation.x - 3.0).abs() < 1e-12);
        assert!((field_to_base.translation.x + 3.0).abs() < 1e-12);
        let identity = base_to_field * field_to_base;
        assert!(identity.translation.vector.norm() < 1e-12);
        assert!(identity.rotation.angle() < 1e-12);
    }

    #[test]
    fn chains_through_a_common_ancestor() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(10));
        buffer
            .insert(&transform("root", "left", 1, 2.0), false)
            .unwrap();
        buffer
            .insert(&transform("root", "right", 1, -3.0), false)
            .unwrap();
        let left_to_right = buffer.lookup("left", "right", stamp(1, 0)).unwrap();
        assert!((left_to_right.translation.x - 5.0).abs() < 1e-12);
    }

    #[test]
    fn interpolates_translation_and_rotation() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(10));
        let mut first = transform("odom", "base_link", 1, 0.0);
        let mut second = transform("odom", "base_link", 3, 2.0);
        let end_rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI);
        let quaternion = end_rotation.quaternion();
        second.transform.rotation = QuaternionMsg {
            x: quaternion.i,
            y: quaternion.j,
            z: quaternion.k,
            w: quaternion.w,
        };
        first.transform.rotation.w = 1.0;
        buffer.insert(&first, false).unwrap();
        buffer.insert(&second, false).unwrap();

        let midpoint = buffer.lookup("base_link", "odom", stamp(2, 0)).unwrap();
        assert!((midpoint.translation.x - 1.0).abs() < 1e-12);
        assert!((midpoint.rotation.angle() - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn rejects_extrapolation_both_directions() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(10));
        buffer
            .insert(&transform("odom", "base_link", 2, 0.0), false)
            .unwrap();
        buffer
            .insert(&transform("odom", "base_link", 3, 1.0), false)
            .unwrap();
        assert!(buffer.lookup("base_link", "odom", stamp(1, 0)).is_err());
        assert!(buffer.lookup("base_link", "odom", stamp(4, 0)).is_err());
    }

    #[test]
    fn static_transform_is_valid_at_any_timestamp() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(1));
        buffer
            .insert(&transform("base_link", "torso", 99, 1.0), true)
            .unwrap();
        assert!(buffer.lookup("torso", "base_link", stamp(1, 0)).is_ok());
        assert!(buffer.lookup("torso", "base_link", stamp(100, 0)).is_ok());
    }

    #[test]
    fn rejects_invalid_tree_and_transform_inputs() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(10));
        buffer
            .insert(&transform("odom", "base_link", 1, 0.0), false)
            .unwrap();
        assert!(
            buffer
                .insert(&transform("field", "base_link", 2, 0.0), false)
                .is_err()
        );
        assert!(
            buffer
                .insert(&transform("base_link", "odom", 2, 0.0), false)
                .is_err()
        );
        assert!(
            buffer
                .insert(&transform("same", "same", 1, 0.0), false)
                .is_err()
        );
        assert!(
            buffer
                .insert(&transform("robot/base", "camera", 1, 0.0), false)
                .is_err()
        );

        let mut bad_rotation = transform("odom", "sensor", 1, 0.0);
        bad_rotation.transform.rotation.w = 2.0;
        assert!(buffer.insert(&bad_rotation, false).is_err());
    }

    #[test]
    fn rejects_static_dynamic_conflict_and_bad_timestamps() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(10));
        buffer
            .insert(&transform("base", "sensor", 0, 0.0), true)
            .unwrap();
        assert!(
            buffer
                .insert(&transform("base", "sensor", 1, 0.0), false)
                .is_err()
        );
        assert!(matches!(
            buffer.insert(&transform("odom", "base_link", 0, 0.0), false),
            Err(TransformBufferError::ZeroDynamicTimestamp { .. })
        ));
        let mut invalid = transform("odom", "base_link", 1, 0.0);
        invalid.header.stamp.nanosec = 1_000_000_000;
        assert!(matches!(
            buffer.insert(&invalid, false),
            Err(TransformBufferError::InvalidTimestamp { .. })
        ));
        let negative = transform("odom", "negative_stamp", -1, 0.0);
        assert!(matches!(
            buffer.insert(&negative, false),
            Err(TransformBufferError::InvalidTimestamp { .. })
        ));

        let subsecond = TransformStamped {
            header: Header {
                stamp: stamp(0, 1),
                frame_id: "odom".to_owned(),
            },
            child_frame_id: "subsecond".to_owned(),
            ..transform("odom", "subsecond", 0, 0.0)
        };
        assert!(buffer.insert(&subsecond, false).is_ok());

        let mut invalid_static = transform("base", "invalid_static", 0, 0.0);
        invalid_static.header.stamp.nanosec = 1_000_000_000;
        assert!(matches!(
            buffer.insert(&invalid_static, true),
            Err(TransformBufferError::InvalidTimestamp { .. })
        ));
    }

    #[test]
    fn clear_dynamic_preserves_static_edges() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(10));
        buffer
            .insert(&transform("base_link", "torso", 0, 1.0), true)
            .unwrap();
        buffer
            .insert(&transform("odom", "base_link", 1, 2.0), false)
            .unwrap();
        buffer.clear_dynamic();
        assert!(buffer.lookup("torso", "base_link", stamp(1, 0)).is_ok());
        assert!(buffer.lookup("base_link", "odom", stamp(1, 0)).is_err());
    }

    #[test]
    fn message_and_matrix_conversions_roundtrip() {
        let original = Isometry3::from_parts(
            Translation3::new(1.0, 2.0, 3.0),
            UnitQuaternion::from_euler_angles(0.1, -0.2, 0.3),
        );
        let message =
            isometry_to_transform_stamped(&original, "odom", "base_link", stamp(1, 2)).unwrap();
        let from_message = transform_stamped_to_isometry(&message).unwrap();
        let matrix = isometry_to_matrix4(&from_message);
        let from_matrix = matrix4_to_isometry(&matrix).unwrap();
        assert!((original.translation.vector - from_matrix.translation.vector).norm() < 1e-12);
        assert!((original.rotation.inverse() * from_matrix.rotation).angle() < 1e-12);

        let point = Vector3::new(0.4, -0.5, 0.6);
        assert!((original * point - from_matrix * point).norm() < 1e-12);
    }
}
