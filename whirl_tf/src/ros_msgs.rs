//! Minimal ROS 2 message types used by the optional ROS transform API.
//!
//! This crate only needs the small transform-related subset of the ROS 2
//! interface definitions, so it keeps equivalent data structures without a
//! ROS installation.

pub mod builtin_interfaces {
    /// A ROS timestamp.
    #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
    pub struct Time {
        pub sec: i32,
        pub nanosec: u32,
    }
}

pub mod std_msgs {
    use super::builtin_interfaces::Time;

    /// The part of a ROS header needed by transform messages.
    #[derive(Clone, Debug, Default, PartialEq, Eq)]
    pub struct Header {
        pub stamp: Time,
        pub frame_id: String,
    }
}

pub mod geometry_msgs {
    use super::std_msgs::Header;

    /// A three-dimensional vector.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    pub struct Vector3 {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    /// A quaternion in ROS x/y/z/w field order.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    pub struct Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    /// A rigid transform.
    #[derive(Clone, Copy, Debug, Default, PartialEq)]
    pub struct Transform {
        pub translation: Vector3,
        pub rotation: Quaternion,
    }

    /// A stamped transform between two named frames.
    #[derive(Clone, Debug, Default, PartialEq)]
    pub struct TransformStamped {
        pub header: Header,
        pub child_frame_id: String,
        pub transform: Transform,
    }
}

pub mod tf2_msgs {
    use super::geometry_msgs::TransformStamped;

    /// A collection of stamped transforms.
    #[derive(Clone, Debug, Default, PartialEq)]
    pub struct TFMessage {
        pub transforms: Vec<TransformStamped>,
    }
}
