//! Canonical names for frames published in the Phase 1 transform tree.

use thiserror::Error;

pub const FIELD: &str = "field";
pub const ODOM: &str = "odom";
pub const BASE_LINK: &str = "base_link";
pub const BASE_FOOTPRINT: &str = "base_footprint";
pub const TORSO: &str = "torso";
pub const GAZE: &str = "gaze";
pub const LEFT_CAMERA_LINK: &str = "left_camera_link";
pub const LEFT_CAMERA_OPTICAL_FRAME: &str = "left_camera_optical_frame";
pub const LEFT_SOLE: &str = "l_sole";
pub const RIGHT_SOLE: &str = "r_sole";

pub const ALL: &[&str] = &[
    FIELD,
    ODOM,
    BASE_LINK,
    BASE_FOOTPRINT,
    TORSO,
    GAZE,
    LEFT_CAMERA_LINK,
    LEFT_CAMERA_OPTICAL_FRAME,
    LEFT_SOLE,
    RIGHT_SOLE,
];

#[derive(Debug, Error, PartialEq, Eq)]
#[error("{interface} requires frame {expected:?}, got {actual:?}")]
pub struct FrameError {
    pub actual: String,
    pub expected: String,
    pub interface: String,
}

pub fn require_frame(actual: &str, expected: &str, interface: &str) -> Result<(), FrameError> {
    if actual == expected {
        return Ok(());
    }
    Err(FrameError {
        actual: actual.to_owned(),
        expected: expected.to_owned(),
        interface: interface.to_owned(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn require_frame_names_the_interface_and_frames() {
        let error = require_frame("body", BASE_LINK, "/fall_state").unwrap_err();
        assert_eq!(error.actual, "body");
        assert_eq!(error.expected, BASE_LINK);
        assert_eq!(error.interface, "/fall_state");
    }
}
