use thiserror::Error;

/// A frame did not match the frame required by an interface.
#[derive(Debug, Error, PartialEq, Eq)]
#[error("{interface} requires frame {expected:?}, got {actual:?}")]
pub struct FrameError {
    pub actual: String,
    pub expected: String,
    pub interface: String,
}

/// Checks that a frame matches the frame required by an interface.
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
    fn accepts_the_required_frame() {
        assert_eq!(require_frame("sensor", "sensor", "/measurements"), Ok(()));
    }

    #[test]
    fn error_names_the_interface_and_frames() {
        let error = require_frame("body", "robot", "/state").unwrap_err();
        assert_eq!(error.actual, "body");
        assert_eq!(error.expected, "robot");
        assert_eq!(error.interface, "/state");
    }
}
