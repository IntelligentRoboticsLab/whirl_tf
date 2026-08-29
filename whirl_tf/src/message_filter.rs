use std::collections::VecDeque;

use crate::buffer::{TransformBuffer, TransformBufferError, normalize_frame};

/// Why a stamped message was rejected by [`MessageFilter`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FilterFailureReason {
    /// The message's source frame was empty.
    EmptyFrameId,
    /// The message's source frame was not a valid whirl-tf frame name.
    InvalidFrameId,
    /// The bounded queue was full, so its oldest message was discarded.
    QueueFull,
}

/// A message removed from a filter without being delivered.
#[derive(Debug)]
pub struct DroppedMessage<M> {
    pub message: M,
    pub reason: FilterFailureReason,
}

#[derive(Debug)]
struct QueuedMessage<M> {
    message: M,
    source_frame: String,
    stamp_ns: u128,
}

/// Holds stamped messages until their required transforms are available.
///
/// Messages are delivered in insertion order. In particular, a later message
/// cannot overtake an earlier message whose transform is still unavailable.
/// This makes the result independent of whether a data callback or transform
/// callback happened to run first.
#[derive(Debug)]
pub struct MessageFilter<M> {
    target_frames: Vec<String>,
    queue_size: usize,
    tolerance_ns: u128,
    messages: VecDeque<QueuedMessage<M>>,
}

impl<M> MessageFilter<M> {
    /// Creates a filter for one target frame.
    ///
    /// A `queue_size` of zero makes the queue unbounded.
    pub fn new(target_frame: &str, queue_size: usize) -> Result<Self, TransformBufferError> {
        let mut filter = Self {
            target_frames: Vec::new(),
            queue_size,
            tolerance_ns: 0,
            messages: VecDeque::new(),
        };
        filter.set_target_frames([target_frame])?;
        Ok(filter)
    }

    /// Replaces the frames that must all be reachable before delivery.
    pub fn set_target_frames<I, S>(&mut self, target_frames: I) -> Result<(), TransformBufferError>
    where
        I: IntoIterator<Item = S>,
        S: AsRef<str>,
    {
        let target_frames = target_frames
            .into_iter()
            .map(|frame| normalize_frame(frame.as_ref()))
            .collect::<Result<Vec<_>, _>>()?;
        if target_frames.is_empty() {
            return Err(TransformBufferError::InvalidFrame {
                frame: String::new(),
                reason: "at least one target frame is required",
            });
        }
        self.target_frames = target_frames;
        Ok(())
    }

    /// Replaces the single target frame.
    pub fn set_target_frame(&mut self, target_frame: &str) -> Result<(), TransformBufferError> {
        self.set_target_frames([target_frame])
    }

    /// Requires transforms at both the message timestamp and this much later.
    pub fn set_tolerance_ns(&mut self, tolerance_ns: u128) {
        self.tolerance_ns = tolerance_ns;
    }

    /// Changes the maximum number of waiting messages. Zero is unbounded.
    pub fn set_queue_size(&mut self, queue_size: usize) -> Vec<DroppedMessage<M>> {
        self.queue_size = queue_size;
        self.enforce_queue_size()
    }

    /// Adds a stamped message and reports any message discarded as a result.
    pub fn add(
        &mut self,
        message: M,
        source_frame: &str,
        stamp_ns: u128,
    ) -> Vec<DroppedMessage<M>> {
        if source_frame.trim_start_matches('/').is_empty() {
            return vec![DroppedMessage {
                message,
                reason: FilterFailureReason::EmptyFrameId,
            }];
        }
        let Ok(source_frame) = normalize_frame(source_frame) else {
            return vec![DroppedMessage {
                message,
                reason: FilterFailureReason::InvalidFrameId,
            }];
        };
        self.messages.push_back(QueuedMessage {
            message,
            source_frame,
            stamp_ns,
        });
        self.enforce_queue_size()
    }

    /// Removes and returns the FIFO prefix whose transforms are available.
    pub fn drain_ready(&mut self, buffer: &TransformBuffer) -> Vec<M> {
        let mut ready = Vec::new();
        while self.messages.front().is_some_and(|message| {
            self.target_frames.iter().all(|target| {
                buffer.can_transform_ns(&message.source_frame, target, message.stamp_ns)
                    && (self.tolerance_ns == 0
                        || message.stamp_ns.checked_add(self.tolerance_ns).is_some_and(
                            |end_stamp| {
                                buffer.can_transform_ns(&message.source_frame, target, end_stamp)
                            },
                        ))
            })
        }) {
            let message = self
                .messages
                .pop_front()
                .expect("the queue front was checked above");
            ready.push(message.message);
        }
        ready
    }

    /// Discards every waiting message without reporting failures.
    pub fn clear(&mut self) {
        self.messages.clear();
    }

    #[must_use]
    pub fn pending_count(&self) -> usize {
        self.messages.len()
    }

    #[must_use]
    pub fn queue_size(&self) -> usize {
        self.queue_size
    }

    #[must_use]
    pub fn tolerance_ns(&self) -> u128 {
        self.tolerance_ns
    }

    #[must_use]
    pub fn target_frames(&self) -> &[String] {
        &self.target_frames
    }

    fn enforce_queue_size(&mut self) -> Vec<DroppedMessage<M>> {
        let mut dropped = Vec::new();
        while self.queue_size != 0 && self.messages.len() > self.queue_size {
            let message = self
                .messages
                .pop_front()
                .expect("an oversized queue cannot be empty");
            dropped.push(DroppedMessage {
                message: message.message,
                reason: FilterFailureReason::QueueFull,
            });
        }
        dropped
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use nalgebra::Isometry3;

    use super::*;

    fn insert(buffer: &mut TransformBuffer, stamp_ns: u128) {
        buffer
            .insert_isometry("map", "sensor", &Isometry3::identity(), stamp_ns, false)
            .unwrap();
    }

    #[test]
    fn waits_for_a_transform_inserted_after_the_message() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(10));
        let mut filter = MessageFilter::new("map", 10).unwrap();
        assert!(filter.add("scan", "sensor", 2).is_empty());
        assert!(filter.drain_ready(&buffer).is_empty());

        insert(&mut buffer, 2);
        assert_eq!(filter.drain_ready(&buffer), ["scan"]);
    }

    #[test]
    fn immediately_delivers_when_the_transform_arrived_first() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(10));
        insert(&mut buffer, 2);
        let mut filter = MessageFilter::new("map", 10).unwrap();
        filter.add("scan", "sensor", 2);
        assert_eq!(filter.drain_ready(&buffer), ["scan"]);
    }

    #[test]
    fn preserves_fifo_order_when_a_later_message_is_ready_first() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(20));
        insert(&mut buffer, 1);
        insert(&mut buffer, 3);
        let mut filter = MessageFilter::new("map", 10).unwrap();
        filter.add("future", "sensor", 10);
        filter.add("ready", "sensor", 2);
        assert!(filter.drain_ready(&buffer).is_empty());

        insert(&mut buffer, 11);
        assert_eq!(filter.drain_ready(&buffer), ["future", "ready"]);
    }

    #[test]
    fn drops_the_oldest_message_when_the_queue_is_full() {
        let mut filter = MessageFilter::new("map", 1).unwrap();
        filter.add("first", "sensor", 1);
        let dropped = filter.add("second", "sensor", 2);
        assert_eq!(dropped.len(), 1);
        assert_eq!(dropped[0].message, "first");
        assert_eq!(dropped[0].reason, FilterFailureReason::QueueFull);
    }

    #[test]
    fn tolerance_requires_the_end_of_the_interval() {
        let mut buffer = TransformBuffer::new(Duration::from_secs(10));
        insert(&mut buffer, 1_000_000_000);
        let mut filter = MessageFilter::new("map", 10).unwrap();
        filter.set_tolerance_ns(200_000_000);
        filter.add("scan", "sensor", 1_000_000_000);
        assert!(filter.drain_ready(&buffer).is_empty());

        insert(&mut buffer, 2_000_000_000);
        assert_eq!(filter.drain_ready(&buffer), ["scan"]);
    }

    #[test]
    fn rejects_empty_and_invalid_source_frames() {
        let mut filter = MessageFilter::new("map", 10).unwrap();
        let empty = filter.add("empty", "", 1);
        assert_eq!(empty[0].reason, FilterFailureReason::EmptyFrameId);
        let invalid = filter.add("invalid", "robot/sensor", 1);
        assert_eq!(invalid[0].reason, FilterFailureReason::InvalidFrameId);
        assert_eq!(filter.pending_count(), 0);
    }
}
