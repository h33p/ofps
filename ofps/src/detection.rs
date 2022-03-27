//! Motion detection module

use crate::prelude::v1::*;

/// Generic motion detector.
pub trait Detector {
    /// Detect motion.
    ///
    /// This function will return `None`, if there is no motion, or a filtered motion field with
    /// the number of vectors in motion.
    fn detect_motion(&self, motion: &[MotionEntry]) -> Option<(usize, MotionField)>;
}
