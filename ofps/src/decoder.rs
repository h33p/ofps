//! # Motion field decoding

use crate::prelude::v1::*;
use bytemuck::{Pod, Zeroable};
use nalgebra as na;

/// RGBA colour structure.
#[repr(C)]
#[derive(Clone, Copy, Debug, Pod, Zeroable)]
pub struct RGBA {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl RGBA {
    /// Convert from a slice containing `[r, g, b]` elements.
    pub fn from_rgb_slice(rgb: &[u8]) -> Self {
        Self {
            r: rgb[0],
            g: rgb[1],
            b: rgb[2],
            a: 255,
        }
    }

    /// Convert from a slice containing [r, g, b, a] elements.
    pub fn from_rgba_slice(rgba: &[u8]) -> Self {
        Self {
            r: rgba[0],
            g: rgba[1],
            b: rgba[2],
            a: rgba[3],
        }
    }
}

/// Pair containing coordinates and motion at them.
pub type MotionEntry = (na::Point2<f32>, na::Vector2<f32>);
/// Vector of `MotionEntry` elements.
pub type MotionVectors = Vec<MotionEntry>;

/// Optical flow decoder.
pub trait Decoder {
    /// Process a single frame in the stream.
    ///
    /// This function will take in a single frame, and attempt extracting motion
    /// vectors from it. If the frame contains MVs, `Ok(true)` is returned, and `field`
    /// gets updated. If there are no motion vectors, `Ok(false)` is returned, and if
    /// there is an error while processing, `Err` is returned.
    ///
    /// If the decoder supports it, `out_frame` may also be written at either of the `Ok` cases.
    fn process_frame(
        &mut self,
        field: &mut MotionVectors,
        out_frame: Option<(&mut Vec<RGBA>, &mut usize)>,
        skip_frames: usize,
    ) -> Result<bool>;

    /// Get the framerate of the stream.
    ///
    /// This will return `Some(framerate)` if it is known. On realtime streams it may
    /// not always be known. In such cases, `None` is returned.
    fn get_framerate(&self) -> Option<f64>;

    /// Get aspect ratio of the stream.
    ///
    /// This will return `Some((width, height))` if the aspect ratio is known. On rare
    /// cases it may not be known, which will then return `None`. Aspect ratio may change
    /// after the first frame is processed.
    fn get_aspect(&self) -> Option<(usize, usize)>;
}
