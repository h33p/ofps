use crate::prelude::v1::*;

pub trait Decoder {
    /// Process a single frame in the stream.
    ///
    /// This function will take in a single frame, and attempt extracting motion
    /// vectors from it. If the frame contains MVs, `Ok(true)` is returned, and `field`
    /// gets updated. If there are no motion vectors, `Ok(false)` is returned, and if
    /// there is an error while processing, `Err` is returned.
    fn process_frame(&mut self, field: &mut MotionField) -> Result<bool>;

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
