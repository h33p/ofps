use crate::prelude::v1::*;

pub trait Decoder {
    fn next_frame(&mut self) -> Result<MotionField>;
}
