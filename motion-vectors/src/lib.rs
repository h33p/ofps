pub mod decoder;
pub mod motion_field;
pub mod motion_model;
pub mod result;
pub mod utils;

pub mod prelude {
    pub mod v1 {
        pub use crate::{
            decoder::{Decoder, RGBA},
            motion_field::{DownscaleMotionField, MotionField},
            motion_model::*,
            result::Result,
        };
        pub use ptrplus;
    }
}
