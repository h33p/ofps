pub mod camera;
pub mod decoder;
pub mod estimator;
pub mod motion_field;
pub mod motion_model;
pub mod reconstruct;
pub mod result;
pub mod utils;

pub mod prelude {
    pub mod v1 {
        pub use crate::{
            camera::*,
            decoder::{Decoder, MotionEntry, MotionVectors, RGBA},
            estimator::Estimator,
            motion_field::{DownscaleMotionField, MotionField},
            motion_model::*,
            reconstruct,
            result::Result,
        };
        pub use ptrplus;
    }
}
