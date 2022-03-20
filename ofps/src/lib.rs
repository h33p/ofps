//! # Optical Flow Processing Library
//!
//! This library provides methods and a framework for processing optical flow fields. There are
//! motion tracking, detection and estimation traits available.
//!
//! The easiest way to use the library is to import its prelude:
//!
//! ```
//! use ofps::prelude::v1::*;
//! ```
//!
//! You may need [`nalgebra`](https://crates.io/crates/nalgebra) to make use of the functionality.

pub mod camera;
pub mod decoder;
pub mod detection;
pub mod estimator;
pub mod motion_field;
#[cfg(feature = "plugins")]
pub mod plugins;
pub mod utils;

pub mod prelude {
    pub mod v1 {
        pub use crate::{
            camera::*,
            decoder::{Decoder, MotionEntry, MotionVectors, RGBA},
            detection::BlockMotionDetection,
            estimator::Estimator,
            motion_field::{MotionField, MotionFieldDensifier},
        };
        #[cfg(feature = "plugins")]
        pub use crate::{
            define_descriptor,
            plugins::{DecoderPlugin, EstimatorPlugin, PluginStore},
        };
        pub use anyhow::{anyhow, Error, Result};
        pub use ptrplus;
    }
}

pub use cglue;
pub use paste;
