//! # Camera motion estimator

use nalgebra as na;

use crate::prelude::v1::*;

/// Generic camera motion estimator
pub trait Estimator {
    /// Estimate single-frame of camera motion.
    ///
    /// This function processes the next motion field and produces rotation and translation
    /// estimates. Not all estimators are stateless, thus this function expects sequential frames.
    ///
    /// # Arguments
    ///
    /// * `motion_vectors` - input optical flow motion field.
    /// * `camera` - camera to use in estimation.
    /// * `move_magnitude` - optional hint for translation magnitude.
    fn estimate(
        &mut self,
        motion_vectors: &[MotionEntry],
        camera: &StandardCamera,
        move_magnitude: Option<f32>,
    ) -> Result<(na::UnitQuaternion<f32>, na::Vector3<f32>)>;

    /// Estimate camera motion and apply it to previous motion.
    ///
    /// This function processes the next motion field and produces rotation and translation
    /// estimates. Not all estimators are stateless, thus this function expects sequential frames.
    ///
    /// # Arguments
    ///
    /// * `motion_vectors` - input optical flow motion field.
    /// * `camera` - camera to use in estimation.
    /// * `move_magnitude` - optional hint for translation magnitude.
    /// * `rot` - camera rotation to modify.
    /// * `pos` - camera position to modify.
    fn motion_step(
        &mut self,
        motion_vectors: &[MotionEntry],
        camera: &StandardCamera,
        move_magnitude: Option<f32>,
        rot: &mut na::UnitQuaternion<f32>,
        pos: &mut na::Point3<f32>,
    ) -> Result<()>
    where
        Self: Sized,
    {
        let (r, tr) = self.estimate(motion_vectors, camera, move_magnitude)?;
        *pos += *rot * tr;
        *rot = r * *rot;
        Ok(())
    }
}
