use nalgebra as na;

use crate::prelude::v1::*;

pub trait Estimator {
    fn estimate(
        &mut self,
        motion_vectors: &[MotionEntry],
        camera: &StandardCamera,
    ) -> Result<(na::UnitQuaternion<f32>, na::Vector3<f32>)>;

    fn motion_step(
        &mut self,
        motion_vectors: &[MotionEntry],
        camera: &StandardCamera,
        rot: &mut na::UnitQuaternion<f32>,
        pos: &mut na::Point3<f32>,
    ) -> Result<()>
    where
        Self: Sized,
    {
        let (r, tr) = self.estimate(motion_vectors, camera)?;
        *pos += *rot * tr;
        *rot = r * *rot;
        Ok(())
    }
}
