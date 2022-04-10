//! # Motion estimator built on the 5-point algorithm.
//!
//! This estimator uses OpenCV's implementation of the 5-point correspondance algorithm for finding
//! essential camera matrix, and attempts to keep consistent translational scale throughout the
//! invokations.

use nalgebra as na;
use ofps::prelude::v1::*;
use opencv::calib3d::{find_essential_mat_matrix, recover_pose_estimated, LMEDS, RANSAC};
use opencv::core::*;

ofps::define_descriptor!(multiview, Estimator, |_| Ok(Box::new(
    MultiviewEstimator::default()
)));

/// Libmv based camera estimator.
pub struct MultiviewEstimator {
    desired_confidence: f32,
    max_error: f32,
    use_ransac: bool,
}

impl Properties for MultiviewEstimator {
    fn props_mut(&mut self) -> Vec<(&str, PropertyMut)> {
        vec![
            (
                "Desired confidence",
                PropertyMut::float(&mut self.desired_confidence, 0.0, 1.0),
            ),
            (
                "Max error",
                PropertyMut::float(&mut self.max_error, 0.00001, 0.1),
            ),
            ("Use ransac", PropertyMut::Bool(&mut self.use_ransac)),
        ]
    }
}

impl MultiviewEstimator {
    pub fn desired_confidence(self, desired_confidence: f32) -> Self {
        Self {
            desired_confidence,
            ..self
        }
    }

    pub fn max_error(self, max_error: f32) -> Self {
        Self { max_error, ..self }
    }

    pub fn use_ransac(self, use_ransac: bool) -> Self {
        Self { use_ransac, ..self }
    }
}

impl Default for MultiviewEstimator {
    fn default() -> Self {
        Self {
            desired_confidence: 0.999,
            max_error: 0.0001,
            use_ransac: true,
        }
    }
}

impl MultiviewEstimator {
    fn essential(
        &self,
        motion: impl Iterator<Item = MotionEntry>,
        camera: &StandardCamera,
    ) -> Result<(Mat, Mat, Mat, Mat, Mat)> {
        let mut p1 = vec![];
        let mut p2 = vec![];

        for (s, e) in motion.map(|(s, m)| (s, s + m)) {
            p1.push(Point2f::new(s.x, s.y));
            p2.push(Point2f::new(e.x, e.y));
        }

        let p1 = Mat::from_slice(&*p1)?;
        let p2 = Mat::from_slice(&*p2)?;

        let cam_matrix = camera.intrinsics();

        let cam_matrix = cam_matrix.transpose();

        let cam_matrix = Mat::from_slice_2d(&[
            cam_matrix.column(0).as_slice(),
            cam_matrix.column(1).as_slice(),
            cam_matrix.column(2).as_slice(),
        ])?;

        let mut inliers = Mat::default();

        let method = if self.use_ransac { RANSAC } else { LMEDS };

        // find_essential_mat allows to specify max_iters, however,
        // it is less portable, and the API is different on debian.
        let essential = find_essential_mat_matrix(
            &p1,
            &p2,
            &cam_matrix,
            method,
            self.desired_confidence as _,
            self.max_error as _,
            &mut inliers,
        )?;

        Ok((essential, p1, p2, cam_matrix, inliers))
    }
}

impl Estimator for MultiviewEstimator {
    fn estimate(
        &mut self,
        motion_vectors: &[MotionEntry],
        camera: &StandardCamera,
        _: Option<f32>,
    ) -> Result<(na::UnitQuaternion<f32>, na::Vector3<f32>)> {
        let (e, p1, p2, cam_matrix, mut inliers) =
            self.essential(motion_vectors.iter().copied(), camera)?;

        let mut r = Mat::default();
        let mut t = Mat::default();

        recover_pose_estimated(&e, &p1, &p2, &cam_matrix, &mut r, &mut t, &mut inliers)?;

        let r = na::Matrix3::from_iterator(r.iter::<f64>()?.map(|(_, v)| v as _));

        // Swap Y and Z axis in the rotation matrix to be line-in-line with
        // the rest of the codebase.
        let r = na::UnitQuaternion::from_matrix(&r).inverse();
        let (x, z, y) = r.euler_angles();
        let r = na::UnitQuaternion::from_euler_angles(x, y, z);

        // Check if rotation is over 90 degrees. Generally, this should not be possible, but is
        // caused by opencv acting weird.
        // TODO: look into the behavior.
        let r = if r.angle() > std::f32::consts::FRAC_PI_2 {
            let (axis, angle) = r.axis_angle().unwrap();
            let new_angle = (angle + std::f32::consts::PI) % (std::f32::consts::PI * 2.0);
            na::UnitQuaternion::from_axis_angle(&axis, new_angle)
        } else {
            r
        };

        Ok((r, Default::default()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn get_grid(
        grid_cnt_x: usize,
        grid_cnt_y: usize,
        camera: &StandardCamera,
    ) -> Vec<na::Point3<f32>> {
        let mut new_points = vec![];

        for x in 1..grid_cnt_x {
            for y in 1..grid_cnt_y {
                let x = x as f32 / grid_cnt_x as f32;
                let y = y as f32 / grid_cnt_y as f32;
                new_points.push(na::Point2::new(x, y));
            }
        }

        let view = calc_view(Default::default(), Default::default());

        new_points
            .into_iter()
            .map(|p| camera.unproject(p, view))
            .collect()
    }

    fn calc_view(rot: na::UnitQuaternion<f32>, pos: na::Point3<f32>) -> na::Matrix4<f32> {
        na::Matrix4::look_at_rh(
            &pos,
            &(pos + rot.transform_vector(&na::Vector3::new(0.0, 1.0, 0.0))),
            &rot.transform_vector(&na::Vector3::new(0.0, 0.0, 1.0)),
        )
    }

    fn project_grid<'a>(
        grid: impl IntoIterator<Item = &'a na::Point3<f32>>,
        camera: &StandardCamera,
        view: na::Matrix4<f32>,
    ) -> Vec<na::Point2<f32>> {
        grid.into_iter().map(|&p| camera.project(p, view)).collect()
    }

    fn calc_field(
        p1: impl IntoIterator<Item = na::Point2<f32>>,
        p2: impl IntoIterator<Item = na::Point2<f32>>,
    ) -> Vec<MotionEntry> {
        p1.into_iter()
            .zip(p2.into_iter())
            .filter(|(p1, p2)| p1.coords.magnitude() <= 0.71 && p2.coords.magnitude() <= 0.71)
            .map(|(p1, p2)| (p1, p2 - p1))
            .collect()
    }

    #[test]
    fn test_rotation() {
        const ROT: f32 = 1.0;

        let angles = [
            (0.0, 0.0, 0.0),
            (ROT, 0.0, 0.0),
            (0.0, ROT, 0.0),
            (0.0, 0.0, ROT),
            (ROT, ROT, 0.0),
            (ROT, 0.0, ROT),
            (0.0, ROT, ROT),
            (ROT, ROT, ROT),
        ];

        let camera = StandardCamera::new(1.0, 90.0);

        let grid = get_grid(50, 50, &camera);

        for rot in angles.iter().map(|&(r, p, y)| {
            na::UnitQuaternion::from_euler_angles(r.to_radians(), p.to_radians(), y.to_radians())
        }) {
            let p1 = project_grid(
                &grid,
                &camera,
                calc_view(Default::default(), Default::default()),
            );
            let p2 = project_grid(&grid, &camera, calc_view(rot, Default::default()));

            let field = calc_field(p1, p2);

            let mut estimator = MultiviewEstimator::default();
            estimator.estimate(&field, &camera, None).unwrap();
        }
    }
}
