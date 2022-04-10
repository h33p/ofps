//! # Motion estimator built on homography estimation.
//!
//! This estimator uses OpenCV's implementation of the algorithm for finding homography matrix,
//! which allows to retrieve camera rotation.

use nalgebra as na;
use ofps::prelude::v1::*;
use opencv::calib3d::{decompose_homography_mat, find_homography_ext, LMEDS, RANSAC};
use opencv::core::*;

ofps::define_descriptor!(homography, Estimator, |_| Ok(Box::new(
    HomographyEstimator::default()
)));

/// Libmv based camera estimator.
pub struct HomographyEstimator {
    desired_confidence: f32,
    max_error: f32,
    max_iters: usize,
    use_ransac: bool,
}

impl Properties for HomographyEstimator {
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
            (
                "Max iters",
                PropertyMut::usize(&mut self.max_iters, 1, 5000),
            ),
            ("Use ransac", PropertyMut::Bool(&mut self.use_ransac)),
        ]
    }
}

impl HomographyEstimator {
    pub fn desired_confidence(self, desired_confidence: f32) -> Self {
        Self {
            desired_confidence,
            ..self
        }
    }

    pub fn max_error(self, max_error: f32) -> Self {
        Self { max_error, ..self }
    }

    pub fn max_iters(self, max_iters: usize) -> Self {
        Self { max_iters, ..self }
    }

    pub fn use_ransac(self, use_ransac: bool) -> Self {
        Self { use_ransac, ..self }
    }
}

impl Default for HomographyEstimator {
    fn default() -> Self {
        Self {
            desired_confidence: 0.997,
            max_error: 0.001,
            max_iters: 2000,
            use_ransac: true,
        }
    }
}

impl HomographyEstimator {
    fn homography(
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
        let cam_matrix = na::Matrix3::from_iterator(cam_matrix.into_iter().map(|v| *v as f64));

        let cam_matrix = Mat::from_slice_2d(&[
            cam_matrix.column(0).as_slice(),
            cam_matrix.column(1).as_slice(),
            cam_matrix.column(2).as_slice(),
        ])?;

        let mut inliers = Mat::default();

        let method = if self.use_ransac { RANSAC } else { LMEDS };

        let homography = find_homography_ext(
            &p1,
            &p2,
            method,
            self.max_error as _,
            &mut inliers,
            self.max_iters as _,
            self.desired_confidence as _,
        )?;

        Ok((homography, p1, p2, cam_matrix, inliers))
    }
}

impl Estimator for HomographyEstimator {
    fn estimate(
        &mut self,
        motion_vectors: &[MotionEntry],
        camera: &StandardCamera,
        _: Option<f32>,
    ) -> Result<(na::UnitQuaternion<f32>, na::Vector3<f32>)> {
        let (h, _, _, cam_matrix, _) = self.homography(motion_vectors.iter().copied(), camera)?;

        let mut r: Vector<Mat> = Default::default();
        let mut t: Vector<Mat> = Default::default();
        let mut n: Vector<Mat> = Default::default();

        decompose_homography_mat(&h, &cam_matrix, &mut r, &mut t, &mut n)?;

        let (r, _) = r
            .iter()
            .zip(t.iter())
            .fold(None, |cr, (r, t)| {
                let dot = t.dot(&t).unwrap_or(0.0);
                match cr {
                    Some((cr, m)) if m < dot => Some((cr, m)),
                    _ => Some((r, dot)),
                }
            })
            .unwrap();

        let r = na::Matrix3::from_iterator(r.iter::<f64>()?.map(|(_, v)| v as _));

        // Swap Y and Z axis in the rotation matrix to be line-in-line with
        // the rest of the codebase.
        let r = na::UnitQuaternion::from_matrix(&r).inverse();
        let (x, z, y) = r.euler_angles();
        let r = na::UnitQuaternion::from_euler_angles(x * -1.0, y * -1.0, z);
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

        let mut grid = get_grid(50, 50, &camera);

        for rot in angles.iter().map(|&(r, p, y)| {
            na::UnitQuaternion::from_euler_angles(r.to_radians(), p.to_radians(), y.to_radians())
        }) {
            println!("NEWWWWWWWWWW::::");
            println!("ROT: {:?}", rot.euler_angles());
            for _ in 0..1 {
                println!("ITER");
                let p1 = project_grid(
                    &grid,
                    &camera,
                    calc_view(Default::default(), Default::default()),
                );
                //println!("{:?}", p1);
                let p2 = project_grid(&grid, &camera, calc_view(rot, Default::default()));
                //println!("{:?}", p2);

                let field = calc_field(p1, p2);

                let mut estimator = HomographyEstimator::default();
                let (r, tr) = estimator.estimate(&field, &camera, None).unwrap();

                println!("ROTATION: {:?}", r.euler_angles());
                println!("DELTA: {:?}", rot.angle_to(&r).to_degrees());
                println!("TR: {:?}", tr);
            }
        }
    }
}
