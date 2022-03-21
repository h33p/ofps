//! # Motion estimator built on the 5-point algorithm.
//!
//! This estimator uses OpenCV's implementation of the 5-point correspondance algorithm for finding
//! essential camera matrix, and attempts to keep consistent translational scale throughout the
//! invokations.

use nalgebra as na;
use ofps::prelude::v1::*;
use opencv::calib3d::{find_essential_mat, recover_pose_estimated, LMEDS, RANSAC};
use opencv::core::*;
use std::collections::BTreeMap;

ofps::define_descriptor!(multiview, Estimator, |_| Ok(Box::new(
    MultiviewEstimator::default()
)));

type Float = noisy_float::NoisyFloat<f32, noisy_float::checkers::NumChecker>;

struct PrevMotion {
    mv: BTreeMap<Float, BTreeMap<Float, MotionEntry>>,
    rot: na::UnitQuaternion<f32>,
    tr: na::Vector3<f32>,
}

impl PrevMotion {
    fn new(
        mv: impl Iterator<Item = MotionEntry>,
        rot: na::UnitQuaternion<f32>,
        tr: na::Vector3<f32>,
    ) -> Self {
        let mut s = Self {
            mv: Default::default(),
            rot,
            tr,
        };

        s.set_mv(mv);

        s
    }

    fn mv_iter(&self) -> impl Iterator<Item = MotionEntry> + '_ {
        self.mv.values().flat_map(|m| m.values().copied())
    }

    fn set_mv(&mut self, mv: impl Iterator<Item = MotionEntry>) {
        self.mv.clear();
        for (pos, motion) in mv {
            let ep = pos + motion;
            let ex = Float::unchecked_new(ep.x);
            let ey = Float::unchecked_new(ep.y);
            *self.mv.entry(ey).or_default().entry(ex).or_default() = (pos, motion);
        }
    }

    fn find_nearest_entry(&self, (pos, _): MotionEntry, range: f32) -> Option<MotionEntry> {
        let mut best_entry: Option<(f32, MotionEntry)> = None;

        for (y, m) in self
            .mv
            .range(Float::unchecked_new(pos.y - range)..Float::unchecked_new(pos.y + range))
        {
            for (x, mv) in
                m.range(Float::unchecked_new(pos.x - range)..Float::unchecked_new(pos.x + range))
            {
                let dist = (pos.y - f32::from(*y)).abs() + (pos.x - f32::from(*x)).abs();
                if let Some((best_dist, _)) = best_entry {
                    if dist < best_dist {
                        best_entry = Some((dist, *mv))
                    }
                } else {
                    best_entry = Some((dist, *mv))
                }
            }
        }

        best_entry.map(|(_, mv)| mv)
    }
}

/// Libmv based camera estimator.
pub struct MultiviewEstimator {
    desired_confidence: f64,
    max_error: f64,
    max_iters: usize,
    use_ransac: bool,
    prev_motion: Option<PrevMotion>,
}

impl MultiviewEstimator {
    pub fn desired_confidence(self, desired_confidence: f64) -> Self {
        Self {
            desired_confidence,
            ..self
        }
    }

    pub fn max_error(self, max_error: f64) -> Self {
        Self { max_error, ..self }
    }

    pub fn max_iters(self, max_iters: usize) -> Self {
        Self { max_iters, ..self }
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
            max_iters: 200,
            use_ransac: true,
            prev_motion: None,
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

        let mut p1 = Mat::from_slice(&*p1)?;
        let mut p2 = Mat::from_slice(&*p2)?;

        let cam_matrix = camera.intrinsics();

        //println!("{:?}", cam_matrix);

        let cam_matrix = cam_matrix.transpose();

        let cam_matrix = Mat::from_slice_2d(&[
            cam_matrix.column(0).as_slice(),
            cam_matrix.column(1).as_slice(),
            cam_matrix.column(2).as_slice(),
        ])?;

        //println!("{:?}", cam_matrix);

        let mut inliers = Mat::default();

        let method = if self.use_ransac { RANSAC } else { LMEDS };

        let essential = find_essential_mat(
            &p1,
            &p2,
            &cam_matrix,
            method,
            self.desired_confidence,
            self.max_error,
            self.max_iters as _,
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
        move_magnitude: Option<f32>,
    ) -> Result<(na::UnitQuaternion<f32>, na::Vector3<f32>)> {
        let (e, p1, p2, cam_matrix, mut inliers) =
            self.essential(motion_vectors.iter().copied(), camera)?;

        let mut r = Mat::default();
        let mut t = Mat::default();

        recover_pose_estimated(&e, &p1, &p2, &cam_matrix, &mut r, &mut t, &mut inliers)?;

        let r = na::Matrix3::from_iterator(r.iter::<f64>()?.map(|(_, v)| v as _));
        let t = na::Vector3::from_iterator(t.iter::<f64>()?.map(|(_, v)| v as _));

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

        let tm = t.magnitude();

        let (t, tm) = if tm == 0.0 {
            (t, tm)
        } else {
            (t.normalize(), 1.0)
        };

        // If tm is zero, then append rotation to prev motion.
        // Else calculate motion and triangulate.

        let sf = if let Some(prev_motion) = self.prev_motion.as_mut() {
            // If there is prev motion, first track motion vectors.
            // Basically take current motion start points, find the nearest endpoint of
            // prev motion, and either a) add motions together or b) calculate new motion that ends
            // at current endpoint.

            let mv = motion_vectors
                .iter()
                .filter_map(|&me| {
                    prev_motion
                        .find_nearest_entry(me, 0.05)
                        .map(|ne| (ne.0, ne.1 + me.1))
                })
                .collect::<Vec<_>>();

            prev_motion.set_mv(mv.into_iter());
            prev_motion.rot = r * prev_motion.rot;

            if tm == 0.0 {
                0.0
            } else {
                /*let (err, f, inliers) = fundamental(
                    prev_motion.mv_iter(),
                    self.outlier_proba,
                    self.max_error,
                    self.algo_points,
                )
                .ok_or("failed to compute secondary fundamental matrix")?;
                let e = camera.essential(f);

                let e = na::Matrix3::from_iterator(e.into_iter().map(|v| *v as _));
                let k =
                    na::Matrix3::from_iterator(camera.intrinsics().into_iter().map(|v| *v as _));
                let point = prev_motion.mv_iter().nth(inliers[0]).unwrap();
                let x1 = na::Point2::new(point.0.x as _, point.0.y as _);
                let x2 = x1 + na::Vector2::new(point.1.x as _, point.1.y as _);

                let (r2, t2) =
                    libmv::multiview::fundamental::motion_from_essential_and_correspondence(
                        e, k, x1, k, x2,
                    )
                    .ok_or("failed to extract secondary motion")?;

                //let r2 = na::Matrix3::from_iterator(r2.into_iter().map(|v| *v as _));
                let t13 = na::Vector3::from_iterator(t2.into_iter().map(|v| *v as f32));

                let t23 = prev_motion.rot * t;

                //TODO: compare rotation estimates to calc error.
                //let r2 = na::UnitQuaternion::from_matrix(&r2);
                //let (x, z, y) = r.euler_angles();
                //let r2 = na::UnitQuaternion::from_euler_angles(x, y, z);

                let scale = ofps::utils::triangulate_scale(prev_motion.tr, t23, t13);

                self.prev_motion = Some(PrevMotion::new(motion_vectors.iter().copied(), r, t * scale));

                scale*/
                1.0
            }
        } else if tm == 0.0 {
            // If translation magnitude is zero we do not have anything to scale!
            0.0
        } else {
            // If there is no prev motion and positive translation magnitude,
            // just set prev motion to current motion.
            self.prev_motion = Some(PrevMotion::new(motion_vectors.iter().copied(), r, t));
            1.0
        };

        Ok((r, t * -sf * 0.0))
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

        let camera = StandardCamera::new(90.0, 90.0);

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

                let mut estimator = MultiviewEstimator::default();
                let (r, tr) = estimator.estimate(&field, &camera, None).unwrap();

                println!("ROTATION: {:?}", r.euler_angles());
                println!("DELTA: {:?}", rot.angle_to(&r).to_degrees());
                println!("TR: {:?}", tr);
            }
        }
    }
}
