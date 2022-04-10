//! # Implementation of "Robust Estimation of Camera Motion Using Optical Flow Models".
//!
//! The following estimator produces relatively accurate rotation estimates, with no translational
//! output.
//!
//! This is not a blind reimplementation of the paper - it contains several improvements to the
//! methods used.

use nalgebra as na;
use ofps::prelude::v1::*;
use rand::seq::SliceRandom;

ofps::define_descriptor!(almeida, Estimator, |_| Ok(Box::new(
    AlmeidaEstimator::default()
)));

const EPS: f32 = 0.001 * std::f32::consts::PI / 180.0;
const ALPHA: f32 = 0.5;

pub trait MotionModel {
    fn roll(&self, coords: na::Point2<f32>, eps: f32) -> na::Vector2<f32>;

    fn pitch(&self, coords: na::Point2<f32>, eps: f32) -> na::Vector2<f32>;

    fn yaw(&self, coords: na::Point2<f32>, eps: f32) -> na::Vector2<f32>;

    fn point_angle(&self, p: na::Point2<f32>) -> na::Vector2<f32>;
}

impl MotionModel for StandardCamera {
    fn roll(&self, coords: na::Point2<f32>, eps: f32) -> na::Vector2<f32> {
        // We use different camera axis compared to what nalgebra considers as RPY
        self.delta(coords, na::Matrix4::from_euler_angles(0.0, eps, 0.0))
    }

    fn pitch(&self, coords: na::Point2<f32>, eps: f32) -> na::Vector2<f32> {
        self.delta(coords, na::Matrix4::from_euler_angles(eps, 0.0, 0.0))
    }

    fn yaw(&self, coords: na::Point2<f32>, eps: f32) -> na::Vector2<f32> {
        self.delta(coords, na::Matrix4::from_euler_angles(0.0, 0.0, -eps))
    }

    fn point_angle(&self, p: na::Point2<f32>) -> na::Vector2<f32> {
        StandardCamera::point_angle(self, p)
    }
}

/// Motion estimator built on a research paper titled "Robust Estimation
/// of Camera Motion Using Optical Flow Models".
///
/// Authors:
///
/// Jurandy Almeida, Rodrigo Minetto, Tiago A. Almeida, Ricardo da S. Torres, and Neucimar J. Leite.
///
/// This estimator only produces rotational output, and no translation.
pub struct AlmeidaEstimator {
    /// True if ransac is used. False to perform least
    /// squares minimisation solution.
    use_ransac: bool,
    /// Number of iterations for ransac.
    num_iters: usize,
    /// Target angle error in degrees for the sample to be considered as inlier.
    inlier_angle: f32,
    /// Number of samples per each ransac iteration.
    ransac_samples: usize,
}

impl Default for AlmeidaEstimator {
    fn default() -> Self {
        Self {
            use_ransac: true,
            num_iters: 200,
            inlier_angle: 0.05,
            ransac_samples: 1000,
        }
    }
}

impl Properties for AlmeidaEstimator {
    fn props_mut(&mut self) -> Vec<(&str, PropertyMut)> {
        vec![
            ("Use ransac", PropertyMut::bool(&mut self.use_ransac)),
            (
                "Ransac iters",
                PropertyMut::usize(&mut self.num_iters, 1, 500),
            ),
            (
                "Inlier threshold",
                PropertyMut::float(&mut self.inlier_angle, 0.01, 1.0),
            ),
            (
                "Ransac samples",
                PropertyMut::usize(&mut self.ransac_samples, 100, 16000),
            ),
        ]
    }
}

impl Estimator for AlmeidaEstimator {
    fn estimate(
        &mut self,
        motion_vectors: &[MotionEntry],
        camera: &StandardCamera,
        _move_magnitude: Option<f32>,
    ) -> Result<(na::UnitQuaternion<f32>, na::Vector3<f32>)> {
        let rot = if self.use_ransac {
            solve_ypr_ransac(
                motion_vectors,
                camera,
                self.num_iters,
                self.inlier_angle,
                self.ransac_samples,
            )
        } else {
            solve_ypr_given(motion_vectors, camera)
        };

        Ok((rot, na::Vector3::default()))
    }
}

fn solve_ypr_given(input: &[MotionEntry], camera: &StandardCamera) -> na::UnitQuaternion<f32> {
    let dot = |a: usize, b: usize| move |vecs: &[na::Vector2<f32>]| vecs[a].dot(&vecs[b]);

    fn dot_map<T: Fn(&[na::Vector2<f32>]) -> f32>(
        motion: &[(na::Point2<f32>, [na::Vector2<f32>; 4])],
    ) -> (impl Fn(T) -> f32 + '_) {
        move |dot| motion.iter().map(|(_, v)| dot(v)).sum::<f32>()
    }

    let limit = (15.0 / ALPHA).ceil() as usize;

    let mut rotation = na::UnitQuaternion::identity();

    // Iterative optimisation loop.
    for i in 0..limit {
        let alpha = if i == limit - 1 { 1.0 } else { ALPHA };

        let rotm = rotation.to_homogeneous();

        let motion = input
            .iter()
            .copied()
            .map(|(pos, motion)| {
                let delta = camera.delta(pos, rotm);
                (
                    pos + delta,
                    [
                        motion - delta,
                        camera.roll(pos, EPS),
                        camera.pitch(pos, EPS),
                        camera.yaw(pos, EPS),
                    ],
                )
            })
            .collect::<Vec<_>>();

        let a = na::Matrix3::from_iterator(
            [
                dot(1, 1),
                dot(1, 2),
                dot(1, 3),
                dot(2, 1),
                dot(2, 2),
                dot(2, 3),
                dot(3, 1),
                dot(3, 2),
                dot(3, 3),
            ]
            .iter()
            .map(dot_map(&motion)),
        );

        let b = na::Vector3::from_iterator(
            [dot(1, 0), dot(2, 0), dot(3, 0)]
                .iter()
                .map(dot_map(&motion)),
        );

        let decomp = a.lu();

        let model = decomp.solve(&b).unwrap_or_default();

        let model = model * EPS * alpha;

        // Apply rotation in YRP order, as it is more correct.

        let roll = na::UnitQuaternion::from_euler_angles(0.0, model.x, 0.0);
        let pitch = na::UnitQuaternion::from_euler_angles(model.y, 0.0, 0.0);
        let yaw = na::UnitQuaternion::from_euler_angles(0.0, 0.0, -model.z);

        let rot = pitch * roll * yaw;

        rotation *= rot;
    }

    // We estimated how points rotate, not how the camera rotates - take inverse.
    rotation.inverse()
}

fn solve_ypr_ransac(
    field: &[MotionEntry],
    camera: &StandardCamera,
    num_iters: usize,
    target_delta: f32,
    num_samples: usize,
) -> na::UnitQuaternion<f32> {
    let mut best_inliers = vec![];
    let target_delta = target_delta.to_radians();

    let rng = &mut rand::thread_rng();

    for _ in 0..num_iters {
        let samples = field.choose_multiple(rng, 4).copied().collect::<Vec<_>>();

        let fit = solve_ypr_given(&samples, camera);

        let motion = field
            .choose_multiple(rng, num_samples)
            .copied()
            .collect::<Vec<_>>();

        let mat = fit.inverse().to_homogeneous();

        let inliers = motion
            .iter()
            .copied()
            .map(|(pos, vec)| {
                let delta = camera.delta(pos, mat);
                ((pos, vec), (pos + delta, vec - delta))
            })
            .filter(|(_, (sample, vec))| {
                let angle = camera.point_angle(*sample);
                let cosang = na::matrix![angle.x.cos(); angle.y.cos()];
                vec.component_mul(&cosang).magnitude_squared() <= target_delta * target_delta
            })
            .map(|(a, _)| a)
            .collect::<Vec<_>>();

        if inliers.len() > best_inliers.len() {
            best_inliers = inliers;
        }
    }

    solve_ypr_given(&best_inliers, camera)
}

#[cfg(test)]
pub mod tests {
    use super::*;

    fn get_grid(
        grid_cnt_x: usize,
        grid_cnt_y: usize,
        camera: &StandardCamera,
    ) -> Vec<na::Point3<f32>> {
        let mut new_points = vec![];

        for x in 0..grid_cnt_x {
            for y in 0..grid_cnt_y {
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
            &(pos + rot.transform_vector(&na::Vector3::new(0.0, -1.0, 0.0))),
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
        let mid = na::Point2::new(0.5, 0.5);
        p1.into_iter()
            .zip(p2.into_iter())
            .filter(|(p1, p2)| (p1 - mid).magnitude() <= 0.71 || (p2 - mid).magnitude() <= 0.71)
            .map(|(p1, p2)| (p1, p2 - p1))
            .collect()
    }

    fn test_rot(mut estimator: AlmeidaEstimator) {
        let camera = StandardCamera::new(1.0, 90.0);

        let grid = get_grid(50, 50, &camera);

        for rot in [0.01f32, 0.1, 1.0, 10.0] {
            let angles = [
                (0.0, 0.0, 0.0),
                (rot, 0.0, 0.0),
                (0.0, rot, 0.0),
                (0.0, 0.0, rot),
                (rot, rot, 0.0),
                (rot, 0.0, rot),
                (0.0, rot, rot),
                (rot, rot, rot),
            ];

            for q in angles.iter().map(|&(r, p, y)| {
                na::UnitQuaternion::from_euler_angles(
                    r.to_radians(),
                    p.to_radians(),
                    y.to_radians(),
                )
            }) {
                let p1 = project_grid(
                    &grid,
                    &camera,
                    calc_view(Default::default(), Default::default()),
                );
                let p2 = project_grid(&grid, &camera, calc_view(q, Default::default()));

                let field = calc_field(p1, p2);

                let (r, _) = estimator.estimate(&field, &camera, None).unwrap();

                let delta = q.angle_to(&r).to_degrees();

                println!("E: {}", delta / rot);

                assert!(
                    delta < 0.1 * rot,
                    "{:?} vs {:?}: {} > {}",
                    q.euler_angles(),
                    r.euler_angles(),
                    delta,
                    0.1 * rot
                );
            }
        }
    }

    #[test]
    fn test_rotation_default() {
        let mut estimator = AlmeidaEstimator::default();
        estimator.use_ransac = false;
        test_rot(estimator);
    }

    #[test]
    fn test_rotation_ransac() {
        let mut estimator = AlmeidaEstimator::default();
        estimator.use_ransac = true;
        estimator.num_iters = 100;
        test_rot(estimator);
    }
}
