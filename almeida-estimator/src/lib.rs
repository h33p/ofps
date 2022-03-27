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

const EPS: f32 = 0.01745329f32;

pub trait MotionModel {
    fn roll(&self, coords: na::Point2<f32>) -> na::Vector2<f32>;

    fn pitch(&self, coords: na::Point2<f32>) -> na::Vector2<f32>;

    fn yaw(&self, coords: na::Point2<f32>) -> na::Vector2<f32>;
}

impl MotionModel for StandardCamera {
    fn roll(&self, coords: na::Point2<f32>) -> na::Vector2<f32> {
        // We use different camera axis compared to what nalgebra considers as RPY
        self.delta(coords, na::Matrix4::from_euler_angles(0.0, 0.0, -EPS))
    }

    fn pitch(&self, coords: na::Point2<f32>) -> na::Vector2<f32> {
        self.delta(coords, na::Matrix4::from_euler_angles(-EPS, 0.0, 0.0))
    }

    fn yaw(&self, coords: na::Point2<f32>) -> na::Vector2<f32> {
        self.delta(coords, na::Matrix4::from_euler_angles(0.0, EPS, 0.0))
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
#[derive(Default)]
pub struct AlmeidaEstimator {
    /// True if ransac is used. False to perform least
    /// squares minimisation solution.
    use_ransac: bool,
    /// Number of iterations for ransac.
    num_iters: usize,
}

impl Properties for AlmeidaEstimator {
    fn props_mut(&mut self) -> Vec<(&str, PropertyMut)> {
        vec![
            ("Use ransac", PropertyMut::bool(&mut self.use_ransac)),
            (
                "Ransac iters",
                PropertyMut::usize(&mut self.num_iters, 1, 500),
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
        let model = if self.use_ransac {
            solve_ypr_ransac(motion_vectors, camera, self.num_iters)
        } else {
            solve_ypr(camera, motion_vectors)
        } * EPS;

        let rotation = na::UnitQuaternion::from_euler_angles(-model.y, model.z, -model.x);

        Ok((rotation, na::Vector3::default()))
    }
}

fn solve_ypr_given(motion: &[(na::Point2<f32>, [na::Vector2<f32>; 4])]) -> na::Vector3<f32> {
    let dot = |a: usize, b: usize| move |vecs: &[na::Vector2<f32>]| vecs[a].dot(&vecs[b]);

    fn dot_map<T: Fn(&[na::Vector2<f32>]) -> f32>(
        motion: &[(na::Point2<f32>, [na::Vector2<f32>; 4])],
    ) -> (impl Fn(T) -> f32 + '_) {
        move |dot| motion.iter().map(|(_, v)| dot(v)).sum::<f32>()
    }

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

    decomp.solve(&b).unwrap_or_default()
}

fn solve_ypr(camera: &impl MotionModel, field: &[MotionEntry]) -> na::Vector3<f32> {
    let motion = field
        .iter()
        .copied()
        .map(|(pos, motion)| {
            (
                pos,
                [motion, camera.yaw(pos), camera.pitch(pos), camera.roll(pos)],
            )
        })
        .collect::<Vec<_>>();

    solve_ypr_given(&motion)
}

fn solve_ypr_ransac(
    field: &[MotionEntry],
    camera: &impl MotionModel,
    num_iters: usize,
) -> na::Vector3<f32> {
    let motion = field
        .iter()
        .copied()
        .map(|(pos, motion)| {
            (
                pos,
                [motion, camera.yaw(pos), camera.pitch(pos), camera.roll(pos)],
            )
        })
        .collect::<Vec<_>>();

    let mut best_fit = Default::default();
    let mut max_inliers = 0;
    let target_delta = 0.0001;

    let rng = &mut rand::thread_rng();

    // Even with 60% of outliers this would not reach this limit...
    for _ in 0..num_iters {
        /*if max_inliers as f32 / motion.len() as f32 > 0.6 {
            break;
        }*/

        let samples = motion.choose_multiple(rng, 4).copied().collect::<Vec<_>>();

        let fit = solve_ypr_given(samples.as_slice());

        let motion = motion
            .choose_multiple(rng, 400)
            .copied()
            .collect::<Vec<_>>();

        let inliers = motion
            .iter()
            .map(|(pos, vec)| (sample_ypr_model(fit, *pos, camera), vec[0], pos, vec))
            .filter(|(sample, actual, pos, vec)| {
                // Sum the (error / target_delta).min(1.0)

                if actual.magnitude() == 0.0 {
                    true
                } else {
                    /*let c = -*sample;
                    let d = *actual - *sample;

                    let t = (c.dot(&d) / d.dot(&d)).abs();

                    //println!("{}", t);

                    t <= target_delta*/
                    (*actual - *sample).magnitude()/* / actual.magnitude()*/ <= target_delta
                }
            })
            .map(|(a, b, pos, vec)| (*pos, *vec))
            .collect::<Vec<_>>();

        if inliers.len() > max_inliers {
            best_fit = solve_ypr_given(inliers.as_slice());
            max_inliers = inliers.len();
            //jkprintln!("{} | {}", max_inliers, best_fit);
        }
    }

    best_fit
}

fn sample_ypr_model(
    model: na::Vector3<f32>,
    pos: na::Point2<f32>,
    camera: &impl MotionModel,
) -> na::Vector2<f32> {
    camera.yaw(pos) * model.x + camera.pitch(pos) * model.y + camera.roll(pos) * model.z
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
            let p1 = project_grid(
                &grid,
                &camera,
                calc_view(Default::default(), Default::default()),
            );
            let p2 = project_grid(&grid, &camera, calc_view(rot, Default::default()));

            let field = calc_field(p1, p2);

            let mut estimator = AlmeidaEstimator::default();
            let (r, tr) = estimator.estimate(&field, &camera, None).unwrap();

            let delta = rot.angle_to(&r).to_degrees();

            assert!(
                delta < 0.1 * ROT,
                "{:?} vs {:?}: {} > {}",
                rot.euler_angles(),
                r.euler_angles(),
                delta,
                0.1 * ROT
            );
        }
    }
}
