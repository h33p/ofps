use crate::prelude::v1::*;
use nalgebra as na;
use rand::seq::SliceRandom;

const EPS: f32 = 0.01745329f32;

pub trait MotionModel {
    fn roll(&self, coords: na::Point2<f32>) -> na::Vector2<f32>;

    fn pitch(&self, coords: na::Point2<f32>) -> na::Vector2<f32>;

    fn yaw(&self, coords: na::Point2<f32>) -> na::Vector2<f32>;
}

impl MotionModel for StandardCamera {
    fn roll(&self, coords: na::Point2<f32>) -> na::Vector2<f32> {
        // We use different camera axis compared to what nalgebra considers as RPY
        self.delta(coords, na::Matrix4::from_euler_angles(0.0, 0.0, EPS))
    }

    fn pitch(&self, coords: na::Point2<f32>) -> na::Vector2<f32> {
        self.delta(coords, na::Matrix4::from_euler_angles(EPS, 0.0, 0.0))
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
    /// num_iters if ransac is used. `None` to perform
    /// least squares minimisation solution.
    ransac_options: Option<usize>,
}

impl Estimator for AlmeidaEstimator {
    fn estimate(
        &mut self,
        motion_vectors: &[MotionEntry],
        camera: &StandardCamera,
    ) -> Result<(na::UnitQuaternion<f32>, na::Vector3<f32>)> {
        let model = if let Some(num_iters) = self.ransac_options {
            solve_ypr_ransac(motion_vectors, camera, num_iters)
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

    let b = na::Matrix3x1::from_iterator(
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
            println!("{} | {}", max_inliers, best_fit);
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
