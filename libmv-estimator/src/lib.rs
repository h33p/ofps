//! # Motion estimator built on Blender's `libmv` library
//!
//! This estimator produces relatively robust estimates, but suffers from continuity issues,
//! because the algorithm computes fundamental rather than essential matrices. In theory one could
//! perform incremental auto-callibration using this method, but that is not implemented as of yet.

use nalgebra as na;

use ofps::prelude::v1::*;
use std::collections::BTreeMap;

ofps::define_descriptor!(libmv, Estimator, |_| Ok(
    Box::new(LibmvEstimator::default())
));

pub fn fundamental(
    entries: impl Iterator<Item = MotionEntry>,
    outlier_proba: f64,
    max_error: f64,
    algo_points: usize,
) -> Option<(f64, na::Matrix3<f32>, Vec<usize>)> {
    let func = match algo_points {
        7 => libmv::multiview::robust_fundamental::from_correspondences_7_point,
        8 => libmv::multiview::robust_fundamental::from_correspondences_8_point,
        _ => return None,
    };
    func(
        entries.map(|(a, m)| (a, a + m)).map(|(a, b)| {
            (
                na::Point2::new(a.x as _, a.y as _),
                na::Point2::new(b.x as _, b.y as _),
            )
        }),
        max_error,
        outlier_proba,
    )
    .map(|(e, m, i)| {
        (
            e as _,
            na::Matrix3::from_iterator(m.into_iter().map(|v| *v as _)),
            i,
        )
    })
}

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
pub struct LibmvEstimator {
    outlier_proba: f32,
    max_error: f32,
    algo_points: usize,
    prev_motion: Option<PrevMotion>,
}

impl Properties for LibmvEstimator {
    fn props_mut(&mut self) -> Vec<(&str, PropertyMut)> {
        vec![
            (
                "Outlier prob.",
                PropertyMut::float(&mut self.outlier_proba, 0.0, 1.0),
            ),
            (
                "Max error",
                PropertyMut::float(&mut self.max_error, 0.00001, 0.1),
            ),
            ("Points", PropertyMut::usize(&mut self.algo_points, 7, 8)),
        ]
    }
}

impl LibmvEstimator {
    pub fn outlier_proba(self, outlier_proba: f32) -> Self {
        Self {
            outlier_proba,
            ..self
        }
    }

    pub fn max_error(self, max_error: f32) -> Self {
        Self { max_error, ..self }
    }

    pub fn algo_points(self, algo_points: usize) -> Self {
        Self {
            algo_points,
            ..self
        }
    }
}

impl Default for LibmvEstimator {
    fn default() -> Self {
        Self {
            outlier_proba: 0.7,
            max_error: 0.0001,
            algo_points: 7,
            prev_motion: None,
        }
    }
}

impl Estimator for LibmvEstimator {
    fn estimate(
        &mut self,
        motion_vectors: &[MotionEntry],
        camera: &StandardCamera,
        _: Option<f32>,
    ) -> Result<(na::UnitQuaternion<f32>, na::Vector3<f32>)> {
        let (_, f, inliers) = fundamental(
            motion_vectors.iter().copied(),
            self.outlier_proba as _,
            self.max_error as _,
            self.algo_points,
        )
        .ok_or_else(|| anyhow!("failed to compute fundamental matrix"))?;
        let e = camera.essential(f);

        // TODO: reimplement in pure Rust
        let e = na::Matrix3::from_iterator(e.into_iter().map(|v| *v as _));
        let k = na::Matrix3::from_iterator(camera.intrinsics().into_iter().map(|v| *v as _));
        let point = motion_vectors[inliers[0]];
        let x1 = na::Point2::new(point.0.x as _, point.0.y as _);
        let x2 = x1 + na::Vector2::new(point.1.x as _, point.1.y as _);

        let (r, t) = libmv::multiview::fundamental::motion_from_essential_and_correspondence(
            e, k, x1, k, x2,
        )
        .ok_or_else(|| anyhow!("failed to extract motion"))?;

        let r = na::Matrix3::from_iterator(r.into_iter().map(|v| *v as _));
        let t = na::Vector3::from_iterator(t.into_iter().map(|v| *v as _));

        // Swap Y and Z axis in the rotation matrix to be line-in-line with
        // the rest of the codebase.
        let r = na::UnitQuaternion::from_matrix(&r);
        let (x, z, y) = r.euler_angles();
        let r = na::UnitQuaternion::from_euler_angles(x * -1.0, y * -1.0, z);

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

            #[allow(clippy::needless_collect)]
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
                let (_, f, inliers) = fundamental(
                    prev_motion.mv_iter(),
                    self.outlier_proba as _,
                    self.max_error as _,
                    self.algo_points,
                )
                .ok_or_else(|| anyhow!("failed to compute secondary fundamental matrix"))?;
                let e = camera.essential(f);

                let e = na::Matrix3::from_iterator(e.into_iter().map(|v| *v as _));
                let k =
                    na::Matrix3::from_iterator(camera.intrinsics().into_iter().map(|v| *v as _));
                let point = prev_motion.mv_iter().nth(inliers[0]).unwrap();
                let x1 = na::Point2::new(point.0.x as _, point.0.y as _);
                let x2 = x1 + na::Vector2::new(point.1.x as _, point.1.y as _);

                let (_, t2) =
                    libmv::multiview::fundamental::motion_from_essential_and_correspondence(
                        e, k, x1, k, x2,
                    )
                    .ok_or_else(|| anyhow!("failed to extract secondary motion"))?;

                let t13 = na::Vector3::from_iterator(t2.into_iter().map(|v| *v as f32));

                let t23 = prev_motion.rot * t;

                let scale = ofps::utils::triangulate_scale(prev_motion.tr, t23, t13);

                self.prev_motion = Some(PrevMotion::new(
                    motion_vectors.iter().copied(),
                    r,
                    t * scale,
                ));

                scale
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

        Ok((r, t * -sf))
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

        let camera = StandardCamera::new(1.0, 170.0);

        let grid = get_grid(50, 50, &camera);

        for rot in angles.iter().map(|&(r, p, y)| {
            na::UnitQuaternion::from_euler_angles(r.to_radians(), p.to_radians(), y.to_radians())
        }) {
            for _ in 0..1 {
                let p1 = project_grid(
                    &grid,
                    &camera,
                    calc_view(Default::default(), Default::default()),
                );
                let p2 = project_grid(&grid, &camera, calc_view(rot, Default::default()));

                let field = calc_field(p1, p2);

                let mut estimator = LibmvEstimator::default();
                let (r, tr) = estimator.estimate(&field, &camera, None).unwrap();
            }
        }
    }
}
