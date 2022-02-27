use nalgebra as na;

use crate::prelude::v1::*;

pub fn fundamental(
    entries: &[MotionEntry],
    outlier_proba: f64,
    max_error: f64,
) -> Option<(f64, na::Matrix3<f32>, Vec<usize>)> {
    libmv::multiview::robust_fundamental::from_correspondences_8_point(
        entries
            .iter()
            .copied()
            .map(|(a, m)| (a, a + m))
            .map(|(a, b)| {
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

/// Libmv based camera estimator.
pub struct MultiviewEstimator {
    outlier_proba: f64,
    max_error: f64,
}

impl Default for MultiviewEstimator {
    fn default() -> Self {
        Self {
            outlier_proba: 0.7,
            max_error: 0.0001,
        }
    }
}

impl Estimator for MultiviewEstimator {
    fn estimate(
        &mut self,
        motion_vectors: &[MotionEntry],
        camera: &StandardCamera,
    ) -> Result<(na::UnitQuaternion<f32>, na::Vector3<f32>)> {
        let (err, f, inliers) = fundamental(motion_vectors, self.outlier_proba, self.max_error)
            .ok_or("failed to compute fundamental matrix")?;
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
        .ok_or("failed to extract motion")?;

        let r = na::Matrix3::from_iterator(r.into_iter().map(|v| *v as _));
        let t = na::Vector3::from_iterator(t.into_iter().map(|v| *v as _));

        // Swap Y and Z axis in the rotation matrix to be line-in-line with
        // the rest of the codebase.
        let r = na::UnitQuaternion::from_matrix(&r);
        let (x, z, y) = r.euler_angles();

        Ok((na::UnitQuaternion::from_euler_angles(x, y, z), t * -0.1))
    }
}
