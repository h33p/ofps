use nalgebra as na;

use super::decoder::MotionEntry;

pub fn fundamental(
    entries: &[MotionEntry],
    outlier_proba: f64,
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
        0.0001,
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
