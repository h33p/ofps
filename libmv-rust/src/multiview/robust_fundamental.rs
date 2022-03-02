use crate::sys;
use nalgebra as na;

pub fn from_correspondences_7_point(
    x: impl Iterator<Item = (na::Point2<f64>, na::Point2<f64>)>,
    max_error: f64,
    outliers_probability: f64,
) -> Option<(f64, na::Matrix3<f64>, Vec<usize>)> {
    let mut x1 = vec![];
    let mut x2 = vec![];

    for (p1, p2) in x {
        x1.push(p1.x);
        x1.push(p1.y);
        x2.push(p2.x);
        x2.push(p2.y);
    }

    let mut inliers = vec![-1; x1.len() / 2];
    let mut inliers_len = 0;
    let mut f = [0.0; 9];

    let error = unsafe {
        sys::fundamental_from_correspondences_7_point_robust(
            x1.as_ptr(),
            x2.as_ptr(),
            (x1.len() / 2) as _,
            max_error,
            f.as_mut_ptr(),
            inliers.as_mut_ptr(),
            &mut inliers_len,
            outliers_probability,
        )
    };

    if error > 10000000.0 {
        None
    } else {
        Some((
            error,
            na::Matrix3::from_iterator(f.iter().copied()),
            inliers
                .into_iter()
                .take(inliers_len as _)
                .map(|i| i as _)
                .collect(),
        ))
    }
}

pub fn from_correspondences_8_point(
    x: impl Iterator<Item = (na::Point2<f64>, na::Point2<f64>)>,
    max_error: f64,
    outliers_probability: f64,
) -> Option<(f64, na::Matrix3<f64>, Vec<usize>)> {
    let mut x1 = vec![];
    let mut x2 = vec![];

    for (p1, p2) in x {
        x1.push(p1.x);
        x1.push(p1.y);
        x2.push(p2.x);
        x2.push(p2.y);
    }

    let mut inliers = vec![-1; x1.len() / 2];
    let mut inliers_len = 0;
    let mut f = [0.0; 9];

    let error = unsafe {
        sys::fundamental_from_correspondences_8_point_robust(
            x1.as_ptr(),
            x2.as_ptr(),
            (x1.len() / 2) as _,
            max_error,
            f.as_mut_ptr(),
            inliers.as_mut_ptr(),
            &mut inliers_len,
            outliers_probability,
        )
    };

    if error > 10000000.0 {
        None
    } else {
        Some((
            error,
            na::Matrix3::from_iterator(f.iter().copied()),
            inliers
                .into_iter()
                .take(inliers_len as _)
                .map(|i| i as _)
                .collect(),
        ))
    }
}
