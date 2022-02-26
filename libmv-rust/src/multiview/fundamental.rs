use crate::sys;
use nalgebra as na;

pub fn motion_from_essential_and_correspondence(
    e: na::Matrix3<f64>,
    k1: na::Matrix3<f64>,
    x1: na::Point2<f64>,
    k2: na::Matrix3<f64>,
    x2: na::Point2<f64>,
) -> Option<(na::Matrix3<f64>, na::Vector3<f64>)> {
    let mut r = [0.0; 9];
    let mut t = [0.0; 3];

    if unsafe {
        sys::motion_from_essential_and_correspondence(
            e.as_slice().as_ptr(),
            k1.as_slice().as_ptr(),
            [x1.x, x1.y].as_ptr(),
            k2.as_slice().as_ptr(),
            [x2.x, x2.y].as_ptr(),
            r.as_mut_ptr(),
            t.as_mut_ptr(),
        )
    } != 0
    {
        Some((na::Matrix3::from_iterator(r), na::Vector3::from_iterator(t)))
    } else {
        None
    }
}
