use crate::prelude::v1::*;
use nalgebra as na;

pub trait MotionModel {
    fn roll(&self, coords: na::Point2<f32>) -> na::Point2<f32>;

    fn yaw(&self, coords: na::Point2<f32>) -> na::Point2<f32>;

    fn pitch(&self, coords: na::Point2<f32>) -> na::Point2<f32>;
}

pub struct StandardCamera {
    fov_x: f32,
    fov_y: f32,
}

impl MotionModel for StandardCamera {
    fn roll(&self, coords: na::Point2<f32>) -> na::Point2<f32> {
        // Calculate
        coords
    }

    fn yaw(&self, coords: na::Point2<f32>) -> na::Point2<f32> {
        coords
    }

    fn pitch(&self, coords: na::Point2<f32>) -> na::Point2<f32> {
        coords
    }
}
