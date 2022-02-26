use crate::prelude::v1::*;
use nalgebra as na;

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
