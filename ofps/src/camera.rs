//! # Camera abstraction

use crate::prelude::v1::*;
use nalgebra as na;

/// Standard pinhole camera
///
/// This camera is defined by x and y angle field-of-view and contains a simple perspective
/// projection matrix.
///
/// The principal point is defined at `(0.5; 0.5)` coordinates.
pub struct StandardCamera {
    fov_x: f32,
    fov_y: f32,
    proj: na::Perspective3<f32>,
}

impl StandardCamera {
    /// Create a new camera
    ///
    /// # Argumenta
    ///
    /// * `fov_x` - horizontal field-of-view (in degrees).
    /// * `fov_y` - vertical field-of-view (in degrees).
    pub fn new(fov_x: f32, fov_y: f32) -> Self {
        let proj = na::Perspective3::new(fov_x / fov_y, fov_y.to_radians(), 0.1, 10.0);

        Self { fov_x, fov_y, proj }
    }

    /// Convert a screen-space point to 3D
    ///
    /// This function will convert a 2D point to 3D at an unspecified distance from camera.
    ///
    /// # Arguments
    ///
    /// * `coords` - screen-space coordinates to unproject.
    /// * `view` - camera view matrix in 3D space.
    pub fn unproject(&self, coords: na::Point2<f32>, view: na::Matrix4<f32>) -> na::Point3<f32> {
        // Transform the point to [-1; 1] range
        let coords = coords * 2.0 - na::Vector2::new(1.0, 1.0);

        // Transform the point into 3D space
        (self.proj.as_matrix() * view)
            .try_inverse()
            .unwrap()
            .transform_point(&na::Point3::new(coords.x, coords.y, 1.0))
    }

    /// Get the underlying projection matrix
    ///
    /// # Remarks
    ///
    /// The projective matrix will assume `(0; 0)` as the principal point point.
    pub fn as_matrix(&self) -> &na::Matrix4<f32> {
        self.proj.as_matrix()
    }

    /// Project a 3D point into screen space
    ///
    /// # Arguments
    ///
    /// * `world` - point to project.
    /// * `view` - camera view matrix.
    pub fn project(&self, world: na::Point3<f32>, view: na::Matrix4<f32>) -> na::Point2<f32> {
        // Transform the point back into 2D
        let mut screen = (self.proj.as_matrix() * view).transform_point(&world);

        // Convert back from homogeneus coordinates
        let screen = na::Point2::new(screen.x / screen.z, screen.y / screen.z);

        // Transform the point back to [0; 1] range
        (screen + na::Vector2::new(1.0, 1.0)) * 0.5
    }

    /// Rotate a 2D point around camera.
    ///
    /// # Arguments
    ///
    /// * `coords` - screen space coordinates of the point to reproject.
    /// * `rotation` - 3D rotation matrix of the point.
    pub fn rotate(&self, coords: na::Point2<f32>, rotation: na::Matrix4<f32>) -> na::Point2<f32> {
        let view = na::Matrix4::identity();

        let world = self.unproject(coords, view);

        // Rotate the point as if it was on a sphere
        let world = rotation.transform_point(&world);

        self.project(world, view)
    }

    /// Calculate screen-space rotation of a 2D point being rotated around the camera.
    pub fn delta(&self, coords: na::Point2<f32>, rotation: na::Matrix4<f32>) -> na::Vector2<f32> {
        self.rotate(coords, rotation) - coords
    }

    /// Get camera intrinsic parameters.
    pub fn intrinsics(&self) -> na::Matrix3<f32> {
        let fx = 1.0 / (self.fov_x.to_radians() / 2.0).tan();
        let fy = 1.0 / (self.fov_y.to_radians() / 2.0).tan();

        na::matrix![
            fx, 0.0, 0.5;
            0.0, fy, 0.5;
            0.0, 0.0, 1.0
        ]
    }

    /// Calculate the essential matrix given a fundamental one.
    ///
    /// # Arguments
    ///
    /// * `f` - input fundamental matrix.
    pub fn essential(&self, f: na::Matrix3<f32>) -> na::Matrix3<f32> {
        let k = self.intrinsics();
        k.transpose() * f * k
    }

    fn preliminary_multiview_reconstruction(
        &self,
        e: na::Matrix3<f32>,
    ) -> Option<([na::Matrix3<f32>; 4], [na::Vector3<f32>; 4])> {
        // Based on https://github.com/libmv/libmv/blob/8040c0f6fa8e03547fd4fbfdfaf6d8ffd5d1988b/src/libmv/multiview/fundamental.cc#L302-L338
        let svd = na::linalg::SVD::new_unordered(e, true, true);

        let mut u = svd.u?;
        let mut v_t = svd.v_t?;

        if u.determinant() < 0.0 {
            u.column_mut(2).neg_mut();
        }

        if v_t.determinant() < 0.0 {
            v_t.column_mut(2).neg_mut();
        }

        let w = na::matrix![
            0.0, -1.0, 0.0;
            1.0, 0.0, 0.0;
            0.0, 0.0, 1.0
        ];

        let uwvt = u * w * v_t;
        let uwtvt = u * w.transpose() * v_t;

        Some((
            [uwvt, uwvt, uwtvt, uwtvt],
            [
                u.column(2).into(),
                (-u.column(2)).into(),
                u.column(2).into(),
                (-u.column(2)).into(),
            ],
        ))
    }
}