use crate::prelude::v1::*;
use nalgebra as na;

pub struct StandardCamera {
    fov_x: f32,
    fov_y: f32,
    proj: na::Perspective3<f32>,
}

impl StandardCamera {
    pub fn new(fov_x: f32, fov_y: f32) -> Self {
        let proj = na::Perspective3::new(fov_x / fov_y, fov_y.to_radians(), 0.01, 10.0);

        Self { fov_x, fov_y, proj }
    }

    pub fn rotate(&self, coords: na::Point2<f32>, rotation: na::Matrix4<f32>) -> na::Point2<f32> {
        // Transform the point to [-1; 1] range
        let coords = coords * 2.0 - na::Vector2::new(1.0, 1.0);

        // Transform the point into 3D space
        let world = self
            .proj
            .unproject_point(&na::Point3::new(coords.x, coords.y, 1.0));

        // Rotate the point as if it was on a sphere
        let world = rotation.transform_point(&world);

        // Transform the point back into 2D
        let screen = self.proj.project_point(&world);

        // Convert back from homogeneus coordinates
        let screen = na::Point2::new(screen.x / screen.z, screen.y / screen.z);

        // Transform the point back to [0; 1] range
        (screen + na::Vector2::new(1.0, 1.0)) * 0.5
    }

    pub fn delta(&self, coords: na::Point2<f32>, rotation: na::Matrix4<f32>) -> na::Vector2<f32> {
        self.rotate(coords, rotation) - coords
    }

    pub fn intrinsics(&self) -> na::Matrix3<f32> {
        let fx = 1.0 / (self.fov_x.to_radians() / 2.0).tan();
        let fy = 1.0 / (self.fov_y.to_radians() / 2.0).tan();

        na::matrix![
            fx, 0.0, 0.5;
            0.0, fy, 0.5;
            0.0, 0.0, 1.0
        ]
    }

    pub fn essential(&self, f: na::Matrix3<f32>) -> na::Matrix3<f32> {
        let k = self.intrinsics();
        k.transpose() * f * k
    }

    pub fn preliminary_multiview_reconstruction(
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

    pub fn reconstruct_multiview(
        &self,
        points: &[MotionEntry],
    ) -> Option<(na::geometry::Rotation3<f32>, na::Vector3<f32>)> {
        let (err, f, inliers) = crate::reconstruct::fundamental(points, 0.7)?;
        let e = self.essential(f);

        // TODO: reimplement in pure Rust
        let e = na::Matrix3::from_iterator(e.into_iter().map(|v| *v as _));
        let k = na::Matrix3::from_iterator(self.intrinsics().into_iter().map(|v| *v as _));
        let point = points[inliers[0]];
        let x1 = na::Point2::new(point.0.x as _, point.0.y as _);
        let x2 = x1 + na::Vector2::new(point.1.x as _, point.1.y as _);

        let (r, t) = libmv::multiview::fundamental::motion_from_essential_and_correspondence(
            e, k, x1, k, x2,
        )?;

        println!("({}) | {:?} | {:?}", e, r, t);

        let r = na::Matrix3::from_iterator(r.into_iter().map(|v| *v as _));
        let t = na::Vector3::from_iterator(t.into_iter().map(|v| *v as _));

        Some((na::geometry::Rotation3::from_matrix(&r), t))
    }
}
