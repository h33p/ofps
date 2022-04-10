//! # Camera abstraction

use nalgebra as na;

/// Standard pinhole camera
///
/// This camera is defined by x and y angle field-of-view and contains a simple perspective
/// projection matrix.
///
/// The principal point is defined at `(0.5; 0.5)` coordinates.
#[derive(Clone, Copy, Debug)]
pub struct StandardCamera {
    aspect: f32,
    fov_y: f32,
    proj: na::Perspective3<f32>,
    inv_proj: na::Matrix4<f32>,
}

impl StandardCamera {
    /// Create a new camera
    ///
    /// # Argumenta
    ///
    /// * `aspect` - screen aspect ratio.
    /// * `fov_y` - vertical field-of-view (in degrees).
    pub fn new(aspect: f32, fov_y: f32) -> Self {
        let proj = na::Perspective3::new(aspect, fov_y.to_radians(), 0.1, 10.0);

        Self {
            aspect,
            fov_y,
            inv_proj: proj.inverse(),
            proj,
        }
    }

    /// Convert a screen-space point to 3D
    ///
    /// This function will convert a 2D point to 3D at an unspecified distance from camera.
    ///
    /// # Arguments
    ///
    /// * `coords` - screen-space coordinates to unproject.
    /// * `inv_view` - inverse of camera view matrix in 3D space.
    pub fn unproject(
        &self,
        coords: na::Point2<f32>,
        inv_view: na::Matrix4<f32>,
    ) -> na::Point3<f32> {
        // Transform the point to [-1; 1] range
        let coords = coords * 2.0 - na::Vector2::new(1.0, 1.0);

        // Transform the point into 3D space
        (inv_view * self.inv_proj).transform_point(&na::Point3::new(coords.x, coords.y, 1.0))
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
        let screen = self.proj.project_point(&view.transform_point(&world));

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
        // Z up, Y forward.
        let view = na::matrix![
            -1.0, 0.0, 0.0, 0.0;
             0.0, 0.0, 1.0, 0.0;
             0.0, 1.0, 0.0, 0.0;
             0.0, 0.0, 0.0, 1.0;
        ];

        /*let view = na::Matrix4::look_at_lh(
            &na::Vector3::new(0.0, 0.0, 0.0).into(),
            &na::Vector3::new(0.0, 1.0, 0.0).into(),
            &na::Vector3::new(0.0, 0.0, 1.0),
        );*/

        //println!("{:?}", view);

        let world = self.unproject(coords, view.transpose());

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
        let fy = 0.5 / (self.fov_y.to_radians() / 2.0).tan();
        let fx = fy / self.aspect;

        na::matrix![
            fx, 0.0, 0.5;
            0.0, fy, 0.5;
            0.0, 0.0, 1.0
        ]
    }

    /// Get horizontal and vertical angle of a point in radians.
    ///
    /// # Arguments
    ///
    /// * `p` - point to have the angles computed for.
    ///
    /// # Examples
    ///
    /// ```
    /// # use assert_approx_eq::assert_approx_eq;
    /// use ofps::camera::StandardCamera;
    /// use nalgebra as na;
    ///
    /// let camera = StandardCamera::new(1.0, 90.0);
    ///
    /// let angle = camera.point_angle(na::matrix![1.0; 0.5].into());
    ///
    /// assert_approx_eq!(angle.x.to_degrees(), 45.0f32, 0.01);
    /// ```
    pub fn point_angle(&self, p: na::Point2<f32>) -> na::Vector2<f32> {
        let intrinsics = self.intrinsics();

        // Center the point.
        let p = p - intrinsics.fixed_slice::<2, 1>(0, 2);

        let tan = p
            .coords
            .component_div(&na::matrix![intrinsics[(0, 0)]; intrinsics[(1, 1)]]);

        na::matrix![tan.x.atan(); tan.y.atan()]
    }

    /// Get the camera's field of view.
    ///
    /// Returns horizontal and vertical field of view in degrees as a tuple.
    pub fn fov(&self) -> (f32, f32) {
        let ty = (self.fov_y.to_radians() / 2.0).tan();
        let tx = self.aspect * ty;
        (tx.atan().to_degrees() * 2.0, self.fov_y)
    }

    /// Get the camera's aspect ratio.
    ///
    /// Returns vertical focal length divided by horizontal focal length.
    pub fn aspect_ratio(&self) -> f32 {
        self.aspect
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
}
