use egui::{Context, Pos2, Vec2};
use nalgebra as na;
use ofps::camera::StandardCamera;
use std::sync::Arc;
use wimrend::{material::Material, mesh::Mesh, Renderer};

enum InMotion {
    None,
    Orbit(Pos2, na::UnitQuaternion<f32>),
    Pan(Pos2, na::Point3<f32>),
}

impl InMotion {
    fn has_motion(&self) -> bool {
        matches!(self, InMotion::None)
    }
}

pub struct CameraController {
    pub fov_y: f32,
    pub focus_point: na::Point3<f32>,
    pub rot: na::UnitQuaternion<f32>,
    pub dist: f32,
    in_motion: InMotion,
    pub scroll_sensitivity: f32,
    pub orbit_sensitivity: f32,
    move_material: Option<Arc<Material>>,
    last_down: bool,
    pressed: bool,
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            fov_y: 90.0,
            focus_point: Default::default(),
            rot: na::UnitQuaternion::from_euler_angles(
                -30f32.to_radians(),
                0.0,
                30f32.to_radians(),
            ),
            dist: 1.0,
            in_motion: InMotion::None,
            scroll_sensitivity: 0.002,
            orbit_sensitivity: 0.01,
            move_material: None,
            last_down: false,
            pressed: false,
        }
    }
}

impl CameraController {
    pub fn update(&mut self, ctx: &Context) {
        if self.in_motion.has_motion() || !(ctx.wants_pointer_input() || ctx.wants_keyboard_input())
        {
            let over_area = ctx.is_pointer_over_area();
            let input = ctx.input();

            self.dist =
                (self.dist * (1.0 - input.scroll_delta.y * self.scroll_sensitivity)).max(0.1);

            let pressed = input.pointer.primary_down();
            self.pressed = pressed && (self.pressed || (!self.last_down && !over_area));
            self.last_down = pressed;

            if let Some((pointer, true)) = input.pointer.interact_pos().map(|p| (p, self.pressed)) {
                let pan_key = input.modifiers.shift;

                match self.in_motion {
                    InMotion::None => {
                        if pan_key {
                            self.in_motion = InMotion::Pan(pointer, self.focus_point);
                        } else {
                            self.in_motion = InMotion::Orbit(pointer, self.rot);
                        }
                    }
                    InMotion::Orbit(pointer_start, start_rot) => {
                        let delta = pointer - pointer_start;

                        // Use euler angles to never have any rolling rotation.
                        let (x, _, z) = start_rot.euler_angles();

                        self.rot = na::UnitQuaternion::from_euler_angles(
                            x - delta.y * self.orbit_sensitivity,
                            0.0,
                            z - delta.x * self.orbit_sensitivity,
                        );

                        if pan_key {
                            self.in_motion = InMotion::Pan(pointer, self.focus_point);
                        }
                    }
                    InMotion::Pan(pointer_start, start_pos) => {
                        let delta = pointer - pointer_start;
                        let screen_size =
                            Vec2::new(input.screen_rect.width(), input.screen_rect.height());
                        let delta = delta / screen_size;
                        let aspect = screen_size.x / screen_size.y;
                        let fov = self.fov_y.to_radians() / 2.0;

                        // A little bit geometry to move the pan the camera in a pixel perfect way.
                        let move_delta = na::matrix![
                            -fov.tan() * delta.x * aspect;
                            0.0;
                            fov.tan() * delta.y
                        ] * (self.dist * 2.0);

                        self.focus_point = start_pos + self.rot * move_delta;

                        if !pan_key {
                            self.in_motion = InMotion::Orbit(pointer, self.rot);
                        }
                    }
                }
            } else {
                self.in_motion = InMotion::None;
            }
        }
    }

    pub fn on_render(&mut self, renderer: &mut Renderer) {
        // Initialise the indicator material if it hasn't already been.
        let move_material = if let Some(move_material) = self.move_material.clone() {
            move_material
        } else {
            let move_material = Arc::new(Material::unlit(renderer.pipeline_manager_mut()));
            self.move_material = Some(move_material.clone());
            move_material
        };

        let uniform = renderer.uniform_mut();

        let dir = self.rot * na::matrix![0.0; -1.0; 0.0];
        let res = uniform.resolution();
        let aspect = res[0] / res[1];

        uniform.update_projection(
            StandardCamera::new(self.fov_y * aspect, self.fov_y).as_matrix(),
            self.focus_point + dir * self.dist,
            self.rot,
        );

        let scale = na::matrix![1.0; 1.0; 1.0] * 0.1 * self.dist;
        let colour = na::matrix![0.7; 0.5; 0.0; 0.5];

        for pos in match self.in_motion {
            InMotion::Orbit(_, _) => [Some(self.focus_point), None],
            InMotion::Pan(_, pos) => [Some(self.focus_point), Some(pos)],
            _ => [None, None],
        }
        .iter()
        .filter_map(|v| *v)
        {
            renderer.obj(
                Mesh::cube(),
                pos,
                na::UnitQuaternion::identity(),
                scale,
                colour,
                move_material.clone(),
            );
        }
    }
}
