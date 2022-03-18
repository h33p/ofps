use super::{
    CreateDecoderUiState, CreateEstimatorUiState, CreatePluginUi, OfpsAppContext, OfpsCtxApp,
};
use egui::*;
use epi::Frame;
use nalgebra as na;
use ofps::prelude::v1::*;
use std::sync::{Arc, Mutex};
use widgets::plot::{Arrows, Bar, BarChart, Line, Plot, Value, Values};
use wimrend::material::Material;
use wimrend::mesh::Mesh;
use wimrend::texture::Texture;
use wimrend::{Renderer, UniformObject};

pub struct EstimatorState {
    scale_factor: f32,
    poses: Vec<(na::Vector3<f32>, na::UnitQuaternion<f32>)>,
    layer_frames: bool,
    layered_frames: Vec<Arc<Texture>>,
}

impl Default for EstimatorState {
    fn default() -> Self {
        Self {
            scale_factor: 0.0,
            poses: vec![],
            layer_frames: false,
            layered_frames: vec![],
        }
    }
}

#[derive(Default)]
pub struct MotionTrackingApp {
    create_decoder_state: CreateDecoderUiState,
    decoder: Option<Box<dyn Decoder>>,
    estimators: Vec<(
        CreateEstimatorUiState,
        Option<(Box<dyn Estimator>, EstimatorState)>,
    )>,
    motion_vectors: Vec<MotionEntry>,
    frame: Vec<RGBA>,
    frame_height: usize,
    frames: usize,
    cube_material: Option<Arc<Material>>,
    camera_controller: CameraController,
}

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
    fov_y: f32,
    focus_point: na::Point3<f32>,
    rot: na::UnitQuaternion<f32>,
    dist: f32,
    in_motion: InMotion,
    scroll_sensitivity: f32,
    orbit_sensitivity: f32,
    move_material: Option<Arc<Material>>,
}

impl Default for CameraController {
    fn default() -> Self {
        Self {
            fov_y: 90.0,
            focus_point: Default::default(),
            rot: na::UnitQuaternion::identity(),
            dist: 5.0,
            in_motion: InMotion::None,
            scroll_sensitivity: 0.002,
            orbit_sensitivity: 0.01,
            move_material: None,
        }
    }
}

impl CameraController {
    fn update(&mut self, ctx: &Context) {
        if self.in_motion.has_motion() || !(ctx.wants_pointer_input() || ctx.wants_keyboard_input())
        {
            let input = ctx.input();

            self.dist =
                (self.dist * (1.0 - input.scroll_delta.y * self.scroll_sensitivity)).max(0.1);

            if let Some((pointer, true)) = input
                .pointer
                .interact_pos()
                .map(|p| (p, input.pointer.primary_down()))
            {
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

    fn on_render(&mut self, renderer: &mut Renderer) {
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

        let scale = na::matrix![1.0; 1.0; 1.0] * 0.2;
        let colour = na::matrix![0.7; 0.5; 0.0; 0.5];

        if let Some(pos) = match self.in_motion {
            InMotion::Orbit(_, _) => Some(self.focus_point),
            InMotion::Pan(_, pos) => Some(pos),
            _ => None,
        } {
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

impl MotionTrackingApp {
    fn tracking_step(&mut self) {
        if let Some(decoder) = &mut self.decoder {
            self.motion_vectors.clear();

            self.frame.clear();
            self.frame_height = 0;

            decoder
                .process_frame(
                    &mut self.motion_vectors,
                    Some((&mut self.frame, &mut self.frame_height)),
                    0,
                )
                .ok();

            self.frames += 1;
        } else {
            self.frames = 0;
            self.frame.clear();
            self.frame_height = 0;

            for (_, state) in self.estimators.iter_mut().filter_map(|(_, v)| v.as_mut()) {
                *state = Default::default();
            }
        }
    }

    fn render(&mut self, renderer: &mut Renderer) {
        let cube_material = if let Some(cube_material) = self.cube_material.clone() {
            cube_material
        } else {
            let cube_material = Arc::new(Material::unlit(renderer.pipeline_manager_mut()));
            self.cube_material = Some(cube_material.clone());
            cube_material
        };

        self.camera_controller.on_render(renderer);

        let start = na::matrix![0.0; 0.0; 0.0].into();

        let end = na::matrix![6.0; 2.0; 4.0].into();

        renderer.line(start, end, 4.0, na::matrix![1.0; 0.0; 0.0; 1.0]);

        renderer.obj(
            Mesh::cube(),
            start,
            na::UnitQuaternion::identity(),
            na::matrix![1.0; 1.0; 1.0],
            na::matrix![1.0; 1.0; 0.0; 1.0],
            cube_material.clone(),
        );

        renderer.obj(
            Mesh::cube(),
            end,
            na::UnitQuaternion::identity(),
            na::matrix![1.0; 1.0; 1.0],
            na::matrix![1.0; 1.0; 0.0; 1.0],
            cube_material.clone(),
        );
    }
}

impl OfpsCtxApp for MotionTrackingApp {
    fn name(&self) -> &str {
        "Tracking"
    }

    fn update(
        &mut self,
        ctx: &Context,
        ofps_ctx: &OfpsAppContext,
        frame: &Frame,
        renderer: &mut Renderer,
    ) {
        self.tracking_step();
        self.camera_controller.update(ctx);

        self.render(renderer);

        egui::SidePanel::left("tracking_settings").show(ctx, |ui| {
            egui::trace!(ui);

            ui.vertical_centered(|ui| {
                ui.heading("Tracking");
            });

            ui.separator();

            ui.heading("Decoder:");

            ui.separator();

            if let Some(decoder) = &mut self.decoder {
                if ui.button("Close decoder").clicked() {
                    self.decoder = None;
                }
            } else {
                match <Box<dyn Decoder>>::create_plugin_ui(
                    ui,
                    ofps_ctx,
                    &mut self.create_decoder_state,
                    0,
                    |_| {},
                ) {
                    Some(Ok(decoder)) => self.decoder = Some(decoder),
                    _ => {}
                }
            }

            ui.separator();

            ui.heading("Estimators:");

            ui.separator();

            let mut to_remove = None;

            for (i, (state, estimator)) in self.estimators.iter_mut().enumerate() {
                if estimator.is_some() {
                    Grid::new(format!("estimator_ui_{i}")).show(ui, |ui| {
                        ui.label(format!("Estimator #{}", i));
                        ui.end_row();
                        if ui.button("Stop").clicked() {
                            *estimator = None;
                        }
                        if ui.button("Remove").clicked() {
                            to_remove = Some(i);
                        }
                        ui.end_row();
                    });
                } else {
                    match <Box<dyn Estimator>>::create_plugin_ui(ui, ofps_ctx, state, i + 1, |ui| {
                        if ui.button("Remove").clicked() {
                            to_remove = Some(i);
                        }
                    }) {
                        Some(Ok(new_estimator)) => {
                            *estimator = Some((new_estimator, Default::default()))
                        }
                        _ => {}
                    }
                }

                ui.separator();
            }

            if let Some(to_remove) = to_remove {
                self.estimators.remove(to_remove);
            }

            if ui.button("New estimator").clicked() {
                self.estimators.push(Default::default());
            }
        });
    }
}
