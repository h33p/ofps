use super::utils::camera_controller::CameraController;
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
use wimrend::Renderer;

pub struct EstimatorSettings {
    scale_factor: f32,
    camera_offset: f32,
    layer_frames: bool,
    layer_angle_delta: f32,
    keep_frames: usize,
}

impl Default for EstimatorSettings {
    fn default() -> Self {
        Self {
            scale_factor: 1.0,
            camera_offset: 1.0,
            layer_frames: true,
            layer_angle_delta: 0.5,
            keep_frames: 100,
        }
    }
}

#[derive(Default)]
pub struct EstimatorState {
    poses: Vec<(na::Point3<f32>, na::UnitQuaternion<f32>)>,
    layered_frames: Vec<(usize, Arc<Material>)>,
}

impl EstimatorState {
    fn apply_pose(
        &self,
        tr: na::Vector3<f32>,
        rot: na::UnitQuaternion<f32>,
    ) -> (na::Point3<f32>, na::UnitQuaternion<f32>) {
        let (pos, old_rot) = self.poses.last().copied().unwrap_or_default();
        (pos + old_rot * tr, rot * old_rot)
    }

    fn layer_frame(
        &self,
        pos: na::Point3<f32>,
        rot: na::UnitQuaternion<f32>,
        settings: &EstimatorSettings,
    ) -> bool {
        settings.layer_frames && {
            self.layered_frames
                .last()
                .map(|(f, _)| {
                    let (op, or) = self.poses[*f];
                    (op - pos).magnitude() > 0.1
                        || rot.angle_to(&or) > settings.layer_angle_delta.to_radians()
                })
                .unwrap_or(true)
        }
    }

    fn push_pose(
        &mut self,
        pos: na::Point3<f32>,
        rot: na::UnitQuaternion<f32>,
        frame: Option<Arc<Material>>,
    ) {
        let idx = self.poses.len();
        self.poses.push((pos, rot));
        if let Some(mat) = frame {
            self.layered_frames.push((idx, mat));
        }
    }

    fn layered_frames(
        &self,
    ) -> impl Iterator<Item = (na::Point3<f32>, na::UnitQuaternion<f32>, Arc<Material>)> + '_ {
        self.layered_frames
            .iter()
            .cloned()
            .map(move |(i, mat)| (self.poses[i].0, self.poses[i].1, mat))
    }

    fn remove_least_significant_frame(&mut self) {
        let (frame, _) = self
            .layered_frames
            .iter()
            .enumerate()
            .map(|(i, s)| (i, self.poses[s.0].1))
            .fold(None, |candidate, (frame, rot)| {
                let dist = self
                    .layered_frames
                    .iter()
                    .map(|s| self.poses[s.0].1.angle_to(&rot))
                    .sum::<f32>();

                if let Some((frame, cur_dist)) = candidate {
                    if dist >= cur_dist {
                        return Some((frame, cur_dist));
                    }
                }

                Some((frame, dist))
            })
            .unwrap_or_default();

        self.layered_frames.remove(frame);
    }
}

pub struct MotionTrackingApp {
    create_decoder_state: CreateDecoderUiState,
    decoder: Option<Box<dyn Decoder>>,
    decoder_camera: StandardCamera,
    estimators: Vec<(
        CreateEstimatorUiState,
        Option<(Box<dyn Estimator>, EstimatorState)>,
        EstimatorSettings,
    )>,
    motion_vectors: Vec<MotionEntry>,
    frame: Vec<RGBA>,
    frame_height: usize,
    frames: usize,
    cube_material: Option<Arc<Material>>,
    camera_controller: CameraController,
}

impl Default for MotionTrackingApp {
    fn default() -> Self {
        Self {
            create_decoder_state: Default::default(),
            decoder: None,
            decoder_camera: StandardCamera::new(39.6, 39.6 * 9.0 / 16.0),
            estimators: vec![],
            motion_vectors: vec![],
            frame: vec![],
            frame_height: 0,
            frames: 0,
            cube_material: None,
            camera_controller: Default::default(),
        }
    }
}

impl MotionTrackingApp {
    fn tracking_step(&mut self, renderer: &mut Renderer) {
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

            let mut mat = None;

            for ((estimator, estimator_state), settings) in self
                .estimators
                .iter_mut()
                .filter_map(|(_, s, settings)| s.as_mut().zip(Some(settings)))
            {
                if let Ok((rot, tr)) =
                    estimator.estimate(&self.motion_vectors, &self.decoder_camera, None)
                {
                    if !settings.layer_frames {
                        estimator_state.layered_frames.clear();
                    }

                    let (pos, rot) = estimator_state.apply_pose(tr, rot);
                    let mat = if estimator_state.layer_frame(pos, rot, settings) {
                        if mat.is_none() && self.frame_height > 0 {
                            if let Ok(texture) = renderer.texture_from_rgba(
                                None,
                                bytemuck::cast_slice(&self.frame),
                                self.frame_height,
                            ) {
                                mat = Some(
                                    Material::textured(
                                        renderer.pipeline_manager_mut(),
                                        texture.into(),
                                    )
                                    .into(),
                                )
                            }
                        }

                        mat.as_ref()
                    } else {
                        None
                    };

                    let mut cnt = 0;
                    while estimator_state.layered_frames.len() > settings.keep_frames && cnt < 20 {
                        estimator_state.remove_least_significant_frame();
                        cnt += 1;
                    }

                    estimator_state.push_pose(pos, rot, mat.cloned());
                }
            }
        } else {
            self.frames = 0;
            self.frame.clear();
            self.frame_height = 0;

            for (_, state) in self
                .estimators
                .iter_mut()
                .filter_map(|(_, v, _)| v.as_mut())
            {
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

        let range = 10isize;

        let line_colour = na::matrix![0.1; 0.1; 0.1; 1.0];
        let line_thickness = 2.0;

        for v in -range..=range {
            let v = v as f32;
            let range = range as f32;
            renderer.line(
                na::matrix![v; -range; 0.0].into(),
                na::matrix![v; range; 0.0].into(),
                line_thickness,
                line_colour,
            );
            renderer.line(
                na::matrix![-range; v; 0.0].into(),
                na::matrix![range; v; 0.0].into(),
                line_thickness,
                line_colour,
            );
        }

        let (offset_factor, x_scale) = {
            let intrinsics = self.decoder_camera.intrinsics();
            (
                intrinsics[(1, 1)] * 0.5,
                intrinsics[(1, 1)] / intrinsics[(0, 0)],
            )
        };

        for ((_, state), settings) in self
            .estimators
            .iter()
            .filter_map(|(_, v, s)| v.as_ref().zip(Some(s)))
        {
            let scale = settings.camera_offset / offset_factor;

            for (pos, rot, mat) in state.layered_frames() {
                renderer.obj(
                    Mesh::centered_quad(),
                    (pos * settings.scale_factor)
                        + rot * na::matrix![0.0; settings.camera_offset; 0.0],
                    rot * na::UnitQuaternion::from_euler_angles(-90.0f32.to_radians(), 0.0, 0.0),
                    na::matrix![x_scale * scale; scale; 1.0],
                    na::matrix![1.0; 1.0; 1.0; 1.0],
                    mat,
                );
            }
        }
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
        self.tracking_step(renderer);
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

            for (i, (state, estimator, settings)) in self.estimators.iter_mut().enumerate() {
                Grid::new(format!("estimator_ui_{i}")).show(ui, |ui| {
                    ui.label(format!("Estimator #{}", i));
                    ui.end_row();

                    ui.checkbox(&mut settings.layer_frames, "Draw frames");
                    ui.end_row();
                    ui.label("Angle delta");
                    ui.add(
                        Slider::new(&mut settings.layer_angle_delta, 0.01..=90.0)
                            .step_by(0.01)
                            .logarithmic(true),
                    );
                    ui.end_row();

                    ui.label("Keep frames");
                    ui.add(Slider::new(&mut settings.keep_frames, 1..=1000));
                    ui.end_row();

                    ui.label("Position scale");
                    ui.add(Slider::new(&mut settings.scale_factor, 0.00..=10.0));
                    ui.end_row();

                    ui.label("Frame offset");
                    ui.add(Slider::new(&mut settings.camera_offset, 0.00..=100.0).step_by(0.01));
                    ui.end_row();

                    if estimator.is_some() {
                        if ui.button("Stop").clicked() {
                            *estimator = None;
                        }
                        if ui.button("Remove").clicked() {
                            to_remove = Some(i);
                        }
                        ui.end_row();
                    }
                });
                if estimator.is_some() {
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
