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

    fn render(&mut self, render_list: &mut Renderer) {
        let cube_material = if let Some(cube_material) = self.cube_material.clone() {
            cube_material
        } else {
            let cube_material = Arc::new(Material::unlit(render_list.pipeline_manager_mut()));
            self.cube_material = Some(cube_material.clone());
            cube_material
        };

        render_list.uniform_mut().update_projection(
            StandardCamera::new(90.0, 45.0).as_matrix(),
            Default::default(),
            na::UnitQuaternion::identity(),
        );

        render_list.line(
            na::matrix![0.0; 4.0; 0.0].into(),
            na::matrix![6.0; 6.0; 4.0].into(),
            4.0,
            na::matrix![1.0; 0.0; 0.0; 1.0],
        );

        render_list.obj(
            Mesh::cube(),
            na::matrix![0.0; 4.0; 0.0].into(),
            na::UnitQuaternion::identity(),
            na::matrix![1.0; 1.0; 1.0],
            na::matrix![1.0; 1.0; 0.0; 1.0],
            cube_material.clone(),
        );

        render_list.obj(
            Mesh::cube(),
            na::matrix![6.0; 6.0; 4.0].into(),
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
        render_list: &mut Renderer,
    ) {
        self.tracking_step();
        self.render(render_list);

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
