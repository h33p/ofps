use super::utils::{
    camera_controller::CameraController,
    perf_stats::{perf_stats_options, perf_stats_windows, DrawPerfStats},
    properties::{properties_grid_ui, properties_ui, transfer_props},
    ui_misc::{jlabel, realtime_processing, realtime_processing_fn, transparent_windows},
};
use super::widgets::{
    CreateDecoderUiConfig, CreateDecoderUiState, CreateEstimatorUiConfig, CreateEstimatorUiState,
    CreatePluginUi, FileLoader, FilePicker,
};
use super::{OfpsAppContext, OfpsCtxApp};
use egui::*;
use nalgebra as na;
use ofps::prelude::v1::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use widgets::plot::{Line, LinkedAxisGroup, Plot, Value, Values};
use wimrend::material::Material;
use wimrend::mesh::Mesh;
use wimrend::Renderer;

mod worker;

use worker::{
    EstimatorSettings, EstimatorState, FrameState, TrackingSettings, TrackingState, TrackingWorker,
};

#[derive(Serialize, Deserialize, Clone, Copy, Default)]
pub struct TrackingErrorStatistics {
    frame: usize,
    error: f32,
    error_r: f32,
    error_p: f32,
    error_y: f32,
}

impl TrackingErrorStatistics {
    fn fold_to_tuple(
        (cnt, e, r, p, y): (usize, f32, f32, f32, f32),
        TrackingErrorStatistics {
            error,
            error_r,
            error_p,
            error_y,
            ..
        }: TrackingErrorStatistics,
    ) -> (usize, f32, f32, f32, f32) {
        (cnt + 1, error + e, error_r + r, error_p + p, error_y + y)
    }
}

#[derive(Serialize, Deserialize, Clone, Copy, Default)]
pub struct TrackingPoseStatistics {
    delta: f32,
    delta_r: f32,
    delta_p: f32,
    delta_y: f32,
    r: f32,
    p: f32,
    y: f32,
}

#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct DrawGroundTruth {
    summary_window: bool,
    delta_window: bool,
    error_window: bool,
    angles_window: bool,
}

impl Default for DrawGroundTruth {
    fn default() -> Self {
        Self {
            summary_window: true,
            delta_window: false,
            error_window: false,
            angles_window: false,
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct MotionTrackingConfig {
    #[serde(default)]
    decoder: (CreateDecoderUiConfig, bool),
    draw_grid: bool,
    view_fov: f32,
    view_focus_point: (f32, f32, f32),
    view_rot: (f32, f32, f32),
    view_dist: f32,
    #[serde(default)]
    ground_truth: (String, bool),
    #[serde(default)]
    draw_ground_truth: DrawGroundTruth,
    #[serde(default)]
    draw_perf_stats: DrawPerfStats,
    #[serde(default)]
    estimators: Vec<(CreateEstimatorUiConfig, bool, EstimatorSettings)>,
    camera_aspect: f32,
    camera_fov_y: f32,
    #[serde(default)]
    realtime_processing: bool,
    #[serde(default)]
    decoder_properties: BTreeMap<String, Property>,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy)]
pub struct GroundTruth {
    frame: usize,
    fov_x: f32,
    fov_y: f32,
    rot_w: f32,
    rot_i: f32,
    rot_j: f32,
    rot_k: f32,
    pos_x: f32,
    pos_y: f32,
    pos_z: f32,
}

impl GroundTruth {
    fn from_csv(reader: std::fs::File) -> Result<Vec<GroundTruth>> {
        csv::Reader::from_reader(reader)
            .deserialize()
            .map(|v| v.map_err(<_>::into))
            .collect::<Result<Vec<GroundTruth>>>()
    }

    fn rot(&self) -> na::UnitQuaternion<f32> {
        na::UnitQuaternion::from_quaternion(na::Quaternion::new(
            self.rot_w, self.rot_i, self.rot_j, self.rot_k,
        ))
    }

    fn deltas<'a>(
        truth: &'a [Self],
        poses: &'a [(na::Point3<f32>, na::UnitQuaternion<f32>)],
    ) -> impl Iterator<Item = (usize, na::UnitQuaternion<f32>)> + 'a {
        truth
            .iter()
            .filter_map(move |t| poses.get(t.frame - 1).zip(Some(t)))
            .map(|((_, rot), truth)| (truth.frame - 1, truth.rot().rotation_to(rot)))
    }

    fn gen_stats<'a>(
        truth: &'a [GroundTruth],
        state: &'a EstimatorState,
    ) -> impl Iterator<Item = (TrackingErrorStatistics, TrackingPoseStatistics)> + 'a {
        Self::calc_err(truth, &state.transforms)
            .zip(Self::deltas(truth, &state.poses))
            .map(move |(stats, (frame, rot))| {
                assert_eq!(stats.frame, frame);
                let total_rot = state.poses[frame].1;
                let (p, r, y) = total_rot.euler_angles();

                let delta = rot.angle();
                let (delta_p, delta_r, delta_y) = rot.euler_angles();

                (
                    stats,
                    TrackingPoseStatistics {
                        delta,
                        delta_r,
                        delta_p,
                        delta_y,
                        r,
                        p,
                        y,
                    },
                )
            })
    }

    fn calc_err<'a>(
        truth: &'a [Self],
        transforms: &'a [(na::Vector3<f32>, na::UnitQuaternion<f32>)],
    ) -> impl Iterator<Item = TrackingErrorStatistics> + 'a {
        truth
            .get(0)
            .into_iter()
            .chain(truth.iter())
            .zip(truth)
            .filter_map(move |(t0, t)| transforms.get(t.frame - 1).zip(Some((t0, t))))
            .map(|((_, rot), (prev_truth, truth))| {
                let q1 = prev_truth.rot();
                let q2 = truth.rot();
                let q = q1.rotation_to(&q2);
                // Swap roll and pitch since data is coming from blender
                let (p, r, y) = rot.euler_angles();
                let (pt, rt, yt) = q.euler_angles();
                let [error_r, error_p, error_y] =
                    [r - rt, p - pt, y - yt].map(|v| v.abs() % std::f32::consts::PI);

                TrackingErrorStatistics {
                    frame: truth.frame - 1,
                    error: rot.angle_to(&q),
                    error_r,
                    error_p,
                    error_y,
                }
            })
    }

    fn calc_avg_err(
        truth: &[Self],
        transforms: &[(na::Vector3<f32>, na::UnitQuaternion<f32>)],
    ) -> (f32, f32, f32, f32) {
        let (c, err, err_r, err_p, err_y) = Self::calc_err(truth, transforms)
            .fold(Default::default(), TrackingErrorStatistics::fold_to_tuple);
        let c = if c == 0 { 1.0 } else { c as f32 };
        (err / c, err_r / c, err_p / c, err_y / c)
    }
}

pub struct MotionTrackingApp {
    create_decoder_state: CreateDecoderUiState,
    app_state: Option<TrackingWorker>,
    app_settings: TrackingSettings,
    estimator_uis: Vec<CreateEstimatorUiState>,
    camera_controller: CameraController,
    draw_grid: bool,
    config_state: FilePicker,
    ground_truth: FileLoader<Vec<GroundTruth>>,
    ground_truth_link_axis: LinkedAxisGroup,
    draw_ground_truth: DrawGroundTruth,
    draw_perf_stats: DrawPerfStats,
}

impl Default for MotionTrackingApp {
    fn default() -> Self {
        Self {
            create_decoder_state: Default::default(),
            app_state: None,
            app_settings: Default::default(),
            estimator_uis: vec![],
            camera_controller: Default::default(),
            draw_grid: true,
            config_state: Default::default(),
            ground_truth: Default::default(),
            ground_truth_link_axis: LinkedAxisGroup::x(),
            draw_ground_truth: Default::default(),
            draw_perf_stats: Default::default(),
        }
    }
}

impl MotionTrackingApp {
    fn tracking_step(&mut self, renderer: &mut Renderer) {
        if let Some(state) = &self.app_state {
            // Load up any frames generated by worker onto GPU.
            for frame in state.frames_to_load.try_iter() {
                if let Ok(mut frame) = frame.lock() {
                    if let FrameState::Pending(buf, height) = &mut *frame {
                        if let Ok(texture) =
                            renderer.texture_from_rgba(None, bytemuck::cast_slice(buf), *height)
                        {
                            *frame = FrameState::Loaded(
                                Material::textured(renderer.pipeline_manager_mut(), texture.into())
                                    .into(),
                            )
                        } else {
                            *frame = FrameState::Pending(vec![], 0);
                        }
                    }
                }
            }
        }
    }

    fn render(&mut self, renderer: &mut Renderer) {
        if self.draw_grid {
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
        }

        let (offset_factor, x_scale) = {
            let intrinsics = self.app_settings.camera.intrinsics();
            (intrinsics[(1, 1)], intrinsics[(1, 1)] / intrinsics[(0, 0)])
        };

        // Render any layered frames
        if let Some(Ok(app_state)) = self.app_state.as_ref().map(|a| a.worker.read()) {
            for (state, settings) in app_state
                .estimators
                .iter()
                .zip(self.app_settings.settings.iter())
                .filter_map(|(v, (_, _, s))| v.as_ref().zip(Some(s)))
                .filter(|(_, s)| s.layer_frames)
            {
                let scale = settings.camera_offset / offset_factor;

                for (pos, rot, mat) in state.layered_frames() {
                    if let Ok(mat) = mat.lock() {
                        // Must ensure the frame is loaded.
                        if let FrameState::Loaded(mat) = &*mat {
                            renderer.obj(
                                Mesh::centered_quad(),
                                (pos * settings.scale_factor)
                                    + rot * na::matrix![0.0; settings.camera_offset; 0.0],
                                rot * na::UnitQuaternion::from_euler_angles(
                                    -90.0f32.to_radians(),
                                    0.0,
                                    0.0,
                                ),
                                na::matrix![x_scale * scale; scale; 1.0],
                                na::matrix![1.0; 1.0; 1.0; 1.0],
                                mat.clone(),
                            );
                        }
                    }
                }
            }
        }
    }

    fn load_cfg(
        &mut self,
        ofps_ctx: &OfpsAppContext,
        MotionTrackingConfig {
            decoder,
            draw_grid,
            view_fov,
            view_focus_point,
            view_rot,
            view_dist,
            ground_truth,
            draw_ground_truth,
            draw_perf_stats,
            estimators,
            camera_aspect,
            camera_fov_y,
            realtime_processing,
            decoder_properties,
        }: MotionTrackingConfig,
    ) {
        self.draw_grid = draw_grid;
        self.camera_controller.fov_y = view_fov;
        self.camera_controller.focus_point =
            na::Vector3::new(view_focus_point.0, view_focus_point.1, view_focus_point.2).into();
        self.camera_controller.rot =
            na::UnitQuaternion::from_euler_angles(view_rot.0, view_rot.1, view_rot.2);
        self.camera_controller.dist = view_dist;

        self.create_decoder_state.config = decoder.0;

        self.app_settings.camera = StandardCamera::new(camera_aspect, camera_fov_y);
        self.app_settings.settings.clear();
        self.app_settings.realtime_processing = realtime_processing;
        self.app_settings.decoder_properties = decoder_properties;

        self.ground_truth.data = None;
        self.ground_truth.path = ground_truth.0;
        if ground_truth.1 {
            self.ground_truth.load(GroundTruth::from_csv);
        }

        self.draw_ground_truth = draw_ground_truth;
        self.draw_perf_stats = draw_perf_stats;

        self.estimator_uis.clear();

        for (cfg, loaded, settings) in estimators {
            let mut ui = CreateEstimatorUiState::default();
            ui.config = cfg;

            let plugin = if loaded {
                match EstimatorPlugin::do_create(ofps_ctx, &mut ui) {
                    Ok(estimator) => Some(estimator),
                    _ => None,
                }
            } else {
                None
            };

            self.estimator_uis.push(ui);

            let loaded = Arc::from(AtomicBool::new(plugin.is_some()));

            self.app_settings
                .settings
                .push((Arc::from(Mutex::new(plugin)), loaded, settings))
        }

        self.app_state = if decoder.1 {
            if let Ok(decoder) = DecoderPlugin::do_create(ofps_ctx, &mut self.create_decoder_state)
            {
                Some(TrackingState::worker(decoder, self.app_settings.clone()))
            } else {
                None
            }
        } else {
            None
        };
    }

    fn save_cfg(&self) -> MotionTrackingConfig {
        let (_, camera_fov_y) = self.app_settings.camera.fov();
        let camera_aspect = self.app_settings.camera.aspect_ratio();

        MotionTrackingConfig {
            decoder: (
                self.create_decoder_state.config.clone(),
                self.app_state.is_some(),
            ),
            draw_grid: self.draw_grid,
            view_fov: self.camera_controller.fov_y,
            view_focus_point: {
                let p = self.camera_controller.focus_point;
                (p.x, p.y, p.z)
            },
            view_rot: self.camera_controller.rot.euler_angles(),
            view_dist: self.camera_controller.dist,
            ground_truth: (
                self.ground_truth.path.clone(),
                self.ground_truth.data.is_some(),
            ),
            draw_ground_truth: self.draw_ground_truth,
            draw_perf_stats: self.draw_perf_stats,
            estimators: self
                .estimator_uis
                .iter()
                .map(|ui| ui.config.clone())
                .zip(self.app_settings.settings.iter())
                .map(|(cfg, (_, loaded, settings))| {
                    (cfg, loaded.load(Ordering::Relaxed), settings.clone())
                })
                .collect(),
            camera_aspect,
            camera_fov_y,
            realtime_processing: self.app_settings.realtime_processing,
            decoder_properties: self.app_settings.decoder_properties.clone(),
        }
    }
}

impl OfpsCtxApp for MotionTrackingApp {
    fn name(&self) -> &str {
        "Tracking"
    }

    fn late_update(&mut self, ctx: &Context, _: &Arc<OfpsAppContext>, renderer: &mut Renderer) {
        self.camera_controller.update(ctx);
        self.camera_controller.on_render(renderer, ctx);
    }

    fn update(&mut self, ctx: &Context, ofps_ctx: &Arc<OfpsAppContext>, renderer: &mut Renderer) {
        self.tracking_step(renderer);

        self.render(renderer);

        egui::SidePanel::left("tracking_settings").show(ctx, |ui| {
            egui::trace!(ui);

            let mut config_state = std::mem::take(&mut self.config_state);
            if let Err(e) = config_state.show_config(
                ui,
                "Tracking",
                Self::load_cfg,
                Self::save_cfg,
                self,
                ofps_ctx,
            ) {
                log::error!("Error while running config widget: {e}");
            }
            self.config_state = config_state;

            ui.separator();

            ScrollArea::vertical()
                .auto_shrink([false; 2])
                .show(ui, |ui| {
                    ui.heading("UI:");

                    ui.separator();

                    Grid::new("tracking_ui".to_string()).show(ui, |ui| {
                        ui.checkbox(&mut self.draw_grid, "Draw grid");
                        ui.end_row();

                        ui.label("View FOV");

                        ui.add(Slider::new(&mut self.camera_controller.fov_y, 0.01..=179.0));
                        ui.end_row();
                    });

                    ui.separator();

                    ui.heading("Decoder:");

                    ui.separator();

                    if let Some(worker) = &mut self.app_state {
                        let settings = &mut self.app_settings;
                        let clicked_close = Grid::new("decoder_settings".to_string())
                            .show(ui, |ui| {
                                let clicked_close = ui.button("Close decoder").clicked();

                                realtime_processing(ui, &mut settings.realtime_processing);

                                ui.end_row();

                                let state = worker.worker.read().ok();

                                properties_ui(
                                    ui,
                                    &mut settings.decoder_properties,
                                    state.as_ref().and_then(|s| s.decoder_properties.as_ref()),
                                );

                                clicked_close
                            })
                            .inner;
                        if clicked_close {
                            self.app_state = None;
                        }
                    } else {
                        if let Some(Ok(decoder)) = DecoderPlugin::create_plugin_ui(
                            ui,
                            ofps_ctx,
                            &mut self.create_decoder_state,
                            0,
                            realtime_processing_fn(&mut self.app_settings.realtime_processing),
                        ) {
                            self.app_state =
                                Some(TrackingState::worker(decoder, self.app_settings.clone()))
                        }

                        properties_grid_ui(
                            ui,
                            "decoder_settings",
                            &mut self.app_settings.decoder_properties,
                            None,
                        );
                    }

                    ui.separator();

                    ui.heading("Camera:");

                    ui.separator();

                    Grid::new("camera_settings".to_string()).show(ui, |ui| {
                        let (_, mut fov_y) = self.app_settings.camera.fov();
                        let mut aspect = self.app_settings.camera.aspect_ratio();

                        ui.label("Aspect ratio");
                        ui.add(Slider::new(&mut aspect, 0.01..=5.0));
                        ui.end_row();

                        ui.label("Vertical FOV");
                        ui.add(Slider::new(&mut fov_y, 0.01..=179.0));
                        ui.end_row();

                        self.app_settings.camera = StandardCamera::new(aspect, fov_y);
                    });

                    ui.separator();

                    ui.heading("Ground truth:");

                    ui.separator();

                    let _ =
                        self.ground_truth
                            .show(ui, "ground_truth", GroundTruth::from_csv, || {
                                rfd::FileDialog::new().add_filter("CSV Files", &["csv"])
                            });

                    Grid::new("draw_ground_truth").show(ui, |ui| {
                        ui.checkbox(&mut self.draw_ground_truth.summary_window, "Draw summary");
                        ui.checkbox(&mut self.draw_ground_truth.angles_window, "Draw angles");
                        ui.end_row();

                        ui.checkbox(&mut self.draw_ground_truth.error_window, "Draw errors");
                        ui.checkbox(&mut self.draw_ground_truth.delta_window, "Draw deltas");
                        ui.end_row();
                    });

                    ui.separator();

                    perf_stats_options(ui, &mut self.draw_perf_stats);

                    ui.heading("Estimators:");

                    ui.separator();

                    let mut to_remove = None;

                    let app_state_lock = self.app_state.as_ref().and_then(|s| s.worker.read().ok());

                    {
                        for (i, (state, (est, exists, settings))) in self
                            .estimator_uis
                            .iter_mut()
                            .zip(self.app_settings.settings.iter_mut())
                            .enumerate()
                        {
                            let is_some = exists.load(Ordering::Relaxed);

                            Grid::new(format!("estimator_ui_{i}")).show(ui, |ui| {
                                ui.label(format!("Estimator #{}", i));
                                ui.end_row();

                                ui.checkbox(&mut settings.layer_frames, "Draw frames");

                                if ui.button("Clear frames").clicked() {
                                    settings.clear_count += 1;
                                }

                                ui.end_row();

                                ui.label("Keep frames");
                                ui.add(Slider::new(&mut settings.keep_frames, 1..=1000));
                                ui.end_row();

                                ui.label("Position scale");
                                ui.add(Slider::new(&mut settings.scale_factor, 0.00..=10.0));
                                ui.end_row();

                                ui.label("Frame offset");
                                ui.add(
                                    Slider::new(&mut settings.camera_offset, 0.00..=100.0)
                                        .step_by(0.01),
                                );
                                ui.end_row();

                                // If there is no app state, then update the properties here, as
                                // opposed to the worker.
                                if app_state_lock.is_none() {
                                    if let Ok(Some(est)) = est.lock().as_deref_mut() {
                                        transfer_props(
                                            est.props_mut(),
                                            &settings.properties.clone(),
                                            &mut settings.properties,
                                        );
                                    }
                                }

                                let props =
                                    app_state_lock.as_ref().and_then(|s| s.estimators.get(i));

                                let props = props
                                    .and_then(Option::as_ref)
                                    .and_then(|s| s.properties.as_ref());

                                properties_ui(ui, &mut settings.properties, props);

                                if is_some {
                                    if ui.button("Stop").clicked() {
                                        *est.lock().unwrap() = None;
                                        exists.store(false, Ordering::Relaxed);
                                    }
                                    if ui.button("Remove").clicked() {
                                        to_remove = Some(i);
                                    }
                                    ui.end_row();
                                }
                            });

                            if !is_some {
                                if let Some(Ok(new_estimator)) = EstimatorPlugin::create_plugin_ui(
                                    ui,
                                    ofps_ctx,
                                    state,
                                    i + 1,
                                    |ui| {
                                        if ui.button("Remove").clicked() {
                                            to_remove = Some(i);
                                        }
                                    },
                                ) {
                                    *est.lock().unwrap() = Some(new_estimator);
                                    exists.store(true, Ordering::Relaxed);
                                }
                            }

                            ui.separator();
                        }

                        if let Some(to_remove) = to_remove {
                            self.app_settings.settings.remove(to_remove);
                            self.estimator_uis.remove(to_remove);
                        }

                        if ui.button("New estimator").clicked() {
                            self.app_settings.settings.push(Default::default());
                            self.estimator_uis.push(Default::default());
                        }

                        if let Some(Ok(mut settings)) =
                            self.app_state.as_ref().map(|a| a.worker.settings())
                        {
                            *settings = self.app_settings.clone();
                        }
                    }
                });
        });

        transparent_windows(ctx, || {
            let state = self.app_state.as_ref().and_then(|a| a.worker.read().ok());
            let estimator_uis = &self.estimator_uis;
            let create_decoder_state = &self.create_decoder_state;

            let decoder = state.as_ref().map(|state| {
                (
                    create_decoder_state.config.selected_plugin.clone(),
                    &*state.decoder_times,
                    state,
                )
            });

            let get_stats = decoder.map(|(dnames, dtimes, state)| {
                (
                    move || {
                        state
                            .estimators
                            .iter()
                            .enumerate()
                            .filter_map(move |(i, est)| {
                                estimator_uis
                                    .get(i)
                                    .map(|ui| format!("{}_{i}", ui.config.selected_plugin))
                                    .zip(est.as_ref().map(|e| &*e.times))
                            })
                    },
                    dnames,
                    dtimes,
                )
            });

            perf_stats_windows(ctx, &mut self.draw_perf_stats, get_stats);

            if let Some(ground_truth) = &self.ground_truth.data {
                // Do copies, because that is easier than isolating self.
                let mut draw_ground_truth = self.draw_ground_truth;

                egui::Window::new("Error Summary")
                    .open(&mut draw_ground_truth.summary_window)
                    .show(ctx, |ui| {
                        ScrollArea::vertical()
                            .auto_shrink([true, true])
                            .show(ui, |ui| {
                                Grid::new("tracking_ui".to_string())
                                    .min_col_width(ui.spacing().interact_size.x + 30.0)
                                    .show(ui, |ui| {
                                        jlabel(ui, "Estimator");
                                        jlabel(ui, "Combo");
                                        jlabel(ui, "Roll");
                                        jlabel(ui, "Pitch");
                                        jlabel(ui, "Yaw");
                                        ui.end_row();

                                        for (i, (est, _)) in self
                                            .estimator_uis
                                            .iter()
                                            .zip(self.app_settings.settings.iter())
                                            .enumerate()
                                            .filter(|(_, (_, set))| set.1.load(Ordering::Relaxed))
                                        {
                                            jlabel(
                                                ui,
                                                format!("{}_{i}", est.config.selected_plugin),
                                            );
                                            if let Some(est) = state
                                                .as_ref()
                                                .and_then(|s| {
                                                    s.estimators.get(i).map(Option::as_ref)
                                                })
                                                .flatten()
                                            {
                                                let (err, err_r, err_p, err_y) =
                                                    GroundTruth::calc_avg_err(
                                                        ground_truth,
                                                        &est.transforms,
                                                    );
                                                for v in [err, err_r, err_p, err_y] {
                                                    jlabel(ui, format!("{:.03}°", v.to_degrees()));
                                                }
                                            } else {
                                                for _ in 0..4 {
                                                    jlabel(ui, "-");
                                                }
                                            }
                                            ui.end_row();
                                        }
                                    });
                            });

                        if let Some(state) = &state {
                            if ui.button("Export all stats").clicked() {
                                let all_stats = state
                                    .estimators
                                    .iter()
                                    .enumerate()
                                    .filter_map(|(i, est)| {
                                        self.estimator_uis
                                            .get(i)
                                            .map(|ui| {
                                                format!("{}_{i}", ui.config.selected_plugin.clone())
                                            })
                                            .zip(est.as_ref().map(|est| {
                                                GroundTruth::gen_stats(ground_truth, est)
                                                    .collect::<Vec<_>>()
                                            }))
                                    })
                                    .collect::<Vec<_>>();

                                std::thread::spawn(move || {
                                    if let Err(e) = (move || {
                                        if let Some(dir) = rfd::FileDialog::new().pick_folder() {
                                            std::fs::create_dir_all(&dir)?;
                                            for (name, stats) in all_stats {
                                                let mut path = dir.to_path_buf();
                                                path.push(format!("{name}.csv"));
                                                let file = std::fs::File::create(path)?;
                                                let mut writer = csv::Writer::from_writer(file);
                                                for stat in stats {
                                                    writer.serialize(stat)?;
                                                }
                                            }
                                        }
                                        std::io::Result::Ok(())
                                    })() {
                                        log::error!("Error while exporting stats: {}", e);
                                    }
                                });
                            }
                        } else {
                            ui.label("Start decoder to export");
                        }
                    });

                egui::Window::new("Rotation Angles")
                    .open(&mut draw_ground_truth.angles_window)
                    .show(ctx, |ui| {
                        Plot::new("rotation_graphs")
                            .legend(Default::default())
                            .link_axis(self.ground_truth_link_axis.clone())
                            .show(ui, |plot_ui| {
                                let mut vals = vec![];

                                let mut gt = [vec![], vec![], vec![]];

                                for truth in ground_truth {
                                    let r = truth.rot();
                                    // Again, swap pitch with roll
                                    let (p, r, y) = r.euler_angles();

                                    for (i, v) in [r, p, y].iter().enumerate() {
                                        gt[i].push(Value::new(
                                            (truth.frame - 1) as f32,
                                            v.to_degrees(),
                                        ));
                                    }
                                }

                                vals.push(("Ground truth".to_string(), gt));

                                if let Some(state) = &state {
                                    for (name, est) in
                                        state.estimators.iter().enumerate().filter_map(
                                            |(i, est)| {
                                                self.estimator_uis
                                                    .get(i)
                                                    .map(|ui| {
                                                        format!("{}_{i}", ui.config.selected_plugin)
                                                    })
                                                    .zip(est.as_ref())
                                            },
                                        )
                                    {
                                        let mut pred = [vec![], vec![], vec![]];

                                        for (frame, (_, rot)) in est.poses.iter().enumerate() {
                                            // Same swap here
                                            let (p, r, y) = rot.euler_angles();
                                            for (i, v) in [r, p, y].iter().enumerate() {
                                                pred[i]
                                                    .push(Value::new(frame as f32, v.to_degrees()));
                                            }
                                        }

                                        vals.push((name.clone(), pred));
                                    }
                                }

                                for (name, v) in vals {
                                    for (ty, v) in
                                        IntoIterator::into_iter(["roll", "pitch", "yaw"]).zip(v)
                                    {
                                        let v = Values::from_values(v);
                                        plot_ui.line(Line::new(v).name(format!("{name} {ty}")));
                                    }
                                }
                            });
                    });

                egui::Window::new("Error Graphs")
                    .open(&mut draw_ground_truth.error_window)
                    .show(ctx, |ui| {
                        if let Some(state) = &state {
                            Plot::new("error_graphs")
                                .legend(Default::default())
                                .link_axis(self.ground_truth_link_axis.clone())
                                .show(ui, |plot_ui| {
                                    for (name, est) in
                                        state.estimators.iter().enumerate().filter_map(
                                            |(i, est)| {
                                                self.estimator_uis
                                                    .get(i)
                                                    .map(|ui| {
                                                        format!("{}_{i}", ui.config.selected_plugin)
                                                    })
                                                    .zip(est.as_ref())
                                            },
                                        )
                                    {
                                        let mut errs = [
                                            ("total err", vec![]),
                                            ("roll err", vec![]),
                                            ("pitch err", vec![]),
                                            ("yaw err", vec![]),
                                        ];
                                        for TrackingErrorStatistics {
                                            frame,
                                            error,
                                            error_r,
                                            error_p,
                                            error_y,
                                        } in
                                            GroundTruth::calc_err(ground_truth, &est.transforms)
                                        {
                                            for (i, err) in [error, error_r, error_p, error_y]
                                                .iter()
                                                .enumerate()
                                            {
                                                errs[i].1.push(Value::new(
                                                    frame as f32,
                                                    err.to_degrees(),
                                                ));
                                            }
                                        }
                                        for (plot_name, errs) in errs {
                                            let errs = Values::from_values(errs);
                                            plot_ui.line(
                                                Line::new(errs).name(format!("{name} {plot_name}")),
                                            );
                                        }
                                    }
                                });
                        }
                    });

                egui::Window::new("Delta Graphs")
                    .open(&mut draw_ground_truth.delta_window)
                    .show(ctx, |ui| {
                        if let Some(state) = &state {
                            Plot::new("delta_graphs")
                                .legend(Default::default())
                                .link_axis(self.ground_truth_link_axis.clone())
                                .show(ui, |plot_ui| {
                                    for (name, est) in
                                        state.estimators.iter().enumerate().filter_map(
                                            |(i, est)| {
                                                self.estimator_uis
                                                    .get(i)
                                                    .map(|ui| {
                                                        format!("{}_{i}", ui.config.selected_plugin)
                                                    })
                                                    .zip(est.as_ref())
                                            },
                                        )
                                    {
                                        let mut dts = [
                                            ("total delta", vec![]),
                                            ("roll delta", vec![]),
                                            ("pitch delta", vec![]),
                                            ("yaw delta", vec![]),
                                        ];
                                        for (frame, rot) in
                                            GroundTruth::deltas(ground_truth, &est.poses)
                                        {
                                            let delta = rot.angle();
                                            let (delta_p, delta_r, delta_y) = rot.euler_angles();

                                            for (i, dt) in [delta, delta_r, delta_p, delta_y]
                                                .iter()
                                                .enumerate()
                                            {
                                                dts[i].1.push(Value::new(
                                                    frame as f32,
                                                    dt.to_degrees(),
                                                ));
                                            }
                                        }
                                        for (plot_name, dts) in dts {
                                            let dts = Values::from_values(dts);
                                            plot_ui.line(
                                                Line::new(dts).name(format!("{name} {plot_name}")),
                                            );
                                        }
                                    }
                                });
                        }
                    });

                self.draw_ground_truth = draw_ground_truth;
            }
        });
    }
}
