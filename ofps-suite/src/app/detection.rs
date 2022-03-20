use super::utils::worker::{AppWorker, Workable};
use super::{
    ConfigState, CreateDecoderUiConfig, CreateDecoderUiState, CreatePluginUi, OfpsAppContext,
    OfpsCtxApp,
};
use egui::*;
use epi::Frame;
use ofps::prelude::v1::*;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use widgets::plot::{Arrows, Bar, BarChart, Line, Plot, Value, Values};
use wimrend::Renderer;

#[derive(Serialize, Deserialize)]
pub struct MotionDetectionConfig {
    decoder: (CreateDecoderUiConfig, bool),
    settings: MotionDetectionSettings,
    overlay_mf: bool,
}

struct MotionDetectionState {
    decoder: DecoderPlugin,
    detector: BlockMotionDetection,
    frames: usize,
    motion_ranges: Vec<(usize, usize)>,
}

impl Workable for MotionDetectionState {
    type Output = MotionDetectionOutput;
    type Settings = MotionDetectionSettings;
}

impl MotionDetectionState {
    fn new(decoder: DecoderPlugin) -> Self {
        Self {
            decoder,
            detector: Default::default(),
            frames: 0,
            motion_ranges: vec![],
        }
    }

    pub fn worker(decoder: DecoderPlugin, settings: MotionDetectionSettings) -> AppWorker<Self> {
        AppWorker::new(Self::new(decoder), settings, MotionDetectionState::update)
    }

    fn update(&mut self, out: &mut MotionDetectionOutput, settings: MotionDetectionSettings) {
        out.motion_vectors.clear();
        out.frame.clear();
        out.frame_height = 0;

        if self
            .decoder
            .process_frame(
                &mut out.motion_vectors,
                Some((&mut out.frame, &mut out.frame_height)),
                0,
            )
            .is_ok()
        {
            self.frames += 1;
        }

        out.frames = self.frames;

        self.detector.min_size = settings.min_size;
        self.detector.subdivide = settings.subdivide;
        self.detector.target_motion = settings.target_motion;

        out.motion = self
            .detector
            .detect_motion(out.motion_vectors.iter().copied());

        if out.motion.is_some() {
            match self.motion_ranges.last_mut() {
                Some((_, e)) if *e == out.frames => *e += 1,
                _ => self.motion_ranges.push((out.frames, out.frames + 1)),
            }
        }

        out.motion_ranges.clear();
        out.motion_ranges.extend(self.motion_ranges.iter().copied());
    }
}

#[derive(Clone, Copy, Serialize, Deserialize)]
struct MotionDetectionSettings {
    min_size: f32,
    subdivide: usize,
    target_motion: f32,
}

impl Default for MotionDetectionSettings {
    fn default() -> Self {
        Self {
            min_size: 0.05,
            subdivide: 3,
            target_motion: 0.003,
        }
    }
}

#[derive(Default)]
struct MotionDetectionOutput {
    motion_vectors: Vec<MotionEntry>,
    frame: Vec<RGBA>,
    frame_height: usize,
    frames: usize,
    motion: Option<(usize, MotionField)>,
    motion_ranges: Vec<(usize, usize)>,
}

#[derive(Default)]
pub struct MotionDetectionApp {
    create_decoder_state: CreateDecoderUiState,
    app_state: Option<AppWorker<MotionDetectionState>>,
    tex_handle: Option<TextureHandle>,
    overlay_mf: bool,
    settings: MotionDetectionSettings,
    config_state: ConfigState,
}

impl MotionDetectionApp {
    fn load_cfg(
        &mut self,
        ofps_ctx: &OfpsAppContext,
        MotionDetectionConfig {
            decoder,
            settings,
            overlay_mf,
        }: MotionDetectionConfig,
    ) {
        self.overlay_mf = overlay_mf;
        self.settings = settings;
        self.create_decoder_state.config = decoder.0;

        self.app_state = None;

        if decoder.1 {
            match DecoderPlugin::do_create(ofps_ctx, &mut self.create_decoder_state) {
                Ok(decoder) => {
                    self.app_state = Some(MotionDetectionState::worker(decoder, self.settings))
                }
                _ => {}
            }
        }
    }

    fn save_cfg(&self) -> MotionDetectionConfig {
        MotionDetectionConfig {
            decoder: (
                self.create_decoder_state.config.clone(),
                self.app_state.is_some(),
            ),
            settings: self.settings,
            overlay_mf: self.overlay_mf,
        }
    }
}

impl OfpsCtxApp for MotionDetectionApp {
    fn name(&self) -> &str {
        "Detection"
    }

    fn update(
        &mut self,
        ctx: &Context,
        ofps_ctx: &Arc<OfpsAppContext>,
        frame: &Frame,
        _render_list: &mut Renderer,
    ) {
        egui::SidePanel::left("detection_settings").show(ctx, |ui| {
            egui::trace!(ui);

            let mut config_state = std::mem::take(&mut self.config_state);
            config_state.run(
                ui,
                "Detection",
                Self::load_cfg,
                Self::save_cfg,
                self,
                ofps_ctx,
            );
            self.config_state = config_state;

            ui.separator();

            Grid::new("motion_settings").show(ui, |ui| {
                ui.label("Min size:");

                ui.add(Slider::new(&mut self.settings.min_size, 0.01..=1.0));

                ui.end_row();

                ui.label("Subdivisions:");

                ui.add(Slider::new(&mut self.settings.subdivide, 1..=16));

                ui.end_row();

                ui.label("Min magnitude:");

                ui.add(Slider::new(&mut self.settings.target_motion, 0.0001..=0.01));

                ui.end_row();

                ui.checkbox(&mut self.overlay_mf, "Overlay Motion");
            });

            ui.separator();

            if let Some(app_state) = &self.app_state {
                let _ = app_state.settings().map(|mut s| *s = self.settings);

                let mut app_state_lock = app_state.read();

                if let Ok(state) = &mut app_state_lock {
                    if !state.frame.is_empty() {
                        let image = ColorImage::from_rgba_unmultiplied(
                            [state.frame.len() / state.frame_height, state.frame_height],
                            bytemuck::cast_slice(&*state.frame),
                        );

                        if let Some(th) = self.tex_handle.as_mut() {
                            th.set(image);
                        } else {
                            self.tex_handle = Some(ui.ctx().load_texture("detector-frame", image));
                        }
                    }

                    let clicked_close = ui.button("Close").clicked();

                    Grid::new("motion_info").show(ui, |ui| {
                        ui.label("Motion:");

                        if let Some((motion, _)) = &state.motion {
                            ui.label(format!("{} blocks", motion));
                        } else {
                            ui.label("None");
                        }
                    });

                    ui.label("Dominant motion:");

                    Plot::new("dominant_motion")
                        .data_aspect(1.0)
                        .view_aspect(1.0)
                        .allow_drag(false)
                        .allow_boxed_zoom(false)
                        .center_x_axis(true)
                        .center_y_axis(true)
                        .show_x(false)
                        .show_y(false)
                        .show(ui, |plot_ui| {
                            if let Some((_, mf)) = &state.motion {
                                let values = mf
                                    .motion_iter()
                                    .filter(|(_, motion)| motion.magnitude() > 0.0)
                                    .map(|(_, motion)| motion * 10.0)
                                    .map(|motion| Value::new(motion.x, -motion.y))
                                    .collect::<Vec<_>>();

                                let arrows = Arrows::new(
                                    Values::from_values(vec![Value::new(0.0, 0.0); values.len()]),
                                    Values::from_values(values),
                                );
                                plot_ui.arrows(arrows);
                            }
                        });

                    std::mem::drop(app_state_lock);

                    if clicked_close {
                        self.app_state = None;
                    }
                } else {
                    std::mem::drop(app_state_lock);
                    self.app_state = None;
                }
            } else {
                ui.label("Open motion vectors");

                match DecoderPlugin::create_plugin_ui(
                    ui,
                    ofps_ctx,
                    &mut self.create_decoder_state,
                    0,
                    |_| {},
                ) {
                    Some(Ok(decoder)) => {
                        self.app_state =
                            Some(MotionDetectionState::worker(decoder, Default::default()))
                    }
                    _ => {}
                }
            }

            ui.separator();
        });

        let mut app_state_lock = self.app_state.as_ref().map(|s| s.read());

        egui::TopBottomPanel::bottom("detection_plot_window").show(ctx, |ui| {
            if let Some(Ok(state)) = &mut app_state_lock {
                ui.horizontal(|ui| {
                    ui.monospace(format!("{:>6}", state.frames));
                    ui.separator();
                    Plot::new("motion_window")
                        .include_x(0.0)
                        .include_x(
                            state
                                .motion_ranges
                                .last()
                                .map(|&(_, e)| e as f64)
                                .unwrap_or(0.0)
                                .max(20.0),
                        )
                        .center_y_axis(true)
                        .allow_drag(false)
                        .allow_boxed_zoom(false)
                        .show_x(false)
                        .show_y(false)
                        .min_size(Vec2::new(1.0, 1.0))
                        .show_axes([true, false])
                        .show(ui, |plot_ui| {
                            plot_ui.bar_chart(
                                BarChart::new(
                                    state
                                        .motion_ranges
                                        .iter()
                                        .copied()
                                        .flat_map(|(s, e)| {
                                            let s = s as f64;
                                            let e = e as f64;
                                            [
                                                Bar::new(s + (e - s) * 0.5, 2.0).width(e - s),
                                                Bar::new(s + (e - s) * 0.5, -2.0).width(e - s),
                                            ]
                                        })
                                        .collect(),
                                )
                                .color(Color32::LIGHT_GREEN),
                            );
                            plot_ui.bar_chart(
                                BarChart::new(
                                    IntoIterator::into_iter([
                                        Bar::new(state.frames as f64 + 0.9995, 2.0).width(0.001),
                                        Bar::new(state.frames as f64 + 0.9995, -2.0).width(0.001),
                                    ])
                                    .collect(),
                                )
                                .color(Color32::RED),
                            );
                        });
                });
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            if let Some(Ok(state)) = &mut app_state_lock {
                let rect = if let Some(th) = self.tex_handle.as_ref() {
                    let aspect = th.aspect_ratio();

                    let aw = ui.available_width();
                    let ah = ui.available_height();

                    let aspect2 = aw / ah;

                    let size = if aspect >= aspect2 {
                        Vec2::new(aw, aw / aspect)
                    } else {
                        Vec2::new(ah * aspect, ah)
                    };

                    let rect = ui
                        .centered_and_justified(|ui| ui.image(th, size))
                        .response
                        .rect;

                    Align2::CENTER_CENTER.align_size_within_rect(size, rect)
                } else {
                    ui.clip_rect()
                };

                if let Some(((_, mf), true)) = state.motion.as_ref().zip(Some(self.overlay_mf)) {
                    let painter = ui.painter_at(rect);
                    let (w, h) = mf.dim();
                    let w = rect.width() / w as f32;
                    let h = rect.height() / h as f32;
                    let block = Vec2::new(w, h);

                    for (x, y, motion) in mf
                        .iter()
                        .filter(|(_, _, motion)| motion.magnitude() > 0.0)
                        .map(|(x, y, motion)| (x as f32, y as f32, motion))
                    {
                        let pos = Vec2::new(x * w, y * h);

                        let angle = (motion.x.atan2(motion.y) + std::f32::consts::PI)
                            / (2.0 * std::f32::consts::PI);
                        let color = color::Hsva::new(angle, 1.0, 1.0, motion.magnitude());

                        painter.rect_filled(
                            Rect {
                                min: rect.min + pos,
                                max: rect.min + pos + block,
                            },
                            Rounding::none(),
                            color,
                        )
                    }
                }
            }
        });
    }
}
