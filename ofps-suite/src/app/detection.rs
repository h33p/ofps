use super::utils::{
    perf_stats::*,
    properties::*,
    timer::Timer,
    ui_misc::{realtime_processing_fn, transparent_windows},
    worker::{AppWorker, Workable},
};
use super::widgets::{
    CreateDecoderUiConfig, CreateDecoderUiState, CreateDetectorUiConfig, CreateDetectorUiState,
    CreatePluginUi, FilePicker,
};
use super::{OfpsAppContext, OfpsCtxApp};
use egui::*;
use epi::Frame;
use itertools::*;
use ofps::prelude::v1::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use widgets::plot::{Arrows, Bar, BarChart, Line, Plot, Value, Values};
use wimrend::Renderer;

#[derive(Serialize, Deserialize, Clone)]
pub struct MotionDetectionAppSettings {
    worker: MotionDetectionSettings,
    overlay_mf: bool,
    max_frame_gap: usize,
    min_frames: usize,
    #[serde(default)]
    draw_perf_stats: DrawPerfStats,
}

impl Default for MotionDetectionAppSettings {
    fn default() -> Self {
        Self {
            worker: Default::default(),
            overlay_mf: false,
            max_frame_gap: 2,
            min_frames: 2,
            draw_perf_stats: Default::default(),
        }
    }
}

#[derive(Serialize, Deserialize)]
pub struct MotionDetectionConfig {
    decoder: (CreateDecoderUiConfig, bool),
    detector: (CreateDetectorUiConfig, bool),
    settings: MotionDetectionAppSettings,
}

struct MotionDetectionState {
    decoder: DecoderPlugin,
    detector: DetectorPlugin,
    frames: usize,
    motion_ranges: Vec<(usize, usize)>,
    detector_times: Vec<Duration>,
    decoder_times: Vec<Duration>,
    decoder_timer: Option<Timer>,
}

impl Workable for MotionDetectionState {
    type Output = MotionDetectionOutput;
    type Settings = MotionDetectionSettings;
}

impl MotionDetectionState {
    fn new(decoder: DecoderPlugin, detector: DetectorPlugin) -> Self {
        Self {
            decoder,
            detector,
            frames: 0,
            motion_ranges: vec![],
            decoder_times: vec![],
            detector_times: vec![],
            decoder_timer: None,
        }
    }

    pub fn worker(
        decoder: DecoderPlugin,
        detector: DetectorPlugin,
        settings: MotionDetectionSettings,
    ) -> AppWorker<Self> {
        AppWorker::new(
            Self::new(decoder, detector),
            settings,
            MotionDetectionState::update,
        )
    }

    fn update(
        &mut self,
        out: &mut MotionDetectionOutput,
        mut settings: MotionDetectionSettings,
    ) -> bool {
        out.motion_vectors.clear();
        out.frame.clear();
        out.frame_height = 0;

        Timer::handle_option(
            &mut self.decoder_timer,
            settings.realtime_processing,
            self.decoder
                .get_framerate()
                .filter(|f| *f > 0.0)
                .map(|f| Duration::from_secs_f64(1.0 / f)),
        );

        let timer = Instant::now();
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
        } else {
            return false;
        }

        self.decoder_times.push(timer.elapsed());
        out.frames = self.frames;

        transfer_props(
            self.detector.props_mut(),
            &settings.detector_properties,
            &mut out.detector_properties,
        );

        transfer_props(
            self.decoder.props_mut(),
            &settings.decoder_properties,
            &mut out.decoder_properties,
        );

        let timer = Instant::now();
        out.motion = self.detector.detect_motion(&out.motion_vectors);
        self.detector_times.push(timer.elapsed());

        if out.motion.is_some() {
            match self.motion_ranges.last_mut() {
                Some((_, e)) if *e == out.frames => *e += 1,
                _ => self.motion_ranges.push((out.frames, out.frames + 1)),
            }
        }

        out.motion_ranges.clear();
        out.motion_ranges.extend(self.motion_ranges.iter().copied());

        out.detector_times.clear();
        out.detector_times
            .extend(self.detector_times.iter().copied());

        out.decoder_times.clear();
        out.decoder_times.extend(self.decoder_times.iter().copied());

        true
    }
}

#[derive(Clone, Default, Serialize, Deserialize)]
struct MotionDetectionSettings {
    #[serde(default)]
    decoder_properties: BTreeMap<String, Property>,
    #[serde(default)]
    detector_properties: BTreeMap<String, Property>,
    #[serde(default)]
    realtime_processing: bool,
}

#[derive(Default)]
struct MotionDetectionOutput {
    motion_vectors: Vec<MotionEntry>,
    frame: Vec<RGBA>,
    frame_height: usize,
    frames: usize,
    motion: Option<(usize, MotionField)>,
    motion_ranges: Vec<(usize, usize)>,
    detector_times: Vec<Duration>,
    decoder_times: Vec<Duration>,
    decoder_properties: BTreeMap<String, Property>,
    detector_properties: BTreeMap<String, Property>,
}

impl MotionDetectionOutput {
    fn filtered_motion_ranges(
        &self,
        max_frame_gap: usize,
        min_frames: usize,
    ) -> impl Iterator<Item = (usize, usize)> + '_ {
        self.motion_ranges
            .iter()
            .copied()
            .coalesce(move |(a1, a2), (b1, b2)| {
                if b1 - a2 <= max_frame_gap {
                    Ok((a1, b2))
                } else {
                    Err(((a1, a2), (b1, b2)))
                }
            })
            .filter(move |(s, e)| e - s >= min_frames)
    }
}

#[derive(Default)]
pub struct MotionDetectionApp {
    create_decoder_state: CreateDecoderUiState,
    create_detector_state: CreateDetectorUiState,
    app_state: Option<AppWorker<MotionDetectionState>>,
    pending_decoder: Option<DecoderPlugin>,
    pending_detector: Option<DetectorPlugin>,
    tex_handle: Option<TextureHandle>,
    settings: MotionDetectionAppSettings,
    config_state: FilePicker,
    export_state: FilePicker,
}

impl MotionDetectionApp {
    fn load_cfg(
        &mut self,
        ofps_ctx: &OfpsAppContext,
        MotionDetectionConfig {
            decoder,
            detector,
            settings,
        }: MotionDetectionConfig,
    ) {
        self.settings = settings;
        self.create_decoder_state.config = decoder.0;

        self.app_state = None;

        if decoder.1 {
            self.pending_decoder =
                DecoderPlugin::do_create(ofps_ctx, &mut self.create_decoder_state).ok();
        }

        if detector.1 {
            self.pending_detector =
                DetectorPlugin::do_create(ofps_ctx, &mut self.create_detector_state).ok();
        }
    }

    fn save_cfg(&self) -> MotionDetectionConfig {
        MotionDetectionConfig {
            decoder: (
                self.create_decoder_state.config.clone(),
                self.app_state.is_some() || self.pending_decoder.is_some(),
            ),
            detector: (
                self.create_detector_state.config.clone(),
                self.app_state.is_some() || self.pending_detector.is_some(),
            ),
            settings: self.settings.clone(),
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
            config_state.show_config(
                ui,
                "Detection",
                Self::load_cfg,
                Self::save_cfg,
                self,
                ofps_ctx,
            );
            self.config_state = config_state;

            ui.separator();

            ScrollArea::vertical()
                .auto_shrink([false; 2])
                .show(ui, |ui| {
                    ui.heading("Decoder:");

                    ui.separator();

                    if let Some(app_state) = &self.app_state {
                        let _ = app_state
                            .settings()
                            .map(|mut s| *s = self.settings.worker.clone());

                        let mut app_state_lock = app_state.read();

                        if let Ok(state) = &mut app_state_lock {
                            let realtime_processing = realtime_processing_fn(
                                &mut self.settings.worker.realtime_processing,
                            );

                            let clicked_close = Grid::new("detector_options")
                                .show(ui, |ui| {
                                    let clicked_close = ui.button("Close").clicked();

                                    realtime_processing(ui);

                                    ui.end_row();

                                    clicked_close
                                })
                                .inner;

                            properties_grid_ui(
                                ui,
                                "decoder_properties",
                                &mut self.settings.worker.decoder_properties,
                                Some(&state.decoder_properties),
                            );

                            std::mem::drop(app_state_lock);

                            if clicked_close {
                                self.app_state = None;
                            }
                        } else {
                            std::mem::drop(app_state_lock);
                            self.app_state = None;
                        }
                    } else {
                        if let Some(decoder) = self.pending_decoder.as_mut() {
                            ui.label("Waiting for detector");
                            let clicked_close = ui.button("Close").clicked();

                            let props = decoder
                                .props()
                                .into_iter()
                                .map(|(n, p)| (n.to_string(), p))
                                .collect();

                            properties_grid_ui(
                                ui,
                                "decoder_properties",
                                &mut self.settings.worker.decoder_properties,
                                Some(&props),
                            );

                            if clicked_close {
                                self.pending_decoder = None;
                            }
                        } else {
                            ui.label("Open motion vectors");

                            match DecoderPlugin::create_plugin_ui(
                                ui,
                                ofps_ctx,
                                &mut self.create_decoder_state,
                                0,
                                realtime_processing_fn(
                                    &mut self.settings.worker.realtime_processing,
                                ),
                            ) {
                                Some(Ok(decoder)) => {
                                    self.pending_decoder = Some(decoder);
                                }
                                _ => {}
                            }

                            properties_grid_ui(
                                ui,
                                "decoder_properties",
                                &mut self.settings.worker.decoder_properties,
                                None,
                            );
                        }
                    }

                    ui.separator();

                    perf_stats_options(ui, &mut self.settings.draw_perf_stats);

                    let detector_props_ui =
                        |ui: &mut Ui,
                         settings: &mut MotionDetectionAppSettings,
                         detector_properties| {
                            Grid::new("detector_properties").show(ui, |ui| {
                                properties_ui(
                                    ui,
                                    &mut settings.worker.detector_properties,
                                    detector_properties,
                                );

                                ui.label("Max gap:");
                                ui.add(Slider::new(&mut settings.max_frame_gap, 0..=200));

                                ui.end_row();

                                ui.label("Min frames:");
                                ui.add(Slider::new(&mut settings.min_frames, 1..=200));

                                ui.end_row();

                                ui.checkbox(&mut settings.overlay_mf, "Overlay Motion");
                            });
                        };

                    ui.heading("Detector:");

                    ui.separator();

                    if let Some(detector) = self.pending_detector.as_mut() {
                        ui.label("Waiting for decoder");

                        let clicked_close = ui.button("Close").clicked();

                        let props = detector
                            .props()
                            .into_iter()
                            .map(|(n, p)| (n.to_string(), p))
                            .collect();

                        detector_props_ui(ui, &mut self.settings, Some(&props));

                        if clicked_close {
                            self.pending_detector = None;
                        }
                    } else {
                        let app_state = self.app_state.as_ref();
                        let output = app_state.and_then(|a| a.read().ok());

                        let clicked_close = if let Some(state) = &output {
                            if !state.frame.is_empty() {
                                let image = ColorImage::from_rgba_unmultiplied(
                                    [state.frame.len() / state.frame_height, state.frame_height],
                                    bytemuck::cast_slice(&*state.frame),
                                );

                                if let Some(th) = self.tex_handle.as_mut() {
                                    th.set(image);
                                } else {
                                    self.tex_handle =
                                        Some(ui.ctx().load_texture("detector-frame", image));
                                }
                            }

                            let export_state = &mut self.export_state;
                            let settings = &self.settings;

                            let clicked_close = Grid::new("motion_info")
                                .show(ui, |ui| {
                                    let clicked_close = ui.button("Close").clicked();

                                    if let Err(e) = export_state.show_custom(
                                        ui,
                                        (),
                                        |_, _, path| {
                                            let ranges = state
                                                .filtered_motion_ranges(
                                                    settings.max_frame_gap,
                                                    settings.min_frames,
                                                )
                                                .collect::<Vec<_>>();

                                            let file = std::fs::File::create(path)?;
                                            let mut writer = csv::Writer::from_writer(file);
                                            for range in ranges {
                                                writer.serialize(range)?;
                                            }
                                            Ok(())
                                        },
                                        |_, ui, is_waiting| {
                                            if is_waiting {
                                                ui.label("Waiting");
                                                None
                                            } else if ui.button("Export").clicked() {
                                                Some(true)
                                            } else {
                                                None
                                            }
                                        },
                                        || rfd::FileDialog::new().add_filter("CSV Files", &["csv"]),
                                        |path| path.with_extension("csv"),
                                    ) {
                                        log::error!("{e}");
                                    }

                                    ui.end_row();

                                    ui.label("Motion:");

                                    if let Some((motion, _)) = &state.motion {
                                        ui.label(format!("{} blocks", motion));
                                    } else {
                                        ui.label("None");
                                    }

                                    clicked_close
                                })
                                .inner;

                            detector_props_ui(
                                ui,
                                &mut self.settings,
                                output.as_ref().map(|a| &a.detector_properties),
                            );

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
                                            Values::from_values(vec![
                                                Value::new(0.0, 0.0);
                                                values.len()
                                            ]),
                                            Values::from_values(values),
                                        );
                                        plot_ui.arrows(arrows);
                                    }
                                });

                            clicked_close
                        } else {
                            ui.label("Open motion detector");

                            match DetectorPlugin::create_plugin_ui(
                                ui,
                                ofps_ctx,
                                &mut self.create_detector_state,
                                1,
                                |_| {},
                            ) {
                                Some(Ok(detector)) => {
                                    self.pending_detector = Some(detector);
                                }
                                _ => {}
                            }

                            detector_props_ui(
                                ui,
                                &mut self.settings,
                                output.as_ref().map(|a| &a.detector_properties),
                            );

                            false
                        };

                        std::mem::drop(output);
                        std::mem::drop(app_state);

                        if clicked_close {
                            self.app_state = None;
                        }
                    }
                });
        });

        if self.pending_decoder.is_some() && self.pending_detector.is_some() {
            let decoder = self.pending_decoder.take().unwrap();
            let detector = self.pending_detector.take().unwrap();

            self.app_state = Some(MotionDetectionState::worker(
                decoder,
                detector,
                self.settings.worker.clone(),
            ));
        }

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
                                        .filtered_motion_ranges(
                                            self.settings.max_frame_gap,
                                            self.settings.min_frames,
                                        )
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

        let draw_perf_stats = &mut self.settings.draw_perf_stats;
        let create_decoder_state = &self.create_decoder_state;
        let tex_handle = self.tex_handle.as_ref();
        let overlay_mf = self.settings.overlay_mf;

        egui::CentralPanel::default().show(ctx, |ui| {
            transparent_windows(ctx, || {
                let decoder = if let Some(Ok(state)) = &app_state_lock {
                    Some((
                        &create_decoder_state.config.selected_plugin,
                        &*state.decoder_times,
                        state,
                    ))
                } else {
                    None
                };

                let get_stats = decoder.as_ref().map(|(dnames, dtimes, state)| {
                    (
                        move || std::iter::once(("detector", &*state.detector_times)),
                        dnames.as_str(),
                        *dtimes,
                    )
                });

                perf_stats_windows(ctx, draw_perf_stats, get_stats);
            });

            if let Some(Ok(state)) = &mut app_state_lock {
                let rect = if let Some(th) = tex_handle {
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

                if let Some(((_, mf), true)) = state.motion.as_ref().zip(Some(overlay_mf)) {
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
