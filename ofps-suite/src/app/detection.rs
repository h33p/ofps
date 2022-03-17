use super::{CreateDecoderUiState, CreatePluginUi, OfpsAppContext, OfpsCtxApp};
use egui::*;
use epi::Frame;
use ofps::prelude::v1::*;
use std::sync::{Arc, Mutex};
use widgets::plot::{Arrows, Bar, BarChart, Line, Plot, Value, Values};
use wimrend::Renderer;

#[derive(Default)]
pub struct MotionDetectionApp {
    create_decoder_state: CreateDecoderUiState,
    decoder: Option<Box<dyn Decoder>>,
    detector: BlockMotionDetection,
    motion_vectors: Vec<MotionEntry>,
    frame: Vec<RGBA>,
    frame_height: usize,
    tex_handle: Option<TextureHandle>,
    mf: Option<MotionField>,
    overlay_mf: bool,
    frames: usize,
    motion_ranges: Vec<(usize, usize)>,
}

impl OfpsCtxApp for MotionDetectionApp {
    fn name(&self) -> &str {
        "Detection"
    }

    fn update(
        &mut self,
        ctx: &Context,
        ofps_ctx: &OfpsAppContext,
        frame: &Frame,
        _render_list: &mut Renderer,
    ) {
        egui::SidePanel::left("detection_settings").show(ctx, |ui| {
            egui::trace!(ui);

            ui.vertical_centered(|ui| {
                ui.heading("Detection");
            });

            ui.separator();

            if let Some(decoder) = &mut self.decoder {
                let clicked_close = ui.button("Close").clicked();

                self.motion_vectors.clear();

                self.frame.clear();
                self.frame_height = 0;

                if decoder
                    .process_frame(
                        &mut self.motion_vectors,
                        Some((&mut self.frame, &mut self.frame_height)),
                        0,
                    )
                    .is_ok()
                {
                    self.frames += 1;
                }

                if !self.frame.is_empty() {
                    let image = ColorImage::from_rgba_unmultiplied(
                        [self.frame.len() / self.frame_height, self.frame_height],
                        bytemuck::cast_slice(&*self.frame),
                    );

                    if let Some(th) = self.tex_handle.as_mut() {
                        th.set(image);
                    } else {
                        self.tex_handle = Some(ui.ctx().load_texture("detector-frame", image));
                    }
                }

                Grid::new("motion_settings").show(ui, |ui| {
                    ui.label("Motion:");

                    if let Some((motion, mf)) = self
                        .detector
                        .detect_motion(self.motion_vectors.iter().copied())
                    {
                        ui.label(format!("{} blocks", motion));
                        self.mf = Some(mf);

                        match self.motion_ranges.last_mut() {
                            Some((_, e)) if *e == self.frames => *e += 1,
                            _ => self.motion_ranges.push((self.frames, self.frames + 1)),
                        }
                    } else {
                        ui.label("None");
                        self.mf = None;
                    }

                    ui.end_row();

                    ui.label("Min size:");

                    ui.add(Slider::new(&mut self.detector.min_size, 0.01..=1.0));

                    ui.end_row();

                    ui.label("Subdivisions:");

                    ui.add(Slider::new(&mut self.detector.subdivide, 1..=16));

                    ui.end_row();

                    ui.label("Min magnitude:");

                    ui.add(Slider::new(&mut self.detector.target_motion, 0.0001..=0.01));

                    ui.end_row();

                    ui.checkbox(&mut self.overlay_mf, "Overlay Motion");
                });

                ui.separator();

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
                        if let Some(mf) = &self.mf {
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

                if clicked_close {
                    self.decoder = None;
                }
            } else {
                ui.label("Open motion vectors");

                self.frames = 0;
                self.tex_handle = None;
                self.motion_ranges.clear();
                self.mf = None;

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
        });

        egui::TopBottomPanel::bottom("detection_plot_window").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.monospace(format!("{:>6}", self.frames));
                ui.separator();
                Plot::new("motion_window")
                    .include_x(0.0)
                    .include_x(
                        self.motion_ranges
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
                                self.motion_ranges
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
                                    Bar::new(self.frames as f64 + 0.9995, 2.0).width(0.001),
                                    Bar::new(self.frames as f64 + 0.9995, -2.0).width(0.001),
                                ])
                                .collect(),
                            )
                            .color(Color32::RED),
                        );
                    });
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
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

            if let Some((mf, true)) = self.mf.as_ref().zip(Some(self.overlay_mf)) {
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
        });
    }
}
