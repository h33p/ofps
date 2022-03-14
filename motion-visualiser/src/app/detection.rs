use super::{CreateDecoderUiState, CreatePluginUi, OfpsAppContext};
use egui::*;
use epi::{App, Frame};
use ofps::prelude::v1::*;
use std::sync::{Arc, Mutex};

#[derive(Default)]
pub struct MotionDetectionApp {
    ctx: OfpsAppContext,
    create_decoder_state: CreateDecoderUiState,
    decoder: Option<Box<dyn Decoder>>,
    detector: BlockMotionDetection,
    motion_vectors: Vec<MotionEntry>,
    frame: Vec<RGBA>,
    frame_height: usize,
    tex_handle: Option<TextureHandle>,
    mf: Option<MotionField>,
    overlay_mf: bool,
}

impl App for MotionDetectionApp {
    fn name(&self) -> &str {
        "Detection"
    }

    fn update(&mut self, ctx: &Context, frame: &Frame) {
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

                decoder
                    .process_frame(
                        &mut self.motion_vectors,
                        Some((&mut self.frame, &mut self.frame_height)),
                        0,
                    )
                    .ok();

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

                Grid::new("show_motion").show(ui, |ui| {
                    ui.label("Motion:");

                    if let Some((motion, mf)) = self
                        .detector
                        .detect_motion(self.motion_vectors.iter().copied())
                    {
                        ui.label(format!("{} blocks", motion));
                        self.mf = Some(mf);
                    } else {
                        ui.label("None");
                        self.mf = None;
                    }

                    ui.end_row();

                    ui.checkbox(&mut self.overlay_mf, "Overlay Motion");
                });

                use widgets::plot::{Arrows, Plot, Value, Values};

                ui.separator();

                ui.label("Dominant motion:");

                Plot::new("dominant_motion")
                    .data_aspect(1.0)
                    .view_aspect(1.0)
                    .allow_drag(false)
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
                    self.frame.clear();
                }
            } else {
                ui.label("Open motion vectors");

                match <Box<dyn Decoder>>::create_plugin_ui(
                    ui,
                    &self.ctx,
                    &mut self.create_decoder_state,
                ) {
                    Some(Ok(decoder)) => self.decoder = Some(decoder),
                    _ => {}
                }
            }

            ui.separator();
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
