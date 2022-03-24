use crate::egui_app::EguiApp;
use egui::*;
use epi::Frame;
use ofps::prelude::v1::*;
use std::sync::Arc;
use wimrend::Renderer;

mod detection;
mod tracking;
mod utils;
mod widgets;

const APPS: &[fn() -> Box<dyn OfpsCtxApp>] = &[
    || Box::new(detection::MotionDetectionApp::default()),
    || Box::new(tracking::MotionTrackingApp::default()),
];

pub struct OfpsAppContext {
    plugin_store: PluginStore,
}

impl Default for OfpsAppContext {
    fn default() -> Self {
        Self {
            plugin_store: PluginStore::new(),
        }
    }
}

pub struct OfpsApp {
    apps: Vec<Box<dyn OfpsCtxApp>>,
    selected_app: usize,
    ofps_ctx: Arc<OfpsAppContext>,
}

impl Default for OfpsApp {
    fn default() -> Self {
        Self {
            apps: APPS.iter().map(|&create| create()).collect(),
            selected_app: 0,
            ofps_ctx: Default::default(),
        }
    }
}

pub trait OfpsCtxApp {
    fn name(&self) -> &str;
    fn update(
        &mut self,
        ctx: &Context,
        ofps_ctx: &Arc<OfpsAppContext>,
        frame: &Frame,
        render_list: &mut Renderer,
    );

    fn late_update(
        &mut self,
        ctx: &Context,
        ofps_ctx: &Arc<OfpsAppContext>,
        frame: &Frame,
        render_list: &mut Renderer,
    ) {
    }
}

impl EguiApp for OfpsApp {
    fn name(&self) -> &str {
        "OFPS Suite"
    }

    fn update(&mut self, ctx: &Context, frame: &Frame, render_list: &mut Renderer) {
        TopBottomPanel::top("ofps_app_top_bar").show(ctx, |ui| {
            egui::trace!(ui);

            ui.horizontal_wrapped(|ui| {
                egui::widgets::global_dark_light_mode_switch(ui);

                ui.separator();

                for (i, app) in self.apps.iter().enumerate() {
                    if ui
                        .selectable_label(self.selected_app == i, app.name())
                        .clicked()
                    {
                        self.selected_app = i;
                    }
                }
            })
        });

        if let Some(app) = self.apps.get_mut(self.selected_app) {
            app.update(ctx, &self.ofps_ctx, frame, render_list);
        }
    }

    fn late_update(&mut self, ctx: &Context, frame: &Frame, render_list: &mut Renderer) {
        if let Some(app) = self.apps.get_mut(self.selected_app) {
            app.late_update(ctx, &self.ofps_ctx, frame, render_list);
        }
    }
}
