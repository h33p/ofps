use crate::egui_app::EguiApp;
use egui::*;
use epi::Frame;
use ofps::prelude::v1::*;
use std::sync::{Arc, Mutex};
use wimrend::Renderer;

mod detection;
mod tracking;
mod utils;

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

#[derive(Default)]
pub struct CreatePluginState<T> {
    selected_plugin: usize,
    arg: Arc<Mutex<String>>,
    extra: T,
}

impl CreatePluginUi for EstimatorPlugin {
    type Extra = ();

    fn available_plugins(ctx: &OfpsAppContext) -> Vec<String> {
        ctx.plugin_store.available_estimators()
    }

    fn create_plugin(ctx: &OfpsAppContext, plugin: &str, arg: String) -> Result<Self> {
        ctx.plugin_store.create_estimator(plugin, arg)
    }

    fn arg_ui(ui: &mut Ui, ctx: &OfpsAppContext, state: &mut CreatePluginState<Self::Extra>) {
        ui.label("Arguments:");

        let mut guard = state.arg.lock().unwrap();
        ui.add(TextEdit::singleline(&mut *guard));

        ui.end_row();
    }
}

impl CreatePluginUi for DecoderPlugin {
    type Extra = bool;

    fn available_plugins(ctx: &OfpsAppContext) -> Vec<String> {
        ctx.plugin_store.available_decoders()
    }

    fn create_plugin(ctx: &OfpsAppContext, plugin: &str, arg: String) -> Result<Self> {
        ctx.plugin_store.create_decoder(plugin, arg)
    }

    fn arg_ui(ui: &mut Ui, ctx: &OfpsAppContext, state: &mut CreatePluginState<Self::Extra>) {
        if ui
            .button(if state.extra {
                "Input Path:"
            } else {
                "Select File:"
            })
            .clicked()
        {
            state.extra = !state.extra;
        }

        let mut guard = state.arg.lock().unwrap();

        if state.extra {
            ui.add(TextEdit::singleline(&mut *guard));
        } else {
            if ui
                .button(if guard.is_empty() { "Open..." } else { &*guard })
                .clicked()
            {
                let arg = state.arg.clone();

                let task = rfd::AsyncFileDialog::new()
                    .add_filter("Video", &["mp4", "mov", "mkv", "h264"])
                    .add_filter("Optical Flow", &["flo"])
                    .add_filter("OFPS Motion Vectors", &["mvec"])
                    .pick_file();

                std::thread::spawn(move || {
                    pollster::block_on(async move {
                        let file = task.await;

                        if let Some(file) = file.as_ref().and_then(|f| f.path().to_str()) {
                            *arg.lock().unwrap() = file.to_string();
                        }
                    })
                });
            }
        }

        ui.end_row();
    }
}

pub type CreateDecoderUiState = CreatePluginState<<DecoderPlugin as CreatePluginUi>::Extra>;
pub type CreateEstimatorUiState = CreatePluginState<<EstimatorPlugin as CreatePluginUi>::Extra>;

pub trait CreatePluginUi: Sized {
    type Extra;

    fn available_plugins(ctx: &OfpsAppContext) -> Vec<String>;
    fn create_plugin(ctx: &OfpsAppContext, plugin: &str, arg: String) -> Result<Self>;
    fn arg_ui(ui: &mut Ui, ctx: &OfpsAppContext, state: &mut CreatePluginState<Self::Extra>);

    fn do_create(
        ctx: &OfpsAppContext,
        state: &mut CreatePluginState<Self::Extra>,
        plugins: Option<Vec<String>>,
    ) -> Result<Self>
    where
        Self: Sized,
    {
        let plugins = plugins.unwrap_or_else(|| Self::available_plugins(ctx));

        Self::create_plugin(
            ctx,
            &plugins[state.selected_plugin],
            state.arg.lock().unwrap().clone(),
        )
    }

    fn create_plugin_ui(
        ui: &mut Ui,
        ctx: &OfpsAppContext,
        state: &mut CreatePluginState<Self::Extra>,
        id: usize,
        extra_ui: impl FnOnce(&mut Ui),
    ) -> Option<Result<Self>> {
        Grid::new(format!("create_plugin_{id}"))
            .show(ui, |ui| {
                let plugins = Self::available_plugins(ctx);

                if !plugins.is_empty() && state.selected_plugin >= plugins.len() {
                    state.selected_plugin = 0;
                }

                ui.label("Plugin to use:");
                ComboBox::from_id_source(format!("plugin_select_{id}"))
                    .selected_text(
                        if let Some(target_plugin) = plugins.get(state.selected_plugin) {
                            target_plugin.clone()
                        } else {
                            "<empty>".to_string()
                        },
                    )
                    .show_ui(ui, |ui| {
                        for (i, plugin) in plugins.iter().enumerate() {
                            ui.selectable_value(&mut state.selected_plugin, i, plugin);
                        }
                    });
                ui.end_row();

                Self::arg_ui(ui, ctx, state);

                let ret = if ui.button("Create").clicked() && state.selected_plugin < plugins.len()
                {
                    Some(Self::do_create(ctx, state, Some(plugins)))
                } else {
                    None
                };

                extra_ui(ui);

                ret
            })
            .inner
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
}
