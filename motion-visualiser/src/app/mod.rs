use egui::*;
use epi::{App, Frame};
use ofps::prelude::v1::*;
use std::sync::{Arc, Mutex};

mod detection;

const APPS: &[fn() -> Box<dyn App>] = &[|| Box::new(detection::MotionDetectionApp::default())];

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

impl CreatePluginUi for Box<dyn Decoder> {
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

pub type CreateDecoderUiState = CreatePluginState<<Box<dyn Decoder> as CreatePluginUi>::Extra>;

pub trait CreatePluginUi: Sized {
    type Extra;

    fn available_plugins(ctx: &OfpsAppContext) -> Vec<String>;
    fn create_plugin(ctx: &OfpsAppContext, plugin: &str, arg: String) -> Result<Self>;
    fn arg_ui(ui: &mut Ui, ctx: &OfpsAppContext, state: &mut CreatePluginState<Self::Extra>);

    fn create_plugin_ui(
        ui: &mut Ui,
        ctx: &OfpsAppContext,
        state: &mut CreatePluginState<Self::Extra>,
    ) -> Option<Result<Self>> {
        Grid::new("create_plugin")
            .show(ui, |ui| {
                let plugins = Self::available_plugins(ctx);

                if !plugins.is_empty() && state.selected_plugin >= plugins.len() {
                    state.selected_plugin = 0;
                }

                ui.label("Plugin to use:");
                ComboBox::from_id_source("plugin_select")
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

                if ui.button("Create").clicked() && state.selected_plugin < plugins.len() {
                    Some(Self::create_plugin(
                        ctx,
                        &plugins[state.selected_plugin],
                        state.arg.lock().unwrap().clone(),
                    ))
                } else {
                    None
                }
            })
            .inner
    }
}

pub struct OfpsApp {
    apps: Vec<Box<dyn App>>,
    selected_app: usize,
}

impl Default for OfpsApp {
    fn default() -> Self {
        Self {
            apps: APPS.iter().map(|&create| create()).collect(),
            selected_app: 0,
        }
    }
}

impl App for OfpsApp {
    fn name(&self) -> &str {
        "OFPS Suite"
    }

    fn update(&mut self, ctx: &Context, frame: &Frame) {
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
            app.update(ctx, frame);
        }
    }
}
