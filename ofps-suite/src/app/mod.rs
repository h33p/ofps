use crate::egui_app::EguiApp;
use egui::*;
use epi::Frame;
use ofps::prelude::v1::*;
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::thread::{spawn, JoinHandle};
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
pub struct FileLoader<T> {
    picker: FilePicker,
    path: String,
    data: Option<T>,
}

impl<T: for<'a> Deserialize<'a>> FileLoader<T> {
    pub fn load<E>(&mut self, deserialize: impl Fn(File) -> std::result::Result<T, E>) {
        if let Ok(Ok(new_data)) = File::open(&self.path).map(deserialize) {
            self.data = Some(new_data);
        }
    }

    pub fn run<E>(
        &mut self,
        ui: &mut Ui,
        id: &str,
        deserialize: impl Fn(File) -> std::result::Result<T, E>,
        dialog_builder: impl Fn() -> rfd::AsyncFileDialog,
    ) -> Result<()>
    where
        anyhow::Error: From<E>,
    {
        self.picker.run_custom(
            ui,
            (&mut self.path, &mut self.data),
            |(path, _), _, file| {
                **path = file.clone();
                Ok(())
            },
            |(path, data), ui, is_waiting| {
                Grid::new(id)
                    .show(ui, |ui| {
                        ui.label(if data.is_some() {
                            "Loaded file:"
                        } else {
                            "Pick file:"
                        });

                        let clicked = if is_waiting {
                            ui.label("Waiting...");
                            false
                        } else {
                            let p = Path::new(&path);
                            let filename = p.file_name().and_then(|p| p.to_str());
                            if data.is_some() {
                                ui.label(filename.unwrap_or(path));
                                false
                            } else {
                                ui.button(if let Some(filename) = &filename {
                                    filename
                                } else {
                                    "Open..."
                                })
                                .clicked()
                            }
                        };

                        ui.end_row();

                        if data.is_some() {
                            if ui.button("Close").clicked() {
                                **data = None;
                            }
                        } else {
                            if ui.button("Load").clicked() {
                                if let Ok(Ok(new_data)) = File::open(path).map(deserialize) {
                                    **data = Some(new_data);
                                }
                            }
                        }

                        ui.end_row();

                        if clicked {
                            Some(false)
                        } else {
                            None
                        }
                    })
                    .inner
            },
            dialog_builder,
            |path| path.into(),
        )
    }
}

#[derive(Default)]
pub struct FilePicker {
    thread: Option<(bool, Receiver<Option<String>>, JoinHandle<()>)>,
}

impl Drop for FilePicker {
    fn drop(&mut self) {
        if let Some((_, _, h)) = self.thread.take() {
            let _ = h.join();
        }
    }
}

impl FilePicker {
    pub fn run_config<I, T: Serialize + for<'a> Deserialize<'a>>(
        &mut self,
        ui: &mut Ui,
        name: &str,
        on_load: impl Fn(&mut I, &OfpsAppContext, T),
        on_save: impl Fn(&I) -> T,
        instance: &mut I,
        ctx: &OfpsAppContext,
    ) -> Result<()> {
        self.run_custom(
            ui,
            (),
            move |_, save, file| -> Result<()> {
                if save {
                    let cfg = on_save(instance);
                    let file = File::create(file)?;
                    serde_json::to_writer_pretty(file, &cfg)?;
                } else {
                    let file = File::open(file)?;
                    let cfg = serde_json::from_reader(file)?;
                    on_load(instance, ctx, cfg)
                }
                Ok(())
            },
            |_, ui, _| {
                ui.vertical_centered_justified(|ui| {
                    ui.menu_button(RichText::new(name).heading(), |ui| {
                        let mut save = None;
                        if ui.button("Load Config").clicked() {
                            save = Some(false);
                            ui.close_menu();
                        }
                        if ui.button("Save Config").clicked() {
                            save = Some(true);
                            ui.close_menu();
                        }
                        save
                    })
                })
                .inner
                .inner
                .flatten()
            },
            || rfd::AsyncFileDialog::new().add_filter("JSON Files", &["json"]),
            |path| path.with_extension("json"),
        )
    }

    pub fn run_custom<S>(
        &mut self,
        ui: &mut Ui,
        mut state: S,
        on_file: impl FnOnce(&mut S, bool, String) -> Result<()>,
        ui_fn: impl FnOnce(&mut S, &mut Ui, bool) -> Option<bool>,
        build_dialog: impl Fn() -> rfd::AsyncFileDialog,
        path_map: impl Fn(&Path) -> PathBuf + Send + 'static,
    ) -> Result<()> {
        if let Some((save, rx, h)) = self.thread.take() {
            if let Ok(file) = rx.try_recv() {
                h.join().expect("Failed to join");

                if let Some(file) = file {
                    on_file(&mut state, save, file)?;
                }
            } else {
                self.thread = Some((save, rx, h));
            }
        }

        if let (Some(save), None) = (ui_fn(&mut state, ui, self.thread.is_some()), &self.thread) {
            let (tx, rx) = mpsc::channel();
            let task = build_dialog();

            let h = spawn(move || {
                pollster::block_on(async move {
                    let file = if save {
                        task.save_file().await
                    } else {
                        task.pick_file().await
                    };

                    tx.send(
                        file.map(|fh| path_map(fh.path()))
                            .as_ref()
                            .and_then(|f| f.to_str())
                            .map(<_>::into),
                    )
                    .expect("Failed to send");
                })
            });
            self.thread = Some((save, rx, h));
        }

        Ok(())
    }
}

#[derive(Default, Clone, Serialize, Deserialize)]
pub struct CreatePluginConfig<T> {
    selected_plugin: String,
    arg: String,
    extra: T,
}

#[derive(Default)]
pub struct CreatePluginState<T> {
    config: CreatePluginConfig<T>,
    picker: FilePicker,
    //thread: Option<(Receiver<Option<String>>, JoinHandle<()>)>,
}

/*impl<T> Drop for CreatePluginState<T> {
    fn drop(&mut self) {
        if let Some((_, h)) = self.thread.take() {
            let _ = h.join();
        }
    }
}*/

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

        ui.add(TextEdit::singleline(&mut state.config.arg));

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
            .button(if state.config.extra {
                "Input Path:"
            } else {
                "Select File:"
            })
            .clicked()
        {
            state.config.extra = !state.config.extra;
        }

        if state.config.extra {
            ui.add(TextEdit::singleline(&mut state.config.arg));
        } else {
            let _ = state.picker.run_custom(
                ui,
                &mut state.config.arg,
                |arg, _, filename| -> Result<()> {
                    **arg = filename;
                    Ok(())
                },
                |arg, ui, is_waiting| {
                    let filename = Path::new(arg);

                    if is_waiting {
                        ui.label("Waiting...");
                        None
                    } else {
                        if ui
                            .button(
                                if let Some(name) = filename.file_name().and_then(|s| s.to_str()) {
                                    name
                                } else {
                                    "Open..."
                                },
                            )
                            .clicked()
                        {
                            Some(false)
                        } else {
                            None
                        }
                    }
                },
                || {
                    rfd::AsyncFileDialog::new()
                        .add_filter("Video", &["mp4", "mov", "mkv", "h264"])
                        .add_filter("Optical Flow", &["flo"])
                        .add_filter("OFPS Motion Vectors", &["mvec"])
                },
                |p| p.into(),
            );
        }

        ui.end_row();
    }
}

pub type CreateDecoderUiConfig = CreatePluginConfig<<DecoderPlugin as CreatePluginUi>::Extra>;
pub type CreateDecoderUiState = CreatePluginState<<DecoderPlugin as CreatePluginUi>::Extra>;

pub type CreateEstimatorUiConfig = CreatePluginConfig<<EstimatorPlugin as CreatePluginUi>::Extra>;
pub type CreateEstimatorUiState = CreatePluginState<<EstimatorPlugin as CreatePluginUi>::Extra>;

pub trait CreatePluginUi: Sized {
    type Extra;

    fn available_plugins(ctx: &OfpsAppContext) -> Vec<String>;
    fn create_plugin(ctx: &OfpsAppContext, plugin: &str, arg: String) -> Result<Self>;
    fn arg_ui(ui: &mut Ui, ctx: &OfpsAppContext, state: &mut CreatePluginState<Self::Extra>);

    fn do_create(ctx: &OfpsAppContext, state: &mut CreatePluginState<Self::Extra>) -> Result<Self>
    where
        Self: Sized,
    {
        Self::create_plugin(ctx, &state.config.selected_plugin, state.config.arg.clone())
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

                let mut selected_plugin = plugins
                    .iter()
                    .enumerate()
                    .filter(|(_, v)| v == &&state.config.selected_plugin)
                    .next()
                    .map(|(i, _)| i)
                    .unwrap_or(0);

                ui.label("Plugin to use:");
                ComboBox::from_id_source(format!("plugin_select_{id}"))
                    .selected_text(if let Some(target_plugin) = plugins.get(selected_plugin) {
                        target_plugin.clone()
                    } else {
                        "<empty>".to_string()
                    })
                    .show_ui(ui, |ui| {
                        for (i, plugin) in plugins.iter().enumerate() {
                            ui.selectable_value(&mut selected_plugin, i, plugin);
                        }
                    });
                ui.end_row();

                state.config.selected_plugin =
                    plugins.get(selected_plugin).cloned().unwrap_or_default();

                Self::arg_ui(ui, ctx, state);

                let ret = if ui.button("Create").clicked() && selected_plugin < plugins.len() {
                    Some(Self::do_create(ctx, state))
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
