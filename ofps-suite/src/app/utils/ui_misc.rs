use egui::*;

pub fn jlabel(ui: &mut Ui, label: impl Into<WidgetText>) {
    ui.vertical_centered_justified(|ui| {
        ui.label(label);
    });
}

pub fn transparent_windows(ctx: &Context, handler: impl FnOnce()) {
    let mut visuals = ctx.style().visuals.clone();
    let prev_visuals = visuals.clone();
    let prev = visuals.widgets.noninteractive.bg_fill;
    visuals.widgets.noninteractive.bg_fill =
        Color32::from_rgba_unmultiplied(prev.r(), prev.g(), prev.b(), 150);
    ctx.set_visuals(visuals);

    handler();

    ctx.set_visuals(prev_visuals);
}

pub fn realtime_processing(ui: &mut Ui, realtime_processing: &mut bool) {
    ui.checkbox(realtime_processing, "Realtime")
        .on_hover_ui(|ui| {
            ui.label("Do not process frames faster than the framerate of the video.");
        });
}

pub fn realtime_processing_fn(realtime_processing: &mut bool) -> impl FnOnce(&mut Ui) + '_ {
    move |ui| self::realtime_processing(ui, realtime_processing)
}
