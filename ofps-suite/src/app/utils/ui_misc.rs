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
