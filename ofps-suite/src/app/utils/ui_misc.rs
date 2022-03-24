use egui::*;

pub fn jlabel(ui: &mut Ui, label: impl Into<WidgetText>) {
    ui.vertical_centered_justified(|ui| {
        ui.label(label);
    });
}
