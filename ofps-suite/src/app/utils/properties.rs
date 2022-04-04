use egui::*;
use ofps::plugins::properties::{Property, PropertyMut};
use std::collections::BTreeMap;

pub fn transfer_props(
    obj: Vec<(&str, PropertyMut)>,
    props: &BTreeMap<String, Property>,
    out: &mut BTreeMap<String, Property>,
) {
    out.clear();

    for (n, mut p) in obj {
        if let Some(np) = props.get(n) {
            p.set(np);
        }
        out.insert(n.into(), p.into());
    }
}

pub fn property_ui(ui: &mut Ui, n: &str, p: &mut Property) {
    ui.label(format!("{n}:"));

    match p {
        Property::String(s) => {
            ui.add(TextEdit::singleline(s));
        }
        Property::Bool(b) => {
            let tf = if *b { "True" } else { "False" };
            ui.checkbox(b, tf);
        }
        Property::Float(v) => {
            ui.add(Slider::new(&mut v.val, v.min..=v.max));
        }
        Property::Usize(v) => {
            ui.add(Slider::new(&mut v.val, v.min..=v.max));
        }
    }
}

pub fn properties_ui(
    ui: &mut Ui,
    settings_properties: &mut BTreeMap<String, Property>,
    in_properties: Option<&BTreeMap<String, Property>>,
) {
    if let Some(in_properties) = in_properties {
        settings_properties.clear();
    }

    // Add any properties available
    if let Some(in_properties) = in_properties {
        for (n, p) in in_properties {
            settings_properties
                .entry(n.clone())
                .or_insert_with(|| p.clone());
        }
    }

    for (n, p) in settings_properties {
        property_ui(ui, n, p);
        ui.end_row();
    }
}

pub fn properties_grid_ui(
    ui: &mut Ui,
    id: &str,
    settings_properties: &mut BTreeMap<String, Property>,
    in_properties: Option<&BTreeMap<String, Property>>,
) {
    Grid::new(id).show(ui, |ui| {
        properties_ui(ui, settings_properties, in_properties)
    });
}
