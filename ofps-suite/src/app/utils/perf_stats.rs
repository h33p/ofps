use super::ui_misc::jlabel;
use egui::*;
use serde::{Deserialize, Serialize};
use std::time::Duration;
use widgets::plot::{Arrows, Bar, BarChart, Line, LinkedAxisGroup, Plot, Value, Values};

#[derive(Clone, Copy, Default, Serialize, Deserialize)]
pub struct DrawPerfStats {
    summary_window: bool,
    graph_window: bool,
}

pub fn perf_stats_options(ui: &mut Ui, stats: &mut DrawPerfStats) {
    ui.heading("Performance:");

    ui.separator();

    Grid::new("draw_performance").show(ui, |ui| {
        ui.checkbox(&mut stats.summary_window, "Draw summary");
        ui.checkbox(&mut stats.graph_window, "Draw graphs");
        ui.end_row();
    });

    ui.separator();
}

fn calc_perf(times: &[Duration]) -> (f32, f32) {
    let total = times.iter().map(Duration::as_secs_f32).sum::<f32>();
    let len = times.len();

    let len = if len > 0 { len as f32 } else { 1.0 };

    (total, total * 1000.0 / len)
}

pub fn perf_stats_windows<
    'a,
    F: Fn() -> I,
    I: Iterator<Item = (T, &'a [Duration])>,
    T: Into<WidgetText> + Clone + ToString + std::fmt::Display,
>(
    ctx: &Context,
    draw_perf_stats: &mut DrawPerfStats,
    estimator_and_decoder_stats: Option<(F, T, &'a [Duration])>,
) {
    egui::Window::new("Performance Summary")
        .open(&mut draw_perf_stats.summary_window)
        .show(ctx, |ui| {
            ScrollArea::vertical()
                .auto_shrink([true, true])
                .show(ui, |ui| {
                    Grid::new(format!("performance_ui"))
                        .min_col_width(ui.spacing().interact_size.x + 25.0)
                        .show(ui, |ui| {
                            jlabel(ui, "Part");
                            jlabel(ui, "Total Time");
                            jlabel(ui, "Avg Time");
                            ui.end_row();

                            if let Some((get_stats, dec_name, dec_times)) =
                                &estimator_and_decoder_stats
                            {
                                jlabel(ui, format!("Decoder {dec_name}"));

                                let (total_s, avg_ms) = calc_perf(dec_times);

                                jlabel(ui, format!("{:.03} s", total_s));
                                jlabel(ui, format!("{:.03} ms", avg_ms));

                                ui.end_row();

                                for (name, times) in get_stats() {
                                    jlabel(ui, name);

                                    let (total_s, avg_ms) = calc_perf(times);

                                    jlabel(ui, format!("{:.03} s", total_s));
                                    jlabel(ui, format!("{:.03} ms", avg_ms));

                                    ui.end_row();
                                }
                            }
                        });

                    if let Some((get_stats, dec_name, dec_times)) = &estimator_and_decoder_stats {
                        if ui.button("Export all stats").clicked() {
                            let all_stats = std::iter::once(("decoder".to_string(), *dec_times))
                                .chain(get_stats().map(|(name, times)| (name.to_string(), times)))
                                .map(|(name, times)| {
                                    (
                                        name,
                                        times
                                            .iter()
                                            .map(|d| d.as_secs_f32() * 1000.0)
                                            .collect::<Vec<_>>(),
                                    )
                                })
                                .collect::<Vec<_>>();

                            let dec_name = dec_name.to_string();

                            std::thread::spawn(move || {
                                if let Err(e) = (move || {
                                    if let Some(dir) = rfd::FileDialog::new().pick_folder() {
                                        std::fs::create_dir_all(&dir)?;
                                        for (name, stats) in all_stats {
                                            let mut path = dir.to_path_buf();
                                            path.push(format!("perf_{name}_{dec_name}.csv"));
                                            let file = std::fs::File::create(path)?;
                                            let mut writer = csv::Writer::from_writer(file);
                                            for stat in stats {
                                                writer.serialize(stat)?;
                                            }
                                        }
                                    }
                                    std::io::Result::Ok(())
                                })() {
                                    log::error!("Error while exporting stats: {}", e);
                                }
                            });
                        }
                    } else {
                        ui.label("Start decoder to export");
                    }
                });
        });

    egui::Window::new("Performance Graph")
        .open(&mut draw_perf_stats.graph_window)
        .show(ctx, |ui| {
            if let Some((get_stats, dec_name, dec_times)) = &estimator_and_decoder_stats {
                Plot::new("performance_graph")
                    .legend(Default::default())
                    //.link_axis(self.ground_truth_link_axis.clone())
                    .show(ui, |plot_ui| {
                        for (name, times) in
                            std::iter::once((dec_name.clone(), *dec_times)).chain(get_stats())
                        {
                            let mut timevals = vec![];

                            for (frame, time) in times.iter().enumerate() {
                                timevals
                                    .push(Value::new(frame as f32, time.as_secs_f32() * 1000.0));
                            }

                            let vals = Values::from_values(timevals);
                            plot_ui.line(Line::new(vals).name(name));
                        }
                    });
            }
        });
}
