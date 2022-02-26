use log::*;
use motion_vectors::prelude::v1::*;
use nalgebra as na;
use rand::seq::SliceRandom;
use std::time::{Duration, Instant};

use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

mod renderer;
mod texture;

use renderer::{RenderError, RenderState};

fn reset_grid(grid: &mut Vec<na::Point2<f32>>) {
    grid.clear();
    fill_grid(grid);
}

fn fill_grid(grid: &mut Vec<na::Point2<f32>>) {
    let grid_cnt_x = 30;
    let grid_cnt_y = 30;

    for x in 1..grid_cnt_x {
        for y in 1..grid_cnt_y {
            let x = x as f32 / grid_cnt_x as f32;
            let y = y as f32 / grid_cnt_y as f32;
            grid.push(na::Point2::new(x, y));
        }
    }
}

fn solve_ypr_given(motion: &[(na::Point2<f32>, [na::Vector2<f32>; 4])]) -> na::Vector3<f32> {
    let dot = |a: usize, b: usize| move |vecs: &[na::Vector2<f32>]| vecs[a].dot(&vecs[b]);

    fn dot_map<T: Fn(&[na::Vector2<f32>]) -> f32>(
        motion: &[(na::Point2<f32>, [na::Vector2<f32>; 4])],
    ) -> (impl Fn(T) -> f32 + '_) {
        move |dot| motion.iter().map(|(_, v)| dot(v)).sum::<f32>()
    }

    let a = na::Matrix3::from_iterator(
        [
            dot(1, 1),
            dot(1, 2),
            dot(1, 3),
            dot(2, 1),
            dot(2, 2),
            dot(2, 3),
            dot(3, 1),
            dot(3, 2),
            dot(3, 3),
        ]
        .iter()
        .map(dot_map(&motion)),
    );

    let b = na::Matrix3x1::from_iterator(
        [dot(1, 0), dot(2, 0), dot(3, 0)]
            .iter()
            .map(dot_map(&motion)),
    );

    let decomp = a.lu();

    decomp.solve(&b).unwrap_or_default()
}

fn solve_ypr(field: &MotionVectors, camera: &impl MotionModel) -> na::Vector3<f32> {
    let motion = field
        .iter()
        .copied()
        .map(|(pos, motion)| {
            (
                pos,
                [motion, camera.yaw(pos), camera.pitch(pos), camera.roll(pos)],
            )
        })
        .collect::<Vec<_>>();

    solve_ypr_given(&motion)
}

fn solve_ypr_ransac(field: &MotionVectors, camera: &impl MotionModel) -> na::Vector3<f32> {
    let motion = field
        .iter()
        .copied()
        .map(|(pos, motion)| {
            (
                pos,
                [motion, camera.yaw(pos), camera.pitch(pos), camera.roll(pos)],
            )
        })
        .collect::<Vec<_>>();

    let mut best_fit = Default::default();
    let mut max_inliers = 0;
    let target_delta = 0.0001;

    let rng = &mut rand::thread_rng();

    // Even with 60% of outliers this would not reach this limit...
    for _ in 0..100 {
        /*if max_inliers as f32 / motion.len() as f32 > 0.6 {
            break;
        }*/

        let samples = motion.choose_multiple(rng, 4).copied().collect::<Vec<_>>();

        let fit = solve_ypr_given(samples.as_slice());

        let motion = motion
            .choose_multiple(rng, 400)
            .copied()
            .collect::<Vec<_>>();

        let inliers = motion
            .iter()
            .map(|(pos, vec)| (sample_ypr_model(fit, *pos, camera), vec[0], pos, vec))
            .filter(|(sample, actual, pos, vec)| {
                // Sum the (error / target_delta).min(1.0)

                if actual.magnitude() == 0.0 {
                    true
                } else {
                    /*let c = -*sample;
                    let d = *actual - *sample;

                    let t = (c.dot(&d) / d.dot(&d)).abs();

                    //println!("{}", t);

                    t <= target_delta*/
                    (*actual - *sample).magnitude()/* / actual.magnitude()*/ <= target_delta
                }
            })
            .map(|(a, b, pos, vec)| (*pos, *vec))
            .collect::<Vec<_>>();

        if inliers.len() > max_inliers {
            best_fit = solve_ypr_given(inliers.as_slice());
            max_inliers = inliers.len();
            println!("{} | {}", max_inliers, best_fit);
        }
    }

    best_fit
}

fn sample_ypr_model(
    model: na::Vector3<f32>,
    pos: na::Point2<f32>,
    camera: &impl MotionModel,
) -> na::Vector2<f32> {
    camera.yaw(pos) * model.x + camera.pitch(pos) * model.y + camera.roll(pos) * model.z
}

fn main() -> Result<()> {
    let input = std::env::args()
        .nth(1)
        .ok_or("Please supply a video file!")?;

    let seek = std::env::args()
        .nth(2)
        .and_then(|v| v.parse::<f64>().ok())
        .unwrap_or(0.0);

    let mut c = motion_loader::create_decoder(&input)?;

    env_logger::init();
    let event_loop = EventLoop::new();
    let window = WindowBuilder::new()
        .with_inner_size(winit::dpi::Size::Logical(winit::dpi::LogicalSize {
            width: 1920.0 * 0.7,
            height: 1080.0 * 0.7,
        }))
        .build(&event_loop)
        .unwrap();

    let mut state = pollster::block_on(RenderState::new(&window))?;

    let mut mf = MotionField::new(300, 300);
    let mut motion_vectors = vec![];

    let rate = match c.get_framerate() {
        Some(x) if x < 1000.0 && x > 0.0 => x,
        _ => 30f64,
    };

    println!("FRAMERATE: {}", rate);

    let interval = 1f64 / rate;

    let start = Instant::now() - Duration::from_secs_f64(seek);

    let mut cnt = 0;

    let mut rgba_frame = vec![];
    let mut rgba_width = 0;
    let mut tracked_objs = vec![];

    reset_grid(&mut tracked_objs);

    let mut contains_buf = vec![];

    let mut mouse_pos = None;

    let fov = 39.6;

    let camera = StandardCamera::new(fov, fov * 9.0 / 16.0);

    event_loop.run(move |event, _, control_flow| match event {
        Event::RedrawRequested(_) => {
            loop {
                let target_time = Duration::from_secs_f64(interval * cnt as f64);
                let curtime = start.elapsed();

                let delta = curtime.saturating_sub(target_time).as_secs_f64();
                let frames = (delta * rate).round() as usize;

                if frames > 0 {
                    let frame = if delta < 1.0 {
                        Some((&mut rgba_frame, &mut rgba_width))
                    } else {
                        None
                    };

                    let break_out = frame.is_some();

                    let skip_frames = 0; //frames - 1;

                    motion_vectors.clear();

                    if let Err(e) = c.process_frame(&mut motion_vectors, frame, skip_frames) {
                        error!("{}", e);
                        *control_flow = ControlFlow::Exit;
                        return;
                    }

                    let mut downscale_mf = mf.new_downscale();

                    for &(pos, motion) in &motion_vectors {
                        downscale_mf.add_vector(pos, motion);
                    }

                    if downscale_mf.interpolate_empty_cells().is_err() {
                        continue;
                    }

                    cnt += frames;

                    contains_buf.clear();
                    contains_buf.resize(mf.size(), 0.0);

                    let motion_estimate =
                        if let Some((r, t)) = camera.reconstruct_multiview(&motion_vectors) {
                            const EPS: f32 = 0.01745329f32;
                            let (r, p, y) = r.euler_angles();
                            na::Vector3::new(-p / EPS, -r / EPS, y / EPS)
                        } else {
                            solve_ypr(&motion_vectors, &camera)
                        };
                    //let mat = reconstruct::fundamental(&motion_vectors, 0.5);
                    //println!("{:?}", mat);

                    /*println!(
                        "{:.02} {:.02} {:.02}",
                        motion_estimate.x, motion_estimate.y, motion_estimate.z
                    );*/

                    let (w, h) = mf.dim();
                    if w > 0 && h > 0 {
                        if cnt < 0 {
                            fill_grid(&mut tracked_objs);
                        }

                        for (i, in_pos) in tracked_objs.iter_mut().enumerate() {
                            let pos = na::clamp(
                                *in_pos,
                                na::Point2::new(0f32, 0f32),
                                na::Point2::new(1f32, 1f32),
                            );
                            let (x, y) = (
                                (pos.x * (w - 1) as f32).round() as usize,
                                (pos.y * (h - 1) as f32).round() as usize,
                            );
                            let motion = mf.get_motion(x, y);
                            //if motion.magnitude() > 0.1 {
                            //println!("{}: ({}; {})", i, x, y);
                            contains_buf[x + y * w] += 1.0;
                            //}

                            let motion = if cnt < 0 {
                                camera.yaw(*in_pos)
                            } else {
                                sample_ypr_model(motion_estimate, *in_pos, &camera)
                                //motion
                            };

                            *in_pos += motion;

                            /*in_pos.x = in_pos.x % 1.0;

                            if in_pos.x < 0.0 {
                                in_pos.x += 1.0;
                            }*/

                            //*in_pos += motion;
                        }
                    }

                    if break_out {
                        break;
                    }
                } else {
                    break;
                }
            }

            state.update();
            match state.render(
                &mf,
                rgba_frame.as_slice(),
                rgba_width,
                contains_buf.as_slice(),
            ) {
                Ok(_) => {}
                Err(RenderError::Surface(e)) => {
                    match e {
                        // Reconfigure the surface if lost
                        wgpu::SurfaceError::Lost => state.resize(None),
                        // The system is out of memory, we should probably quit
                        wgpu::SurfaceError::OutOfMemory => *control_flow = ControlFlow::Exit,
                        // All other errors (Outdated, Timeout) should be resolved by the next frame
                        e => eprintln!("{:?}", e),
                    }
                }
                Err(RenderError::Other(e)) => {
                    error!("{}", e);
                    *control_flow = ControlFlow::Exit
                }
            }
        }
        Event::WindowEvent {
            ref event,
            window_id,
        } if window_id == window.id() && !state.input(event) => match event {
            WindowEvent::CloseRequested
            | WindowEvent::KeyboardInput {
                input:
                    KeyboardInput {
                        state: ElementState::Pressed,
                        virtual_keycode: Some(VirtualKeyCode::Escape),
                        ..
                    },
                ..
            } => *control_flow = ControlFlow::Exit,
            WindowEvent::KeyboardInput {
                input:
                    KeyboardInput {
                        state: ElementState::Pressed,
                        virtual_keycode: Some(VirtualKeyCode::Space),
                        ..
                    },
                ..
            } => reset_grid(&mut tracked_objs),
            WindowEvent::CursorMoved { position, .. } => {
                mouse_pos = Some(na::Point2::new(
                    position.x as f32 / state.size.width as f32,
                    position.y as f32 / state.size.height as f32,
                ))
            }
            WindowEvent::MouseInput {
                state: ElementState::Pressed,
                button: MouseButton::Left,
                ..
            } => {
                if let Some(pos) = mouse_pos {
                    tracked_objs.push(pos);
                }
            }
            WindowEvent::Resized(physical_size) => {
                state.resize(Some(*physical_size));
            }
            WindowEvent::ScaleFactorChanged { new_inner_size, .. } => {
                // new_inner_size is &&mut so we have to dereference it twice
                state.resize(Some(**new_inner_size));
            }
            _ => {}
        },
        Event::MainEventsCleared => {
            // RedrawRequested will only trigger once, unless we manually
            // request it.
            window.request_redraw();
        }
        _ => {}
    });
}
