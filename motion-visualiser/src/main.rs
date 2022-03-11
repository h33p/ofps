use almeida_estimator::*;
use libmv_estimator::*;
use log::*;
use multiview_estimator::*;
use nalgebra as na;
use ofps::prelude::v1::*;
use std::time::{Duration, Instant};

use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

mod renderer;
mod texture;

use renderer::{RenderError, RenderState};

fn reset_grid(
    grid: &mut Vec<na::Point3<f32>>,
    camera: &StandardCamera,
    rotation: na::UnitQuaternion<f32>,
    origin: na::Point3<f32>,
) {
    grid.clear();
    fill_grid(grid, camera, rotation, origin);
}

fn calc_view(rot: na::UnitQuaternion<f32>, pos: na::Point3<f32>) -> na::Matrix4<f32> {
    na::Matrix4::look_at_rh(
        &pos,
        &(pos + rot.transform_vector(&na::Vector3::new(0.0, 1.0, 0.0))),
        &rot.transform_vector(&na::Vector3::new(0.0, 0.0, 1.0)),
    )
}

fn fill_grid(
    grid: &mut Vec<na::Point3<f32>>,
    camera: &StandardCamera,
    rotation: na::UnitQuaternion<f32>,
    origin: na::Point3<f32>,
) {
    let grid_cnt_x = 30;
    let grid_cnt_y = 30;

    let mut new_points = vec![];

    for x in 1..grid_cnt_x {
        for y in 1..grid_cnt_y {
            let x = x as f32 / grid_cnt_x as f32;
            let y = y as f32 / grid_cnt_y as f32;
            new_points.push(na::Point2::new(x, y));
            /*let x = x * 2.0 - 1.0;
            let y = y * 2.0 - 1.0;
            let p = na::Vector3::new(x, 3.0, y);
            grid.push(origin + rotation * p);*/
        }
    }

    let view = calc_view(rotation, origin);

    grid.extend(
        new_points
            .into_iter()
            .inspect(|p| println!("{:?} | {:?}", p, camera.unproject(*p, view)))
            .map(|p| camera.unproject(p, view)),
    )
}

fn main() -> Result<()> {
    let input = std::env::args()
        .nth(1)
        .ok_or_else(|| anyhow!("Please supply a video file!"))?;

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

    let mut contains_buf = vec![];

    let mut mouse_pos = None;

    let fov = 39.6;

    let camera = StandardCamera::new(fov, fov * 9.0 / 16.0);

    let mut rot: na::UnitQuaternion<f32> = Default::default();
    let mut pos: na::Point3<f32> = Default::default();

    reset_grid(&mut tracked_objs, &camera, rot, pos);

    let mut estimator = MultiviewEstimator::default();
    //let mut estimator = LibmvEstimator::default();
    //let mut estimator = AlmeidaEstimator::default();

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

                    if let Err(e) = estimator.motion_step(
                        &motion_vectors,
                        &camera,
                        Some(0.1),
                        &mut rot,
                        &mut pos,
                    ) {
                        println!("Failed to estimate motion: {}", e);
                        continue;
                    }

                    //rot = Default::default();
                    //pos = Default::default();

                    //pos.x -= 0.1;
                    //rot *= na::UnitQuaternion::from_euler_angles(0.0, 0.01, 0.0);

                    println!("{:?} | {:?}", rot, pos);

                    let mut densify_mf = mf.new_densifier();

                    for &(pos, motion) in &motion_vectors {
                        densify_mf.add_vector(pos, motion);
                    }

                    if densify_mf.interpolate_empty_cells().is_err() {
                        continue;
                    }

                    cnt += frames;

                    contains_buf.clear();
                    contains_buf.resize(mf.size(), 0.0);

                    let (w, h) = mf.dim();
                    if w > 0 && h > 0 {
                        if cnt < 0 {
                            fill_grid(&mut tracked_objs, &camera, rot, pos);
                        }

                        let view = calc_view(rot, pos);

                        println!("{:?}", view);

                        for (i, in_pos) in tracked_objs.iter_mut().enumerate() {
                            //let in_pos = camera.project(*in_pos, view);
                            let in_pos = camera.project(*in_pos, view);

                            let pos = na::clamp(
                                in_pos,
                                na::Point2::new(0f32, 0f32),
                                na::Point2::new(1f32, 1f32),
                            );

                            let (x, y) = (
                                (pos.x * (w - 1) as f32).round() as usize,
                                (pos.y * (h - 1) as f32).round() as usize,
                            );
                            contains_buf[x + y * w] += 1.0;
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
            } => {
                rot = Default::default();
                pos = Default::default();
                //reset_grid(&mut tracked_objs, &camera, rot, pos),
            }
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
                if let Some(npos) = mouse_pos {
                    tracked_objs.push(camera.unproject(npos, calc_view(rot, pos)));
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
