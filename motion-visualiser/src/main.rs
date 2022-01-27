use log::*;
use motion_vectors::prelude::v1::*;
use nalgebra as na;
use std::time::{Duration, Instant};

use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

mod renderer;
mod texture;

use renderer::{RenderError, RenderState};

use std::net::{TcpListener, TcpStream};

fn open_file(input: &str) -> Result<Box<dyn std::io::Read>> {
    if input.starts_with("tcp://") {
        let input = input.strip_prefix("tcp://").expect("Cannot strip prefix");
        let (addr, port) = input.split_once(":").ok_or("Invalid format")?;
        let port: usize = str::parse(port)?;

        let stream = if addr == "@" {
            let listener = TcpListener::bind(format!("0.0.0.0:{}", port))?;
            let (sock, addr) = listener.accept()?;
            println!("Accept {}", addr);
            sock
        } else {
            println!("Connecting to {}", input);
            TcpStream::connect(input)?
        };

        println!("Got stream!");

        Ok(Box::new(stream))
    } else {
        std::fs::File::open(input)
            .map(|i| Box::new(i) as _)
            .map_err(Into::into)
    }
}

fn create_decoder(input: &str) -> Result<impl Decoder> {
    let max_size = (300, 300);

    #[cfg(feature = "mvec-av")]
    let c = {
        let f = open_file(input)?;

        // Mul width by 2, because of character width
        let mut c = mvec_av::AvDecoder::try_new(f, (1, 1), max_size)?;

        c.av_ctx.dump_format();

        c
    };

    #[cfg(feature = "mvec-cv")]
    let c = mvec_cv::CvDecoder::try_new(input, (1, 1), max_size)?;

    Ok(c)
}

fn reset_grid(grid: &mut Vec<na::Point2<f32>>) {
    grid.clear();

    let grid_cnt = 20;

    for x in 1..grid_cnt {
        for y in 1..grid_cnt {
            let x = x as f32 / grid_cnt as f32;
            let y = y as f32 / grid_cnt as f32;
            grid.push(na::Point2::new(x, y));
        }
    }
}

fn main() -> Result<()> {
    let input = std::env::args()
        .nth(1)
        .ok_or("Please supply a video file!")?;

    let seek = std::env::args()
        .nth(2)
        .and_then(|v| v.parse::<f64>().ok())
        .unwrap_or(0.0);

    let mut c = create_decoder(&input)?;

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

    let mut mf = MotionField::new(0, 0);

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

                    let skip_frames = frames - 1;

                    if let Err(e) = c.process_frame(&mut mf, frame, skip_frames) {
                        error!("{}", e);
                        *control_flow = ControlFlow::Exit;
                        return;
                    }

                    cnt += frames;

                    contains_buf.clear();
                    contains_buf.resize(mf.size(), 0.0);

                    let (w, h) = mf.dim();
                    if w > 0 && h > 0 {
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
                            println!("{}: ({}; {})", i, x, y);
                            contains_buf[x + y * w] += 1.0;
                            //}
                            *in_pos += motion; // * 0.21 * 1.0;
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
