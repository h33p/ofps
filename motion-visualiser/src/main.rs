use log::*;
use motion_vectors::prelude::v1::*;
use mvec_av::*;
use std::fs::File;
use std::time::{Duration, Instant};

use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

mod renderer;
mod texture;

use renderer::{RenderError, RenderState};

fn create_decoder(input: &str) -> Result<impl Decoder> {
    let f = File::open(input)?;

    let max_size = (80, 80);

    // Mul width by 2, because of character width
    let mut c = AvDecoder::try_new(f.into(), (1, 1), max_size)?;

    c.av_ctx.dump_format();

    Ok(c)
}

fn main() -> Result<()> {
    let input = std::env::args()
        .nth(1)
        .ok_or("Please supply a video file!")?;

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

    let rate = c.get_framerate().unwrap_or(24f64);

    println!("FRAMERATE: {}", rate);

    let interval = 1f64 / rate;

    let start = Instant::now();

    let mut cnt = 0;

    let mut rgba_frame = vec![];
    let mut rgba_width = 0;

    event_loop.run(move |event, _, control_flow| match event {
        Event::RedrawRequested(_) => {
            let target_time = Duration::from_secs_f64(interval * cnt as f64);
            let curtime = start.elapsed();

            if curtime >= target_time {
                if let Err(e) = c.process_frame(&mut mf, Some((&mut rgba_frame, &mut rgba_width))) {
                    error!("{}", e);
                    *control_flow = ControlFlow::Exit;
                    return;
                }

                cnt += 1;
            }

            state.update();
            match state.render(&mf, rgba_frame.as_slice(), rgba_width) {
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
