use almeida_estimator::*;
use libmv_estimator::*;
use log::*;
use multiview_estimator::*;
use nalgebra as na;
use ofps::prelude::v1::*;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

mod app;
mod renderer;
mod texture;

use renderer::{GuiEvent, GuiRepaintSignal, RenderError, RenderState};

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
    env_logger::init();

    let event_loop = EventLoop::with_user_event();
    let window = WindowBuilder::new()
        .with_inner_size(winit::dpi::Size::Logical(winit::dpi::LogicalSize {
            width: 1920.0 * 0.7,
            height: 1080.0 * 0.7,
        }))
        .build(&event_loop)
        .unwrap();

    let app = app::OfpsApp::default();

    let mut state = pollster::block_on(RenderState::new(window, app.into()))?;

    let repaint_signal = Arc::new(GuiRepaintSignal(Mutex::new(event_loop.create_proxy())));

    event_loop.run(move |event, _, control_flow| match event {
        Event::RedrawRequested(_) => {
            state.update(&repaint_signal);

            match state.render() {
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
        } if window_id == state.window.id() && !state.input(event) => match event {
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
        Event::MainEventsCleared | Event::UserEvent(GuiEvent::RequestRedraw) => {
            // RedrawRequested will only trigger once, unless we manually
            // request it.
            state.window.request_redraw();
        }
        _ => {}
    });
}
