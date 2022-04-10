use log::*;
use ofps::prelude::v1::*;
use std::sync::{Arc, Mutex};

use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

mod app;
mod egui_app;

use egui_app::{EguiRenderState, GlobalEvent, GlobalRepaintSignal};
use wgpu::SurfaceError;
use wimrend::render_state::RenderSubState;

fn main() -> Result<()> {
    env_logger::init();

    let event_loop = EventLoop::with_user_event();
    let window = WindowBuilder::new()
        .with_inner_size(winit::dpi::Size::Logical(winit::dpi::LogicalSize {
            width: 1920.0 * 0.7,
            height: 1080.0 * 0.7,
        }))
        .with_title("OFPS Suite")
        .build(&event_loop)
        .unwrap();

    let app = app::OfpsApp::default();

    let repaint_signal = Arc::new(GlobalRepaintSignal(Mutex::new(event_loop.create_proxy())));

    let mut state =
        pollster::block_on(EguiRenderState::create_state(window, (app, repaint_signal)))?;

    event_loop.run(move |event, _, control_flow| match event {
        Event::RedrawRequested(_) => {
            state.update();

            match state.render() {
                Ok(_) => {}
                Err(e) => {
                    if let Some(e) = e.downcast_ref::<SurfaceError>() {
                        match e {
                            // Reconfigure the surface if lost
                            wgpu::SurfaceError::Lost => state.resize(None),
                            // The system is out of memory, we should probably quit
                            wgpu::SurfaceError::OutOfMemory => *control_flow = ControlFlow::Exit,
                            // All other errors (Outdated, Timeout) should be resolved by the next frame
                            e => eprintln!("{:?}", e),
                        }
                    } else {
                        error!("{}", e);
                        *control_flow = ControlFlow::Exit
                    }
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
        Event::MainEventsCleared | Event::UserEvent(GlobalEvent::RequestRedraw) => {
            // RedrawRequested will only trigger once, unless we manually
            // request it.
            state.window.request_redraw();
        }
        _ => {}
    });
}
