use anyhow::Result;
use egui::Context;
use egui_wgpu_backend::{RenderPass, ScreenDescriptor};
use egui_winit::State;
use epi::Frame;
use std::sync::{Arc, Mutex};
use wgpu::*;
use winit::{event::*, window::Window};

use wimrend::render_state::RenderSubState;
use wimrend::Renderer;

// Adapted from
// https://github.com/hasenbanck/egui_example/

pub trait EguiApp {
    fn name(&self) -> &str;
    fn update(&mut self, ctx: &Context, frame: &Frame, render_list: &mut Renderer);
}

/// GUI part of the render state.
pub struct EguiRenderState<T> {
    render_pass: RenderPass,
    state: State,
    context: egui::Context,
    app: T,
    repaint_signal: Arc<GlobalRepaintSignal>,
}

impl<T: EguiApp> RenderSubState for EguiRenderState<T> {
    type InitData = (T, Arc<GlobalRepaintSignal>);

    fn create(
        device: &Arc<Device>,
        format: TextureFormat,
        window: &Window,
        msaa_samples: u32,
        (app, repaint_signal): (T, Arc<GlobalRepaintSignal>),
    ) -> Self {
        Self {
            render_pass: RenderPass::new(device, format, msaa_samples),
            state: State::new(4096, window),
            context: Default::default(),
            app,
            repaint_signal,
        }
    }

    fn input(&mut self, event: &WindowEvent) -> bool {
        self.state.on_event(&self.context, event)
    }

    fn update(&mut self, window: &Window, renderer: &mut Renderer) {
        let input = self.state.take_egui_input(&window);
        self.context.begin_frame(input);
        let app_output = epi::backend::AppOutput::default();

        let frame = epi::Frame::new(epi::backend::FrameData {
            info: epi::IntegrationInfo {
                name: "ofps_suite",
                web_info: None,
                cpu_usage: None,
                native_pixels_per_point: Some(window.scale_factor() as _),
                prefer_dark_mode: None,
            },
            output: app_output,
            repaint_signal: self.repaint_signal.clone(),
        });

        self.app.update(&self.context, &frame, renderer);
    }

    fn render(
        &mut self,
        encoder: &mut CommandEncoder,
        color_attachment: &TextureView,
        config: &SurfaceConfiguration,
        device: &Device,
        window: &Window,
        queue: &Queue,
    ) -> Result<()> {
        // End the UI frame. We could now handle the output and draw the UI with the backend.
        let output = self.context.end_frame();
        let paint_jobs = self.context.tessellate(output.shapes);

        let screen_descriptor = ScreenDescriptor {
            physical_width: config.width,
            physical_height: config.height,
            scale_factor: window.scale_factor() as f32,
        };

        // Update textures.
        self.render_pass
            .add_textures(&device, &queue, &output.textures_delta)?;
        self.render_pass.remove_textures(output.textures_delta)?;
        self.render_pass
            .update_buffers(&device, &queue, &paint_jobs, &screen_descriptor);

        // Draw GUI.
        self.render_pass
            .execute(
                encoder,
                color_attachment,
                &paint_jobs,
                &screen_descriptor,
                None,
            )
            .map_err(<_>::into)
    }
}

/// Custom redraw event to be sent to the parent winit event loop.
pub enum GlobalEvent {
    RequestRedraw,
}

/// This is the repaint signal type that egui needs for requesting a repaint from another thread.
///
/// It sends the custom RequestRedraw event to the winit event loop.
pub struct GlobalRepaintSignal(pub Mutex<winit::event_loop::EventLoopProxy<GlobalEvent>>);

impl epi::backend::RepaintSignal for GlobalRepaintSignal {
    fn request_repaint(&self) {
        self.0
            .lock()
            .unwrap()
            .send_event(GlobalEvent::RequestRedraw)
            .ok();
    }
}
