use anyhow::Result;
use egui::Context;
use egui_wgpu_backend::{RenderPass, ScreenDescriptor};
use egui_winit::State;
use std::sync::Arc;
use wgpu::*;
use winit::{event::*, window::Window};

use wimrend::render_state::RenderSubState;
use wimrend::Renderer;

// Adapted from
// https://github.com/hasenbanck/egui_example/

pub trait EguiApp {
    fn name(&self) -> &str;
    fn update(&mut self, ctx: &Context, render_list: &mut Renderer);
    fn late_update(&mut self, ctx: &Context, render_list: &mut Renderer);
}

/// GUI part of the render state.
pub struct EguiRenderState<T> {
    render_pass: RenderPass,
    state: State,
    context: egui::Context,
    app: T,
}

impl<T: EguiApp> RenderSubState for EguiRenderState<T> {
    type InitData = T;

    fn create(
        device: &Arc<Device>,
        format: TextureFormat,
        window: &Window,
        msaa_samples: u32,
        app: T,
    ) -> Self {
        Self {
            render_pass: RenderPass::new(device, format, msaa_samples),
            state: State::new(4096, window),
            context: Default::default(),
            app,
        }
    }

    fn input(&mut self, event: &WindowEvent) -> bool {
        self.state.on_event(&self.context, event)
    }

    fn update(&mut self, window: &Window, renderer: &mut Renderer) {
        let input = self.state.take_egui_input(window);
        self.context.begin_frame(input);
        self.app.update(&self.context, renderer);
        self.app.late_update(&self.context, renderer);
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
            .add_textures(device, queue, &output.textures_delta)?;
        self.render_pass.remove_textures(output.textures_delta)?;
        self.render_pass
            .update_buffers(device, queue, &paint_jobs, &screen_descriptor);

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
