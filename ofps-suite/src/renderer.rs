use crate::texture::Texture;
use epi::App;
use log::*;
use ofps::prelude::v1::{Error, *};
use std::sync::{Arc, Mutex};
use wgpu::util::DeviceExt;
use wgpu::*;
use winit::{event::*, window::Window};

use egui_wgpu_backend::{RenderPass, ScreenDescriptor};
use egui_winit::State;

// Adapted from: https://sotrh.github.io/learn-wgpu/
// and https://github.com/hasenbanck/egui_example/

pub enum GuiEvent {
    RequestRedraw,
}

/// This is the repaint signal type that egui needs for requesting a repaint from another thread.
/// It sends the custom RequestRedraw event to the winit event loop.
pub struct GuiRepaintSignal(pub Mutex<winit::event_loop::EventLoopProxy<GuiEvent>>);

impl epi::backend::RepaintSignal for GuiRepaintSignal {
    fn request_repaint(&self) {
        self.0
            .lock()
            .unwrap()
            .send_event(GuiEvent::RequestRedraw)
            .ok();
    }
}

pub struct GuiRenderState {
    render_pass: RenderPass,
    state: State,
    context: egui::Context,
    app: Box<dyn App>,
}

impl GuiRenderState {
    fn new(device: &Device, format: TextureFormat, window: &Window, app: Box<dyn App>) -> Self {
        let size = window.inner_size();

        Self {
            render_pass: RenderPass::new(device, format, 1),
            state: State::new(4096, window),
            context: Default::default(),
            app,
        }
    }

    fn update(&mut self, window: &Window, repaint_signal: Arc<GuiRepaintSignal>) {
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
            repaint_signal,
        });

        self.app.update(&self.context, &frame);
    }

    fn render<'a>(
        &'a mut self,
        rpass: &mut wgpu::RenderPass<'a>,
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

        self.render_pass
            .add_textures(&device, &queue, &output.textures_delta)?;
        self.render_pass.remove_textures(output.textures_delta)?;
        self.render_pass
            .update_buffers(&device, &queue, &paint_jobs, &screen_descriptor);

        self.render_pass
            .execute_with_renderpass(rpass, &paint_jobs, &screen_descriptor)
            .map_err(<_>::into)
    }
}

pub struct RenderState {
    surface: Surface,
    device: Device,
    pub window: Window,
    queue: Queue,
    config: SurfaceConfiguration,
    pub size: winit::dpi::PhysicalSize<u32>,
    gui_state: GuiRenderState,
    render_pipeline: RenderPipeline,
    texture_bind_group_layout: wgpu::BindGroupLayout,
    vertex_buffer: Option<Buffer>,
    vertex_motion_buffer: Option<Buffer>,
    contains_buffer: Option<Buffer>,
    index_buffer: Option<Buffer>,
    texture: Option<Texture>,
    num_indices: u32,
}

impl RenderState {
    fn desc<'a, T, const N: u32>() -> VertexBufferLayout<'a> {
        VertexBufferLayout {
            array_stride: std::mem::size_of::<T>() as BufferAddress,
            step_mode: VertexStepMode::Vertex,
            attributes: &vertex_attr_array![N => Float32x2],
        }
    }

    // Creating some of the wgpu types requires async code
    pub async fn new(window: Window, app: Box<impl App + 'static>) -> Result<Self> {
        let size = window.inner_size();

        // The instance is a handle to our GPU
        // Backends::all => Vulkan + Metal + DX12 + Browser WebGPU
        let instance = Instance::new(Backends::all());
        let surface = unsafe { instance.create_surface(&window) };
        let adapter = instance
            .request_adapter(&RequestAdapterOptions {
                power_preference: PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .ok_or_else(|| anyhow!("Failed to get adapter"))?;

        let (device, queue) = adapter
            .request_device(
                &DeviceDescriptor {
                    features: Features::empty(),
                    limits: Limits::default(),
                    label: None,
                },
                None, // Trace path
            )
            .await?;

        let surface_format = surface
            .get_preferred_format(&adapter)
            .ok_or_else(|| anyhow!("Failed to get preferred format"))?;

        let config = SurfaceConfiguration {
            usage: TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width,
            height: size.height,
            present_mode: PresentMode::Fifo,
        };

        surface.configure(&device, &config);

        let gui_state = GuiRenderState::new(&device, surface_format, &window, app);

        let texture_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Texture {
                            multisampled: false,
                            view_dimension: wgpu::TextureViewDimension::D2,
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                        count: None,
                    },
                ],
                label: Some("texture_bind_group_layout"),
            });

        let shader = device.create_shader_module(&ShaderModuleDescriptor {
            label: Some("Screen Shader"),
            source: ShaderSource::Wgsl(include_str!("shaders/screen_shader.wgsl").into()),
        });

        let render_pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: Some("Render Pipeline Layout"),
            bind_group_layouts: &[&texture_bind_group_layout],
            push_constant_ranges: &[],
        });

        let render_pipeline = device.create_render_pipeline(&RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[
                    Self::desc::<[f32; 2], 0>(),
                    Self::desc::<[f32; 2], 1>(),
                    Self::desc::<f32, 2>(),
                ],
            },
            fragment: Some(FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[ColorTargetState {
                    format: config.format,
                    blend: Some(BlendState::REPLACE),
                    write_mask: ColorWrites::ALL,
                }],
            }),
            primitive: PrimitiveState {
                topology: PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: FrontFace::Ccw,
                cull_mode: Some(Face::Back),
                // Setting this to anything other than Fill requires Features::NON_FILL_POLYGON_MODE
                polygon_mode: PolygonMode::Fill,
                // Requires Features::DEPTH_CLIP_CONTROL
                unclipped_depth: false,
                // Requires Features::CONSERVATIVE_RASTERIZATION
                conservative: false,
            },
            depth_stencil: None,
            multisample: MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
        });

        Ok(Self {
            surface,
            device,
            window,
            queue,
            config,
            size,
            texture_bind_group_layout,
            gui_state,
            render_pipeline,
            vertex_buffer: None,
            vertex_motion_buffer: None,
            contains_buffer: None,
            index_buffer: None,
            texture: None,
            num_indices: 0,
        })
    }

    pub fn resize(&mut self, new_size: Option<winit::dpi::PhysicalSize<u32>>) {
        let new_size = new_size.unwrap_or(self.size);

        if new_size.width > 0 && new_size.height > 0 {
            self.size = new_size;
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            trace!("Resize {} {}", new_size.width, new_size.height);
            self.surface.configure(&self.device, &self.config);
        }
    }

    pub fn input(&mut self, event: &WindowEvent) -> bool {
        self.gui_state
            .state
            .on_event(&self.gui_state.context, event)
    }

    pub fn update(&mut self, repaint_signal: &Arc<GuiRepaintSignal>) {
        self.gui_state.update(&self.window, repaint_signal.clone())
    }

    pub fn render(&mut self) -> std::result::Result<(), RenderError> {
        let output = self.surface.get_current_texture()?;

        let view = output
            .texture
            .create_view(&TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        let bufs = match (
            &mut self.vertex_buffer,
            &mut self.vertex_motion_buffer,
            &mut self.contains_buffer,
            &mut self.index_buffer,
        ) {
            (Some(vbuf), Some(vmbuf), Some(cbuf), Some(ibuf)) => Some((vbuf, vmbuf, cbuf, ibuf)),
            _ => None,
        };

        /*if let Some((_, vmbuf, cbuf, _)) = &bufs {
            self.queue
                .write_buffer(vmbuf, 0, bytemuck::cast_slice(mfield.as_slice()));
            self.queue
                .write_buffer(cbuf, 0, bytemuck::cast_slice(contains_buf));
        }*/

        {
            let mut render_pass = encoder.begin_render_pass(&RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: Operations {
                        load: LoadOp::Clear(Color {
                            r: 0.1,
                            g: 0.2,
                            b: 0.3,
                            a: 1.0,
                        }),
                        store: true,
                    },
                }],
                depth_stencil_attachment: None,
            });

            if let Some((vbuf, vmbuf, cbuf, ibuf)) = bufs {
                render_pass.set_pipeline(&self.render_pipeline);
                if let Some(tex) = &self.texture {
                    render_pass.set_bind_group(0, &tex.bind_group, &[]);
                }
                render_pass.set_vertex_buffer(0, vbuf.slice(..));
                render_pass.set_vertex_buffer(1, vmbuf.slice(..));
                render_pass.set_vertex_buffer(2, cbuf.slice(..));
                render_pass.set_index_buffer(ibuf.slice(..), IndexFormat::Uint32);
                render_pass.draw_indexed(0..self.num_indices, 0, 0..1);
            }

            self.gui_state.render(
                &mut render_pass,
                &self.config,
                &self.device,
                &self.window,
                &self.queue,
            )?;
        }

        // submit will accept anything that implements IntoIter
        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}

pub enum RenderError {
    Surface(SurfaceError),
    Other(Error),
}

impl From<SurfaceError> for RenderError {
    fn from(err: SurfaceError) -> Self {
        Self::Surface(err)
    }
}

impl From<Error> for RenderError {
    fn from(err: Error) -> Self {
        Self::Other(err)
    }
}
