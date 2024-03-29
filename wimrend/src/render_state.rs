//! Complete render state.

use anyhow::{anyhow, Error, Result};
use log::*;
use std::sync::Arc;
use wgpu::*;
use winit::{event::*, window::Window};

use super::multisampler::Multisampler;
use super::texture::RawTexture;
use super::Renderer;

/// Render state including window being drawn and the GUI.
pub struct RenderState<T> {
    surface: Surface,
    device: Arc<Device>,
    /// Window being drawn to.
    pub window: Window,
    queue: Arc<Queue>,
    config: SurfaceConfiguration,
    /// Size of the window.
    pub size: winit::dpi::PhysicalSize<u32>,
    sub_state: T,
    depth_texture: RawTexture,
    renderer: Renderer,
    multisampler: Option<Multisampler>,
}

impl<T: RenderSubState> RenderState<T> {
    /// Create a new instance of the render state.
    ///
    /// # Arguments
    ///
    /// * `window` - window to draw to.
    /// * `app` - GUI app to draw.
    pub async fn new(window: Window, sub_state_data: T::InitData) -> Result<Self> {
        let size = window.inner_size();

        // The instance is a handle to our GPU
        let instance = Instance::new(util::backend_bits_from_env().unwrap_or_else(Backends::all));
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

        let device = Arc::new(device);
        let queue = Arc::new(queue);

        let surface_format = *surface
            .get_supported_formats(&adapter)
            .get(0)
            .ok_or_else(|| anyhow!("Failed to get preferred format"))?;

        let config = SurfaceConfiguration {
            usage: TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: size.width,
            height: size.height,
            present_mode: PresentMode::Fifo,
        };

        surface.configure(&device, &config);

        // Gl backend is buggy, no rendering occurs with depth texture and MSAA.
        let msaa_samples = if adapter.get_info().backend == Backend::Gl {
            1
        } else {
            4
        };

        let sub_state = T::create(
            &device,
            surface_format,
            &window,
            msaa_samples,
            sub_state_data,
        );
        let renderer = Renderer::new(device.clone(), queue.clone(), surface_format, msaa_samples);
        let depth_texture =
            RawTexture::create_depth_texture(&device, &config, "depth_texture", msaa_samples);

        let multisampler = if msaa_samples <= 1 {
            None
        } else {
            Some(Multisampler::new(surface_format, msaa_samples))
        };

        Ok(Self {
            surface,
            device,
            window,
            queue,
            config,
            size,
            depth_texture,
            sub_state,
            renderer,
            multisampler,
        })
    }

    /// Call when the window resizes.
    ///
    /// # Arguments
    ///
    /// * `new_size` - new dimensions of the window.
    pub fn resize(&mut self, new_size: Option<winit::dpi::PhysicalSize<u32>>) {
        let new_size = new_size.unwrap_or(self.size);

        if new_size.width > 0 && new_size.height > 0 {
            self.size = new_size;
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            trace!("Resize {} {}", new_size.width, new_size.height);
            self.surface.configure(&self.device, &self.config);
            self.depth_texture = RawTexture::create_depth_texture(
                &self.device,
                &self.config,
                "depth_texture",
                self.multisampler
                    .as_ref()
                    .map(Multisampler::sample_count)
                    .unwrap_or(1),
            );
        }
    }

    /// Handle input event.
    ///
    /// Returns true if the event should be handled exclusively by this instance and nobody else.
    ///
    /// # Arguments
    ///
    /// * `event` - event to check.
    pub fn input(&mut self, event: &WindowEvent) -> bool {
        self.sub_state.input(event)
    }

    /// Frame (pre-render) update.
    pub fn update(&mut self) {
        // Update screen resolution passed to shaders.
        self.renderer.uniform_mut().update_internal(&self.config);

        self.sub_state.update(&self.window, &mut self.renderer)
    }

    /// Render the frame to screen.
    pub fn render(&mut self) -> std::result::Result<(), Error> {
        let output = self.surface.get_current_texture()?;

        // Get the framebuffer view.
        let view = output
            .texture
            .create_view(&TextureViewDescriptor::default());

        // Load any new meshes onto GPU.
        self.renderer.mesh_manager.apply_changes();

        // Load all object information onto GPU.
        self.renderer.update_buffers();

        // Use multisampled texture, if it is enabled.
        let (view, resolve_target) = if let Some(msaa) = &mut self.multisampler {
            let new_view = msaa.target(&self.device, &self.config);
            (new_view, Some(&view))
        } else {
            (&view, None)
        };

        // Begin draw commands.
        let mut encoder = self
            .device
            .create_command_encoder(&CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        // The first render pass of the 3D scene.
        {
            let mut render_pass = encoder.begin_render_pass(&RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[Some(RenderPassColorAttachment {
                    view,
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
                })],
                depth_stencil_attachment: Some(RenderPassDepthStencilAttachment {
                    view: &self.depth_texture.view,
                    depth_ops: Some(Operations {
                        load: LoadOp::Clear(1.0),
                        store: false,
                    }),
                    stencil_ops: None,
                }),
            });

            self.renderer.render(&mut render_pass);
        }

        // Draw GUI on second render pass.
        // We could draw them all in one go, but egui does not expect to have depth images.
        self.sub_state.render(
            &mut encoder,
            view,
            &self.config,
            &self.device,
            &self.window,
            &self.queue,
        )?;

        // Resolve MSAA. We do it here, because, the substate can only have one view.
        if resolve_target.is_some() {
            let _ = encoder.begin_render_pass(&RenderPassDescriptor {
                label: Some("Resolve Pass"),
                color_attachments: &[Some(RenderPassColorAttachment {
                    view,
                    resolve_target,
                    ops: Operations {
                        load: LoadOp::Load,
                        store: false,
                    },
                })],
                depth_stencil_attachment: None,
            });
        }

        // Submit all commands.
        self.queue.submit(std::iter::once(encoder.finish()));

        // And display a new frame.
        output.present();

        // Clear any temporary state of the renderer.
        self.renderer.reset();

        Ok(())
    }
}

/// Additional substate.
///
/// This trait allows to add additional render elements, such as GUIs to the rendering process.
pub trait RenderSubState {
    /// Data needed to initialise this substate.
    type InitData;

    /// Create a render state with this substate.
    ///
    /// The render state creation function is async, thus a boxed future is being returned.
    ///
    /// # Arguments
    ///
    /// * `window` - window to be passed to the renderer.
    /// * `init_data` - initialisation data for this substate.
    fn create_state<'a>(
        window: Window,
        init_data: Self::InitData,
    ) -> core::pin::Pin<Box<dyn std::future::Future<Output = Result<RenderState<Self>>> + 'a>>
    where
        Self: Sized + 'a,
    {
        Box::pin(RenderState::new(window, init_data))
    }

    /// Create the substate.
    ///
    /// # Arguments
    ///
    /// * `device` - device that that will be rendererd to.
    /// * `surface_format` - format for all the textures.
    /// * `window` - window to be rendered to.
    /// * `init_data` - custom initialisation data of the substate.
    fn create(
        device: &Arc<Device>,
        surface_format: TextureFormat,
        window: &Window,
        msaa_samples: u32,
        init_data: Self::InitData,
    ) -> Self;

    /// Handle input event.
    ///
    /// Returns true if the event should be handled exclusively by this instance and nobody else.
    ///
    /// # Arguments
    ///
    /// * `event` - event to check.
    fn input(&mut self, event: &WindowEvent) -> bool;

    /// Render a frame of the substate.
    ///
    /// # Arguments
    ///
    /// * `encoder` - command encoder to use.
    /// * `view` - view to the framebuffer.
    /// * `config` - information about the screen.
    /// * `device` - device to render to.
    /// * `window` - window to draw to.
    /// * `queue` - command queue.
    fn render(
        &mut self,
        encoder: &mut CommandEncoder,
        view: &TextureView,
        config: &SurfaceConfiguration,
        device: &Device,
        window: &Window,
        queue: &Queue,
    ) -> Result<()>;

    /// Pre-render update.
    ///
    /// # Arguments
    ///
    /// * `window` - window that will be drawn to.
    /// * `renderer` - instance of the renderer.
    fn update(&mut self, window: &Window, renderer: &mut Renderer);
}
