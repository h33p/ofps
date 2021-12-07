use log::*;
use motion_vectors::prelude::v1::*;
use wgpu::util::DeviceExt;
use wgpu::*;
use winit::{event::*, window::Window};

// Adapted from: https://sotrh.github.io/learn-wgpu/

pub struct RenderState {
    surface: Surface,
    device: Device,
    queue: Queue,
    config: SurfaceConfiguration,
    size: winit::dpi::PhysicalSize<u32>,
    render_pipeline: RenderPipeline,
    vertex_buffer: Option<Buffer>,
    vertex_motion_buffer: Option<Buffer>,
    index_buffer: Option<Buffer>,
    num_indices: u32,
}

impl RenderState {
    fn desc<'a, const N: u32>() -> VertexBufferLayout<'a> {
        VertexBufferLayout {
            array_stride: std::mem::size_of::<[f32; 2]>() as BufferAddress,
            step_mode: VertexStepMode::Vertex,
            attributes: &vertex_attr_array![N => Float32x2],
        }
    }

    // Creating some of the wgpu types requires async code
    pub async fn new(window: &Window) -> Result<Self> {
        let size = window.inner_size();

        // The instance is a handle to our GPU
        // Backends::all => Vulkan + Metal + DX12 + Browser WebGPU
        let instance = Instance::new(Backends::all());
        let surface = unsafe { instance.create_surface(window) };
        let adapter = instance
            .request_adapter(&RequestAdapterOptions {
                power_preference: PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .ok_or("Failed to get adapter")?;

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

        let config = SurfaceConfiguration {
            usage: TextureUsages::RENDER_ATTACHMENT,
            format: surface
                .get_preferred_format(&adapter)
                .ok_or("Failed to get preferred format")?,
            width: size.width,
            height: size.height,
            present_mode: PresentMode::Fifo,
        };

        surface.configure(&device, &config);

        let shader = device.create_shader_module(&ShaderModuleDescriptor {
            label: Some("Shader"),
            source: ShaderSource::Wgsl(include_str!("screen_shader.wgsl").into()),
        });

        let render_pipeline_layout = device.create_pipeline_layout(&PipelineLayoutDescriptor {
            label: Some("Render Pipeline Layout"),
            bind_group_layouts: &[],
            push_constant_ranges: &[],
        });

        let render_pipeline = device.create_render_pipeline(&RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[Self::desc::<0>(), Self::desc::<1>()],
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
                // Requires Features::DEPTH_CLAMPING
                clamp_depth: false,
                // Requires Features::CONSERVATIVE_RASTERIZATION
                conservative: false,
            },
            depth_stencil: None,
            multisample: MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
        });

        Ok(Self {
            surface,
            device,
            queue,
            config,
            size,
            render_pipeline,
            vertex_buffer: None,
            vertex_motion_buffer: None,
            index_buffer: None,
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
        false
    }

    pub fn update(&mut self) {}

    pub fn render(&mut self, mfield: &MotionField) -> std::result::Result<(), SurfaceError> {
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
            &mut self.index_buffer,
        ) {
            (Some(vbuf), Some(vmbuf), Some(ibuf)) => Some((vbuf, vmbuf, ibuf)),
            (vbuf, vmbuf, ibuf) if mfield.size() != 0 => {
                let vertex_motion_buffer = self.device.create_buffer(&BufferDescriptor {
                    label: Some("Vertex Motion Buffer"),
                    size: (mfield.size() * std::mem::size_of::<[f32; 2]>()) as _,
                    usage: BufferUsages::VERTEX | BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });

                let (width, height) = mfield.dim();

                let vertices = (0..height)
                    .flat_map(|y| (0..width).map(move |x| (x, y)))
                    .map(|(x, y)| {
                        [
                            // -1 is left and 1 is right
                            (x as f32 / (width - 1) as f32) * 2.0 - 1.0,
                            // 1 is up, and -1 is down
                            1.0 - (y as f32 / (height - 1) as f32) * 2.0,
                        ]
                    })
                    .enumerate()
                    .map(|(_, a)| a)
                    .collect::<Vec<_>>();

                let vertex_buffer = self.device.create_buffer_init(&util::BufferInitDescriptor {
                    label: Some("Vertex Buffer"),
                    contents: bytemuck::cast_slice(vertices.as_slice()),
                    usage: BufferUsages::VERTEX,
                });

                let vmap = |x, y| width as u32 * y + x;

                let indices = (0..(height - 1))
                    .flat_map(|y| (0..(width - 1)).map(move |x| (x, y)))
                    .map(|(x, y)| (x as u32, y as u32))
                    .flat_map(|(x, y)| {
                        std::array::IntoIter::new([
                            vmap(x, y),
                            vmap(x, y + 1),
                            vmap(x + 1, y),
                            vmap(x + 1, y),
                            vmap(x, y + 1),
                            vmap(x + 1, y + 1),
                        ])
                    })
                    .collect::<Vec<u32>>();

                self.num_indices = indices.len() as u32;
                println!("{} ({} {})", self.num_indices, width, height);

                let index_buffer = self.device.create_buffer_init(&util::BufferInitDescriptor {
                    label: Some("Index Buffer"),
                    contents: bytemuck::cast_slice(indices.as_slice()),
                    usage: BufferUsages::INDEX,
                });

                *vbuf = Some(vertex_buffer);
                *vmbuf = Some(vertex_motion_buffer);
                *ibuf = Some(index_buffer);

                Some((
                    vbuf.as_mut().unwrap(),
                    vmbuf.as_mut().unwrap(),
                    ibuf.as_mut().unwrap(),
                ))
            }
            _ => None,
        };

        if let Some((_, vmbuf, _)) = &bufs {
            self.queue
                .write_buffer(vmbuf, 0, bytemuck::cast_slice(mfield.as_slice()));
        }

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

            if let Some((vbuf, vmbuf, ibuf)) = bufs {
                render_pass.set_pipeline(&self.render_pipeline);
                render_pass.set_vertex_buffer(0, vbuf.slice(..));
                render_pass.set_vertex_buffer(1, vmbuf.slice(..));
                render_pass.set_index_buffer(ibuf.slice(..), IndexFormat::Uint32);
                render_pass.draw_indexed(0..self.num_indices, 0, 0..1);
            }
        }

        // submit will accept anything that implements IntoIter
        self.queue.submit(std::iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}
