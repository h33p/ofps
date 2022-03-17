//! Manager of render pipelines.

use std::collections::HashMap;
use std::sync::Arc;
use wgpu::*;

use super::texture::RawTexture;
use super::vertex::Vertex;
use super::RenderObject;

#[derive(PartialEq, Eq, Hash, Clone, Copy)]
pub struct RenderPipelineInfo {
    /// WGSL shader source code.
    pub shader: &'static str,
    /// Optional name of the shader.
    pub label: Option<&'static str>,
    /// Whether the pipeline should have a texture binding.
    pub has_texture: bool,
}

/// Pipeline manager allows to create new render pipelines on demand, without duplication.
pub struct RenderPipelineManager {
    device: Arc<Device>,
    texture_bind_group_layout: BindGroupLayout,
    uniform_bind_group_layout: BindGroupLayout,
    pipelines: HashMap<RenderPipelineInfo, Arc<RenderPipeline>>,
    surface_format: TextureFormat,
}

impl RenderPipelineManager {
    /// Create a new pipeline manager.
    ///
    /// # Arguments
    ///
    /// * `device` - WGPU device to use.
    /// * `surface_format` - Texture surface format for textured pipelines.
    pub fn new(device: Arc<Device>, surface_format: TextureFormat) -> Self {
        // Bind a texture and its sampler to fragment shader.
        let texture_bind_group_layout =
            device.create_bind_group_layout(&BindGroupLayoutDescriptor {
                entries: &[
                    BindGroupLayoutEntry {
                        binding: 0,
                        visibility: ShaderStages::FRAGMENT,
                        ty: BindingType::Texture {
                            multisampled: false,
                            view_dimension: TextureViewDimension::D2,
                            sample_type: TextureSampleType::Float { filterable: true },
                        },
                        count: None,
                    },
                    BindGroupLayoutEntry {
                        binding: 1,
                        visibility: ShaderStages::FRAGMENT,
                        ty: BindingType::Sampler(SamplerBindingType::Filtering),
                        count: None,
                    },
                ],
                label: Some("texture_bind_group_layout"),
            });

        // Bind uniform buffer to vertex shader.
        let uniform_bind_group_layout =
            device.create_bind_group_layout(&BindGroupLayoutDescriptor {
                entries: &[BindGroupLayoutEntry {
                    binding: 0,
                    visibility: ShaderStages::VERTEX,
                    ty: BindingType::Buffer {
                        ty: BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
                label: Some("uniform_bind_group_layout"),
            });

        Self {
            device,
            texture_bind_group_layout,
            uniform_bind_group_layout,
            surface_format,
            pipelines: Default::default(),
        }
    }

    /// Get the underlying WGPU device.
    pub fn device(&self) -> &Arc<Device> {
        &self.device
    }

    /// Get the layout of texture bind group.
    pub fn texture_bind_group_layout(&self) -> &BindGroupLayout {
        &self.texture_bind_group_layout
    }

    /// Get the layout of camera's uniform buffer bind group.
    pub fn uniform_bind_group_layout(&self) -> &BindGroupLayout {
        &self.uniform_bind_group_layout
    }

    /// Get a render pipeline that matches the provided info.
    ///
    /// # Arguments
    ///
    /// * `info` - shader and binding information for the pipeline.
    pub fn pipeline(&mut self, info: RenderPipelineInfo) -> Arc<RenderPipeline> {
        // Destruct self to access fields in the closure
        let RenderPipelineManager {
            texture_bind_group_layout,
            uniform_bind_group_layout,
            pipelines,
            device,
            surface_format,
        } = self;

        pipelines
            .entry(info)
            .or_insert_with_key(|info| {
                let shader = device.create_shader_module(&ShaderModuleDescriptor {
                    label: info.label,
                    source: ShaderSource::Wgsl(info.shader.into()),
                });

                // Depending on whether the pipeline needs textures, use one of these layouts.
                let textured_bind_group_layouts =
                    &[&*uniform_bind_group_layout, &*texture_bind_group_layout][..];
                let untextured_bind_group_layouts = &[&*uniform_bind_group_layout][..];

                let render_pipeline_layout =
                    device.create_pipeline_layout(&PipelineLayoutDescriptor {
                        label: Some("Render Pipeline Layout"),
                        bind_group_layouts: if info.has_texture {
                            textured_bind_group_layouts
                        } else {
                            untextured_bind_group_layouts
                        },
                        push_constant_ranges: &[],
                    });

                // Create the render pipeline
                device
                    .create_render_pipeline(&RenderPipelineDescriptor {
                        label: Some("Render Pipeline"),
                        layout: Some(&render_pipeline_layout),
                        vertex: VertexState {
                            module: &shader,
                            entry_point: "vs_main",
                            buffers: &[Vertex::desc(), RenderObject::desc()],
                        },
                        fragment: Some(FragmentState {
                            module: &shader,
                            entry_point: "fs_main",
                            targets: &[ColorTargetState {
                                format: *surface_format,
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
                        depth_stencil: Some(DepthStencilState {
                            format: RawTexture::DEPTH_FORMAT,
                            depth_write_enabled: true,
                            depth_compare: CompareFunction::Less,
                            stencil: StencilState::default(),
                            bias: DepthBiasState::default(),
                        }),
                        multisample: MultisampleState {
                            count: 1,
                            mask: !0,
                            alpha_to_coverage_enabled: false,
                        },
                        multiview: None,
                    })
                    .into()
            })
            .clone()
    }
}
