//! # wimrend - WgpuImmediateModeRENDerer
//!
//! This crate provides a simple immediate mode renderer for use in wgpu and winit environments.
//!
//! Adapted from [wgpu tutorial](https://sotrh.github.io/learn-wgpu/), but contains mostly custom
//! code.

use std::collections::BTreeMap;
use std::mem;
use std::sync::Arc;
use wgpu::*;

use nalgebra as na;

pub mod material;
pub mod mesh;
pub mod mesh_manager;
pub mod render_pipeline_manager;
pub mod render_state;
pub mod texture;
pub mod vertex;

use material::Material;
use mesh::Mesh;
use mesh_manager::{MeshDescriptor, MeshManager};
use render_pipeline_manager::RenderPipelineManager;
use texture::Texture;

/// Arc for when hashing the underlying object is too costly.
///
/// This arc essentially compares and hashes the pointer of the object as opposed to the object
/// itself. This becomes very useful when singleton arcs are widely used.
#[repr(transparent)]
struct UniqueArc<T>(Arc<T>);

impl<T> core::ops::Deref for UniqueArc<T> {
    type Target = Arc<T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> PartialEq for UniqueArc<T> {
    fn eq(&self, other: &Self) -> bool {
        Arc::as_ptr(self) == Arc::as_ptr(other)
    }
}

impl<T> Eq for UniqueArc<T> {}

impl<T> Ord for UniqueArc<T> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        Arc::as_ptr(self).cmp(&Arc::as_ptr(other))
    }
}

impl<T> PartialOrd for UniqueArc<T> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T> From<Arc<T>> for UniqueArc<T> {
    fn from(arc: Arc<T>) -> Self {
        Self(arc)
    }
}

#[repr(C)]
#[derive(Default, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct UniformData {
    proj: [f32; 4 * 4],
    resolution: [f32; 4],
}

/// Data bound to all shaders in the form of a uniform buffer.
///
/// This structure contains camera data, as well as other properties to be bound as a uniform
/// buffer.
pub struct UniformObject {
    data: UniformData,
    uniform_buffer: Buffer,
    uniform_bind_group: BindGroup,
}

fn calc_view(pos: na::Point3<f32>, rot: na::UnitQuaternion<f32>) -> na::Matrix4<f32> {
    na::Matrix4::look_at_rh(
        &pos,
        &(pos + rot.transform_vector(&na::Vector3::new(0.0, 1.0, 0.0))),
        &rot.transform_vector(&na::Vector3::new(0.0, 0.0, 1.0)),
    )
}

impl UniformObject {
    fn new(pipeline_manager: &mut RenderPipelineManager) -> Self {
        let uniform_buffer = pipeline_manager.device().create_buffer(&BufferDescriptor {
            label: Some("Uniform Buffer"),
            size: mem::size_of::<UniformData>() as BufferAddress,
            usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let uniform_bind_group =
            pipeline_manager
                .device()
                .create_bind_group(&BindGroupDescriptor {
                    layout: pipeline_manager.uniform_bind_group_layout(),
                    entries: &[BindGroupEntry {
                        binding: 0,
                        resource: uniform_buffer.as_entire_binding(),
                    }],
                    label: Some("uniform_bind_group"),
                });

        Self {
            data: Default::default(),
            uniform_buffer,
            uniform_bind_group,
        }
    }

    /// Update camera projection matrix.
    ///
    /// # Arguments
    ///
    /// * `proj` - projection matrix.
    /// * `pos` - position of the camera.
    /// * `rot` - rotation of the camera.
    pub fn update_projection(
        &mut self,
        proj: &na::Matrix4<f32>,
        pos: na::Point3<f32>,
        rot: na::UnitQuaternion<f32>,
    ) {
        let view = calc_view(pos, rot);
        #[rustfmt::skip]
        pub const OPENGL_TO_WGPU_MATRIX: na::Matrix4<f32> = na::matrix![
            1.0, 0.0, 0.0, 0.0;
            0.0, 1.0, 0.0, 0.0;
            0.0, 0.0, 0.5, 0.0;
            0.0, 0.0, 0.5, 1.0
        ];
        let mat = OPENGL_TO_WGPU_MATRIX * proj * view;
        self.data.proj.copy_from_slice(mat.as_slice());
    }

    fn update_internal(&mut self, screen: &SurfaceConfiguration) {
        self.data.resolution[0] = screen.width as f32;
        self.data.resolution[1] = screen.height as f32;
    }

    fn update_buffers(&self, queue: &Queue) {
        queue.write_buffer(&self.uniform_buffer, 0, bytemuck::bytes_of(&self.data));
    }
}

pub struct Renderer {
    device: Arc<Device>,
    pipeline_manager: RenderPipelineManager,
    mesh_manager: MeshManager,
    obj_count: usize,
    objects: BTreeMap<
        UniqueArc<RenderPipeline>,
        BTreeMap<
            Option<UniqueArc<Texture>>,
            BTreeMap<UniqueArc<MeshDescriptor>, Vec<RenderObject>>,
        >,
    >,
    instance_buffer: Option<(usize, Buffer)>,
    line_material: Arc<Material>,
    camera: UniformObject,
}

impl Renderer {
    /// Create a new renderer.
    ///
    /// # Arguments
    ///
    /// * `device` - device to render to.
    /// * `surface_format` - format of all textures.
    pub fn new(device: Arc<Device>, surface_format: TextureFormat) -> Self {
        let mut pipeline_manager = RenderPipelineManager::new(device.clone(), surface_format);
        Self {
            mesh_manager: MeshManager::new(device.clone()),
            camera: UniformObject::new(&mut pipeline_manager),
            line_material: Material::line(&mut pipeline_manager).into(),
            pipeline_manager,
            device,
            obj_count: 0,
            objects: Default::default(),
            instance_buffer: None,
        }
    }

    fn reset(&mut self) {
        self.obj_count = 0;
        self.objects.clear();
    }

    /// Update all buffers before rendering.
    ///
    /// # Arguments
    ///
    /// * `queue` - queue of write operations.
    pub fn update_buffers(&mut self, queue: &Queue) {
        self.camera.update_buffers(queue);

        if self.instance_buffer.as_ref().map(|(s, _)| *s).unwrap_or(0) < self.obj_count {
            let buffer = self.device.create_buffer(&BufferDescriptor {
                label: Some("Camera Buffer"),
                size: RenderObject::desc().array_stride * self.obj_count as BufferAddress,
                usage: BufferUsages::VERTEX | BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });

            self.instance_buffer = Some((self.obj_count, buffer));
        }

        if let Some((_, buf)) = &self.instance_buffer {
            let mut offset = 0;
            for o in self
                .objects
                .values()
                .flat_map(|m| m.values().flat_map(|m| m.values().flat_map(|m| m.iter())))
            {
                offset = o.write_buffer(buf, offset, queue);
            }
        }
    }

    /// Render all objects.
    ///
    /// # Arguments
    ///
    /// * `render_pass` - render pass to draw in.
    pub fn render<'a>(&'a self, render_pass: &mut wgpu::RenderPass<'a>) {
        // First setup any buffers used.
        self.mesh_manager.set_buffers(render_pass);

        if let Some((_, ibuf)) = &self.instance_buffer {
            render_pass.set_vertex_buffer(1, ibuf.slice(..));
        }

        let mut count = 0;

        // Instanced draw loop.
        for (pipeline, objects) in &self.objects {
            // Pipeline for all the objects following.
            render_pass.set_pipeline(pipeline);

            // Bind the uniform buffer.
            render_pass.set_bind_group(0, &self.camera.uniform_bind_group, &[]);

            for (texture, objects) in objects {
                // Bind a texture if there is any.
                if let Some(texture) = texture {
                    render_pass.set_bind_group(1, &texture.bind_group, &[]);
                }
                for (mesh, objects) in objects {
                    // Perform instanced draw of all objects with matching material and mesh.
                    let c2 = objects.len() as u32;
                    render_pass.draw_indexed(
                        mesh.indices.clone(),
                        mesh.base_vertex,
                        count..(count + c2),
                    );
                    count += c2;
                }
            }
        }
    }

    /// Get mutable reference to underlying camera.
    pub fn uniform_mut(&mut self) -> &mut UniformObject {
        &mut self.camera
    }

    /// Get mutable reference to underlying pipeline manager.
    ///
    /// This manager needs to be passed upon creating materials.
    pub fn pipeline_manager_mut(&mut self) -> &mut RenderPipelineManager {
        &mut self.pipeline_manager
    }

    /// Draw an object in scene.
    ///
    /// # Arguments
    ///
    /// * `mesh` - 3D mesh to draw.
    /// * `transform` - Combined translation, rotation and scale transformation.
    /// * `colour` - RGBA colour of this object instance.
    /// * `material` - Material used by the object.
    pub fn obj_raw(
        &mut self,
        mesh: Arc<Mesh>,
        transform: na::Matrix4<f32>,
        colour: na::Vector4<f32>,
        material: Arc<Material>,
    ) {
        let mesh = self.mesh_manager.descriptor(mesh.clone());

        self.obj_count += 1;

        self.objects
            .entry(material.pipeline().clone().into())
            .or_default()
            .entry(material.texture().clone().map(<_>::into))
            .or_default()
            .entry(mesh.clone().into())
            .or_default()
            .push(RenderObject { transform, colour });
    }

    /// Draw an object in scene.
    ///
    /// # Arguments
    ///
    /// * `mesh` - 3D mesh to draw.
    /// * `position` - Position of the object.
    /// * `rotation` - Rotation of the object.
    /// * `scale` - Scale of the object.
    /// * `colour` - RGBA colour of this object instance.
    /// * `material` - Material used by the object.
    pub fn obj(
        &mut self,
        mesh: Arc<Mesh>,
        position: na::Point3<f32>,
        rotation: na::UnitQuaternion<f32>,
        scale: na::Vector3<f32>,
        colour: na::Vector4<f32>,
        material: Arc<Material>,
    ) {
        let tr = na::Matrix4::new_translation(&position.coords);
        let rot = rotation.to_homogeneous();
        let scale = na::matrix![
            scale.x, 0.0, 0.0, 0.0;
            0.0, scale.y, 0.0, 0.0;
            0.0, 0.0, scale.z, 0.0;
            0.0, 0.0, 0.0, 1.0
        ];
        self.obj_raw(mesh, tr * rot * scale, colour, material)
    }

    /// Draw a line in scene.
    ///
    /// # Arguments
    ///
    /// * `start` - starting point of the line.
    /// * `end` - ending point of the line.
    /// * `thickness` - thickness of line in pixels.
    /// * `colour` - RGBA colour of the line.
    pub fn line(
        &mut self,
        start: na::Point3<f32>,
        end: na::Point3<f32>,
        thickness: f32,
        colour: na::Vector4<f32>,
    ) {
        // The matrix in the line shader is merely start and end positions,
        // not an actual transformation.

        let transform = na::matrix![
            start.x, start.y, start.z, 1.0;
            end.x, end.y, end.z, 1.0;
            thickness, 0.0, 0.0, 0.0;
            0.0, 0.0, 0.0, 0.0
        ]
        .transpose();

        self.obj_raw(Mesh::quad(), transform, colour, self.line_material.clone());
    }
}

struct RenderObject {
    pub transform: na::Matrix4<f32>,
    pub colour: na::Vector4<f32>,
}

impl RenderObject {
    pub fn desc<'a>() -> VertexBufferLayout<'a> {
        VertexBufferLayout {
            array_stride: mem::size_of::<[f32; 4 * 5]>() as BufferAddress,
            step_mode: VertexStepMode::Instance,
            attributes: &[
                VertexAttribute {
                    offset: 0,
                    shader_location: 4,
                    format: VertexFormat::Float32x4,
                },
                VertexAttribute {
                    offset: mem::size_of::<[f32; 4]>() as BufferAddress,
                    shader_location: 5,
                    format: VertexFormat::Float32x4,
                },
                VertexAttribute {
                    offset: mem::size_of::<[f32; 4 * 2]>() as BufferAddress,
                    shader_location: 6,
                    format: VertexFormat::Float32x4,
                },
                VertexAttribute {
                    offset: mem::size_of::<[f32; 4 * 3]>() as BufferAddress,
                    shader_location: 7,
                    format: VertexFormat::Float32x4,
                },
                VertexAttribute {
                    offset: mem::size_of::<[f32; 4 * 4]>() as BufferAddress,
                    shader_location: 8,
                    format: VertexFormat::Float32x4,
                },
            ],
        }
    }

    pub fn write_buffer(
        &self,
        buffer: &Buffer,
        offset: BufferAddress,
        queue: &Queue,
    ) -> BufferAddress {
        queue.write_buffer(
            buffer,
            offset,
            bytemuck::cast_slice(self.transform.as_slice()),
        );
        queue.write_buffer(
            buffer,
            offset + mem::size_of::<[f32; 4 * 4]>() as BufferAddress,
            bytemuck::cast_slice(self.colour.as_slice()),
        );
        offset + mem::size_of::<[f32; 4 * 5]>() as BufferAddress
    }
}
