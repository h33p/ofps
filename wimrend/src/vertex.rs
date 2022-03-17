//! Describes vertices.

use wgpu::*;

/// A single vertex.
#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub colour: [f32; 4],
    pub pos: [f32; 3],
    pub normal: [f32; 3],
    pub uv: [f32; 2],
}

impl Default for Vertex {
    fn default() -> Self {
        Self {
            pos: Default::default(),
            normal: Default::default(),
            colour: [1.0, 1.0, 1.0, 1.0],
            uv: Default::default(),
        }
    }
}

impl PartialEq for Vertex {
    fn eq(&self, other: &Vertex) -> bool {
        bytemuck::bytes_of(self) == bytemuck::bytes_of(other)
    }
}

impl Eq for Vertex {}

impl std::hash::Hash for Vertex {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        bytemuck::bytes_of(self).hash(state)
    }
}

impl From<([f32; 3], [f32; 2])> for Vertex {
    fn from((pos, uv): ([f32; 3], [f32; 2])) -> Self {
        Self {
            pos,
            uv,
            ..Default::default()
        }
    }
}

impl Vertex {
    /// Description for shaders.
    pub fn desc<'a>() -> VertexBufferLayout<'a> {
        use std::mem;
        VertexBufferLayout {
            array_stride: mem::size_of::<Vertex>() as BufferAddress,
            step_mode: VertexStepMode::Vertex,
            attributes: &[
                VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: VertexFormat::Float32x4,
                },
                VertexAttribute {
                    offset: mem::size_of::<[f32; 4]>() as BufferAddress,
                    shader_location: 1,
                    format: VertexFormat::Float32x3,
                },
                VertexAttribute {
                    offset: mem::size_of::<[f32; 8]>() as BufferAddress,
                    shader_location: 2,
                    format: VertexFormat::Float32x3,
                },
                VertexAttribute {
                    offset: mem::size_of::<[f32; 10]>() as BufferAddress,
                    shader_location: 3,
                    format: VertexFormat::Float32x2,
                },
            ],
        }
    }
}
