//! Manager of on-GPU meshes.

use super::mesh::Mesh;
use core::ops::Range;
use std::collections::HashMap;
use std::sync::Arc;
use wgpu::util::DeviceExt;
use wgpu::*;

use super::vertex::Vertex;

/// Manager of on-GPU meshes.
///
/// This manager will load up any meshes onto the GPU vertex and index buffers. However, unloading
/// is currently unsupported.
pub struct MeshManager {
    device: Arc<Device>,
    vertex_buffer: Option<Buffer>,
    index_buffer: Option<Buffer>,
    loaded_meshes: HashMap<Arc<Mesh>, Arc<MeshDescriptor>>,
    vertex_count: i32,
    index_count: u32,
    bufs_dirty: bool,
}

impl MeshManager {
    /// Create a new mesh manager.
    ///
    /// # Arguments
    ///
    /// * `device` - device to load the meshes on.
    pub fn new(device: Arc<Device>) -> Self {
        Self {
            device,
            vertex_buffer: None,
            index_buffer: None,
            loaded_meshes: Default::default(),
            vertex_count: 0,
            index_count: 0,
            bufs_dirty: false,
        }
    }

    /// Set buffers up in the render pass.
    ///
    /// This will set the vertex buffer on slot 0, if it exists.
    ///
    /// If the buffers have not been allocated, it means that no meshes are being drawn, thus they
    /// are not being bound. In such cases no draw calls should ever happen.
    ///
    /// # Arguments
    ///
    /// * `render_pass` - render pass to set the buffers on.
    pub fn set_buffers<'a>(&'a self, render_pass: &mut RenderPass<'a>) {
        if let Some(vbuf) = &self.vertex_buffer {
            render_pass.set_vertex_buffer(0, vbuf.slice(..));
        }

        if let Some(ibuf) = &self.index_buffer {
            render_pass.set_index_buffer(ibuf.slice(..), IndexFormat::Uint32);
        }
    }

    /// Mark the mesh for loading and get its on-GPU location.
    ///
    /// This function will find the mesh in the vertex buffer and return its descriptor, or insert
    /// it at the end of the buffer.
    ///
    /// Before rendering, [`apply_changes`](MeshManager::apply_changes) must be called to allocate
    /// new buffers and write the changes.
    ///
    /// # Arguments
    ///
    /// * `mesh` - mesh to load.
    pub fn descriptor(&mut self, mesh: Arc<Mesh>) -> Arc<MeshDescriptor> {
        let MeshManager {
            loaded_meshes,
            vertex_count,
            index_count,
            bufs_dirty,
            ..
        } = self;

        loaded_meshes
            .entry(mesh)
            .or_insert_with_key(|mesh| {
                *bufs_dirty = true;
                MeshDescriptor {
                    mesh: mesh.clone(),
                    base_vertex: {
                        let ret = *vertex_count;
                        *vertex_count += mesh.vertices.len() as i32;
                        ret
                    },
                    indices: {
                        let ret = *index_count;
                        *index_count += mesh.indices.len() as u32;
                        ret..*index_count
                    },
                }
                .into()
            })
            .clone()
    }

    /// Commit any buffer changes to GPU.
    ///
    /// This function will create a new set of vertex and index buffers in case new meshes have
    /// been loaded up.
    pub fn apply_changes(&mut self) {
        if self.bufs_dirty {
            self.bufs_dirty = false;

            let mut vertex_buffer = vec![Vertex::default(); self.vertex_count as usize];
            let mut index_buffer = vec![0u32; self.index_count as usize];

            for MeshDescriptor {
                mesh,
                base_vertex,
                indices,
            } in self.loaded_meshes.values().map(|v| &**v)
            {
                let vertex_buffer =
                    &mut vertex_buffer[*base_vertex as usize..][..mesh.vertices.len()];
                let index_buffer = &mut index_buffer[indices.start as usize..indices.end as usize];
                vertex_buffer.copy_from_slice(&mesh.vertices);
                index_buffer.copy_from_slice(&mesh.indices);
            }

            self.vertex_buffer =
                Some(self.device.create_buffer_init(&util::BufferInitDescriptor {
                    label: Some("Vertex Buffer"),
                    contents: bytemuck::cast_slice(vertex_buffer.as_slice()),
                    usage: BufferUsages::VERTEX,
                }));

            self.index_buffer = Some(self.device.create_buffer_init(&util::BufferInitDescriptor {
                label: Some("Index Buffer"),
                contents: bytemuck::cast_slice(index_buffer.as_slice()),
                usage: BufferUsages::INDEX,
            }));
        }
    }
}

/// Describe a on-GPU mesh.
pub struct MeshDescriptor {
    /// The mesh that is being described.
    pub mesh: Arc<Mesh>,
    /// Offset within the vertex buffer for the mesh.
    pub base_vertex: i32,
    /// Indices within the index buffer for the mesh.
    pub indices: Range<u32>,
}
