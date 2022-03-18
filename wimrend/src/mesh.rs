//! 3D Meshes.

use super::vertex::Vertex;
use once_cell::sync::OnceCell;
use std::sync::Arc;

/// Describes a 3D mesh.
#[derive(Clone, Hash, PartialEq, Eq)]
pub struct Mesh {
    /// Vertices of the mesh.
    pub vertices: Vec<Vertex>,
    /// Indices of the mesh.
    pub indices: Vec<u32>,
}

static QUAD: OnceCell<Arc<Mesh>> = OnceCell::new();
static CENTERED_QUAD: OnceCell<Arc<Mesh>> = OnceCell::new();
static CUBE: OnceCell<Arc<Mesh>> = OnceCell::new();
static HEXAGON: OnceCell<Arc<Mesh>> = OnceCell::new();

impl Mesh {
    /// Create a mesh from vertice and index iterators.
    ///
    /// # Arguments
    ///
    /// * `vertices` - iterator of vertices.
    /// * `indices` - iterator of indices.
    pub fn from_parts<T: Into<Vertex>>(
        vertices: impl IntoIterator<Item = T>,
        indices: impl IntoIterator<Item = u32>,
    ) -> Self {
        Self {
            vertices: vertices.into_iter().map(T::into).collect(),
            indices: indices.into_iter().collect(),
        }
    }

    /// Get a 2 triangle square.
    pub fn quad() -> Arc<Mesh> {
        QUAD.get_or_init(|| {
            Self::from_parts(
                [
                    ([0.0, 0.0, 0.0], [0.0, 0.0]),
                    ([0.0, 1.0, 0.0], [0.0, 1.0]),
                    ([1.0, 0.0, 0.0], [1.0, 0.0]),
                    ([1.0, 1.0, 0.0], [1.0, 1.0]),
                ],
                [0, 1, 2, 1, 3, 2],
            )
            .into()
        })
        .clone()
    }

    /// Get a 2 triangle square.
    pub fn centered_quad() -> Arc<Mesh> {
        CENTERED_QUAD
            .get_or_init(|| {
                Self::from_parts(
                    [
                        ([-0.5, -0.5, 0.0], [0.0, 0.0]),
                        ([-0.5, 0.5, 0.0], [0.0, 1.0]),
                        ([0.5, -0.5, 0.0], [1.0, 0.0]),
                        ([0.5, 0.5, 0.0], [1.0, 1.0]),
                    ],
                    [0, 1, 2, 1, 3, 2],
                )
                .into()
            })
            .clone()
    }
    /// Get a centered 3D cube.
    pub fn cube() -> Arc<Mesh> {
        CUBE.get_or_init(|| {
            Self::from_parts(
                [
                    ([-0.5, -0.5, -0.5], [0.0, 0.0]),
                    ([0.5, -0.5, -0.5], [1.0, 0.0]),
                    ([0.5, 0.5, -0.5], [1.0, 1.0]),
                    ([-0.5, 0.5, -0.5], [0.0, 1.0]),
                    ([-0.5, -0.5, 0.5], [0.0, 1.0]),
                    ([0.5, -0.5, 0.5], [1.0, 1.0]),
                    ([0.5, 0.5, 0.5], [1.0, 0.0]),
                    ([-0.5, 0.5, 0.5], [0.0, 0.0]),
                ],
                [
                    0, 2, 1, 0, 3, 2, 1, 2, 6, 6, 5, 1, 4, 5, 6, 6, 7, 4, 2, 3, 6, 6, 3, 7, 0, 7,
                    3, 0, 4, 7, 0, 1, 5, 0, 5, 4,
                ],
            )
            .into()
        })
        .clone()
    }

    /// Get a flat hexagon.
    pub fn hexagon() -> Arc<Mesh> {
        HEXAGON
            .get_or_init(|| {
                Self::from_parts(
                    [
                        ([-0.5, -0.5, -0.5], [0.86603, -0.5]),
                        ([0.86603, 0.5, 0.0], [0.86603, 0.5]),
                        ([0.0, 1.0, 0.0], [0.0, 1.0]),
                        ([-0.86603, 0.5, 0.0], [-0.86603, 0.5]),
                        ([-0.86603, -0.5, 0.0], [-0.86603, -0.5]),
                        ([0.0, -1.0, 0.0], [0.0, -1.0]),
                    ],
                    [0, 4, 5, 1, 2, 3, 0, 1, 3, 0, 3, 4],
                )
                .into()
            })
            .clone()
    }
}
