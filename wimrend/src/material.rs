//! Materials used to shade objects.

use std::sync::Arc;
use wgpu::*;

use super::render_pipeline_manager::{RenderPipelineInfo, RenderPipelineManager};
use super::texture::Texture;

/// Shading material to be applied during rendering.
pub struct Material {
    pipeline: Arc<RenderPipeline>,
    texture: Option<Arc<Texture>>,
}

impl Material {
    /// Get the underlying render pipeline used.
    pub fn pipeline(&self) -> &Arc<RenderPipeline> {
        &self.pipeline
    }

    /// Get the texture used by the material (if any).
    pub fn texture(&self) -> &Option<Arc<Texture>> {
        &self.texture
    }

    /// Create a custom material.
    ///
    /// # Arguments
    ///
    /// * `pipeline_manager` - Controls where the pipeline should be created.
    /// * `shader` - WGSL shader to use.
    /// * `label` - Optional label to identify the shader.
    /// * `texture` - Optional texture to bind to the shader.
    pub fn custom(
        pipeline_manager: &mut RenderPipelineManager,
        shader: &'static str,
        label: Option<&'static str>,
        texture: Option<Arc<Texture>>,
    ) -> Self {
        Self {
            pipeline: pipeline_manager.pipeline(RenderPipelineInfo {
                shader,
                label,
                has_texture: texture.is_some(),
            }),
            texture,
        }
    }

    /// Create a textured material.
    ///
    /// # Arguments
    ///
    /// * `pipeline_manager` - Controls where the pipeline should be created.
    /// * `texture` - Texture to bind to the shader.
    pub fn textured(pipeline_manager: &mut RenderPipelineManager, texture: Arc<Texture>) -> Self {
        Self {
            pipeline: pipeline_manager.pipeline(RenderPipelineInfo {
                shader: include_str!("shaders/textured.wgsl"),
                label: Some("textured"),
                has_texture: true,
            }),
            texture: Some(texture),
        }
    }

    /// Create a line renderer material.
    ///
    /// # Arguments
    ///
    /// * `pipeline_manager` - Controls where the pipeline should be created.
    ///
    /// # Remarks
    ///
    /// Note that this material expects a non-standard transformation matrix. The first row of the
    /// matrix defines starting position of the line, the second row defines ending position, and
    /// the first element of the third row defines line thickness.
    pub fn line(pipeline_manager: &mut RenderPipelineManager) -> Self {
        Self {
            pipeline: pipeline_manager.pipeline(RenderPipelineInfo {
                shader: include_str!("shaders/line.wgsl"),
                label: Some("line"),
                has_texture: false,
            }),
            texture: None,
        }
    }

    /// Create a untextured and unlit material.
    ///
    /// # Arguments
    ///
    /// * `pipeline_manager` - Controls where the pipeline should be created.
    pub fn unlit(pipeline_manager: &mut RenderPipelineManager) -> Self {
        Self {
            pipeline: pipeline_manager.pipeline(RenderPipelineInfo {
                shader: include_str!("shaders/unlit.wgsl"),
                label: Some("unlit"),
                has_texture: false,
            }),
            texture: None,
        }
    }
}
