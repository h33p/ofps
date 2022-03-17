//! Multisampling texture manager.

use crate::texture::RawTexture;
use wgpu::*;

/// Multisampling texture manager.
pub struct Multisampler {
    sample_count: u32,
    surface_format: TextureFormat,
    tex: Option<RawTexture>,
}

impl Multisampler {
    /// Create a new multisampler.
    ///
    /// # Arguments
    ///
    /// * `surface_format` - Framebuffer texture format.
    /// * `sample_count` - Number of MSAA samples.
    pub fn new(surface_format: TextureFormat, sample_count: u32) -> Self {
        Self {
            sample_count,
            surface_format,
            tex: None,
        }
    }

    /// Get number of MSAA samples.
    pub fn sample_count(&self) -> u32 {
        self.sample_count
    }

    /// Get multisampled framebuffer.
    ///
    /// # Arguments
    ///
    /// * `device` - Device to update the buffer on if dimensions change.
    /// * `config` - Current screen configuration.
    pub fn target(&mut self, device: &Device, config: &SurfaceConfiguration) -> &TextureView {
        let dimensions = (config.width, config.height);

        match &self.tex {
            Some(tex) if tex.dimensions == dimensions => {}
            _ => {
                self.tex = Some(RawTexture::create_custom(
                    device,
                    Some("msaa"),
                    dimensions,
                    self.sample_count,
                    self.surface_format,
                    wgpu::TextureUsages::RENDER_ATTACHMENT,
                ));
            }
        }

        self.tex.as_ref().map(|tex| &tex.view).unwrap()
    }
}
