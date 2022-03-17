//! On-GPU textures.

use anyhow::Result;

/// Raw texture without a bind group.
pub struct RawTexture {
    pub texture: wgpu::Texture,
    pub view: wgpu::TextureView,
    pub sampler: wgpu::Sampler,
    pub label: Option<&'static str>,
    pub dimensions: (u32, u32),
}

impl RawTexture {
    pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float; // 1.

    /// Create a depth texture.
    ///
    /// # Arguments
    ///
    /// * `device` - Device to create the texture on.
    /// * `config` - Framebuffer configuration.
    /// * `label` - Optional label to identify the texture.
    pub fn create_depth_texture(
        device: &wgpu::Device,
        config: &wgpu::SurfaceConfiguration,
        label: &'static str,
    ) -> Self {
        let size = wgpu::Extent3d {
            width: config.width,
            height: config.height,
            depth_or_array_layers: 1,
        };
        let desc = wgpu::TextureDescriptor {
            label: Some(label),
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: Self::DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        };
        let texture = device.create_texture(&desc);

        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Nearest,
            compare: Some(wgpu::CompareFunction::LessEqual),
            lod_min_clamp: -100.0,
            lod_max_clamp: 100.0,
            ..Default::default()
        });

        Self {
            texture,
            view,
            sampler,
            label: Some(label),
            dimensions: (config.width, config.height),
        }
    }

    /// Create a texture from RGBA image.
    ///
    /// # Arguments
    ///
    /// * `device` - Device to create the texture on.
    /// * `queue` - Command queue to write the texture through.
    /// * `label` - Optional identification label.
    /// * `rgba` - Image data.
    /// * `height` - Height of the texture.
    pub fn from_rgba_frame(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        label: Option<&'static str>,
        rgba: &[u8],
        height: usize,
    ) -> Result<Self> {
        let height = height as u32;
        let width = (rgba.len() / 4) as u32 / height;

        let dimensions = (width, height);

        let size = wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        };

        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label,
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        });

        queue.write_texture(
            wgpu::ImageCopyTexture {
                aspect: wgpu::TextureAspect::All,
                texture: &texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
            },
            rgba,
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: std::num::NonZeroU32::new(4 * width),
                rows_per_image: std::num::NonZeroU32::new(height),
            },
            size,
        );

        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Nearest,
            mipmap_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        Ok(Self {
            texture,
            view,
            sampler,
            label,
            dimensions,
        })
    }
}

/// Texture with a bind group.
///
/// This is the typical texture object to be used, as the bind group is needed for applying the
/// texture to rendered objects.
pub struct Texture {
    pub raw: RawTexture,
    pub bind_group: wgpu::BindGroup,
}

impl Texture {
    /// Update the texture with new image.
    ///
    /// This function will update the existing buffers if the dimensions match, or create new
    /// buffers with new dimensions.
    ///
    /// # Arguments
    ///
    /// * `device` - Device of the texture. Must match the original one.
    /// * `queue` - Command queue for writing the data. Must match the original one.
    /// * `bind_group_layout` - Layout for binding this texture to a shader. Must match the
    /// original one.
    /// * `rgba` - New image data.
    /// * `height` - Height of this image.
    pub fn update_texture(
        self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        bind_group_layout: &wgpu::BindGroupLayout,
        rgba: &[u8],
        height: usize,
    ) -> Result<Self> {
        let height = height as u32;
        let width = (rgba.len() / 4) as u32 / height;

        let dimensions = (width, height);

        if dimensions == self.raw.dimensions {
            let size = wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            };

            queue.write_texture(
                wgpu::ImageCopyTexture {
                    aspect: wgpu::TextureAspect::All,
                    texture: &self.raw.texture,
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                },
                rgba,
                wgpu::ImageDataLayout {
                    offset: 0,
                    bytes_per_row: std::num::NonZeroU32::new(4 * width),
                    rows_per_image: std::num::NonZeroU32::new(height),
                },
                size,
            );

            Ok(self)
        } else {
            Self::from_rgba_frame(
                device,
                queue,
                bind_group_layout,
                self.raw.label,
                rgba,
                height as usize,
            )
        }
    }

    /// Create a texture from RGBA image.
    ///
    /// # Arguments
    ///
    /// * `device` - Device to create the texture on.
    /// * `queue` - Command queue to write the texture through.
    /// * `bind_group_layout` - Layout for binding this texture.
    /// * `label` - Optional identification label.
    /// * `rgba` - Image data.
    /// * `height` - Height of the texture.
    pub fn from_rgba_frame(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        bind_group_layout: &wgpu::BindGroupLayout,
        label: Option<&'static str>,
        rgba: &[u8],
        height: usize,
    ) -> Result<Self> {
        let raw = RawTexture::from_rgba_frame(device, queue, label, rgba, height)?;

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&raw.view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&raw.sampler),
                },
            ],
            label: Some("diffuse_bind_group"),
        });

        Ok(Self { raw, bind_group })
    }
}

impl core::ops::Deref for Texture {
    type Target = RawTexture;

    fn deref(&self) -> &Self::Target {
        &self.raw
    }
}
