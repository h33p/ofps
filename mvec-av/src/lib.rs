//! FFMPEG's AV Decoder

use ::core::ffi::c_void;
use ::core::ops::{Deref, DerefMut};
use ::core::{ptr, slice};
use c_str_macro::c_str;
use ffmpeg_sys::*;
use libc::c_int;
use log::*;
use motion_vectors::prelude::v1::{ptrplus::*, Result, *};
use motion_vectors::utils::*;
use nalgebra as na;
use std::io::*;
use std::mem::MaybeUninit;

pub struct AvBuf(&'static mut [u8]);

impl AvBuf {
    pub fn try_new(size: usize) -> Result<Self> {
        let buf = unsafe { av_malloc(size) as *mut u8 };

        if buf.is_null() {
            Err("Failed to allocate buffer".into())
        } else {
            Ok(Self(unsafe { slice::from_raw_parts_mut(buf, size) }))
        }
    }
}

impl Drop for AvBuf {
    fn drop(&mut self) {
        unsafe { av_free(self.0.as_mut_ptr() as *mut _) }
    }
}

impl Deref for AvBuf {
    type Target = [u8];

    fn deref(&self) -> &[u8] {
        self.0
    }
}

impl DerefMut for AvBuf {
    fn deref_mut(&mut self) -> &mut [u8] {
        self.0
    }
}

pub struct AvContext<T> {
    _stream: Box<T>,
    pub fmt_ctx: &'static mut AVFormatContext,
    pub avio_ctx: &'static mut AVIOContext,
}

impl<T> Drop for AvContext<T> {
    fn drop(&mut self) {
        // SAFETY: the references will be dangling,
        // but after the drop nobody will read them.
        unsafe {
            avformat_close_input(&mut (self.fmt_ctx as *mut _));
            av_free((*self.avio_ctx).buffer as *mut _);
            av_free(self.avio_ctx as *mut _ as *mut c_void);
        }
    }
}

impl<T: Read + Seek> AvContext<T> {
    pub fn try_new(mut stream: Box<T>) -> Result<Self> {
        let mut buf = AvBuf::try_new(8196)?;

        let avio_ctx = unsafe {
            avio_alloc_context(
                buf.as_mut_ptr(),
                buf.len() as _,
                0,
                (&mut *stream) as *mut T as *mut _,
                Some(Self::read_callback),
                None,
                None,
            )
            .as_mut()
        }
        .ok_or("Failed to allocate AVIOContext")?;

        let mut fmt_ctx = unsafe { avformat_alloc_context().as_mut() }.ok_or_else(|| {
            unsafe { av_free((*avio_ctx).buffer as *mut _) };
            unsafe { av_free(avio_ctx as *mut AVIOContext as *mut _) };
            "Failed to allocate AVFormatContext"
        })?;

        fmt_ctx.pb = avio_ctx;

        match unsafe {
            avformat_open_input(
                fmt_ctx.as_mut_ptr(),
                ptr::null(),
                ptr::null_mut(),
                ptr::null_mut(),
            )
        } {
            0 => {
                std::mem::forget(buf);
                Ok(Self {
                    _stream: stream,
                    fmt_ctx,
                    avio_ctx,
                })
            }
            e => {
                unsafe { av_free((*avio_ctx).buffer as *mut _) };
                unsafe { av_free(avio_ctx as *mut AVIOContext as *mut _) };
                Err(format!("Unable to open input ({:x})", e).into())
            }
        }
    }

    unsafe extern "C" fn read_callback(
        opaque: *mut c_void,
        buf: *mut u8,
        buf_size: c_int,
    ) -> c_int {
        (*(opaque as *mut T))
            .read(slice::from_raw_parts_mut(buf, buf_size as _))
            .map(|r| r as c_int)
            .map(|r| {
                trace!("{}", r);
                r
            })
            .map_err(|e| {
                error!("{}", e);
                e
            })
            .unwrap_or(-1)
    }

    pub fn dump_format(&mut self) {
        unsafe { av_dump_format(self.fmt_ctx, 0, std::ptr::null(), 0) };
    }
}

pub struct AvDecoder<T> {
    pub av_ctx: AvContext<T>,
    codec_ctx: &'static mut AVCodecContext,
    av_frame: &'static mut AVFrame,
    stream_idx: i32,
    framerate: f64,
    aspect_ratio: (usize, usize),
    aspect_ratio_scale: (usize, usize),
    max_mfield_size: (usize, usize),
}

impl<T> Drop for AvDecoder<T> {
    fn drop(&mut self) {
        // SAFETY: the references will be dangling,
        // but after the drop nobody will read them.
        unsafe {
            av_frame_free(&mut (self.av_frame as *mut _));
            avcodec_free_context(&mut (self.codec_ctx as *mut _));
        };
    }
}

struct RefFrame<'a> {
    frame: &'a mut AVFrame,
}

impl<'a> Drop for RefFrame<'a> {
    fn drop(&mut self) {
        unsafe { av_frame_unref(self.frame) };
    }
}

impl<'a> RefFrame<'a> {
    fn new(codec_ctx: &mut AVCodecContext, frame: &'a mut AVFrame) -> Result<Option<Self>> {
        match unsafe { avcodec_receive_frame(codec_ctx, frame) } {
            // TODO: match non-fatal errors
            -11 => Ok(None),
            e if e < 0 => return Err(format!("Failed to recv frame ({})", e).into()),
            _ => Ok(Some(Self { frame })),
        }
    }
}

impl<'a> Deref for RefFrame<'a> {
    type Target = AVFrame;

    fn deref(&self) -> &Self::Target {
        self.frame
    }
}

impl<'a> DerefMut for RefFrame<'a> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.frame
    }
}

impl<T: Read + Seek> AvDecoder<T> {
    pub fn try_new(
        stream: Box<T>,
        aspect_ratio_scale: (usize, usize),
        max_mfield_size: (usize, usize),
    ) -> Result<Self> {
        let av_ctx = AvContext::try_new(stream)?;

        let mut decoder: Option<&mut AVCodec> = None;

        let stream_idx = match unsafe {
            av_find_best_stream(
                av_ctx.fmt_ctx,
                AVMediaType::AVMEDIA_TYPE_VIDEO,
                -1,
                -1,
                decoder.as_mut_ptr(),
                0,
            )
        } {
            e if e < 0 => Err(format!("Failed to find a stream ({})", e)),
            i => Ok(i),
        }?;

        let mut codec_ctx = unsafe { avcodec_alloc_context3(decoder.as_ptr()).as_mut() }
            .ok_or("Failed to allocate codec context")?;

        let stream = unsafe { (*av_ctx.fmt_ctx.streams.offset(stream_idx as _)).as_mut() }
            .ok_or("Stream info null")?;

        println!("{:?}", stream);

        let framerate = if stream.avg_frame_rate.den != 0 && stream.avg_frame_rate.num != 0 {
            stream.avg_frame_rate.num as f64 / stream.avg_frame_rate.den as f64
        } else {
            stream.time_base.den as f64 / stream.time_base.num as f64
        };

        match unsafe { avcodec_parameters_to_context(codec_ctx, stream.codecpar) } {
            e if e < 0 => {
                unsafe { avcodec_free_context(codec_ctx.as_mut_ptr()) };
                return Err(format!("Failed to get codec parameters ({})", e).into());
            }
            _ => {}
        }

        let mut av_opts: Option<&mut AVDictionary> = None;

        unsafe {
            av_dict_set(
                av_opts.as_mut_ptr(),
                c_str!("flags2").as_ptr(),
                c_str!("+export_mvs").as_ptr(),
                0,
            );
        }

        match unsafe { avcodec_open2(codec_ctx, decoder.as_ptr(), av_opts.as_mut_ptr()) } {
            e if e < 0 => {
                unsafe { avcodec_free_context(codec_ctx.as_mut_ptr()) };
                return Err(format!("Failed to open codec ({})", e).into());
            }
            _ => {}
        }

        let av_frame = unsafe { av_frame_alloc().as_mut() }.ok_or("Unable to allocate frame")?;

        Ok(Self {
            av_ctx,
            codec_ctx,
            av_frame,
            stream_idx,
            framerate,
            aspect_ratio: (0, 0),
            aspect_ratio_scale,
            max_mfield_size,
        })
    }

    // https://stackoverflow.com/questions/67828088/pyav-ffmpeg-libav-access-side-data-without-decoding-the-video
    // For now we will decode the packets, but later we should be able to do it without a decoder.

    // HEVC uses CTUs: https://en.wikipedia.org/wiki/Coding_tree_unit

    pub fn extract_mvs(&mut self, packet: &mut AVPacket, mf: &mut MotionField) -> Result<bool> {
        match unsafe { avcodec_send_packet(self.codec_ctx, packet) } {
            e if e < 0 => return Err(format!("Failed to send packet ({})", e).into()),
            _ => {}
        }

        if let Some(frame) = RefFrame::new(&mut self.codec_ctx, &mut self.av_frame)? {
            self.aspect_ratio = (frame.width as usize, frame.height as usize);

            if mf.dim() == (0, 0) {
                let ratio = (
                    self.aspect_ratio.0 * self.aspect_ratio_scale.0,
                    self.aspect_ratio.1 * self.aspect_ratio_scale.1,
                );
                let (w, h) = self.max_mfield_size;
                let width_based = (w, w * ratio.1 / ratio.0);
                let height_based = (h * ratio.0 / ratio.1, h);
                println!("{:?} {:?} | {} {}", width_based, height_based, w, h);
                let (w, h) = if width_based.0 < height_based.0 {
                    width_based
                } else {
                    height_based
                };
                *mf = MotionField::new(w, h)
            }

            if let Some(side_data) = unsafe {
                av_frame_get_side_data(&*frame, AVFrameSideDataType::AV_FRAME_DATA_MOTION_VECTORS)
                    .as_ref()
            } {
                let size = side_data.size as usize / std::mem::size_of::<AVMotionVector>();
                trace!("GOT MVS! {:?} {}", side_data.data, size);
                let motion_vectors =
                    unsafe { slice::from_raw_parts(side_data.data as *const AVMotionVector, size) };

                let mut downscale_mf = mf.new_downscale();

                let frame_norm =
                    na::Vector2::new(1f32 / frame.width as f32, 1f32 / frame.height as f32);

                for mv in motion_vectors {
                    let pos = na::Vector2::new(mv.src_x as f32, mv.src_y as f32)
                        .component_mul(&frame_norm)
                        .into();
                    let motion = na::Vector2::new(mv.motion_x as f32, mv.motion_y as f32)
                        .component_mul(&frame_norm);

                    downscale_mf.add_vector(pos, motion);
                }

                mf.from_downscale(&downscale_mf);

                Ok(true)
            } else {
                trace!("NO MVS :(");
                Ok(false)
            }
        } else {
            println!("NOPE FRAME NOPE");
            Ok(false)
        }
    }
}

impl<T: Read + Seek> Decoder for AvDecoder<T> {
    fn process_frame(&mut self, mf: &mut MotionField) -> Result<bool> {
        let mut packet = MaybeUninit::uninit();
        let mut reached_stream = false;
        let mut ret = false;

        while !reached_stream {
            match unsafe { av_read_frame(self.av_ctx.fmt_ctx, packet.as_mut_ptr()) } {
                e if e < 0 => return Err(format!("Failed to read frame ({})", e).into()),
                _ => {
                    let packet = unsafe { packet.assume_init_mut() };

                    trace!("Read packet: {} {}", packet.stream_index, packet.size);

                    if packet.stream_index == self.stream_idx {
                        debug!("Reached wanted stream!");
                        reached_stream = true;
                        ret = self.extract_mvs(packet, mf)?;
                    }

                    unsafe { av_packet_unref(packet) };
                }
            }
        }

        Ok(ret)
    }

    fn get_framerate(&self) -> Option<f64> {
        Some(self.framerate)
    }

    fn get_aspect(&self) -> Option<(usize, usize)> {
        Some(self.aspect_ratio)
    }
}
