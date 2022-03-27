//! OpenCV motion vector field implementation

use ::core::ffi::c_void;
use ::core::ops::{Deref, DerefMut};
use ::core::{ptr, slice};
use c_str_macro::c_str;
use libc::c_int;
use log::*;
use nalgebra as na;
use ofps::prelude::v1::{ptrplus::*, Result, *};
use ofps::utils::*;
use opencv::core::{Point2f, Size, Vec3b};
use opencv::imgproc;
use opencv::prelude::*;
use opencv::videoio::*;
use std::io::*;
use std::mem::MaybeUninit;

ofps::define_descriptor!(cv, Decoder, |args| CvDecoder::try_new(
    &args,
    (1, 1),
    (150, 150)
)
.map(|d| Box::new(d) as _));

pub struct CvDecoder {
    capture: VideoCapture,
    frame: Mat,
    gray_tmp: Mat,
    gray: Mat,
    old_gray: Mat,
    flow: Mat,
    aspect_ratio_scale: (usize, usize),
    max_mfield_size: (usize, usize),
}

impl Properties for CvDecoder {
    fn props_mut(&mut self) -> Vec<(&str, PropertyMut)> {
        vec![
            (
                "Width",
                PropertyMut::usize(&mut self.max_mfield_size.0, 1, 200),
            ),
            (
                "Height",
                PropertyMut::usize(&mut self.max_mfield_size.1, 1, 200),
            ),
        ]
    }
}

impl CvDecoder {
    pub fn try_new(
        stream: &str,
        aspect_ratio_scale: (usize, usize),
        max_mfield_size: (usize, usize),
    ) -> Result<Self> {
        let capture = VideoCapture::from_file(stream, 0)?;

        Ok(Self {
            capture,
            frame: Default::default(),
            gray: Default::default(),
            gray_tmp: Default::default(),
            old_gray: Default::default(),
            flow: Default::default(),
            aspect_ratio_scale,
            max_mfield_size,
        })
    }
}

impl Decoder for CvDecoder {
    fn process_frame(
        &mut self,
        mf: &mut MotionVectors,
        out_frame: Option<(&mut Vec<RGBA>, &mut usize)>,
        skip: usize,
    ) -> Result<bool> {
        let mut cnt = 0;

        let skip = 0;

        while cnt <= skip {
            if self.capture.read(&mut self.frame)? {
                cnt += 1;
                core::mem::swap(&mut self.old_gray, &mut self.gray);
                imgproc::cvt_color(&self.frame, &mut self.gray_tmp, imgproc::COLOR_BGR2GRAY, 0)?;

                let (dx, dy) = {
                    let (ax, ay) = self.get_aspect().unwrap();
                    let ratio = (
                        ax * self.aspect_ratio_scale.0,
                        ay * self.aspect_ratio_scale.1,
                    );
                    let (w, h) = self.max_mfield_size;
                    let width_based = (w, w * ratio.1 / ratio.0);
                    let height_based = (h * ratio.0 / ratio.1, h);
                    if width_based.0 < height_based.0 {
                        width_based
                    } else {
                        height_based
                    }
                };

                let (dx, dy) = if dx * 4 < self.gray_tmp.cols() as usize {
                    (dx * 4, dy * 4)
                } else {
                    (dx, dy)
                };

                imgproc::resize(
                    &self.gray_tmp,
                    &mut self.gray,
                    Size::new(dx as _, dy as _),
                    0.0,
                    0.0,
                    imgproc::INTER_LINEAR,
                )?;
            } else {
                return Err(anyhow!("Failed to grab frame"));
            }
        }

        if let Some((out_frame, out_height)) = out_frame {
            *out_height = self.frame.rows() as _;
            out_frame.clear();

            for y in 0..self.frame.rows() {
                for x in 0..self.frame.cols() {
                    let bgr: &Vec3b = self.frame.at_2d(y, x)?;
                    out_frame.push(RGBA::from_rgb_slice(&[bgr[2], bgr[1], bgr[0]]));
                }
            }
        }

        if self.gray.rows() != self.old_gray.rows() || self.gray.cols() != self.old_gray.cols() {
            return Ok(false);
        }

        opencv::video::calc_optical_flow_farneback(
            &self.old_gray,
            &self.gray,
            &mut self.flow,
            0.5,
            5,
            11,
            3,
            5,
            1.1,
            0,
        )?;

        let frame_norm = na::Vector2::new(
            1f32 / self.gray.cols() as f32,
            1f32 / self.gray.rows() as f32,
        );

        for y in 0..self.gray.rows() {
            for x in 0..self.gray.cols() {
                let dir: &Point2f = self.flow.at_2d(y, x)?;

                let pos = na::Vector2::new(x as f32, y as f32)
                    .component_mul(&frame_norm)
                    .into();

                let motion =
                    na::Vector2::new(dir.x as f32, dir.y as f32).component_mul(&frame_norm);

                mf.push((pos, motion));
            }
        }

        Ok(true)
    }

    fn get_framerate(&self) -> Option<f64> {
        self.capture.get(CAP_PROP_FPS).ok()
    }

    fn get_aspect(&self) -> Option<(usize, usize)> {
        Some((self.frame.cols() as _, self.frame.rows() as _))
    }
}
