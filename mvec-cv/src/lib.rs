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
    old_frame: Mat,
    frame_tmp: Mat,
    gray: Mat,
    old_gray: Mat,
    sobel: Mat,
    thresh: Mat,
    mask: Mat,
    flow: Mat,
    aspect_ratio_scale: (usize, usize),
    max_mfield_size: (usize, usize),
    use_rlof: bool,
    process_fullres: bool,
}

impl Properties for CvDecoder {
    fn props_mut(&mut self) -> Vec<(&str, PropertyMut)> {
        vec![
            (
                "Width",
                PropertyMut::usize(&mut self.max_mfield_size.0, 1, 2000),
            ),
            (
                "Height",
                PropertyMut::usize(&mut self.max_mfield_size.1, 1, 2000),
            ),
            ("RLOF", PropertyMut::bool(&mut self.use_rlof)),
            (
                "Process Fullres",
                PropertyMut::bool(&mut self.process_fullres),
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
            old_frame: Default::default(),
            frame_tmp: Default::default(),
            gray: Default::default(),
            old_gray: Default::default(),
            sobel: Default::default(),
            mask: Default::default(),
            thresh: Default::default(),
            flow: Default::default(),
            aspect_ratio_scale,
            max_mfield_size,
            use_rlof: false,
            process_fullres: true,
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

        let mut dxy = (0, 0);

        while cnt <= skip {
            if self.capture.read(&mut self.frame_tmp)? {
                cnt += 1;
                core::mem::swap(&mut self.old_frame, &mut self.frame);
                core::mem::swap(&mut self.old_gray, &mut self.gray);

                let (dx, dy) = {
                    let (ax, ay) = (
                        self.frame_tmp.cols() as usize,
                        self.frame_tmp.rows() as usize,
                    );
                    let ratio = (
                        ax * self.aspect_ratio_scale.0,
                        ay * self.aspect_ratio_scale.1,
                    );
                    let (w, h) = self.max_mfield_size;

                    let (w, h) = (
                        std::cmp::min(w, self.frame_tmp.cols() as _),
                        std::cmp::min(h, self.frame_tmp.rows() as _),
                    );

                    let width_based = (w, w * ratio.1 / ratio.0);
                    let height_based = (h * ratio.0 / ratio.1, h);
                    if width_based.0 < height_based.0 {
                        width_based
                    } else {
                        height_based
                    }
                };

                dxy = (dx, dy);

                if self.process_fullres {
                    std::mem::swap(&mut self.frame_tmp, &mut self.frame);
                } else {
                    imgproc::resize(
                        &self.frame_tmp,
                        &mut self.frame,
                        Size::new(dx as _, dy as _),
                        0.0,
                        0.0,
                        imgproc::INTER_LINEAR,
                    )?;
                }

                imgproc::cvt_color(&self.frame, &mut self.gray, imgproc::COLOR_BGR2GRAY, 0)?;
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

        let winsize = 5;

        let flags = if self.flow.cols() > 0 && self.flow.rows() > 0 {
            opencv::video::OPTFLOW_USE_INITIAL_FLOW
        } else {
            0
        };

        if self.use_rlof {
            opencv::optflow::calc_optical_flow_dense_rlof(
                &self.old_frame,
                &self.frame,
                &mut self.flow,
                opencv::optflow::RLOFOpticalFlowParameter::create()?,
                0.0,
                opencv::core::Size::new(8, 8),
                opencv::optflow::InterpolationType::INTERP_EPIC,
                128,
                0.05,
                100.0,
                15,
                100,
                true,
                500.0,
                1.5,
                false,
            )?;
        } else {
            opencv::video::calc_optical_flow_farneback(
                &self.old_gray,
                &self.gray,
                &mut self.flow,
                0.5,
                5,
                winsize + 8,
                3,
                7,
                1.5,
                flags,
            )?;
        }

        // Calculate sobel mask when using Gunnar-Farneback
        if !self.use_rlof {
            opencv::imgproc::sobel(
                &self.gray,
                &mut self.sobel,
                opencv::core::CV_32F,
                1,
                1,
                5,
                1.0,
                0.0,
                opencv::core::BORDER_DEFAULT,
            )?;

            opencv::imgproc::threshold(
                &self.sobel,
                &mut self.thresh,
                20.0,
                255.0,
                opencv::imgproc::THRESH_BINARY,
            )?;

            opencv::imgproc::dilate(
                &self.thresh,
                &mut self.mask,
                &opencv::imgproc::get_structuring_element(
                    opencv::imgproc::MORPH_ELLIPSE,
                    opencv::core::Size::new(winsize * 2 + 1, winsize * 2 + 1),
                    opencv::core::Point::new(winsize, winsize),
                )?,
                opencv::core::Point::new(-1, -1),
                1,
                opencv::core::BORDER_DEFAULT,
                opencv::imgproc::morphology_default_border_value()?,
            )?;
        }

        let frame_norm = na::Vector2::new(
            1f32 / self.gray.cols() as f32,
            1f32 / self.gray.rows() as f32,
        );

        let mut points = std::collections::BTreeSet::new();

        let mut downsample = if self.process_fullres {
            MotionFieldDensifier::new(dxy.0, dxy.1)
        } else {
            MotionFieldDensifier::new(0, 0)
        };

        for y in 0..self.gray.rows() {
            for x in 0..self.gray.cols() {
                if !self.use_rlof {
                    let mask: &f32 = self.mask.at_2d(y, x).unwrap();

                    if *mask < 0.1 {
                        continue;
                    }
                }

                let dir: &Point2f = self.flow.at_2d(y, x)?;

                let pos = na::Vector2::new(x as f32 + 0.5, y as f32 + 0.5)
                    .component_mul(&frame_norm)
                    .into();

                let motion =
                    na::Vector2::new(dir.x as f32, dir.y as f32).component_mul(&frame_norm);

                if self.process_fullres {
                    points.insert(downsample.add_vector(pos, motion));
                } else {
                    mf.push((pos, motion));
                }
            }
        }

        let downsampled = MotionField::from(downsample);

        let frame_norm = na::Vector2::new(1f32 / dxy.0 as f32, 1f32 / dxy.1 as f32);

        for (x, y) in points {
            let motion = downsampled.get_motion(x, y);

            let pos = na::Vector2::new(x as f32 + 0.5, y as f32 + 0.5)
                .component_mul(&frame_norm)
                .into();

            mf.push((pos, motion));
        }

        Ok(true)
    }

    fn get_framerate(&self) -> Option<f64> {
        self.capture.get(CAP_PROP_FPS).ok()
    }

    fn get_aspect(&self) -> Option<(usize, usize)> {
        Some((self.gray.cols() as _, self.gray.rows() as _))
    }
}
