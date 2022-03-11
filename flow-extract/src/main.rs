//! Extract motion vectors to .flo files

use ofps::prelude::v1::{Result, *};
use opencv::core::*;
use opencv::imgproc::*;
use opencv::types::VectorOfMat;
use opencv::video::*;
use std::fs::File;
use std::io::{BufWriter, Write};

fn flow_to_display(flow: &Mat) -> Result<Mat> {
    let mut flow_split = VectorOfMat::new();
    flow_split.push(Mat::default());
    flow_split.push(Mat::default());
    let mut magnitude0 = Mat::default();
    let mut angle = Mat::default();
    let mut hsv_split = VectorOfMat::new();
    let mut hsv = Mat::default();
    let mut rgb = Mat::default();
    split(flow, &mut flow_split)?;
    let flow_split = [flow_split.get(0)?, flow_split.get(1)?];
    cart_to_polar(
        &flow_split[0],
        &flow_split[1],
        &mut magnitude0,
        &mut angle,
        true,
    )?;
    let mut magnitude = Mat::default();
    normalize(
        &magnitude0,
        &mut magnitude,
        0.0,
        1.0,
        NORM_MINMAX,
        -1,
        &mut no_array(),
    )?;
    let sz = angle.size()?;
    let typ = angle.typ();
    hsv_split.push(angle); // already in degrees - no normalization needed
    hsv_split.push(Mat::ones_size(sz, typ)?.to_mat()?);
    hsv_split.push(magnitude);
    merge(&hsv_split, &mut hsv)?;
    cvt_color(&hsv, &mut rgb, COLOR_HSV2BGR, 0)?;
    Ok(rgb)
}

fn main() -> Result<()> {
    let input = std::env::args()
        .nth(1)
        .ok_or_else(|| anyhow!("Please supply a video file!"))?;

    let input2 = std::env::args()
        .nth(3)
        .ok_or_else(|| anyhow!("Please supply a video file!"))?;

    let output = std::env::args().nth(2);
    let output = output.as_deref().unwrap_or(&input);

    std::fs::create_dir_all(output)?;

    let mut c = motion_loader::create_decoder(&input)?;

    let mut motion_vectors = vec![];
    let mut mf = MotionField::new(616 / 2, 186 / 2);
    let (x, y) = mf.dim();
    let mut flow = Mat::new_size_with_default(
        Size_ {
            width: x as _,
            height: y as _,
        },
        CV_32FC2,
        Default::default(),
    )?;
    let mut flow2 = flow.clone();

    let mut cnt = 0usize;

    while let Ok(filled) = c.process_frame(&mut motion_vectors, None, 0) {
        // If motion vectors were filled, update the dense field.
        // Else, reuse the previous values (this typically happens on an I frame).
        if filled {
            // Densify the field.
            let mut densify_mf = mf.new_densifier();

            for &(pos, motion) in &motion_vectors {
                densify_mf.add_vector(pos, motion);
            }

            // Interpolate any empty cells. TODO: configure this?
            let _ = densify_mf.interpolate_empty_cells();

            mf.from_densifier(&densify_mf);

            //mf.gaussian_blur(5);

            // Convert into CV Mat.
            for (x, y, motion) in mf.iter() {
                // TODO: report Rust ICE once open source:
                // flow.at_2d_mut::<f32>(y as _, x as _)? = motion.x;
                let pt = flow.at_2d_mut::<Point2f>(y as _, x as _)?;
                pt.x = motion.x;
                pt.y = motion.y;
            }

            gaussian_blur(
                &flow,
                &mut flow2,
                Size_ {
                    width: 11,
                    height: 11,
                },
                0.0,
                0.0,
                BorderTypes::BORDER_REPLICATE as _,
            )?;

            // Clear the field
            motion_vectors.clear();
        }

        let flw = flow_to_display(&flow2)?;

        let flow2 = read_optical_flow(&format!("{input2}/{cnt:06}.flo"))?;
        let flw2 = flow_to_display(&flow2)?;

        if opencv::highgui::wait_key(1)? >= 1 {
            break;
        }

        opencv::highgui::imshow("fl1", &flw)?;
        opencv::highgui::imshow("fl2", &flw2)?;

        if !write_optical_flow(&format!("{output}/{cnt:06}.flo"), &flow)? {
            return Err(anyhow!("Fail to write"));
        }

        cnt += 1;
    }

    Ok(())
}
