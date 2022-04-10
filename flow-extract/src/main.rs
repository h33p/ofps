//! Extract motion vectors to .flo files

use clap::*;
use ofps::prelude::v1::{Result, *};
use opencv::core::*;
use opencv::imgproc::*;
use opencv::types::VectorOfMat;
use opencv::video::*;

fn main() -> Result<()> {
    let matches = Command::new("flow-extract")
        .version(crate_version!())
        .author(crate_authors!())
        .arg(
            Arg::new("width")
                .long("width")
                .short('w')
                .takes_value(true)
                .required(true),
        )
        .arg(
            Arg::new("height")
                .long("height")
                .short('h')
                .takes_value(true)
                .required(true),
        )
        .arg(
            Arg::new("input")
                .long("input")
                .short('i')
                .takes_value(true)
                .required(true),
        )
        .arg(
            Arg::new("draw-flow")
                .long("draw-flow")
                .short('d')
                .required(false),
        )
        .arg(Arg::new("output").takes_value(true).required(true))
        .get_matches();

    let input = matches.value_of("input").unwrap();
    let output = matches.value_of("output").unwrap();
    let width: usize = matches.value_of("width").unwrap().parse()?;
    let height: usize = matches.value_of("height").unwrap().parse()?;
    let draw_flow = matches.occurrences_of("draw-flow") > 0;

    let mut c = motion_loader::create_decoder(input, None)?;

    std::fs::create_dir_all(output)?;

    let mut motion_vectors = vec![];
    let mut mf = MotionField::new(width, height);
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

        if draw_flow {
            let flw = flow_to_display(&flow2)?;

            if opencv::highgui::wait_key(1)? >= 1 {
                break;
            }

            opencv::highgui::imshow("fl1", &flw)?;
        }

        if !write_optical_flow(&format!("{output}/{cnt:06}.flo"), &flow)? {
            return Err(anyhow!("Fail to write"));
        }

        cnt += 1;
    }

    Ok(())
}

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
        &no_array(),
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
