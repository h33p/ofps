//! Analyse accuracy of motion predictions

use motion_vectors::prelude::v1::*;
use std::fs::File;
use std::io::{BufWriter, Write};

fn main() -> Result<()> {
    let input = std::env::args()
        .nth(1)
        .ok_or("Please supply an input file!")?;

    let ground_truth = std::env::args()
        .nth(2)
        .ok_or("Please supply a ground truth file!")?;

    let mut c = motion_loader::create_decoder(&input)?;

    let mut motion_vectors = vec![];

    while let Ok(got_vecs) = {
        motion_vectors.clear();
        c.process_frame(&mut motion_vectors, None, 0)
    } {
        if got_vecs {}
    }

    Ok(())
}
