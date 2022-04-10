//! Extract motion vectors into a easy-to-read file.

use ofps::prelude::v1::*;
use std::fs::File;
use std::io::{BufWriter, Write};

fn main() -> Result<()> {
    let input = std::env::args()
        .nth(1)
        .ok_or_else(|| anyhow!("Please supply a video file!"))?;

    // Output file must always end with `.mvec` for the loader to detect it.
    let output = std::env::args().nth(2);
    let output = output.as_deref().unwrap_or(&input);
    let output = format!("{output}.mvec");

    let mut c = motion_loader::create_decoder(&input, None)?;

    let out = File::create(output)?;
    let mut motion_vectors = vec![];
    let mut out = BufWriter::new(out);

    while c.process_frame(&mut motion_vectors, None, 0).is_ok() {
        // First encode number of MVs in a 32-bit LE integer.
        out.write_all(&(motion_vectors.len() as u32).to_le_bytes())?;
        // Then write each MV as a 4 f32 groups (in LE).
        for v in motion_vectors
            .iter()
            .flat_map(|(a, m)| [a.x, a.y, m.x, m.y])
        {
            out.write_all(&v.to_le_bytes())?;
        }
        // Clear the buffer.
        motion_vectors.clear();
    }

    Ok(())
}
