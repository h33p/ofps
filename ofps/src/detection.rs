//! Motion detection module

use crate::prelude::v1::*;
use nalgebra as na;

/// Block subdivision based motion detector.
///
/// This detector splits up each frame to blocks that are of area `min_size / (subdivide ^ 2)` and
/// checks average motion within each block to be at least of `target_motion` magnitude.
pub struct BlockMotionDetection {
    min_size: f32,
    subdivide: usize,
    target_motion: f32,
}

impl Default for BlockMotionDetection {
    fn default() -> Self {
        Self {
            min_size: 0.1,
            subdivide: 6,
            target_motion: 0.003,
        }
    }
}

impl BlockMotionDetection {
    pub fn detect_motion(
        &self,
        motion: impl Iterator<Item = MotionEntry>,
    ) -> Option<(usize, MotionField)> {
        // Calculate the motion field size, rounded up.
        let block_width = self.min_size.sqrt() / self.subdivide as f32;
        let block_dim = (1.0 / block_width).ceil() as usize;

        // Add all the motion to the field densifier.
        let mut mf = MotionFieldDensifier::new(block_dim, block_dim);
        motion.for_each(|(pos, motion)| mf.add_vector(pos, motion));
        let mf = MotionField::from(mf);

        let mut map = vec![vec![false; block_dim]; block_dim];

        // Compute which blocks have motion
        mf.iter()
            .filter(|(_, _, motion)| motion.magnitude() >= self.target_motion)
            .for_each(|(x, y, _)| map[y][x] = true);

        // Flood fill compute the area of the biggest motion.
        let mut biggest_area = 0;
        let mut biggest_area_mf = None;

        for y in 0..block_dim {
            for x in 0..block_dim {
                if map[y][x] {
                    let mut area = 0;
                    let mut mf2 = MotionField::new(block_dim, block_dim);

                    map[y][x] = false;
                    let mut to_fill = vec![(x, y); 1];

                    while let Some((x, y)) = to_fill.pop() {
                        area += 1;

                        let neighbor_offs = (-1..=1).flat_map(|x| (-1..=1).map(move |y| (x, y)));

                        // Go through each neighbor and add any unvisited and over threshold
                        // entries.
                        for (x, y) in neighbor_offs
                            .map(|(ox, oy)| (x as isize + ox, y as isize + oy))
                            .filter(|&(ox, oy)| {
                                (0..block_dim as isize).contains(&ox)
                                    && (0..block_dim as isize).contains(&oy)
                            })
                            .map(|(x, y)| (x as usize, y as usize))
                        {
                            if map[y][x] {
                                mf2.set_motion(x, y, mf.get_motion(x, y));
                                to_fill.push((x, y));
                                map[y][x] = false;
                            }
                        }
                    }

                    if area > biggest_area {
                        biggest_area = area;
                        biggest_area_mf = Some(mf2);
                    }
                }
            }
        }

        if biggest_area as f32 / (block_dim * block_dim) as f32 >= self.min_size {
            biggest_area_mf.map(|mf| (biggest_area, mf))
        } else {
            None
        }
    }
}
