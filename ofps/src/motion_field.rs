//! # Fixed size motion field

use anyhow::Result;
use nalgebra::*;

/// Fixed size optical flow motion field.
pub struct MotionField {
    vf: Matrix2xX<f32>,
    width: usize,
}

impl MotionField {
    /// Create a new motion field.
    ///
    /// # Arguments
    ///
    /// * `width` - width of the field.
    /// * `height` - height of the field.
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            vf: Matrix2xX::repeat(width * height, 0f32),
            width,
        }
    }

    /// Get width and height of the motion field.
    pub fn dim(&self) -> (usize, usize) {
        if self.width == 0 {
            (0, 0)
        } else {
            (self.width, self.vf.ncols() / self.width)
        }
    }

    /// Get size of the motion field.
    ///
    /// This is the same as `width * height`
    pub fn size(&self) -> usize {
        self.vf.ncols()
    }

    /// Get the motion field in row-major order.
    ///
    /// The elements returned are in the following order:
    ///
    /// `field[0,0].x, field[0,0].y, field[0,1].x, ... field[0,N].y, field[1,0].x, ... field[N,N].y`
    pub fn as_slice(&self) -> &[f32] {
        self.vf.as_slice()
    }

    /// Set motion at given position.
    ///
    /// # Arguments
    ///
    /// * `x` - horizontal coordinate to set at.
    /// * `y` - vertical coordinate to set at.
    /// * `motion` - motion to set.
    pub fn set_motion(&mut self, x: usize, y: usize, motion: Vector2<f32>) {
        self.vf.set_column(self.width * y + x, &motion);
    }

    /// Create a new densification structure.
    ///
    /// `MotionFieldDensifier` takes arbitrary amount of motion vectors and densifies them to a
    /// fixed size motion field.
    pub fn new_densifier(&self) -> MotionFieldDensifier {
        let (w, h) = self.dim();
        MotionFieldDensifier::new(w, h)
    }

    /// Finalise the motion field densifier.
    ///
    /// # Arguments
    ///
    /// * `densifier` - motion field densifier.
    pub fn from_densifier(&mut self, densifier: &MotionFieldDensifier) {
        assert_eq!(densifier.mf.dim(), self.dim());
        self.vf = densifier.mf.vf.component_div(&densifier.counts);
    }

    /// Get motion at coordinates.
    ///
    /// # Arguments
    ///
    /// * `x` - horizontal coordinate.
    /// * `y` - vertical coordinate.
    pub fn get_motion(&self, x: usize, y: usize) -> Vector2<f32> {
        self.vf.column(self.width * y + x).into()
    }

    /// Iterate every element of the motion field.
    ///
    /// The resulting iterator yields `(x, y, motion)` entries.
    pub fn iter(&self) -> impl Iterator<Item = (usize, usize, Vector2<f32>)> + '_ {
        let (width, height) = self.dim();
        (0..height).into_iter().flat_map(move |y| {
            (0..width)
                .into_iter()
                .map(move |x| (x, y, self.get_motion(x, y)))
        })
    }

    /// Iterate every element of the motion field.
    ///
    /// The resulting iterator yields `MotionEntry` elements.
    pub fn motion_iter(&self) -> impl Iterator<Item = (Point2<f32>, Vector2<f32>)> + '_ {
        let (width, height) = self.dim();
        self.iter().map(move |(x, y, motion)| {
            (
                Point2::new(x as f32 / width as f32, y as f32 / height as f32),
                motion,
            )
        })
    }
}

/// Densify arbitrary number of motion vectors.
///
/// This structure provides facilities to add an arbitrary amount of motion vectors together and
/// then convert them to a fixed size motion field.
pub struct MotionFieldDensifier {
    mf: MotionField,
    counts: Matrix2xX<f32>,
}

impl MotionFieldDensifier {
    /// Create a new densifier
    ///
    /// # Arguments
    ///
    /// * `width` - width of the output motion field.
    /// * `heighy` - height of the output motion field.
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            mf: MotionField::new(width, height),
            counts: Matrix2xX::repeat(width * height, std::f32::EPSILON),
        }
    }

    /// Add a motion vector at specified index.
    fn add_vector_idx(&mut self, idx: usize, motion: Vector2<f32>, weight: f32) {
        self.counts[(0, idx)] += weight;
        self.counts[(1, idx)] += weight;
        self.mf
            .vf
            .set_column(idx, &(motion * weight + self.mf.vf.column(idx)));
    }

    /// Add a motion vector at specified motion field coordinates
    fn add_vector_pos(&mut self, x: usize, y: usize, motion: Vector2<f32>, weight: f32) {
        let idx = y * self.mf.width + x;
        self.add_vector_idx(idx, motion, weight);
    }

    /// Add a motion vector with custom weight.
    ///
    /// Returns cell of insertion.
    ///
    /// # Arguments
    ///
    /// * `pos` - starting position of the motion vector, in 0-1 range.
    /// * `motion` - motion of the vector.
    /// * `weight` - weight of the motion vector.
    pub fn add_vector_weighted(
        &mut self,
        pos: Point2<f32>,
        motion: Vector2<f32>,
        weight: f32,
    ) -> (usize, usize) {
        let pos = clamp(pos, Point2::new(0f32, 0f32), Point2::new(1f32, 1f32));
        let (w, h) = self.mf.dim();
        let (x, y) = (
            (pos.x * (w - 1) as f32).round() as usize,
            (pos.y * (h - 1) as f32).round() as usize,
        );
        self.add_vector_pos(x, y, motion, weight);
        (x, y)
    }

    /// Add a motion vector to the field.
    ///
    /// Returns cell of insertion.
    ///
    /// # Arguments
    ///
    /// * `pos` - starting position of the motion vector, in 0-1 range.
    /// * `motion` - motion of the vector.
    pub fn add_vector(&mut self, pos: Point2<f32>, motion: Vector2<f32>) -> (usize, usize) {
        self.add_vector_weighted(pos, motion, 1.0)
    }

    /// Calculate empty cells from non-empty neghbors.
    pub fn interpolate_empty_cells(&mut self) -> Result<()> {
        #[derive(PartialOrd, PartialEq, Ord, Clone, Copy, Eq, Debug, Default)]
        struct InterpCell {
            neighbors: isize,
            idx: usize,
        }

        impl std::borrow::Borrow<usize> for InterpCell {
            fn borrow(&self) -> &usize {
                &self.idx
            }
        }

        let (width, height) = self.mf.dim();

        let neighbors = [(-1, 0), (0, -1), (-1, -1), (1, 0), (0, 1), (1, 1)];

        let calc_counts = |s: &Self, i| {
            let mut cnt = 0;

            let (x, y) = (i % width, i / width);

            for (ox, oy) in neighbors {
                let (x, y) = (x as isize + ox, y as isize + oy);
                if x >= 0
                    && x < width as isize
                    && y >= 0
                    && y < height as isize
                    && s.counts[(0, x as usize + y as usize * width)] > 0.1
                {
                    cnt += 1;
                }
            }

            cnt
        };

        let mut queue = self
            .counts
            .as_slice()
            .chunks_exact(2)
            .enumerate()
            .filter(|(_, c)| c[0] < 0.5)
            .map(|(i, _)| InterpCell {
                idx: i,
                neighbors: -calc_counts(self, i),
            })
            .collect::<std::collections::BTreeSet<_>>();

        // Special case when there are no motion vectors at all
        if queue.len() == self.mf.size() {
            return Ok(());
        }

        while let Some(cell) = queue.take(&queue.iter().copied().next().unwrap_or_default()) {
            let i = cell.idx;

            let (x, y) = (i % width, i / width);

            let mut added = false;

            // Interpolate with all neighbors
            for (ox, oy) in neighbors {
                let (x, y) = (x as isize + ox, y as isize + oy);
                if x >= 0 && x < width as isize && y >= 0 && y < height as isize {
                    let idx = x as usize + y as usize * width;
                    let cnt = self.counts[(0, idx)];
                    if cnt > 0.1 {
                        let scale = 1.0 - ((ox * ox + oy * oy) as f32).sqrt() * 0.5;
                        let inv_cnt = 1.0 / cnt;
                        self.add_vector_idx(i, scale * inv_cnt * self.mf.vf.column(idx), scale);
                        added = true;
                    }
                }
            }

            if !added {
                queue.insert(cell);
            } else {
                // Recalculate all neighbor weights if we interpolate self
                for (ox, oy) in neighbors {
                    let (x, y) = (x as isize + ox, y as isize + oy);
                    if x >= 0 && x < width as isize && y >= 0 && y < height as isize {
                        let idx = x as usize + y as usize * width;
                        // Add one to account for new neighbor
                        let cnt = -calc_counts(self, idx) + 1;
                        if let Some(mut neighbor) = queue.take(&InterpCell {
                            idx,
                            neighbors: cnt,
                        }) {
                            neighbor.neighbors -= 1;
                            queue.insert(neighbor);
                        } else if cnt != 0 && self.counts[(0, idx)] < 0.1 {
                            unreachable!("{} {}", idx, cnt);
                        }
                    }
                }
            }
        }

        Ok(())
    }
}

impl From<MotionFieldDensifier> for MotionField {
    fn from(
        MotionFieldDensifier {
            mf: MotionField { mut vf, width },
            counts,
        }: MotionFieldDensifier,
    ) -> Self {
        vf.component_div_assign(&counts);

        Self { vf, width }
    }
}
