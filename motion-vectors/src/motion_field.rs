use crate::result::Result;
use nalgebra::*;

pub struct MotionField {
    vf: Matrix2xX<f32>,
    width: usize,
}

impl MotionField {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            vf: Matrix2xX::repeat(width * height, 0f32),
            width,
        }
    }

    pub fn dim(&self) -> (usize, usize) {
        if self.width == 0 {
            (0, 0)
        } else {
            (self.width, self.vf.ncols() / self.width)
        }
    }

    pub fn size(&self) -> usize {
        self.vf.ncols()
    }

    pub fn as_slice(&self) -> &[f32] {
        self.vf.as_slice()
    }

    pub fn new_downscale(&self) -> DownscaleMotionField {
        let (w, h) = self.dim();
        DownscaleMotionField::new(w, h)
    }

    pub fn from_downscale(&mut self, downscale: &DownscaleMotionField) {
        assert_eq!(downscale.mf.dim(), self.dim());
        self.vf = downscale.mf.vf.component_div(&downscale.counts);
    }

    pub fn get_motion(&self, x: usize, y: usize) -> Vector2<f32> {
        self.vf.column(self.width * y + x).into()
    }
}

pub struct DownscaleMotionField {
    mf: MotionField,
    counts: Matrix2xX<f32>,
}

impl DownscaleMotionField {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            mf: MotionField::new(width, height),
            counts: Matrix2xX::repeat(width * height, std::f32::EPSILON),
        }
    }

    pub fn add_vector_idx(&mut self, idx: usize, motion: Vector2<f32>, weight: f32) {
        self.counts[(0, idx)] += weight;
        self.counts[(1, idx)] += weight;
        self.mf
            .vf
            .set_column(idx, &(Matrix2x1::from(motion) + self.mf.vf.column(idx)));
    }

    pub fn add_vector_pos(&mut self, x: usize, y: usize, motion: Vector2<f32>, weight: f32) {
        let idx = y * self.mf.width + x;
        self.add_vector_idx(idx, motion, weight);
    }

    pub fn add_vector_weighted(&mut self, pos: Point2<f32>, motion: Vector2<f32>, weight: f32) {
        let pos = clamp(pos, Point2::new(0f32, 0f32), Point2::new(1f32, 1f32));
        let (w, h) = self.mf.dim();
        let (x, y) = (
            (pos.x * (w - 1) as f32).round() as usize,
            (pos.y * (h - 1) as f32).round() as usize,
        );
        self.add_vector_pos(x, y, motion, weight);
    }

    pub fn add_vector(&mut self, pos: Point2<f32>, motion: Vector2<f32>) {
        self.add_vector_weighted(pos, motion, 1.0);
    }

    /// Calculate empty cells from non-empty neghbors
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
            //.collect::<std::collections::VecDeque<_>>();
            .collect::<std::collections::BTreeSet<_>>();

        println!("{} {}", queue.len(), self.mf.size());

        // Special case when there are no motion vectors at all
        if queue.len() == self.mf.size() {
            return Ok(());
            //return Err("No motion vectors!".into())
        }

        //queue.make_contiguous().sort();

        while let Some(cell) = queue.take(&queue.iter().copied().next().unwrap_or_default()) {
            //c += 1;
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
                        self.add_vector_idx(
                            i,
                            (scale * inv_cnt * self.mf.vf.column(idx)).into(),
                            scale,
                        );
                        added = true;
                    }
                }
            }

            if !added {
                queue.insert(cell);
                panic!("Uh oh");
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
                            unreachable!("UH OH {} {}", idx, cnt);
                        }
                    }
                }
            }
        }

        println!("POST: {}", queue.len());

        Ok(())
    }
}

impl From<DownscaleMotionField> for MotionField {
    fn from(
        DownscaleMotionField {
            mf: MotionField { mut vf, width },
            counts,
        }: DownscaleMotionField,
    ) -> Self {
        vf.component_mul_assign(&counts);

        Self { vf, width }
    }
}
