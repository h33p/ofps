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

    pub fn add_vector(&mut self, pos: Point2<f32>, motion: Vector2<f32>) {
        let pos = clamp(pos, Point2::new(0f32, 0f32), Point2::new(1f32, 1f32));
        let (w, h) = self.mf.dim();
        let (x, y) = (
            (pos.x * (w - 1) as f32).round() as usize,
            (pos.y * (h - 1) as f32).round() as usize,
        );
        let idx = y * w + x;
        self.counts[(0, idx)] += 1f32;
        self.counts[(1, idx)] += 1f32;
        self.mf
            .vf
            .set_column(idx, &(Matrix2x1::from(motion) + self.mf.vf.column(idx)));
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
