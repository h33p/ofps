//! Analyse accuracy of motion predictions

use almeida_estimator::*;
use libmv_estimator::*;
use nalgebra as na;
use ofps::prelude::v1::*;
use rayon::prelude::*;
use std::fs::File;
use std::io::{BufWriter, Write};

fn create_states() -> Vec<AnalysisState> {
    vec![
        AlmeidaEstimator::default().into(),
        LibmvEstimator::default().into(),
        LibmvEstimator::default()
            .outlier_proba(0.7)
            .max_error(0.0001)
            .algo_points(8)
            .into(),
    ]
}

fn main() -> Result<()> {
    let input = std::env::args()
        .nth(1)
        .ok_or_else(|| anyhow!("Please supply an input file!"))?;

    //let ground_truth = std::env::args()
    //    .nth(2)
    //    .ok_or("Please supply a ground truth file!")?;

    let fov = 60.0;
    let camera = StandardCamera::new(fov, fov * 9.0 / 16.0);

    let mut c = motion_loader::create_decoder(&input)?;

    let mut motion_vectors = vec![];

    let mut ground_truth = EstimatorState::from(AlmeidaEstimator::default());
    let mut states = create_states();
    states[0].state.scale = 0.5;

    // Gather the data and perform initial analysis
    while let Ok(got_vecs) = {
        motion_vectors.clear();
        c.process_frame(&mut motion_vectors, None, 0)
    } {
        ground_truth.on_frame(&motion_vectors, &camera);
        states
            .par_iter_mut()
            .for_each(|a| a.on_frame(&motion_vectors, &camera, &ground_truth));
    }

    // Perform final steps of analysis where all of the data is needed
    // (translation comparison etc.)
    for s in &mut states {
        s.finalise(&ground_truth);
        println!("Similarity: {}", s.metric());
    }

    Ok(())
}

#[derive(Clone, Copy, Debug)]
struct AbsoluteDeltas {
    cumulative_q: f32,
    q: f32,
    r: f32,
    p: f32,
    y: f32,
}

struct AnalysisState {
    state: EstimatorState,
    deltas: Vec<AbsoluteDeltas>,
    pos_delta: Vec<na::Vector3<f32>>,
    tr_delta: Vec<na::Vector3<f32>>,
    cos_similarity: Vec<f32>,
    cos_pos_similarity: Vec<f32>,
    max_tr: f32,
    max_tr_ground_truth: f32,
}

impl<T: Into<EstimatorState>> From<T> for AnalysisState {
    fn from(state: T) -> Self {
        Self {
            state: state.into(),
            deltas: vec![],
            pos_delta: vec![],
            tr_delta: vec![],
            cos_similarity: vec![],
            cos_pos_similarity: vec![],
            max_tr: 0.0,
            max_tr_ground_truth: 0.0,
        }
    }
}

impl AnalysisState {
    fn on_frame(
        &mut self,
        motion_vectors: &[MotionEntry],
        camera: &StandardCamera,
        ground_truth: &EstimatorState,
    ) {
        self.state.on_frame(motion_vectors, camera);

        let self_state = self.state.states.last().unwrap();
        let ground_truth_state = ground_truth.states.last().unwrap();

        self.max_tr = self
            .max_tr
            .max(self_state.pos.x)
            .max(self_state.pos.y)
            .max(self_state.pos.z);

        self.max_tr_ground_truth = self
            .max_tr_ground_truth
            .max(ground_truth_state.pos.x)
            .max(ground_truth_state.pos.y)
            .max(ground_truth_state.pos.z);

        let (r1, p1, y1) = self_state.prev_r.euler_angles();
        let (r2, p2, y2) = ground_truth_state.prev_r.euler_angles();

        let deltas = AbsoluteDeltas {
            cumulative_q: self_state.rot.angle_to(&ground_truth_state.rot),
            q: self_state.prev_r.angle_to(&ground_truth_state.prev_r),
            r: r2 - r1,
            p: p2 - p1,
            y: y2 - y1,
        };

        self.deltas.push(deltas);

        let srv = self_state.prev_r * na::matrix![1.0; 0.0; 0.0];
        let s = na::Vector6::from_iterator(
            srv.iter()
                .copied()
                .chain(self_state.prev_tr.iter().copied()),
        );
        let trv = ground_truth_state.prev_r * na::matrix![1.0; 0.0; 0.0];
        let t = na::Vector6::from_iterator(
            trv.iter()
                .copied()
                .chain(ground_truth_state.prev_tr.iter().copied()),
        );

        let st = s.dot(&t);
        let inv = (s.dot(&s) * t.dot(&t)).sqrt();
        let cosang = if inv != 0.0 {
            st / inv
        } else if st == 0.0 {
            1.0
        } else {
            0.0
        };

        println!("CA: {}", cosang);

        self.cos_similarity.push(cosang);

        let s = self_state.prev_tr;
        let t = ground_truth_state.prev_tr;

        let st = s.dot(&t);
        let inv = (s.dot(&s) * t.dot(&t)).sqrt();
        let cosang = if inv != 0.0 {
            st / inv
        } else if s == t {
            1.0
        } else {
            0.0
        };

        println!("CA: {}", cosang);

        self.cos_pos_similarity.push(cosang);
    }

    fn finalise(&mut self, ground_truth: &EstimatorState) {
        let inv_max_tr = if self.max_tr == 0.0 {
            0.0
        } else {
            1.0 / self.max_tr
        };
        let inv_max_tr_ground_truth = if self.max_tr_ground_truth == 0.0 {
            0.0
        } else {
            1.0 / self.max_tr_ground_truth
        };

        let mut tr_self = vec![];
        let mut tr_ground_truth = vec![];
        let mut pos_self = vec![];
        let mut pos_ground_truth = vec![];

        for (self_state, ground_truth_state) in
            self.state.states.iter().zip(ground_truth.states.iter())
        {
            tr_self.push(self_state.prev_tr * inv_max_tr);
            tr_ground_truth.push(ground_truth_state.prev_tr * inv_max_tr_ground_truth);
            pos_self.push(self_state.pos * inv_max_tr);
            pos_ground_truth.push(ground_truth_state.pos * inv_max_tr_ground_truth);
        }

        // Calculate Mean Squared Error for rotation and position deltas.
        let mut mse: na::VectorN<f32, na::U11> = Default::default();

        for (deltas, ((tr_self, tr_ground_truth), (pos_self, pos_ground_truth))) in
            self.deltas.iter().zip(
                tr_self
                    .iter()
                    .zip(tr_ground_truth.iter())
                    .zip(pos_self.iter().zip(pos_ground_truth.iter())),
            )
        {
            let dtr = tr_ground_truth - tr_self;
            let dpos = pos_ground_truth - pos_self;
            let elem = na::VectorN::<f32, na::U11>::from_iterator([
                deltas.cumulative_q,
                deltas.q,
                deltas.r,
                deltas.p,
                deltas.y,
                dtr.x,
                dtr.y,
                dtr.z,
                dpos.x,
                dpos.y,
                dpos.z,
            ]);
            mse += elem.component_mul(&elem);
        }

        println!("{:?}", self.deltas.last());

        mse = na::VectorN::<f32, na::U11>::from_iterator(mse.iter().map(|v| v.sqrt()))
            .scale(1.0 / self.deltas.len() as f32);

        println!("MSE: {:?}", mse);

        // Calculate rotation correlation

        assert_eq!(self.state.states.len(), ground_truth.states.len());

        let mut means_s = vec![];
        let mut means_t = vec![];

        let mut mean_s = na::Vector3::default();
        let mut mean_t = na::Vector3::default();

        for (i, (self_state, truth_state)) in self
            .state
            .states
            .iter()
            .zip(ground_truth.states.iter())
            .enumerate()
        {
            let s = self_state.prev_r * na::matrix![1.0; 0.0; 0.0];
            let t = truth_state.prev_r * na::matrix![1.0; 0.0; 0.0];
            mean_s += s;
            mean_t += t;
            means_s.push(mean_s.scale(1.0 / (i + 1) as f32));
            means_t.push(mean_t.scale(1.0 / (i + 1) as f32));
        }

        let mut nom = 0.0;
        let mut sqr_denom_s = na::Vector3::default();
        let mut sqr_denom_t = na::Vector3::default();

        fn similarity_measure_base(pred: f32, truth: f32) -> f32 {
            use std::f32::consts::{FRAC_PI_2, PI};
            let sgn = truth.signum();
            if pred < truth.abs() {
                sgn * (FRAC_PI_2 * pred / truth.abs()).sin()
            } else if pred < PI - truth.abs() {
                sgn * (FRAC_PI_2 - FRAC_PI_2 * (pred - truth.abs()) / (FRAC_PI_2 - truth.abs()))
            } else {
                sgn * (-FRAC_PI_2 + FRAC_PI_2 * (pred - PI + truth.abs()) / truth.abs())
            }
        }

        fn similarity_measure(pred: f32, truth: f32, tolerance: f32) -> f32 {
            use std::f32::consts::PI;
            if (pred - truth).abs() <= tolerance {
                1.0
            } else {
                similarity_measure_base((pred + PI) % PI, truth)
                    .max(similarity_measure_base((pred - tolerance + PI) % PI, truth))
                    .max(similarity_measure_base((pred + tolerance) % PI, truth))
            }
        }

        let mut rot_similarity = na::Vector3::default();

        for (i, (self_state, truth_state)) in self
            .state
            .states
            .iter()
            .zip(ground_truth.states.iter())
            .enumerate()
        {
            let (r1, p1, y1) = self_state.prev_r.euler_angles();
            let (r2, p2, y2) = truth_state.prev_r.euler_angles();
            rot_similarity += na::matrix![
                similarity_measure(r1, r2, 0.001);
                similarity_measure(p1, p2, 0.001);
                similarity_measure(y1, y2, 0.001)
            ];
        }

        println!(
            "ROT SIM {:?}",
            rot_similarity.scale(1.0 / (self.state.states.len() as f32))
        );

        for ((self_state, truth_state), (mean_s, mean_t)) in self
            .state
            .states
            .iter()
            .zip(ground_truth.states.iter())
            .zip(means_s.iter().copied().zip(means_t.iter().copied()))
        {
            let mean_s = means_s.last().unwrap();
            let mean_t = means_t.last().unwrap();
            let s = self_state.prev_r * na::matrix![1.0; 0.0; 0.0];
            let t = truth_state.prev_r * na::matrix![1.0; 0.0; 0.0];
            //let s = s - mean_s;
            //let t = t - mean_t;
            nom += s.dot(&t);
            // ZNCC defined by Almeida does (F-\bar{F})^2 * (F-\bar{G})^2, but that seems wrong.
            sqr_denom_s += s.component_mul(&s);
            sqr_denom_t += t.component_mul(&t);
        }

        let denom = sqr_denom_s.dot(&sqr_denom_t).sqrt();

        //let denom = na::Vector3::from_iterator(sqr_denom.iter().map(|v| v.sqrt()));

        let correlation = nom / denom;

        println!("Rotation: {:?}", correlation);

        let s2 = self
            .cos_similarity
            .iter()
            .map(|v| (1.0 - *v))
            .map(|v| v * v)
            .sum::<f32>()
            .sqrt();

        println!(
            "ROT2: {s2} | {} {}",
            s2 / (self.cos_similarity.len() as f32),
            self.cos_similarity.iter().copied().fold(1.0, f32::min)
        );

        let s2 = self.cos_pos_similarity.iter().sum::<f32>();

        println!(
            "ROT2: {s2} | {} {}",
            s2 / (self.cos_pos_similarity.len() as f32),
            self.cos_pos_similarity.iter().copied().fold(1.0, f32::min)
        );
    }

    fn metric(&self) -> f32 {
        0.0
    }
}

struct EstimatorState {
    estimator: Box<dyn Estimator + Send + Sync>,
    states: Vec<MotionState>,
    scale: f32,
}

impl<T: Estimator + Send + Sync + 'static> From<T> for EstimatorState {
    fn from(estimator: T) -> Self {
        Self {
            estimator: Box::new(estimator),
            states: vec![],
            scale: 1.0,
        }
    }
}

impl EstimatorState {
    fn on_frame(&mut self, motion_vectors: &[MotionEntry], camera: &StandardCamera) {
        let mut motion = self
            .estimator
            .estimate(motion_vectors, camera, None)
            .unwrap_or_default();

        motion.0 = na::UnitQuaternion::identity().nlerp(&motion.0, self.scale);
        motion.1 *= self.scale;
        let new_state = if let Some(old_state) = self.states.last() {
            (motion, old_state).into()
        } else {
            motion.into()
        };
        self.states.push(new_state);
    }
}

struct MotionState {
    prev_r: na::UnitQuaternion<f32>,
    prev_tr: na::Vector3<f32>,
    rot: na::UnitQuaternion<f32>,
    pos: na::Vector3<f32>,
}

impl From<((na::UnitQuaternion<f32>, na::Vector3<f32>), &Self)> for MotionState {
    fn from(((r, tr), prev): ((na::UnitQuaternion<f32>, na::Vector3<f32>), &Self)) -> Self {
        println!("{:?}", prev.rot.euler_angles());
        Self {
            prev_r: r,
            prev_tr: tr,
            rot: (r * prev.rot),
            pos: (prev.pos + prev.rot * tr),
        }
    }
}

impl From<(na::UnitQuaternion<f32>, na::Vector3<f32>)> for MotionState {
    fn from((r, tr): (na::UnitQuaternion<f32>, na::Vector3<f32>)) -> Self {
        Self {
            prev_r: r,
            prev_tr: tr,
            rot: r,
            pos: tr,
        }
    }
}
