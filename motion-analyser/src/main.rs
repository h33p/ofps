//! Analyse accuracy of motion predictions

use motion_vectors::prelude::v1::*;
use nalgebra as na;
use rayon::prelude::*;
use std::fs::File;
use std::io::{BufWriter, Write};

fn create_states() -> Vec<AnalysisState> {
    vec![
        AlmeidaEstimator::default().into(),
        MultiviewEstimator::default().into(),
    ]
}

fn main() -> Result<()> {
    let input = std::env::args()
        .nth(1)
        .ok_or("Please supply an input file!")?;

    //let ground_truth = std::env::args()
    //    .nth(2)
    //    .ok_or("Please supply a ground truth file!")?;

    let fov = 60.0;
    let camera = StandardCamera::new(fov, fov * 9.0 / 16.0);

    let mut c = motion_loader::create_decoder(&input)?;

    let mut motion_vectors = vec![];

    let mut ground_truth = EstimatorState::from(AlmeidaEstimator::default());
    let mut states = create_states();

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
    }

    fn metric(&self) -> f32 {
        0.0
    }
}

struct EstimatorState {
    estimator: Box<dyn Estimator + Send + Sync>,
    states: Vec<MotionState>,
}

impl<T: Estimator + Send + Sync + 'static> From<T> for EstimatorState {
    fn from(estimator: T) -> Self {
        Self {
            estimator: Box::new(estimator),
            states: vec![],
        }
    }
}

impl EstimatorState {
    fn on_frame(&mut self, motion_vectors: &[MotionEntry], camera: &StandardCamera) {
        let motion = self
            .estimator
            .estimate(motion_vectors, camera)
            .unwrap_or_default();
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
