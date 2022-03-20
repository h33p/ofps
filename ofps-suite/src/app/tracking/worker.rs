use super::super::utils::worker::{AppWorker, Workable};
use nalgebra as na;
use ofps::prelude::v1::*;
use serde::{Deserialize, Serialize};
use std::sync::{
    atomic::AtomicBool,
    mpsc::{self, Receiver, Sender},
    Arc, Mutex,
};
use wimrend::material::Material;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct EstimatorSettings {
    pub scale_factor: f32,
    pub camera_offset: f32,
    pub layer_frames: bool,
    pub layer_angle_delta: f32,
    pub keep_frames: usize,
}

impl Default for EstimatorSettings {
    fn default() -> Self {
        Self {
            scale_factor: 1.0,
            camera_offset: 1.0,
            layer_frames: true,
            layer_angle_delta: 0.5,
            keep_frames: 100,
        }
    }
}

pub enum FrameState {
    Pending(Vec<RGBA>, usize),
    Loaded(Arc<Material>),
}

#[derive(Default, Clone)]
pub struct EstimatorState {
    pub poses: Vec<(na::Point3<f32>, na::UnitQuaternion<f32>)>,
    pub layered_frames: Vec<(usize, Arc<Mutex<FrameState>>)>,
}

impl EstimatorState {
    fn apply_pose(
        &self,
        tr: na::Vector3<f32>,
        rot: na::UnitQuaternion<f32>,
    ) -> (na::Point3<f32>, na::UnitQuaternion<f32>) {
        let (pos, old_rot) = self.poses.last().copied().unwrap_or_default();
        (pos + old_rot * tr, rot * old_rot)
    }

    fn layer_frame(
        &self,
        pos: na::Point3<f32>,
        rot: na::UnitQuaternion<f32>,
        settings: &EstimatorSettings,
    ) -> bool {
        settings.layer_frames && {
            self.layered_frames
                .last()
                .map(|(f, _)| {
                    let (op, or) = self.poses[*f];
                    (op - pos).magnitude() > 0.1
                        || rot.angle_to(&or) > settings.layer_angle_delta.to_radians()
                })
                .unwrap_or(true)
        }
    }

    fn push_pose(
        &mut self,
        pos: na::Point3<f32>,
        rot: na::UnitQuaternion<f32>,
        frame: Option<Arc<Mutex<FrameState>>>,
    ) {
        let idx = self.poses.len();
        self.poses.push((pos, rot));
        if let Some(mat) = frame {
            self.layered_frames.push((idx, mat));
        }
    }

    pub fn layered_frames(
        &self,
    ) -> impl Iterator<
        Item = (
            na::Point3<f32>,
            na::UnitQuaternion<f32>,
            Arc<Mutex<FrameState>>,
        ),
    > + '_ {
        self.layered_frames
            .iter()
            .cloned()
            .map(move |(i, mat)| (self.poses[i].0, self.poses[i].1, mat))
    }

    fn remove_least_significant_frame(&mut self) {
        let (frame, _) = self
            .layered_frames
            .iter()
            .enumerate()
            .map(|(i, s)| (i, self.poses[s.0].1))
            .fold(None, |candidate, (frame, rot)| {
                let dist = self
                    .layered_frames
                    .iter()
                    .map(|s| self.poses[s.0].1.angle_to(&rot))
                    .sum::<f32>();

                if let Some((frame, cur_dist)) = candidate {
                    if dist >= cur_dist {
                        return Some((frame, cur_dist));
                    }
                }

                Some((frame, dist))
            })
            .unwrap_or_default();

        self.layered_frames.remove(frame);
    }
}

/// Tracking worker state (hidden from UI).
pub struct TrackingState {
    decoder: DecoderPlugin,
    estimator_states: Vec<Option<EstimatorState>>,
    frames_to_load: Sender<Arc<Mutex<FrameState>>>,
    motion_vectors: Vec<MotionEntry>,
    frame: Vec<RGBA>,
    frame_height: usize,
    frames: usize,
}

/// UI side of the worker.
pub struct TrackingWorker {
    pub worker: AppWorker<TrackingState>,
    pub frames_to_load: Receiver<Arc<Mutex<FrameState>>>,
}

impl TrackingState {
    fn new(decoder: DecoderPlugin, frames_to_load: Sender<Arc<Mutex<FrameState>>>) -> Self {
        Self {
            decoder,
            estimator_states: vec![],
            frames_to_load,
            motion_vectors: vec![],
            frame: vec![],
            frame_height: 0,
            frames: 0,
        }
    }

    pub fn worker(decoder: DecoderPlugin, settings: TrackingSettings) -> TrackingWorker {
        let (tx, frames_to_load) = mpsc::channel();
        TrackingWorker {
            worker: AppWorker::new(Self::new(decoder, tx), settings, TrackingState::update),
            frames_to_load,
        }
    }

    fn update(&mut self, out: &mut TrackingOutput, settings: TrackingSettings) {
        // Ensure the states are of consistent size with the settings size.
        self.estimator_states.resize(settings.settings.len(), None);

        // Reset that states to default values when needed.
        for (s, e) in self
            .estimator_states
            .iter_mut()
            .zip(settings.settings.iter().map(|(e, _, _)| e.lock()))
        {
            if let Ok(false) = e.map(|v| v.is_some()) {
                *s = None;
            } else if s.is_none() {
                *s = Some(Default::default());
            }
        }

        // Get the motion field.
        self.motion_vectors.clear();

        self.frame.clear();
        self.frame_height = 0;

        self.decoder
            .process_frame(
                &mut self.motion_vectors,
                Some((&mut self.frame, &mut self.frame_height)),
                0,
            )
            .ok();

        self.frames += 1;

        let mut mat = None;

        // Go through each estimator and execute it.
        for (estimator_state, (estimator, _, est_settings)) in self
            .estimator_states
            .iter_mut()
            .zip(settings.settings.iter())
            .filter_map(|(s, settings)| s.as_mut().zip(Some(settings)))
        {
            if let Ok(Some(estimator)) = estimator.lock().as_deref_mut() {
                if let Ok((rot, tr)) =
                    estimator.estimate(&self.motion_vectors, &settings.camera, None)
                {
                    if !est_settings.layer_frames {
                        estimator_state.layered_frames.clear();
                    }

                    let (pos, rot) = estimator_state.apply_pose(tr, rot);
                    let mat = if estimator_state.layer_frame(pos, rot, est_settings) {
                        if mat.is_none() && self.frame_height > 0 {
                            mat = Some(Arc::new(Mutex::new(FrameState::Pending(
                                self.frame.clone(),
                                self.frame_height,
                            ))));
                        }

                        mat.as_ref()
                    } else {
                        None
                    };

                    let mut cnt = 0;
                    while estimator_state.layered_frames.len() > est_settings.keep_frames
                        && cnt < 20
                    {
                        estimator_state.remove_least_significant_frame();
                        cnt += 1;
                    }

                    estimator_state.push_pose(pos, rot, mat.cloned());
                }
            }
        }

        // Send a frame for UI to load as a texture.
        if let Some(mat) = mat {
            self.frames_to_load.send(mat).unwrap();
        }

        out.estimators = self.estimator_states.clone();
    }
}

impl Workable for TrackingState {
    type Output = TrackingOutput;
    type Settings = TrackingSettings;
}

#[derive(Default)]
pub struct TrackingOutput {
    pub estimators: Vec<Option<EstimatorState>>,
}

#[derive(Clone)]
pub struct TrackingSettings {
    pub settings: Vec<(
        Arc<Mutex<Option<EstimatorPlugin>>>,
        Arc<AtomicBool>,
        EstimatorSettings,
    )>,
    pub camera: StandardCamera,
}

impl Default for TrackingSettings {
    fn default() -> Self {
        Self {
            settings: vec![],
            camera: StandardCamera::new(39.6, 39.6 * 9.0 / 16.0),
        }
    }
}
