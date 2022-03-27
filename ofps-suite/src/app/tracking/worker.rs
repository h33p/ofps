use super::super::utils::{
    properties::transfer_props,
    timer::Timer,
    worker::{AppWorker, Workable},
};
use nalgebra as na;
use ofps::prelude::v1::*;
use once_cell::sync::OnceCell;
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc::{self, Receiver, Sender, SyncSender},
    Arc, Mutex,
};
use std::thread::{spawn, JoinHandle};
use std::time::{Duration, Instant};
use wimrend::material::Material;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EstimatorSettings {
    pub scale_factor: f32,
    pub camera_offset: f32,
    pub layer_frames: bool,
    pub layer_angle_delta: f32,
    pub keep_frames: usize,
    #[serde(skip)]
    pub clear_count: usize,
    #[serde(default)]
    pub properties: BTreeMap<String, Property>,
}

impl Default for EstimatorSettings {
    fn default() -> Self {
        Self {
            scale_factor: 1.0,
            camera_offset: 1.0,
            layer_frames: true,
            layer_angle_delta: 0.5,
            keep_frames: 100,
            clear_count: 0,
            properties: Default::default(),
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
    pub transforms: Vec<(na::Vector3<f32>, na::UnitQuaternion<f32>)>,
    pub times: Vec<Duration>,
    pub layered_frames: Vec<(usize, Arc<Mutex<FrameState>>)>,
    pub clear_count: usize,
    pub properties: BTreeMap<String, Property>,
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
        tr: na::Vector3<f32>,
        frot: na::UnitQuaternion<f32>,
        frame: Option<Arc<Mutex<FrameState>>>,
        time: Duration,
    ) {
        let idx = self.poses.len();

        self.poses.push((pos, rot));
        self.transforms.push((tr, frot));
        self.times.push(time);

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
        let frame = if self.layered_frames.len() <= 2 {
            0
        } else {
            self.layered_frames
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
                .unwrap_or_default()
                .0
        };

        self.layered_frames.remove(frame);
    }
}

struct DecoderResult {
    frame: Result<(MotionVectors, Vec<RGBA>, usize)>,
    time: Duration,
    props: BTreeMap<String, Property>,
}

#[derive(Default)]
struct DecoderSettings {
    props: BTreeMap<String, Property>,
    realtime_processing: bool,
}

fn decoder_job(
    mut decoder: DecoderPlugin,
    flag: Arc<AtomicBool>,
    sender: SyncSender<DecoderResult>,
    settings: Arc<Mutex<DecoderSettings>>,
) {
    let mut motion_vectors = vec![];
    let mut motion_vectors_2 = vec![];
    let mut frame = vec![];
    let mut frame_2 = vec![];
    let mut frame_height = 0;
    let mut frame_height_2;
    let mut decoder_timer = None;

    while flag.load(Ordering::Relaxed) {
        let timer = Instant::now();

        let mut props = Default::default();

        if let Ok(settings) = settings.lock() {
            transfer_props(decoder.props_mut(), &settings.props, &mut props);

            Timer::handle_option(
                &mut decoder_timer,
                settings.realtime_processing,
                decoder
                    .get_framerate()
                    .filter(|f| *f > 0.0)
                    .map(|f| Duration::from_secs_f64(1.0 / f)),
            );
        }

        // Get the motion field.
        motion_vectors_2.clear();

        frame_2.clear();
        frame_height_2 = 0;

        let frame = match decoder.process_frame(
            &mut motion_vectors_2,
            Some((&mut frame_2, &mut frame_height_2)),
            0,
        ) {
            Ok(s) => {
                if s {
                    std::mem::swap(&mut motion_vectors, &mut motion_vectors_2);
                    std::mem::swap(&mut frame, &mut frame_2);
                    std::mem::swap(&mut frame_height, &mut frame_height_2);
                }

                Ok((motion_vectors.clone(), frame.clone(), frame_height))
            }
            Err(e) => Err(e),
        };

        let time = timer.elapsed();

        if sender.send(DecoderResult { frame, time, props }).is_err() {
            break;
        }
    }
}

struct DecoderJob {
    flag: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
    results: Receiver<DecoderResult>,
    settings: Arc<Mutex<DecoderSettings>>,
}

impl Drop for DecoderJob {
    fn drop(&mut self) {
        self.flag.store(false, Ordering::Relaxed);

        if let Some(handle) = self.handle.take() {
            handle.join().unwrap();
        }
    }
}

impl From<DecoderPlugin> for DecoderJob {
    fn from(decoder: DecoderPlugin) -> Self {
        let flag = Arc::new(AtomicBool::new(true));

        let (sender, results) = mpsc::sync_channel(0);
        let settings = Arc::new(Mutex::new(Default::default()));

        let handle = Some({
            let flag = flag.clone();
            let settings = settings.clone();
            spawn(move || decoder_job(decoder, flag, sender, settings))
        });

        Self {
            flag,
            handle,
            results,
            settings,
        }
    }
}

/// Tracking worker state (hidden from UI).
pub struct TrackingState {
    decoder: DecoderJob,
    decoder_times: Vec<Duration>,
    estimator_states: Vec<Option<EstimatorState>>,
    frames_to_load: Sender<Arc<Mutex<FrameState>>>,
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
            decoder: decoder.into(),
            decoder_times: vec![],
            estimator_states: vec![],
            frames_to_load,
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

    fn update(&mut self, out: &mut TrackingOutput, settings: TrackingSettings) -> bool {
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

        if let Ok(mut dec_settings) = self.decoder.settings.lock() {
            dec_settings.props = settings.decoder_properties;
            dec_settings.realtime_processing = settings.realtime_processing;
        }

        let (motion_vectors, frame, frame_height, time) = match self.decoder.results.recv() {
            Ok(DecoderResult { frame, time, props }) => {
                out.decoder_properties = props;
                match frame {
                    Ok((mv, f, fh)) => (mv, f, fh, time),
                    Err(_) => return false,
                }
            }
            Err(_) => return false,
        };

        self.decoder_times.push(time);
        self.frames += 1;

        let mut mat = OnceCell::new();
        let camera = &settings.camera;

        // Go through each estimator and execute it.
        self.estimator_states
            .iter_mut()
            .zip(settings.settings.iter())
            .filter_map(|(s, settings)| s.as_mut().zip(Some(settings)))
            .par_bridge()
            .for_each(|(estimator_state, (estimator, _, est_settings))| {
                if let Ok(Some(estimator)) = estimator.lock().as_deref_mut() {
                    transfer_props(
                        estimator.props_mut(),
                        &est_settings.properties,
                        &mut estimator_state.properties,
                    );

                    let timer = Instant::now();
                    if let Ok((frot, tr)) = estimator.estimate(&motion_vectors, camera, None) {
                        if estimator_state.clear_count != est_settings.clear_count {
                            estimator_state.layered_frames.clear();
                            estimator_state.clear_count = est_settings.clear_count;
                        }

                        let (pos, rot) = estimator_state.apply_pose(tr, frot);
                        let mat = if estimator_state.layer_frame(pos, rot, est_settings) {
                            if frame_height > 0 {
                                mat.get_or_init(|| {
                                    Arc::new(Mutex::new(FrameState::Pending(
                                        frame.clone(),
                                        frame_height,
                                    )))
                                });
                            }

                            mat.get()
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

                        estimator_state.push_pose(
                            pos,
                            rot,
                            tr,
                            frot,
                            mat.cloned(),
                            timer.elapsed(),
                        );
                    }
                }
            });

        // Send a frame for UI to load as a texture.
        if let Some(mat) = mat.take() {
            self.frames_to_load.send(mat).unwrap();
        }

        out.estimators = self.estimator_states.clone();
        out.decoder_times = self.decoder_times.clone();

        true
    }
}

impl Workable for TrackingState {
    type Output = TrackingOutput;
    type Settings = TrackingSettings;
}

#[derive(Default)]
pub struct TrackingOutput {
    pub estimators: Vec<Option<EstimatorState>>,
    pub decoder_times: Vec<Duration>,
    pub decoder_properties: BTreeMap<String, Property>,
}

#[derive(Clone)]
pub struct TrackingSettings {
    pub settings: Vec<(
        Arc<Mutex<Option<EstimatorPlugin>>>,
        Arc<AtomicBool>,
        EstimatorSettings,
    )>,
    pub camera: StandardCamera,
    pub realtime_processing: bool,
    pub decoder_properties: BTreeMap<String, Property>,
}

impl Default for TrackingSettings {
    fn default() -> Self {
        Self {
            settings: vec![],
            camera: StandardCamera::new(39.6, 39.6 * 9.0 / 16.0),
            realtime_processing: false,
            decoder_properties: Default::default(),
        }
    }
}
