use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, LockResult, Mutex, MutexGuard, RwLock,
};
use std::thread::{spawn, JoinHandle};

pub trait Workable: Send + 'static {
    type Output: Send + Sync + 'static + Default;
    type Settings: Send + Clone + 'static;
}

pub struct AppWorker<T: Workable> {
    output: Arc<RwLock<T::Output>>,
    settings: Arc<Mutex<T::Settings>>,
    signal: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl<T: Workable> core::ops::Deref for AppWorker<T> {
    type Target = RwLock<T::Output>;

    fn deref(&self) -> &Self::Target {
        &self.output
    }
}

impl<T: Workable> AppWorker<T> {
    pub fn settings(&self) -> LockResult<MutexGuard<T::Settings>> {
        self.settings.lock()
    }

    pub fn new(
        mut state: T,
        settings: T::Settings,
        mut loop_fn: impl FnMut(&mut T, &mut T::Output, T::Settings) + Send + 'static,
    ) -> Self {
        let output = Arc::new(RwLock::new(Default::default()));
        let settings = Arc::new(Mutex::new(settings));
        let signal = Arc::new(AtomicBool::new(true));

        let handle = Some({
            let output = output.clone();
            let signal = signal.clone();
            let settings = settings.clone();
            let mut tmp_out = Default::default();
            spawn(move || loop {
                if !signal.load(Ordering::Relaxed) {
                    break;
                } else {
                    let settings = (*settings.lock().unwrap()).clone();
                    loop_fn(&mut state, &mut tmp_out, settings);
                    if let Ok(mut output) = output.write() {
                        std::mem::swap(&mut *output, &mut tmp_out);
                    }
                }
            })
        });

        Self {
            output,
            settings,
            signal,
            handle,
        }
    }
}

impl<T: Workable> Drop for AppWorker<T> {
    fn drop(&mut self) {
        self.signal.store(false, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}
