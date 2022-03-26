use std::time::{Duration, Instant};

pub struct Timer {
    start: Instant,
    target: Duration,
}

impl Default for Timer {
    fn default() -> Self {
        Self {
            start: Instant::now(),
            target: Default::default(),
        }
    }
}

impl Timer {
    pub fn sleep(&self) {
        if let Some(duration) = self.target.checked_sub(self.start.elapsed()) {
            std::thread::sleep(duration);
        }
    }

    pub fn add(&mut self, duration: Duration) {
        self.target += duration;
    }

    pub fn handle_option(timer: &mut Option<Self>, enable: bool, add_time: Option<Duration>) {
        if enable {
            if timer.is_none() {
                *timer = Some(Default::default());
            }

            let decoder_timer = timer.as_mut().unwrap();

            decoder_timer.sleep();

            if let Some(add_time) = add_time {
                decoder_timer.add(add_time);
            }
        } else {
            *timer = None;
        }
    }
}
