use motion_vectors::prelude::v1::*;
use mvec_av::*;
use std::fs::File;
use std::time::{Duration, Instant};
use terminal_size::{terminal_size, Height, Width};

const CHAR_MAP: &str = "$@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/\\|()1{}[]?-_+~<>i!lI;:,\"^`'. ";

fn motion_to_char(magnitude: f32) -> char {
    let max_range = 0.2f32;
    let range = 0f32.max(magnitude.min(max_range)) / max_range;
    let idx = (1f32 - range) * (CHAR_MAP.len() - 1) as f32;
    CHAR_MAP.chars().nth(idx.round() as usize).unwrap()
}

fn main() -> Result<()> {
    let input = std::env::args()
        .nth(1)
        .ok_or("Please supply a video file!")?;

    let f = File::open(&input)?;

    let max_size = if let Some((Width(w), Height(h))) = terminal_size() {
        (w as usize, h as usize)
    } else {
        (30, 30)
    };

    // Mul width by 2, because of character width
    let mut c = AvDecoder::try_new(f.into(), (2, 1), max_size)?;

    c.av_ctx.dump_format();

    let mut cnt = 0;

    let mut mf = MotionField::new(0, 0);

    let rate = c.get_framerate();

    println!("FRAMERATE: {}", rate);

    let interval = 1f64 / rate;

    std::thread::sleep(Duration::from_secs(1));

    let mut child = std::process::Command::new("ffplay")
        .args([&input])
        .spawn()?;

    std::thread::sleep(Duration::from_millis(30));

    let start = Instant::now();

    loop {
        let target_time = Duration::from_secs_f64(interval * cnt as f64);
        let curtime = start.elapsed();

        if curtime < target_time {
            std::thread::sleep(target_time - curtime);
        }

        match c.process_frame(&mut mf) {
            Err(e) => {
                println!("{}", e);
                break;
            }
            Ok(true) => {
                let (w, h) = mf.dim();
                for y in 0..h {
                    for x in 0..w {
                        print!("{}", motion_to_char(mf.get_motion(x, y).magnitude()));
                    }
                    println!()
                }
            }
            _ => {}
        }

        cnt += 1;
    }

    child.kill().ok();
    child.wait()?;

    Ok(())
}
