//! Common `Decoder` instance loader.

use ofps::prelude::v1::*;

use nalgebra as na;
use std::io::{BufReader, Read};
use std::net::{TcpListener, TcpStream};

fn open_file(input: &str) -> Result<Box<dyn Read>> {
    if input.starts_with("tcp://") {
        let input = input.strip_prefix("tcp://").expect("Cannot strip prefix");
        let (addr, port) = input
            .split_once(":")
            .ok_or_else(|| anyhow!("Invalid format"))?;
        let port: usize = str::parse(port)?;

        let stream = if addr == "@" {
            let listener = TcpListener::bind(format!("0.0.0.0:{}", port))?;
            let (sock, addr) = listener.accept()?;
            println!("Accept {}", addr);
            sock
        } else {
            println!("Connecting to {}", input);
            TcpStream::connect(input)?
        };

        println!("Got stream!");

        Ok(Box::new(stream))
    } else {
        std::fs::File::open(input)
            .map(|i| Box::new(i) as _)
            .map_err(Into::into)
    }
}

/// Create a decoder depending on the input.
///
/// If the input ends with `.mvec`, it will be interpreted as a motion vector file.
///
/// In MPEG mode, `tcp://` will be interpreted as a TCP network stream rather than a regular file.
pub fn create_decoder(input: &str) -> Result<Box<dyn Decoder>> {
    let max_size = (150, 150);

    if input.ends_with(".mvec") {
        let reader = open_file(input)?;
        let reader = BufReader::new(reader);

        let decoder = MvecFile { reader };

        return Ok(Box::new(decoder));
    }

    #[cfg(feature = "mvec-av")]
    let c = {
        let f = open_file(input)?;

        // Mul width by 2, because of character width
        let mut c = mvec_av::AvDecoder::try_new(f, (1, 1), max_size)?;

        c.av_ctx.dump_format();

        c
    };

    #[cfg(feature = "mvec-cv")]
    let c = mvec_cv::CvDecoder::try_new(input, (1, 1), max_size)?;

    Ok(Box::new(c))
}

struct MvecFile<T> {
    reader: T,
}

impl<T: Read> Decoder for MvecFile<T> {
    /// Process a single frame in the stream.
    ///
    /// This function will take in a single frame, and attempt extracting motion
    /// vectors from it. If the frame contains MVs, `Ok(true)` is returned, and `field`
    /// gets updated. If there are no motion vectors, `Ok(false)` is returned, and if
    /// there is an error while processing, `Err` is returned.
    ///
    /// If the decoder supports it, `out_frame` may also be written at either of the `Ok` cases.
    fn process_frame(
        &mut self,
        field: &mut MotionVectors,
        out_frame: Option<(&mut Vec<RGBA>, &mut usize)>,
        skip_frames: usize,
    ) -> Result<bool> {
        let mut vecs = [0u8; std::mem::size_of::<u32>()];
        self.reader.read_exact(&mut vecs)?;
        for _ in 0..u32::from_le_bytes(vecs) {
            let mut data = [[0u8; std::mem::size_of::<f32>()]; 4];
            for b in &mut data {
                self.reader.read_exact(&mut *b)?;
            }
            field.push((
                na::Point2::new(f32::from_le_bytes(data[0]), f32::from_le_bytes(data[1])),
                na::Vector2::new(f32::from_le_bytes(data[2]), f32::from_le_bytes(data[3])),
            ))
        }
        Ok(true)
    }

    /// Get the framerate of the stream.
    ///
    /// This will return `Some(framerate)` if it is known. On realtime streams it may
    /// not always be known. In such cases, `None` is returned.
    fn get_framerate(&self) -> Option<f64> {
        None
    }

    /// Get aspect ratio of the stream.
    ///
    /// This will return `Some((width, height))` if the aspect ratio is known. On rare
    /// cases it may not be known, which will then return `None`. Aspect ratio may change
    /// after the first frame is processed.
    fn get_aspect(&self) -> Option<(usize, usize)> {
        None
    }
}
