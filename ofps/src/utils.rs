//! # Utility module

use anyhow::{anyhow, Result};
use nalgebra as na;
use std::io::Read;
use std::net::{TcpListener, TcpStream};

pub trait AsMutPtr {
    type Mut;

    fn as_mut_ptr(&mut self) -> &mut *mut Self::Mut;
}

impl<T: AsMutPtr> AsMutPtr for Option<T> {
    type Mut = T::Mut;

    fn as_mut_ptr(&mut self) -> &mut *mut Self::Mut {
        unsafe { std::mem::transmute(self) }
    }
}

impl<T> AsMutPtr for &mut T {
    type Mut = T;

    fn as_mut_ptr(&mut self) -> &mut *mut Self::Mut {
        unsafe { std::mem::transmute(self) }
    }
}

impl<T: AsMutPtr> AsMutPtr for *mut T {
    type Mut = T::Mut;

    fn as_mut_ptr(&mut self) -> &mut *mut Self::Mut {
        unsafe { std::mem::transmute(self) }
    }
}

/// Triangulate scale factor for `bc`, so that it is consistent with `ab`.
///
/// This will compute the closest solution. If there isn't any (lines are parallel), 1 is returned.
///
/// # Arguments
///
/// * `ab` - ground truth vector to be scaled against.
/// * `bc` - vector going from ab to the end point. Its scale factor is computed.
/// * `ac` - vector going directly from origin to the endpoint.
pub fn triangulate_scale(ab: na::Vector3<f32>, bc: na::Vector3<f32>, ac: na::Vector3<f32>) -> f32 {
    // P_bc = ab
    // Q_bc = P_bc + t_bc * bc
    //
    // P_bc + t_bc * bc + t_cross * cross = P_ac + t_ac * ac
    //
    // Solve the system of equations to get t_bc
    //
    // Px_bc + t_bc * bcx + t_cross * crossx = Px_ac + t_ac * acx
    // Py_bc + t_bc * bcy + t_cross * crossy = Py_ac + t_ac * acy
    // Pz_bc + t_bc * bcz + t_cross * crossz = Pz_ac + t_ac * acz
    //
    // Px_bc + t_bc * bcx + t_cross * crossx - Px_ac = t_ac * acx /
    // Py_bc + t_bc * bcy + t_cross * crossy - Py_ac = t_ac * acy =
    // (Px_bc + t_bc * bcx + t_cross * crossx - Px_ac) / (Py_bc + t_bc * bcy + t_cross * crossy - Py_ac) = acx / acy
    //
    // Px_bc + t_bc * bcx + t_cross * crossx - Px_ac = (acx / acy) * (Py_bc + t_bc * bcy + t_cross * crossy - Py_ac)
    // Pz_bc + t_bc * bcz + t_cross * crossz = Pz_ac + t_ac * acz
    //
    // t_cross * (crossx - crossy * (acx / acy)) = Px_ac + (acx / acy) * (Py_bc + t_bc * bcy - Py_ac) /
    // t_cross * crossz = Pz_ac + t_ac * acz - Pz_bc + t_bc * bcz =
    // (crossx - crossy * (acx / acy)) / crossz = (Px_ac + (acx / acy) * (Py_bc + t_bc * bcy - Py_ac)) / (Pz_ac + t_ac * acz - Pz_bc + t_bc * bcz)
    //
    // (crossx - crossy * (acx / acy)) / crossz * (Pz_ac + t_ac * acz - Pz_bc + t_bc * bcz) = (Px_ac + (acx / acy) * (Py_bc + t_bc * bcy - Py_ac))
    //
    // (crossx - crossy * (acx / acy)) / crossz * t_bc * bcz - (acx / acy) * t_bc * bcy = (Px_ac + (acx / acy) * (Py_bc - Py_ac)) - ((crossx - crossy * (acx / acy)) / crossz) * (Pz_ac + t_ac * acz - Pz_bc)
    //
    // t_bc * ((crossx - crossy * (acx / acy)) / crossz * bcz - (acx / acy) * bcy) = (Px_ac + (acx / acy) * (Py_bc - Py_ac)) - ((crossx - crossy * (acx / acy)) / crossz) * (Pz_ac + t_ac * acz - Pz_bc)
    //
    // t_bc = ((Px_ac + (acx / acy) * (Py_bc - Py_ac)) - ((crossx - crossy * (acx / acy)) / crossz) * (Pz_ac + t_ac * acz - Pz_bc)) / ((crossx - crossy * (acx / acy)) / crossz * bcz - (acx / acy) * bcy)
    //
    // Given P_ac = (0, 0, 0), P_bc = ab
    //
    // t_bc = (((acx / acy) * aby) - ((crossx - crossy * (acx / acy)) / crossz) * (t_ac * acz - abz)) / ((crossx - crossy * (acx / acy)) / crossz * bcz - (acx / acy) * bcy)

    let cross = bc.cross(&ac);

    let lhs = na::Matrix3::from_columns(&[-bc, ac, cross]);

    let lu = lhs.lu();

    lu.solve(&ab).map(|v| v.x).unwrap_or(1.0)
}

/// Open a file or an input stream.
pub fn open_file(input: &str) -> Result<Box<dyn Read + Send>> {
    if input.starts_with("tcp://") {
        let input = input.strip_prefix("tcp://").expect("Cannot strip prefix");
        let (addr, port) = input
            .split_once(':')
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scale_triangulation_simple() {
        let triangle = [
            na::matrix![0.0; 1.0; 1.0],
            na::matrix![0.0; 1.0; -1.0],
            na::matrix![0.0; 2.0; 0.0],
        ];

        for i in 1..100 {
            for o in 1..100 {
                let s1 = i as f32 / 10.0;
                let s2 = o as f32 / 10.0;
                let s = triangulate_scale(triangle[0], triangle[1] * s1, triangle[2] * s2);
                assert!((s - 1.0 / s1).abs() <= 0.0001, "{} vs {}", s, 1.0 / s1,);
            }
        }
    }

    #[test]
    fn scale_triangulation_parallel() {
        let triangle = [
            na::matrix![0.0; 1.0; 1.0],
            na::matrix![0.0; 1.0; -1.0],
            na::matrix![0.0; 2.0; -2.0],
        ];

        for i in 1..100 {
            for o in 1..100 {
                let s1 = i as f32 / 10.0;
                let s2 = o as f32 / 10.0;
                let s = triangulate_scale(triangle[0], triangle[1] * s1, triangle[2] * s2);
                assert_eq!(s, 1.0);
            }
        }
    }
}
