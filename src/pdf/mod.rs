pub mod constant;
pub mod cosine;
pub mod hitable;
pub mod mixture;

use crate::aliases::{RandGen, Vec2, Vec3};
use rand::Rng;
use std::f32::consts::PI;

/// A probability distribution function on directions which can be a delta distribution.
pub enum SingularPdf {
    Delta { dir: Vec3 },      // delta distribution
    Finite { pdf: Box<Pdf> }, // finite distribution
}

/// Probability distribution function on directions
pub trait Pdf {
    /// Probability density at a direction.
    /// dir is not required to be normalized
    fn density(&self, dir: &Vec3) -> f32;
    /// Generates a radom direction following this pdf.
    fn generate(&self, rng: &mut RandGen) -> Vec3;
}

// ToDo: (a,b,c) -> a^{1/3}(2b-1, \sqrt{1-(2b-1)^2}\cos(2PIc), \sqrt{1-(2b-1)^2}\sin(2PIc))
// or use random_in_cone()
pub fn rnd_in_unit_sphere(rng: &mut RandGen) -> Vec3 {
    loop {
        let p = Vec3::new(
            2.0 * rng.gen::<f32>() - 1.0,
            2.0 * rng.gen::<f32>() - 1.0,
            2.0 * rng.gen::<f32>() - 1.0,
        );
        if p.norm() < 1.0 {
            return p;
        }
    }
}

// ToDo: (a,b) -> (\sqrt{a}\cos(2PIb), \sqrt{a}\sin(2PIb))
pub fn rnd_in_unit_disc(rnd_in_unit_disc: &mut RandGen) -> Vec2 {
    loop {
        let p = Vec2::new(
            2.0 * rnd_in_unit_disc.gen::<f32>() - 1.0,
            2.0 * rnd_in_unit_disc.gen::<f32>() - 1.0,
        );
        if p.norm() < 1.0 {
            return p;
        }
    }
}

/// Calculates a random point on a unit hemisphere (x^2+y^2+z^2=1, z>=0)
/// such that pdf(d)/sin(t) ~= cos(t) where t is angle between d and n=(0,0,1)
pub fn random_cosine_direction(rng: &mut RandGen) -> Vec3 {
    let r0 = rng.gen::<f32>();
    let rr0 = r0.sqrt();
    let r1 = rng.gen::<f32>();
    let angle = 2.0 * PI * r1;
    let z = (1.0 - r0).sqrt();
    let x = angle.cos() * rr0;
    let y = angle.sin() * rr0;
    Vec3::new(x, y, z)
}

/// Generate uniformly a random (normalized) direction vector in a cone
pub fn random_in_cone(cos_half_angle: f32, rng: &mut RandGen) -> Vec3 {
    let r1 = rng.gen::<f32>();
    let r2 = rng.gen::<f32>();
    let z = 1.0 + r1 * (cos_half_angle - 1.0);
    let sine = (1.0 - z.powf(2.0)).sqrt();
    let phi = 2.0 * PI * r2;
    let x = phi.cos() * sine;
    let y = phi.sin() * sine;
    Vec3::new(x, y, z)
}
