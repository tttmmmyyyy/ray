use crate::aliases::{RandGen, Vec3};
use crate::onb::Onb;
use crate::pdf::random_cosine_direction;
use crate::pdf::Pdf;
use std::f64::consts::PI;

/// Pdf such that pdf(d)/sin(t) ~= max(cos(t), 0) in a specified local coordinate (u,v,w)
pub struct CosinePdf(Onb);

impl CosinePdf {
    pub fn new(w: &Vec3) -> Self {
        CosinePdf {
            0: Onb::build_from_w(w),
        }
    }
    pub fn boxed(w: &Vec3) -> Box<Self> {
        Box::new(Self::new(w))
    }
}

impl Pdf for CosinePdf {
    fn density(&self, dir: &Vec3) -> f64 {
        let cosine = dir.normalize().dot(self.0.w());
        (cosine / PI).max(0.0) // since integral of cos(theta) on a unit hemisphere = pi.
    }
    fn generate(&self, rng: &mut RandGen) -> Vec3 {
        self.0.local_to_global_vec(&random_cosine_direction(rng))
    }
}
