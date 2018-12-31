use super::Pdf;
use crate::aliases::{RandGen, Vec3};
use std::f32::consts::PI;

pub struct Constant;

impl Pdf for Constant {
    fn density(&self, _dir: &Vec3) -> f32 {
        1.0 / (4.0 * PI)
    }
    fn generate(&self, _rng: &mut RandGen) -> Vec3 {
        unimplemented!()
    }
}

impl Constant {
    pub fn new() -> Self {
        Constant {}
    }
}
