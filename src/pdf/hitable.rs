use crate::aliases::{RandGen, Vec3};
use crate::hitable::Hitable;
use crate::pdf::Pdf;

pub struct HitablePdf<'a> {
    origin: Vec3,
    hitable: &'a Hitable,
}

impl<'a> HitablePdf<'a> {
    pub fn new(hitable: &'a Hitable, origin: &Vec3) -> Self {
        HitablePdf {
            hitable: hitable,
            origin: *origin,
        }
    }
}

impl<'a> Pdf for HitablePdf<'a> {
    fn density(&self, dir: &Vec3) -> f32 {
        self.hitable.direction_density(&self.origin, &dir)
    }
    fn generate(&self, rng: &mut RandGen) -> Vec3 {
        let ret = self.hitable.random_direction_from(&self.origin, rng);
        debug_assert!(ret.norm().is_finite() && ret.norm() > 0.0);
        ret
    }
}
