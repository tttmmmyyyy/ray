use crate::affine::Affine;
use crate::aliases::Vec3;

#[derive(Clone, Copy)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
    pub time: f32, // time at which ray is generated (utilized for motion blurring)
}

impl Ray {
    pub fn new(origin: &Vec3, direction: &Vec3, time: f32) -> Self {
        Ray {
            origin: *origin,
            direction: *direction,
            time: time,
        }
    }
    pub fn evaluate(&self, t: f32) -> Vec3 {
        self.origin + t * self.direction
    }
    pub fn get_transformed(&self, tr: &Affine) -> Ray {
        Ray::new(
            &tr.act_point(&self.origin),
            &tr.act_vec(&self.direction),
            self.time,
        )
    }
}
