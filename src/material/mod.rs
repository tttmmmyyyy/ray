pub mod diffuse_light;
pub mod glass;
pub mod lambertian;
pub mod lbp;
pub mod metal;
pub mod phong;

use crate::aliases::{RandGen, Vec3};
use crate::hit_record::HitRecord;
use crate::ray::Ray;
use crate::scatter_record::ScatterRecord;

pub trait Material: Send + Sync {
    /// Calculates informations to determine the next ray or
    /// None when this Material does not scatter rays.
    fn scatter(&self, ray: &Ray, rec: &HitRecord, rng: &mut RandGen) -> Option<ScatterRecord>;
    /// Calculates the emitted light from HitRecord.
    fn emitted(&self, _ray_in: &Ray, _rec: &HitRecord) -> Vec3 {
        Vec3::new(0.0, 0.0, 0.0)
    }
    /// RGB component-wise BRDF function
    /// * `in_ray` - the direction (not normalized) of the incoming ray carrying outgoing light.
    /// * `out_ray` - the direction (not normalized) of the outgoing ray carrying incoming light.
    fn brdf(&self, in_ray: &Vec3, out_ray: &Vec3, rec: &HitRecord, in_light: &Vec3) -> Vec3;
}
