pub mod diffuse_light;
pub mod glass;
pub mod lambertian;
pub mod metal;

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
    /// A characteristic function of scattering process.
    /// BRDF * cos(angle between scattered and rec.normal).
    // fn scattering_pdf(&self, ray: &Ray, scattered: &Ray, rec: &HitRecord) -> f32;
    /// ToDo: write a comment.
    fn brdf(&self, ray: &Ray, scattered: &Ray, rec: &HitRecord, in_light: &Vec3) -> Vec3;
}
