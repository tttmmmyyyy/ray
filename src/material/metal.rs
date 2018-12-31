use crate::aliases::{RandGen, Vec3};
use crate::hit_record::HitRecord;
use crate::material::Material;
use crate::pdf::rnd_in_unit_sphere;
use crate::pdf::SingularPdf;
use crate::ray::Ray;
use crate::scatter_record::ScatterRecord;

pub struct Metal {
    pub albedo: Vec3,
    pub fuzziness: f64,
}

impl Metal {
    pub fn new(albedo: &Vec3, fuzziness: f64) -> Self {
        Metal {
            albedo: *albedo,
            fuzziness: fuzziness,
        }
    }
}

impl Material for Metal {
    fn scatter(&self, ray: &Ray, rec: &HitRecord, rng: &mut RandGen) -> Option<ScatterRecord> {
        let dotted = ray.direction.dot(&rec.normal);
        let fuz = if self.fuzziness == 0.0 {
            Vec3::new(0.0, 0.0, 0.0)
        } else {
            self.fuzziness * rnd_in_unit_sphere(rng)
        };
        let reflected = ray.direction - 2.0 * dotted * rec.normal + fuz;
        if reflected.dot(&rec.normal) <= 0.0 {
            return None;
        }
        Some(ScatterRecord {
            attenuation: self.albedo,
            important_dir: SingularPdf::Delta { dir: reflected },
        })
    }
    fn scattering_pdf(&self, _ray: &Ray, _scattered: &Ray, _rec: &HitRecord) -> f64 {
        panic!("scattering_pdf called for Metal.")
    }
}
