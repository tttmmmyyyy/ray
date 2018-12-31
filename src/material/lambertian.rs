use crate::aliases::RandGen;
use crate::hit_record::HitRecord;
use crate::material::Material;
use crate::pdf::cosine::CosinePdf;
use crate::pdf::SingularPdf;
use crate::ray::Ray;
use crate::scatter_record::ScatterRecord;
use crate::texture::Texture;
use std::f32::consts::PI;
use std::sync::Arc;

pub struct Lambertian {
    pub albedo: Arc<Texture>,
}

impl Lambertian {
    pub fn new(texture: Arc<Texture>) -> Self {
        Lambertian { albedo: texture }
    }
}

impl Material for Lambertian {
    fn scatter(&self, _ray: &Ray, rec: &HitRecord, _rng: &mut RandGen) -> Option<ScatterRecord> {
        Some(ScatterRecord {
            attenuation: self.albedo.value(&rec.tex_coord, &rec.point),
            important_dir: SingularPdf::Finite {
                pdf: CosinePdf::boxed(&rec.normal),
            },
        })
    }
    fn scattering_pdf(&self, _ray: &Ray, scattered: &Ray, rec: &HitRecord) -> f32 {
        let cosine = rec.normal.dot(&scattered.direction.normalize());
        (cosine / PI).max(0.0)
    }
}
