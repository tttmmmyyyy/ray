use crate::aliases::RandGen;
use crate::aliases::Vec3;
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
            pdf: SingularPdf::Finite {
                pdf: Box::new(CosinePdf::new(&rec.normal)),
            },
        })
    }
    fn brdf(&self, _in_ray: &Vec3, _out_ray: &Vec3, rec: &HitRecord, in_light: &Vec3) -> Vec3 {
        ((1.0 / PI) * self.albedo.value(&rec.tex_coord, &rec.point)).component_mul(in_light)
    }
}
