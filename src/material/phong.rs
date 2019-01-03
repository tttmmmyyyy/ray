use crate::aliases::{RandGen, Vec3};
use crate::hit_record::HitRecord;
use crate::material::glass::reflect;
use crate::material::Material;
use crate::pdf::cosine::CosineNPdf;
use crate::pdf::cosine::CosinePdf;
use crate::pdf::mixture::MixturePdfBox;
use crate::pdf::SingularPdf;
use crate::ray::Ray;
use crate::scatter_record::ScatterRecord;
use crate::texture::Texture;
use std::f32::consts::PI;
use std::sync::Arc;

/// Lambert-Phong reflection model
pub struct Phong {
    diffuse_coef: Arc<Texture>,
    specular_coef: Arc<Texture>,
    exponent: i32,
    specular_normalizer: f32,
    specular_importance_weight: f32,
}

impl Phong {
    pub fn new(
        diffuse_coef: Arc<Texture>,
        specular_coef: Arc<Texture>,
        exponent: i32,
        specular_importance_weight: f32,
    ) -> Self {
        let specular_normalizer = (exponent as f32 + 2.0) / (2.0 * PI);
        Self {
            diffuse_coef,
            specular_coef,
            exponent,
            specular_normalizer,
            specular_importance_weight,
        }
    }
}

impl Material for Phong {
    fn scatter(&self, ray: &Ray, rec: &HitRecord, _rng: &mut RandGen) -> Option<ScatterRecord> {
        let pdf = MixturePdfBox {
            mix: self.specular_importance_weight,
            a_pdf: Box::new(CosineNPdf::new(
                &reflect(&ray.direction, &rec.normal),
                self.exponent,
            )),
            b_pdf: Box::new(CosinePdf::new(&rec.normal)),
        };
        Some(ScatterRecord {
            attenuation: Vec3::new(0.0, 0.0, 0.0), // ToDo: remove attenuation field from ScatterRecord
            important_dir: SingularPdf::Finite { pdf: Box::new(pdf) },
        })
    }
    fn emitted(&self, _ray_in: &Ray, _rec: &HitRecord) -> Vec3 {
        Vec3::new(0.0, 0.0, 0.0)
    }
    fn brdf(&self, ray: &Ray, scattered: &Ray, rec: &HitRecord, in_light: &Vec3) -> Vec3 {
        let diffuse = (1.0 / PI)
            * self
                .diffuse_coef
                .value(&rec.tex_coord, &rec.point)
                .component_mul(in_light);
        let specular = self.specular_normalizer
            * reflect(&ray.direction, &rec.normal)
                .normalize()
                .dot(&scattered.direction.normalize())
                .powf(self.exponent as f32)
            * self
                .specular_coef
                .value(&rec.tex_coord, &rec.point)
                .component_mul(in_light);
        diffuse + specular
    }
}
