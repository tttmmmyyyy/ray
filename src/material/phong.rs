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
    diffuse_texture: Arc<Texture>,
    k_diffuse: f32,
    k_specular: f32,
    exponent: i32,
    specular_normalizer: f32,
    specular_importance_weight: f32,
}

impl Phong {
    pub fn new(
        diffuse_texture: Arc<Texture>,
        k_diffuse: f32,
        k_specular: f32,
        exponent: i32,
        specular_importance_weight: f32,
    ) -> Self {
        let specular_normalizer = (exponent as f32 + 2.0) / (2.0 * PI);
        Self {
            diffuse_texture,
            k_diffuse,
            k_specular,
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
            pdf: SingularPdf::Finite { pdf: Box::new(pdf) },
        })
    }
    fn emitted(&self, _ray_in: &Ray, _rec: &HitRecord) -> Vec3 {
        Vec3::new(0.0, 0.0, 0.0)
    }
    fn brdf(&self, in_ray: &Vec3, out_ray: &Vec3, rec: &HitRecord, in_light: &Vec3) -> Vec3 {
        let diffuse = (1.0 / PI)
            * self.k_diffuse
            * self
                .diffuse_texture
                .value(&rec.tex_coord, &rec.point)
                .component_mul(in_light);
        let specular = self.specular_normalizer
            * self.k_specular
            * reflect(in_ray, &rec.normal)
                .normalize()
                .dot(&out_ray.normalize())
                .powf(self.exponent as f32)
            * in_light;
        diffuse + specular
    }
}
