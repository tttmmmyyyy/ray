use crate::aliases::{RandGen, Vec3};
use crate::hit_record::HitRecord;
use crate::material::Material;
use crate::pdf::cosine::CosinePdf;
use crate::pdf::SingularPdf;
use crate::ray::Ray;
use crate::scatter_record::ScatterRecord;
use crate::texture::Texture;
use std::f32::consts::PI;
use std::sync::Arc;

// pub struct Lambertian {
//     pub albedo: Arc<Texture>,
// }

// impl Lambertian {
//     pub fn new(texture: Arc<Texture>) -> Self {
//         Lambertian { albedo: texture }
//     }
// }

/// Lambert-Blinn-Phong reflection model
pub struct LBP {
    diffuse_coef: Arc<Texture>,
    specular_coef: Arc<Texture>,
    exponent: i32,
    specular_normalizer: f32,
}

impl LBP {
    pub fn new(diffuse_coef: Arc<Texture>, specular_coef: Arc<Texture>, exponent: i32) -> Self {
        let specular_normalizer = (exponent as f32 + 2.0) * (exponent as f32 + 4.0)
            / (8.0 * PI * (2.0f32.powf(-0.5 * exponent as f32) + exponent as f32));
        Self {
            diffuse_coef,
            specular_coef,
            exponent,
            specular_normalizer,
        }
    }
}

impl Material for LBP {
    fn scatter(&self, ray: &Ray, rec: &HitRecord, rng: &mut RandGen) -> Option<ScatterRecord> {
        Some(ScatterRecord {
            attenuation: Vec3::new(0.0, 0.0, 0.0), // ToDo: remove attenuation field from ScatterRecord
            important_dir: SingularPdf::Finite {
                pdf: CosinePdf::boxed(&rec.normal),
            },
            // ToDo: improve importace sampling, considering specular part.
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
        let half_vec =
            ((-ray.direction.normalize() + scattered.direction.normalize()) / 2.0).normalize();
        let specular = self.specular_normalizer
            * half_vec.dot(&rec.normal).powi(self.exponent)
            * self
                .specular_coef
                .value(&rec.tex_coord, &rec.point)
                .component_mul(in_light);
        diffuse + specular
    }
}