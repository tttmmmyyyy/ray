use crate::aliases::{RandGen, Vec3};
use crate::hit_record::HitRecord;
use crate::material::Material;
use crate::pdf::rnd_in_unit_sphere;
use crate::pdf::SingularPdf;
use crate::ray::Ray;
use crate::scatter_record::ScatterRecord;
use rand::Rng;

/// * `n` - must be normalized
pub fn reflect(v: &Vec3, n: &Vec3) -> Vec3 {
    debug_assert!((n.norm() - 1.0).abs() < 1e-3);
    v - 2.0 * v.dot(&n) * n
}

pub fn refract(v: &Vec3, n: &Vec3, r: f32) -> Option<Vec3> {
    let uv = v.normalize();
    let dt = uv.dot(n);
    let d = 1.0 - r * r * (1.0 - dt * dt);
    if d > 0.0 {
        Some(r * (uv - n * dt) - n * f32::sqrt(d))
    } else {
        None
    }
}

pub struct Glass {
    pub ref_idx: f32,
    pub fuziness: f32,
}

impl Glass {
    pub fn new(ref_idx: f32, fuziness: f32) -> Self {
        Glass {
            ref_idx: ref_idx,
            fuziness: fuziness,
        }
    }
}

/// Approximation formula of probability of reflection when a light enter into a material
pub fn schlick_formula(cosine: f32, ref_idx: f32) -> f32 {
    let r0 = ((1.0 - ref_idx) / (1.0 + ref_idx)).powf(2.0);
    r0 + (1.0 - r0) * f32::powf(1.0 - cosine, 5.0)
}

impl Material for Glass {
    fn scatter(&self, ray: &Ray, rec: &HitRecord, rng: &mut RandGen) -> Option<ScatterRecord> {
        // r = relative refractive index
        // n = a normal vector
        // c = cosine(angle of incidence)
        let in_dir_dot_normal = ray.direction.dot(&rec.normal);
        let (r, n, c) = if in_dir_dot_normal > 0.0 {
            (
                self.ref_idx,
                -rec.normal,
                in_dir_dot_normal / ray.direction.norm(),
            )
        } else {
            (
                1.0 / self.ref_idx,
                rec.normal,
                -in_dir_dot_normal / ray.direction.norm(),
            )
        };
        let op_refracted = refract(&ray.direction, &n, r);
        if op_refracted.is_some() && rng.gen::<f32>() > schlick_formula(c, r) {
            let mut refracted = op_refracted.unwrap();
            if self.fuziness > 0.0 {
                refracted = refracted.normalize();
                refracted += self.fuziness * rnd_in_unit_sphere(rng);
            }
            Some(ScatterRecord {
                attenuation: Vec3::new(1.0, 1.0, 1.0),
                important_dir: SingularPdf::Delta { dir: refracted },
            })
        } else {
            let mut reflected = reflect(&ray.direction, &n);
            if self.fuziness > 0.0 {
                reflected = reflected.normalize();
                reflected += self.fuziness * rnd_in_unit_sphere(rng);
            }
            Some(ScatterRecord {
                attenuation: Vec3::new(1.0, 1.0, 1.0),
                important_dir: SingularPdf::Delta { dir: reflected },
            })
        }
    }
    fn brdf(&self, _ray: &Ray, _scattered: &Ray, _rec: &HitRecord, _in_light: &Vec3) -> Vec3 {
        panic!("brdf called for Glass.")
    }
}
