pub mod aabb;
pub mod affine;
pub mod aliases;
pub mod background;
pub mod camera;
pub mod hit_record;
pub mod hitable;
pub mod material;
pub mod obj_file;
pub mod onb;
pub mod pdf;
pub mod ray;
pub mod scatter_record;
pub mod scene;
pub mod texture;
pub mod util;

use crate::aliases::{RandGen, Vec3};
use crate::hit_record::HitRecord;
use crate::hitable::Hitable;
use crate::pdf::hitable::HitablePdf;
use crate::pdf::mixture::MixturePdf;
use crate::pdf::{Pdf, SingularPdf};
use crate::ray::Ray;
use crate::scene::Scene;

// fn mix_importance_hitable_pdf<'m, 's, 'p>(
//     material_pdf: &'m dyn Pdf,
//     weight_imp_hit_pdf: &'s Option<(f32, HitablePdf)>,
// ) -> MixturePdf<'s, 'm> {
//     match weight_imp_hit_pdf {
//         Some((weight, imp_hit_pdf)) => {
//             debug_assert!(*weight > 0.0);
//             MixturePdf::new(*weight, imp_hit_pdf, material_pdf)
//         }
//         None => MixturePdf::zero(material_pdf),
//     }
// }

// fn create_importance_hitable_pdf<'s>(
//     scene: &'s Scene,
//     hit_pt: &Vec3,
// ) -> Option<(f32, HitablePdf<'s>)> {
//     if scene.importance_weight > 0.0 {
//         Some((
//             scene.importance_weight,
//             HitablePdf::new(&(*scene.importance), hit_pt),
//         ))
//     } else {
//         None
//     }
// }

pub fn next_event_estimation(
    ray: &Ray,
    rec: &HitRecord,
    scene: &Scene,
    light_out: &mut Vec3,
    rng: &mut RandGen,
) {
    // ToDo: 改善する。現状は実験的雑実装。
    if scene.light.is_none() {
        return;
    }
    let light = &**(scene.light.as_ref().unwrap()); // ToDo: かっこいるの？
    let pdf = HitablePdf::new(light, &rec.point);
    let dir = pdf.generate(rng);
    let shadow_ray = Ray::new(&rec.point, &dir, ray.time);
    let light_hit_rec = light.hit(&shadow_ray, 0.0, std::f32::MAX);
    if light_hit_rec.is_none() {
        return;
    }
    let light_hit_rec = light_hit_rec.as_ref().unwrap();
    if scene.hitables.is_hit(
        &shadow_ray,
        0.0001,
        light_hit_rec.t - std::f32::MIN_POSITIVE,
    ) {
        return;
    }
    let cosine = rec.normal.dot(&dir.normalize());
    if cosine <= 0.0 {
        return;
    }
    let density = pdf.density(&dir);
    if density <= 0.0 {
        // Mathematically Prob(density == 0.0) is zero (but occurres sometimes),
        // and therefore just ignoring such cases to avoid Inf is harmless.
        return;
    }
    let emitted = light_hit_rec.material.emitted(&shadow_ray, &light_hit_rec);
    *light_out += (cosine / density) * rec.material.brdf(&ray.direction, &dir, rec, &emitted);
}

pub fn calc_color(
    ray: &Ray,
    scene: &Scene,
    rng: &mut RandGen,
    depth: i32,
    is_ray_diffused: bool,
) -> Vec3 {
    let mut light_out = Vec3::new(0.0, 0.0, 0.0);
    let rec = scene.hitables.hit(&ray, 0.0001, std::f32::MAX);
    if rec.is_none() {
        light_out += scene.bg.color(ray);
        return light_out;
    }
    let rec = rec.as_ref().unwrap();
    if !is_ray_diffused {
        light_out += rec.material.emitted(ray, rec);
    }
    if depth == 0 {
        return light_out;
    }
    let scatter = rec.material.scatter(ray, rec, rng);
    if scatter.is_none() {
        return light_out;
    }
    let scatter = scatter.as_ref().unwrap();
    match scatter.pdf {
        SingularPdf::Finite {
            pdf: ref material_pdf,
        } => {
            // NEE (Next Event Estimation)
            next_event_estimation(ray, rec, scene, &mut light_out, rng);
            let dir = material_pdf.generate(rng);
            let cosine = rec.normal.dot(&dir.normalize());
            if cosine > 0.0 {
                let density = material_pdf.density(&dir);
                debug_assert!(density > 0.0);
                let out_ray = Ray::new(&rec.point, &dir, ray.time);
                let in_light = calc_color(&out_ray, &scene, rng, depth - 1, true);
                let brdf = rec.material.brdf(&ray.direction, &dir, rec, &in_light);
                debug_assert!(cosine.is_finite());
                debug_assert!(cosine > 0.0);
                debug_assert!(density.is_finite());
                debug_assert!(density > 0.0);
                light_out += (cosine / density) * brdf;
            }
        }
        SingularPdf::Delta { ref dir } => {
            let out_ray = &Ray::new(&rec.point, dir, ray.time);
            let in_light = calc_color(&out_ray, &scene, rng, depth - 1, false);
            light_out += rec.material.brdf(&ray.direction, dir, rec, &in_light)
        }
    };
    light_out
}
