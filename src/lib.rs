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

pub fn calc_color(
    ray: &Ray,
    scene: &Scene,
    rng: &mut RandGen,
    depth: i32,
    is_ray_diffused: bool,
) -> Vec3 {
    let mut light_out = Vec3::new(0.0, 0.0, 0.0);
    if let Some(ref rec) = scene.hitables.hit(&ray, 0.0001, std::f32::MAX) {
        if !is_ray_diffused {
            light_out += rec.material.emitted(ray, rec);
        }
        let scatter = rec.material.scatter(ray, rec, rng);
        if depth > 0 && scatter.is_some() {
            let scatter = scatter.as_ref().unwrap();
            match scatter.pdf {
                SingularPdf::Finite {
                    pdf: ref material_pdf,
                } => {
                    // NEE (Next Event Estimation)
                    if let Some(ref light) = scene.light {
                        // ToDo: 改善する。現状は実験的雑実装。
                        let pdf = HitablePdf::new(&**light, &rec.point);
                        let dir = pdf.generate(rng);
                        let shadow_ray = Ray::new(&rec.point, &dir, ray.time);
                        if let Some(ref light_hit_rec) = light.hit(&shadow_ray, 0.0, std::f32::MAX)
                        {
                            if scene
                                .hitables
                                .hit(&shadow_ray, 0.0001, light_hit_rec.t) // ToDo: 特にここで無駄な計算をしている
                                .is_none()
                            {
                                let cosine = rec.normal.dot(&dir.normalize());
                                if cosine > 0.0 {
                                    let density = pdf.density(&dir);
                                    if density > 0.0 {
                                        // Mathematically Prob(density == 0.0) is zero (but occurres sometimes),
                                        // and therefore just ignoring such cases to avoid Inf is harmless.
                                        let light_in = light_hit_rec
                                            .material
                                            .emitted(&shadow_ray, &light_hit_rec);
                                        light_out += (cosine / density)
                                            * rec.material.brdf(
                                                &ray.direction,
                                                &dir,
                                                rec,
                                                &light_in,
                                            );
                                    }
                                }
                            }
                        }
                    }
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
        }
    } else {
        // if ray hits no hitable,
        light_out += scene.bg.color(ray)
    }
    light_out
}
