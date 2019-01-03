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

fn mix_importance_hitable_pdf<'m, 's, 'p>(
    material_pdf: &'m dyn Pdf,
    weight_imp_hit_pdf: &'s Option<(f32, HitablePdf)>,
) -> MixturePdf<'s, 'm> {
    match weight_imp_hit_pdf {
        Some((weight, imp_hit_pdf)) => {
            debug_assert!(*weight > 0.0);
            MixturePdf::new(*weight, imp_hit_pdf, material_pdf)
        }
        None => MixturePdf::zero(material_pdf),
    }
}

fn create_importance_hitable_pdf<'s>(
    scene: &'s Scene,
    hit_pt: &Vec3,
) -> Option<(f32, HitablePdf<'s>)> {
    if scene.importance_weight > 0.0 {
        Some((
            scene.importance_weight,
            HitablePdf::new(&(*scene.importance), hit_pt),
        ))
    } else {
        None
    }
}

pub fn calc_color(ray: &Ray, scene: &Scene, rng: &mut RandGen, depth: i32) -> Vec3 {
    if let Some(ref rec) = scene.hitables.hit(&ray, 0.0001, std::f32::MAX) {
        let emitted = rec.material.emitted(ray, rec);
        let scatter = rec.material.scatter(ray, rec, rng);
        if depth > 0 && scatter.is_some() {
            let scatter = scatter.as_ref().unwrap();
            let light = match scatter.pdf {
                SingularPdf::Finite {
                    pdf: ref material_pdf,
                } => {
                    let imp_objs_pdf = create_importance_hitable_pdf(scene, &rec.point);
                    let pdf = mix_importance_hitable_pdf(&**material_pdf, &imp_objs_pdf);
                    let dir = pdf.generate(rng);
                    let density = pdf.density(&dir);
                    let out_ray = Ray::new(&rec.point, &dir, ray.time);
                    let in_light = calc_color(&out_ray, &scene, rng, depth - 1);
                    let cosine = rec.normal.dot(&dir.normalize()).max(0.0);
                    (cosine / density) * rec.material.brdf(&ray.direction, &dir, rec, &in_light)
                }
                SingularPdf::Delta { ref dir } => {
                    let out_ray = &Ray::new(&rec.point, dir, ray.time);
                    let in_light = calc_color(&out_ray, &scene, rng, depth - 1);
                    rec.material.brdf(&ray.direction, dir, rec, &in_light)
                }
            };
            emitted + light
        } else {
            // if this material does not scatter ray (or depth == 0),
            emitted
        }
    } else {
        // if ray hits no hitable,
        scene.bg.color(ray)
    }
}
