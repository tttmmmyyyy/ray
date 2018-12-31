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
    weight_imp_hit_pdf: &'s Option<(f64, HitablePdf)>,
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
) -> Option<(f64, HitablePdf<'s>)> {
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
    if let Some(ref rec) = scene.hitables.hit(&ray, 0.0001, std::f64::MAX) {
        let emitted = rec.material.emitted(ray, rec);
        let opt_s_rec = rec.material.scatter(ray, rec, rng);
        if depth > 0 && opt_s_rec.is_some() {
            let s_rec = opt_s_rec.as_ref().unwrap();
            let l_i = match s_rec.important_dir {
                SingularPdf::Finite {
                    pdf: ref material_pdf,
                } => {
                    // let imp_objs_pdf = HitablePdf::new(&(*scene.importance), &rec.point);
                    // let pdf =
                    //     MixturePdf::new(scene.importance_weight, &imp_objs_pdf, &**material_pdf);
                    // let pdf = mix_pdf_importance_hitable(&**material_pdf, scene, &rec.point);
                    let imp_objs_pdf = create_importance_hitable_pdf(scene, &rec.point);
                    let pdf = mix_importance_hitable_pdf(&**material_pdf, &imp_objs_pdf);
                    let dir = pdf.generate(rng);
                    // println!("{}", dir);
                    let density = pdf.density(&dir);
                    let scat = Ray::new(&rec.point, &dir, rec.t);
                    let scat_pdf = rec.material.scattering_pdf(ray, &scat, rec);
                    (scat_pdf / density * s_rec.attenuation).component_mul(&calc_color(
                        &scat,
                        &scene,
                        rng,
                        depth - 1,
                    ))
                }
                SingularPdf::Delta { ref dir } => s_rec.attenuation.component_mul(&calc_color(
                    &Ray::new(&rec.point, dir, rec.t),
                    scene,
                    rng,
                    depth - 1,
                )),
            };
            emitted + l_i
        } else {
            emitted
        }
    } else {
        scene.bg.color(ray)
    }
}
