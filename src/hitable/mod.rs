pub mod bvh_node;
pub mod empty;
pub mod hitable_list;
pub mod hitable_ref;
pub mod obvh;
pub mod rectangle;
pub mod sphere;
pub mod transform;
pub mod triangle;

use crate::aabb::Aabb;
use crate::aliases::{RandGen, Vec3};
use crate::hit_record::HitRecord;
use crate::hitable::hitable_list::HitableList;
use crate::hitable::rectangle::Rectangle;
use crate::material::Material;
use crate::ray::Ray;
use std::sync::Arc;

pub trait Hitable: Send + Sync {
    /// Calculates HitRecord.
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, t_max: f32) -> Option<HitRecord<'s>>;
    /// Judge the ray hits self or not.
    /// Each Hitable should provide a more efficient implementation than the default.
    fn is_hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, t_max: f32) -> bool {
        self.hit(ray, t_min, t_max).is_some()
    }
    /// Axis-aligned bounding box of this Hitable.
    /// Returns None if this does not have bounding box (e.g., infinite plane)
    /// For moving objects, returns the unite of all boxes while the time interval [t0, t1]
    fn bounding_box(&self, time_0: f32, time_1: f32) -> Option<Aabb>;
    /// Generate a random direction vector from a specified point to this hitable.
    fn random_direction_from(&self, _origin: &Vec3, _rng: &mut RandGen) -> Vec3 {
        unimplemented!()
    }
    /// Pdf of directions generated by random_direction_from().
    fn direction_density(&self, _origin: &Vec3, _dir: &Vec3) -> f32 {
        unimplemented!()
    }
}

pub fn cube(size: &Vec3, material: Arc<Material>) -> impl Hitable {
    let mut recs = Vec::<Arc<Hitable>>::new();
    recs.push(Arc::new(Rectangle::new(
        &Vec3::new(0.0, 0.0, 0.0).component_mul(size),
        &Vec3::new(0.0, 1.0, 0.0).component_mul(size),
        &Vec3::new(1.0, 0.0, 0.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    recs.push(Arc::new(Rectangle::new(
        &Vec3::new(0.0, 0.0, 0.0).component_mul(size),
        &Vec3::new(0.0, 0.0, 1.0).component_mul(size),
        &Vec3::new(0.0, 1.0, 0.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    recs.push(Arc::new(Rectangle::new(
        &Vec3::new(0.0, 0.0, 0.0).component_mul(size),
        &Vec3::new(1.0, 0.0, 0.0).component_mul(size),
        &Vec3::new(0.0, 0.0, 1.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    recs.push(Arc::new(Rectangle::new(
        &Vec3::new(1.0, 1.0, 1.0).component_mul(size),
        &Vec3::new(-1.0, 0.0, 0.0).component_mul(size),
        &Vec3::new(0.0, -1.0, 0.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    recs.push(Arc::new(Rectangle::new(
        &Vec3::new(1.0, 1.0, 1.0).component_mul(size),
        &Vec3::new(0.0, -1.0, 0.0).component_mul(size),
        &Vec3::new(0.0, 0.0, -1.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    recs.push(Arc::new(Rectangle::new(
        &Vec3::new(1.0, 1.0, 1.0).component_mul(size),
        &Vec3::new(0.0, 0.0, -1.0).component_mul(size),
        &Vec3::new(-1.0, 0.0, 0.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    HitableList::new(recs)
}

pub fn cube_rectangles(pos: &Vec3, size: &Vec3, material: Arc<Material>) -> Vec<Arc<Hitable>> {
    // ToDo: Vec<Arc<Hitable>>ではなくVec<Rectangle>を返すようにする
    let mut recs = Vec::<Arc<Hitable>>::default();
    recs.push(Arc::new(Rectangle::new(
        &(pos + &Vec3::new(0.0, 0.0, 0.0).component_mul(size)),
        &Vec3::new(0.0, 1.0, 0.0).component_mul(size),
        &Vec3::new(1.0, 0.0, 0.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    recs.push(Arc::new(Rectangle::new(
        &(pos + &Vec3::new(0.0, 0.0, 0.0).component_mul(size)),
        &Vec3::new(0.0, 0.0, 1.0).component_mul(size),
        &Vec3::new(0.0, 1.0, 0.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    recs.push(Arc::new(Rectangle::new(
        &(pos + &Vec3::new(0.0, 0.0, 0.0).component_mul(size)),
        &Vec3::new(1.0, 0.0, 0.0).component_mul(size),
        &Vec3::new(0.0, 0.0, 1.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    recs.push(Arc::new(Rectangle::new(
        &(pos + &Vec3::new(1.0, 1.0, 1.0).component_mul(size)),
        &Vec3::new(-1.0, 0.0, 0.0).component_mul(size),
        &Vec3::new(0.0, -1.0, 0.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    recs.push(Arc::new(Rectangle::new(
        &(pos + &Vec3::new(1.0, 1.0, 1.0).component_mul(size)),
        &Vec3::new(0.0, -1.0, 0.0).component_mul(size),
        &Vec3::new(0.0, 0.0, -1.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    recs.push(Arc::new(Rectangle::new(
        &(pos + &Vec3::new(1.0, 1.0, 1.0).component_mul(size)),
        &Vec3::new(0.0, 0.0, -1.0).component_mul(size),
        &Vec3::new(-1.0, 0.0, 0.0).component_mul(size),
        material.clone(),
        0.000,
    )));
    recs
}
