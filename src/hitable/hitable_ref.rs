use crate::aabb::Aabb;
use crate::aliases::RandGen;
use crate::aliases::Vec3;
use crate::hit_record::HitRecord;
use crate::hitable::Hitable;
use crate::Ray;
use std::sync::Arc;

pub struct HitableRef(Arc<Hitable>);

impl Hitable for HitableRef {
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, t_max: f32) -> Option<HitRecord<'s>> {
        self.0.hit(ray, t_min, t_max)
    }
    fn is_hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, t_max: f32) -> bool {
        self.0.hit(ray, t_min, t_max).is_some()
    }
    fn bounding_box(&self, time_0: f32, time_1: f32) -> Option<Aabb> {
        self.0.bounding_box(time_0, time_1)
    }
    fn random_direction_from(&self, origin: &Vec3, rng: &mut RandGen) -> Vec3 {
        self.0.random_direction_from(origin, rng)
    }
    fn direction_density(&self, origin: &Vec3, dir: &Vec3) -> f32 {
        self.0.direction_density(origin, dir)
    }
}
