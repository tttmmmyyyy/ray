use crate::aabb::Aabb;
use crate::aliases::{RandGen, Vec3};
use crate::hit_record::HitRecord;
use crate::hitable::Hitable;
use crate::ray::Ray;
use rand::Rng;
use std::sync::Arc;

pub struct HitableList {
    pub list: Vec<Arc<Hitable>>,
}

impl HitableList {
    pub fn new(objs: Vec<Arc<Hitable>>) -> Self {
        HitableList { list: objs }
    }
}

impl Hitable for HitableList {
    fn hit(&self, ray: &Ray, t_min: f64, t_max: f64) -> Option<HitRecord> {
        let mut res: Option<HitRecord> = None;
        let mut closest_so_far = t_max;
        for obj in &self.list {
            if let Some(ref tmp_rec) = obj.hit(ray, t_min, closest_so_far) {
                closest_so_far = tmp_rec.t;
                res = Some(*tmp_rec);
            }
        }
        return res;
    }
    fn bounding_box(&self, time_0: f64, time_1: f64) -> Option<Aabb> {
        if self.list.len() == 0 {
            return Some(Aabb::empty());
        }
        let mut cur = Aabb::empty();
        for hitable in &self.list {
            if let Some(ref aabb) = hitable.bounding_box(time_0, time_1) {
                cur = Aabb::unite(&cur, aabb);
            } else {
                return None;
            }
        }
        return Some(cur);
    }
    fn random_direction_from(&self, origin: &Vec3, rng: &mut RandGen) -> Vec3 {
        let idx = (rng.gen::<f64>() * self.list.len() as f64) as usize;
        self.list[idx].random_direction_from(origin, rng)
    }
    fn direction_density(&self, origin: &Vec3, dir: &Vec3) -> f64 {
        let sum: f64 = (&(*self.list))
            .into_iter()
            .map(|o| o.direction_density(origin, dir))
            .sum();
        sum / self.list.len() as f64
    }
}
