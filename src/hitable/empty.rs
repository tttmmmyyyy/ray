use crate::aabb::Aabb;
use crate::hit_record::HitRecord;
use crate::hitable::Hitable;
use crate::ray::Ray;

pub struct Empty;

impl Empty {
    pub fn new() -> Self {
        Empty {}
    }
}

impl Hitable for Empty {
    fn hit(&self, _ray: &Ray, _t_min: f64, _t_max: f64) -> Option<HitRecord> {
        None
    }
    fn bounding_box(&self, _time_0: f64, _time_1: f64) -> Option<Aabb> {
        Some(Aabb::empty())
    }
}
