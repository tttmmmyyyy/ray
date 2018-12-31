use crate::aabb::Aabb;
use crate::affine::Affine;
use crate::aliases::{RandGen, Vec3};
use crate::hit_record::HitRecord;
use crate::hitable::Hitable;
use crate::ray::Ray;
use std::sync::Arc;

pub struct Transform {
    original: Arc<Hitable>,
    transform: Affine,
    inv_transform: Affine,
    bbox: Option<Aabb>,
}

impl Transform {
    /// Constructor.
    /// time_0, time_1 is used for moving hitables.
    pub fn new(original: Arc<Hitable>, tr: &Affine, time_0: f32, time_1: f32) -> Self {
        let bbox = original
            .bounding_box(time_0, time_1)
            .map(|bbox| bbox.get_transformed(tr));
        Transform {
            original: original,
            transform: *tr,
            inv_transform: tr.inverse(),
            bbox: bbox,
        }
    }
}

impl Hitable for Transform {
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, t_max: f32) -> Option<HitRecord<'s>> {
        self.original
            .hit(&ray.get_transformed(&self.inv_transform), t_min, t_max)
            .map(|rec| rec.get_transformed(&self.transform))
    }
    fn bounding_box(&self, _time_0: f32, _time_1: f32) -> Option<Aabb> {
        self.bbox
    }
    fn random_direction_from(&self, _origin: &Vec3, _rng: &mut RandGen) -> Vec3 {
        unimplemented!()
    }
    fn direction_density(&self, _origin: &Vec3, _dir: &Vec3) -> f32 {
        unimplemented!()
    }
}
