use crate::aabb::Aabb;
use crate::aliases::{RandGen, Vec2, Vec3};
use crate::hit_record::HitRecord;
use crate::hitable::Hitable;
use crate::material::Material;
use crate::ray::Ray;
use crate::util::{max_vec3, min_vec3};
use rand::Rng;
use std;
use std::sync::Arc;

pub struct Rectangle {
    origin: Vec3,
    // edge_0, edge_1: must be orthogonal.
    edge_0: Vec3,
    edge_1: Vec3,
    // normal must be (edge_0 x edge_1).normalize().
    // This defines the front side of Rectangle.
    normal: Vec3,
    material: Arc<Material>,
    // a small positive to inflate the bounding_box()
    bdb_margin: f64,
}

impl Rectangle {
    pub fn new(
        origin: &Vec3,
        edge_0: &Vec3,
        edge_1: &Vec3,
        material: Arc<Material>,
        bdb_margin: f64,
    ) -> Self {
        Rectangle {
            origin: *origin,
            edge_0: *edge_0,
            edge_1: *edge_1,
            normal: edge_0.cross(edge_1).normalize(),
            material: material,
            bdb_margin: bdb_margin,
        }
    }
}

impl Hitable for Rectangle {
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f64, t_max: f64) -> Option<HitRecord<'s>> {
        let denom = ray.direction.dot(&self.normal);
        if denom == 0.0 {
            return None;
        }
        let t = (self.origin - ray.origin).dot(&self.normal) / denom;
        if t <= t_min || t_max <= t {
            return None;
        }
        let point = ray.evaluate(t);
        let rel_pt = point - self.origin;
        let u = rel_pt.dot(&self.edge_0) / self.edge_0.norm_squared();
        let v = rel_pt.dot(&self.edge_1) / self.edge_1.norm_squared();
        if 0.0 <= u && u <= 1.0 && 0.0 <= v && v <= 1.0 {
            Some(HitRecord {
                t: t,
                point: point,
                tex_coord: Vec2::new(u, v),
                normal: self.normal,
                material: self.material.as_ref(),
            })
        } else {
            None
        }
    }
    fn bounding_box(&self, _time_0: f64, _time_1: f64) -> Option<Aabb> {
        let p0 = self.origin;
        let p1 = self.origin + self.edge_0;
        let p2 = self.origin + self.edge_1;
        let p3 = self.origin + self.edge_0 + self.edge_1;
        let margin = Vec3::new(self.bdb_margin, self.bdb_margin, self.bdb_margin);
        let mn = min_vec3(&p0, &min_vec3(&p1, &min_vec3(&p2, &p3))) - margin;
        let mx = max_vec3(&p0, &max_vec3(&p1, &max_vec3(&p2, &p3))) + margin;
        Some(Aabb::new(&mn, &mx))
    }
    fn random_direction_from(&self, origin: &Vec3, rng: &mut RandGen) -> Vec3 {
        self.origin + rng.gen::<f64>() * self.edge_0 + rng.gen::<f64>() * self.edge_1 - origin
    }
    fn direction_density(&self, origin: &Vec3, dir: &Vec3) -> f64 {
        if let Some(ref rec) = self.hit(
            &Ray::new(origin, &dir, 0.0),
            0.0, /* Note: or 0.001? */
            std::f64::MAX,
        ) {
            let area = self.edge_0.cross(&self.edge_1).norm();
            let dist_squared = (rec.point - origin).norm_squared();
            let cosine = dir.normalize().dot(&rec.normal).abs();
            dist_squared / (cosine * area)
        } else {
            0.0
        }
    }
}
