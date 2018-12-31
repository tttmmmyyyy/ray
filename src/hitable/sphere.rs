use crate::aabb::Aabb;
use crate::aliases::{RandGen, Vec2, Vec3};
use crate::hit_record::HitRecord;
use crate::hitable::Hitable;
use crate::material::Material;
use crate::onb::Onb;
use crate::pdf::random_in_cone;
use crate::ray::Ray;
use nalgebra as na;
use std::f32::consts::PI;
use std::sync::Arc;

pub struct Sphere {
    center: Vec3,
    radius: f32,
    material: Arc<Material>,
}

impl Sphere {
    pub fn new(center: &Vec3, radius: f32, material: Arc<Material>) -> Self {
        Sphere {
            center: *center,
            radius: radius,
            material: material,
        }
    }
    /// Calculates the parameter t of the ray at which it hits this Sphere.
    pub fn hit_core(center: &Vec3, radius: f32, ray: &Ray, t_min: f32, t_max: f32) -> Option<f32> {
        let oc = ray.origin - center;
        let a = na::dot(&ray.direction, &ray.direction);
        let b = na::dot(&oc, &ray.direction);
        let c = na::dot(&oc, &oc) - radius * radius;
        let disc: f32 = b * b - a * c;
        if disc <= 0.0 {
            return None;
        }
        let disc_rt = f32::sqrt(disc);
        let mut t: f32;
        t = (-b - disc_rt) / a;
        if t_min < t && t < t_max {
            return Some(t);
        }
        t = (-b + disc_rt) / a;
        if t_min < t && t < t_max {
            return Some(t);
        }
        return None;
    }
    /// convert a point on unit sphere to a uv coordinate
    pub fn get_uv(p: &Vec3) -> Vec2 {
        let phi = f32::atan2(p[2], p[0]);
        let theta = f32::asin(p[1].min(1.0).max(-1.0));
        Vec2::new(0.5 - 0.5 * (phi / PI), 0.5 + theta / PI)
    }
}

impl Hitable for Sphere {
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, t_max: f32) -> Option<HitRecord<'s>> {
        Sphere::hit_core(&self.center, self.radius, ray, t_min, t_max).map(|t| {
            let point = ray.evaluate(t);
            let normal = (point - self.center) / self.radius;
            let uv = Sphere::get_uv(&normal);
            HitRecord {
                t: t,
                point: point,
                tex_coord: uv,
                normal: normal,
                material: self.material.as_ref(),
            }
        })
    }
    fn bounding_box(&self, _time_0: f32, _time_1: f32) -> Option<Aabb> {
        let rad_vec = Vec3::new(self.radius, self.radius, self.radius);
        return Some(Aabb::new(
            &(self.center - rad_vec),
            &(self.center + rad_vec),
        ));
    }
    fn random_direction_from(&self, origin: &Vec3, rng: &mut RandGen) -> Vec3 {
        let cosine = (1.0 - self.radius.powf(2.0) / (self.center - origin).norm_squared()).sqrt();
        let vec_local = random_in_cone(cosine, rng);
        let onb = Onb::build_from_w(&(self.center - origin));
        onb.local_to_global_vec(&vec_local)
    }
    fn direction_density(&self, origin: &Vec3, dir: &Vec3) -> f32 {
        let cosine = (self.center - origin).normalize().dot(&dir.normalize());
        let cosine_max =
            (1.0 - self.radius.powf(2.0) / (origin - self.center).norm_squared()).sqrt();
        if cosine > cosine_max {
            1.0 / (2.0 * PI * (1.0 - cosine_max))
        } else {
            0.0
        }
    }
}

pub struct MovingSphere {
    center_0: Vec3, // center at time = 0
    center_1: Vec3, // center at time = 1
    radius: f32,
    material: Box<Material>,
}

impl MovingSphere {
    pub fn new(center_0: &Vec3, center_1: &Vec3, radius: f32, material: Box<Material>) -> Self {
        MovingSphere {
            center_0: *center_0,
            center_1: *center_1,
            radius: radius,
            material: material,
        }
    }
}

impl MovingSphere {
    pub fn center_at(&self, time: f32) -> Vec3 {
        self.center_0 + time * (self.center_1 - self.center_0)
    }
    pub fn bounding_box_at(&self, time: f32) -> Aabb {
        let rad_vec = Vec3::new(self.radius, self.radius, self.radius);
        let center = self.center_at(time);
        Aabb::new(&(center - rad_vec), &(center + rad_vec))
    }
}

impl Hitable for MovingSphere {
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, t_max: f32) -> Option<HitRecord<'s>> {
        let center = self.center_at(ray.time);
        Sphere::hit_core(&center, self.radius, ray, t_min, t_max).map(|t| {
            let point = ray.evaluate(t);
            let normal = (point - center) / self.radius;
            let uv = Sphere::get_uv(&normal);
            HitRecord {
                t: t,
                point: point,
                tex_coord: uv,
                normal: normal,
                material: self.material.as_ref(),
            }
        })
    }
    fn bounding_box(&self, time_0: f32, time_1: f32) -> Option<Aabb> {
        Some(Aabb::unite(
            &self.bounding_box_at(time_0),
            &self.bounding_box_at(time_1),
        ))
    }
}
