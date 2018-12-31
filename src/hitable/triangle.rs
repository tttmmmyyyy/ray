use crate::aabb::Aabb;
use crate::aliases::{RandGen, Vec2, Vec3};
use crate::hit_record::HitRecord;
use crate::hitable::Hitable;
use crate::material::Material;
use crate::ray::Ray;
use std::sync::Arc;

pub struct Triangle {
    vertices: [Vec3; 3],              // a,b,c
    perpendicular_normals: [Vec3; 3], // (aから辺bcへの単位垂線ベクトル, bから辺caへの..., cから辺abへの...)
    perpendicular_lengths: [f32; 3],  // 垂線の長さ
    normal: Vec3,                     // 単位法線ベクトル。(b-a).cross(c-a).normalize
    material: Arc<Material>,
    vertex_normals: Option<[Vec3; 3]>, // Normal vectors at vertices. If this is Some, the hit() returns HitRecord with linearly interpolated normal vector.
}

impl Triangle {
    pub fn new(
        vertices: &[Vec3; 3],
        vertex_normals: &Option<[Vec3; 3]>,
        material: Arc<Material>,
    ) -> Self {
        let a = vertices[0];
        let b = vertices[1];
        let c = vertices[2];
        let v = (b - a).cross(&(c - a));
        let n = v.normalize();
        let na = v.cross(&(b - c)).normalize();
        let nb = v.cross(&(c - a)).normalize();
        let nc = v.cross(&(a - b)).normalize();
        let ha = v.norm() / (b - c).norm();
        let hb = v.norm() / (c - a).norm();
        let hc = v.norm() / (a - b).norm();
        Self {
            vertices: [a, b, c],
            normal: n,
            perpendicular_normals: [na, nb, nc],
            perpendicular_lengths: [ha, hb, hc],
            vertex_normals: *vertex_normals,
            material: material,
        }
    }
}

impl Hitable for Triangle {
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, t_max: f32) -> Option<HitRecord<'s>> {
        let denom = self.normal.dot(&ray.direction);
        if denom == 0.0 {
            return None;
        }
        let t = (self.vertices[0] - ray.origin).dot(&self.normal) / denom;
        if t <= t_min || t_max <= t {
            return None;
        }
        let p = ray.evaluate(t);
        let mut weights: [f32; 3] = [0.0, 0.0, 0.0];
        for i in 0..3 {
            weights[i] = 1.0
                - (p - self.vertices[i]).dot(&self.perpendicular_normals[i])
                    / self.perpendicular_lengths[i];
        }
        if weights[0] < 0.0 || weights[1] < 0.0 || weights[2] < 0.0 {
            return None;
        }
        let normal = if let Some(ref vertex_normals) = self.vertex_normals {
            (weights[0] * vertex_normals[0]
                + weights[1] * vertex_normals[1]
                + weights[2] * vertex_normals[2])
                .normalize()
        } else {
            self.normal
        };
        Some(HitRecord {
            t: t,
            point: p,
            tex_coord: Vec2::new(0.0, 0.0),
            normal: normal,
            material: self.material.as_ref(),
        })
    }
    fn bounding_box(&self, _time_0: f32, _time_1: f32) -> Option<Aabb> {
        Some(Aabb::from_points(vec![
            self.vertices[0],
            self.vertices[1],
            self.vertices[2],
        ]))
    }
    fn random_direction_from(&self, _origin: &Vec3, _rng: &mut RandGen) -> Vec3 {
        unimplemented!()
    }
    fn direction_density(&self, _origin: &Vec3, _dir: &Vec3) -> f32 {
        unimplemented!()
    }
}
