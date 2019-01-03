use crate::aliases::{RandGen, Vec3};
use crate::hit_record::HitRecord;
use crate::material::Material;
use crate::ray::Ray;
use crate::scatter_record::ScatterRecord;
use crate::texture::Texture;
use std::sync::Arc;

pub struct DiffuseLight {
    pub emit: Arc<Texture>,
}

impl DiffuseLight {
    pub fn new(emit: Arc<Texture>) -> Self {
        DiffuseLight { emit: emit }
    }
}

impl Material for DiffuseLight {
    fn scatter(&self, _ray: &Ray, _rec: &HitRecord, _rng: &mut RandGen) -> Option<ScatterRecord> {
        None
    }
    fn emitted(&self, ray_in: &Ray, rec: &HitRecord) -> Vec3 {
        if rec.normal.dot(&ray_in.direction) < 0.0 {
            self.emit.value(&rec.tex_coord, &rec.point)
        } else {
            Vec3::new(0.0, 0.0, 0.0)
        }
    }
    fn brdf(&self, _in_ray: &Vec3, _out_ray: &Vec3, _rec: &HitRecord, _in_light: &Vec3) -> Vec3 {
        panic!("brdf called for Metal.")
    }
}
