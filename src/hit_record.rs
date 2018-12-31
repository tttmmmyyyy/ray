use crate::affine::Affine;
use crate::aliases::{Vec2, Vec3};
use crate::material::Material;

#[derive(Clone, Copy)]
pub struct HitRecord<'a> {
    pub t: f64,
    pub point: Vec3,
    pub tex_coord: Vec2,
    pub normal: Vec3,
    pub material: &'a Material,
}

impl<'a> HitRecord<'a> {
    pub fn get_transformed(&self, tr: &Affine) -> HitRecord<'a> {
        HitRecord {
            t: self.t,
            point: tr.act_point(&self.point),
            tex_coord: self.tex_coord,
            normal: tr.act_2_vec(&self.normal).normalize(),
            material: self.material,
        }
    }
}
