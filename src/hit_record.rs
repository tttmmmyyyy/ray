use crate::affine::Affine;
use crate::aliases::{Vec2, Vec3};
use crate::material::Material;

#[derive(Clone, Copy)]
pub struct HitRecord<'a> {
    pub t: f32,
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
    pub fn min_opt<'r, 'x: 'r, 'y: 'r>(
        x: &Option<HitRecord<'x>>,
        y: &Option<HitRecord<'y>>,
    ) -> Option<HitRecord<'r>> {
        if x.is_none() {
            return *y;
        }
        if y.is_none() {
            return *x;
        }
        Some(Self::min(x.as_ref().unwrap(), y.as_ref().unwrap()))
    }
    pub fn min<'r, 'x: 'r, 'y: 'r>(a: &HitRecord<'x>, b: &HitRecord<'y>) -> HitRecord<'r> {
        if a.t < b.t {
            *b
        } else {
            *a
        }
    }
    pub fn replace_to_some_min<'s>(dst: &mut Option<HitRecord<'s>>, src: &Option<HitRecord<'s>>) {
        if src.is_none() {
            return;
        }
        if dst.is_none() {
            *dst = *src;
            return;
        }
        if src.as_ref().unwrap().t < dst.as_ref().unwrap().t {
            *dst = *src;
            return;
        }
    }
}
