use crate::aliases::{Vec2, Vec3};
use crate::texture::Texture;
use std::f32::consts::PI;
use std::sync::Arc;

pub struct CheckerTexture {
    length: f32,
    even: Arc<Texture>,
    odd: Arc<Texture>,
}

impl CheckerTexture {
    pub fn new(even: Arc<Texture>, odd: Arc<Texture>, length: f32) -> Self {
        CheckerTexture {
            even: even,
            odd: odd,
            length: length,
        }
    }
}

impl Texture for CheckerTexture {
    fn value(&self, uv: &Vec2, p: &Vec3) -> Vec3 {
        let sines = f32::sin(PI * p[0] / self.length)
            * f32::sin(PI * p[1] / self.length)
            * f32::sin(PI * p[2] / self.length);
        if sines < 0.0 {
            self.odd.value(uv, p)
        } else {
            self.even.value(uv, p)
        }
    }
}
