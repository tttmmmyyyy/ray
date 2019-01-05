use crate::aliases::{Vec2, Vec3};
use crate::texture::Texture;
use std::f32::consts::PI;
use std::sync::Arc;

pub struct CheckerTexture {
    length: f32,
    even: Arc<Texture>,
    odd: Arc<Texture>,
    phase: Vec3,
}

impl CheckerTexture {
    pub fn new(even: Arc<Texture>, odd: Arc<Texture>, length: f32, phase: &Vec3) -> Self {
        CheckerTexture {
            even: even,
            odd: odd,
            length: length,
            phase: *phase,
        }
    }
}

impl Texture for CheckerTexture {
    fn value(&self, uv: &Vec2, p: &Vec3) -> Vec3 {
        let x = PI * p / self.length + self.phase;
        let sines = f32::sin(x[0]) * f32::sin(x[1]) * f32::sin(x[2]);
        if sines < 0.0 {
            self.odd.value(uv, p)
        } else {
            self.even.value(uv, p)
        }
    }
}
