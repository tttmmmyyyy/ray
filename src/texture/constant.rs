use crate::aliases::{Vec2, Vec3};
use crate::texture::Texture;

pub struct ConstantTexture(Vec3);

impl ConstantTexture {
    pub fn new(color: &Vec3) -> Self {
        ConstantTexture { 0: *color }
    }
    pub fn rgb(r: f32, g: f32, b: f32) -> Self {
        ConstantTexture {
            0: Vec3::new(r, g, b),
        }
    }
    pub fn boxed(color: &Vec3) -> Box<Self> {
        Box::new(Self::new(color))
    }
}

impl Texture for ConstantTexture {
    fn value(&self, _uv: &Vec2, _p: &Vec3) -> Vec3 {
        self.0
    }
}
