pub mod checker;
pub mod constant;
pub mod image;
pub mod noise;

use crate::aliases::{Vec2, Vec3};

pub trait Texture: Send + Sync {
    // (u,v) for 2d texture, p for 3d texture
    fn value(&self, uv: &Vec2, p: &Vec3) -> Vec3;
}
