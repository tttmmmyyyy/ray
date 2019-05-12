mod cornellbox;
mod manyspheres;
mod menger;
mod teapot;

use ray::scene::Scene;

#[allow(dead_code)]
pub enum ScenesType {
    CornellBox,
    ManySpheres,
    Teapot,
    Menger,
}

pub fn get(scene_type: ScenesType, aspect_ratio: f32) -> Scene {
    match scene_type {
        ScenesType::CornellBox => self::cornellbox::scene(aspect_ratio),
        ScenesType::ManySpheres => self::manyspheres::scene(aspect_ratio),
        ScenesType::Teapot => self::teapot::scene(aspect_ratio),
        ScenesType::Menger => self::menger::scene(aspect_ratio),
    }
}
