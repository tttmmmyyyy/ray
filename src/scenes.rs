mod cornellbox;
mod manyspheres;
mod teapot;

use ray::scene::Scene;

#[allow(dead_code)]
pub enum ScenesType {
    CornellBox,
    ManySpheres,
    Teapot,
}

pub fn get(scene_type: ScenesType, aspect_ratio: f64) -> Scene {
    match scene_type {
        ScenesType::CornellBox => self::cornellbox::scene(aspect_ratio),
        ScenesType::ManySpheres => self::manyspheres::scene(aspect_ratio),
        ScenesType::Teapot => self::teapot::scene(aspect_ratio)
    }
}
