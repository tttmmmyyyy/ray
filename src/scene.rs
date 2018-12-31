use crate::background::Background;
use crate::camera::Camera;
use crate::hitable::Hitable;
use std::sync::Arc;

pub struct Scene {
    pub hitables: Arc<Hitable>,   // rendered hitables
    pub importance: Arc<Hitable>, // hitables used for importance sampling method
    pub importance_weight: f64,
    pub camera: Camera,
    pub bg: Arc<Background>,
}
