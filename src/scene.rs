use crate::background::Background;
use crate::camera::Camera;
use crate::hit_record::HitRecord;
use crate::hitable::Hitable;
use crate::ray::Ray;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// シーン
/// * light - Next Event Estimation の対象となる光源。
/// lightはhitablesにも入れておかないと、NEEを使わない経路（カメラに直接光源が入るなど）に反映されなくなり不適切な結果になる。
pub struct Scene {
    pub hitables: Arc<Hitable>, // rendered hitables
    pub light: Option<Arc<Hitable>>,
    pub camera: Camera,
    pub bg: Arc<Background>,
}

fn duration_to_secs(dur: &Duration) -> f64 {
    dur.as_secs() as f64 + dur.subsec_millis() as f64 * 0.001 + dur.subsec_nanos() as f64 * 0.000001
}

impl Scene {
    pub fn debug_hit<'s>(&'s self, ray: &'s Ray) -> Option<HitRecord<'s>> {
        let debug_time = Instant::now();
        let res = self.hitables.hit(&ray, 0.0001, std::f32::MAX);
        println!("hit: {}", duration_to_secs(&debug_time.elapsed()));
        res
    }
}
