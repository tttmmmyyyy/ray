use crate::aliases::{Mat4, Vec3};
use std::time::Duration;

pub fn zipwith_vec3(lhs: &Vec3, rhs: &Vec3, zipper: impl Fn(f32, f32) -> f32) -> Vec3 {
    Vec3::new(
        zipper(lhs[0], rhs[0]),
        zipper(lhs[1], rhs[1]),
        zipper(lhs[2], rhs[2]),
    )
}

pub fn min_vec3(lhs: &Vec3, rhs: &Vec3) -> Vec3 {
    zipwith_vec3(lhs, rhs, f32::min)
}

pub fn max_vec3(lhs: &Vec3, rhs: &Vec3) -> Vec3 {
    zipwith_vec3(lhs, rhs, f32::max)
}

pub fn proj_transform(mat: &Mat4, pt: &Vec3) -> Vec3 {
    Vec3::from_homogeneous(mat * pt.to_homogeneous()).unwrap()
}

pub fn duration_to_secs(dur: &Duration) -> f64 {
    dur.as_secs() as f64 + dur.subsec_millis() as f64 * 1.0e-3 + dur.subsec_nanos() as f64 * 1.0e-6
}

pub fn pretty_print_f32(val: f32) -> String {
if val.is_nan() {
                "NaN".to_string()
            } else if val == std::f32::MAX {
                "MAX".to_string()
            } else if val == std::f32::MIN {
                "MIN".to_string()
            } else if val == std::f32::INFINITY {
                "+INF".to_string()
            } else if val == std::f32::NEG_INFINITY {
                "-INF".to_string()
            } else {
                format!("{:.3}", val)
            }
}