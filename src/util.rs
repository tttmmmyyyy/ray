use crate::aliases::{Mat4, Vec2, Vec3};
use std::cmp::Eq;
use std::hash::{Hash, Hasher};
use std::mem::transmute;
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

pub fn zipwith_vec2(lhs: &Vec2, rhs: &Vec2, zipper: impl Fn(f32, f32) -> f32) -> Vec2 {
    Vec2::new(zipper(lhs[0], rhs[0]), zipper(lhs[1], rhs[1]))
}

pub fn min_vec2(lhs: &Vec2, rhs: &Vec2) -> Vec2 {
    zipwith_vec2(lhs, rhs, f32::min)
}

pub fn max_vec2(lhs: &Vec2, rhs: &Vec2) -> Vec2 {
    zipwith_vec2(lhs, rhs, f32::max)
}

pub fn proj_transform(mat: &Mat4, pt: &Vec3) -> Vec3 {
    Vec3::from_homogeneous(mat * pt.to_homogeneous()).unwrap()
}

pub fn duration_to_secs(dur: &Duration) -> f64 {
    dur.as_secs() as f64 + dur.subsec_millis() as f64 * 1.0e-3 + dur.subsec_nanos() as f64 * 1.0e-9
}

pub fn compare_vec3_le(lhs: &Vec3, rhs: &Vec3) -> bool {
    // Node: nalgebra's operator <= is 'using column-major lexicographic ordering'.
    lhs[0] <= rhs[0] && lhs[1] <= rhs[1] && lhs[2] <= rhs[2]
}

pub fn compare_vec2_le(lhs: &Vec2, rhs: &Vec2) -> bool {
    // Node: nalgebra's operator <= is 'using column-major lexicographic ordering'.
    lhs[0] <= rhs[0] && lhs[1] <= rhs[1]
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

#[derive(PartialEq)]
pub struct HashVec3(pub Vec3);

impl Hash for HashVec3 {
    fn hash<H: Hasher>(&self, state: &mut H) {
        unsafe {
            for i in 0..3 {
                let x = self.0[i];
                if x == 0.0 {
                    state.write_u32(transmute(0.0f32));
                } else {
                    state.write_u32(transmute(x));
                }
                // NaNs are distinguished.
            }
        }
    }
}

// lift PartialEq to Eq
impl Eq for HashVec3 {}
