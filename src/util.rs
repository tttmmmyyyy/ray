use crate::aliases::{Mat4, Vec3};

pub fn zipwith_vec3(lhs: &Vec3, rhs: &Vec3, zipper: impl Fn(f64, f64) -> f64) -> Vec3 {
    Vec3::new(
        zipper(lhs[0], rhs[0]),
        zipper(lhs[1], rhs[1]),
        zipper(lhs[2], rhs[2]),
    )
}

pub fn min_vec3(lhs: &Vec3, rhs: &Vec3) -> Vec3 {
    zipwith_vec3(lhs, rhs, f64::min)
}

pub fn max_vec3(lhs: &Vec3, rhs: &Vec3) -> Vec3 {
    zipwith_vec3(lhs, rhs, f64::max)
}

pub fn proj_transform(mat: &Mat4, pt: &Vec3) -> Vec3 {
    Vec3::from_homogeneous(mat * pt.to_homogeneous()).unwrap()
}
