use crate::aliases::{Mat3, Vec3};
use nalgebra;

/// 3d invertible affine transformation
#[derive(Clone, Copy)]
pub struct Affine {
    a: Mat3,
    b: Vec3,
    a_inv: Mat3, // the inverce matrix of a
    a_det: f64,  // the deteminant of a
}

impl Affine {
    /// The basic constructor.
    /// Note: currently
    /// - the ScatterRecords returned from Metal or Glass, and
    /// - Transform.direction_density
    /// are only compatible with similarity transformation
    /// So new cannot be public since it returns one which is not a similarity transformation.
    fn new(a: &Mat3, b: &Vec3) -> Self {
        Affine {
            a: *a,
            b: *b,
            a_inv: a.try_inverse().unwrap(),
            a_det: a.determinant(),
        }
    }
    /// Linear transformation.
    fn linear(linear: &Mat3, origin: &Vec3) -> Self {
        Affine::new(linear, &(-linear * origin + origin))
    }
    /// Isotropic scaling
    pub fn scale(scaling: f64, origin: &Vec3) -> Self {
        Affine::scale_axis(&Vec3::new(scaling, scaling, scaling), origin)
    }
    fn scale_axis(scaling: &Vec3, origin: &Vec3) -> Self {
        Affine::linear(&Mat3::from_diagonal(scaling), origin)
    }
    /// Creates inverse transformation of self
    pub fn inverse(&self) -> Affine {
        Affine {
            a: self.a_inv,
            b: -self.a_inv * self.b,
            a_inv: self.a,
            a_det: 1.0 / self.a_det,
        }
    }
    /// Act on a point.
    pub fn act_point(&self, pt: &Vec3) -> Vec3 {
        self.a * pt + self.b
    }
    /// Act on a 1-vector (e.g., difference of two points)
    pub fn act_vec(&self, v: &Vec3) -> Vec3 {
        self.a * v
    }
    /// Act on a 2-vector (e.g., the cross product of two 1-vectors, such as normal to a surface)
    pub fn act_2_vec(&self, tv: &Vec3) -> Vec3 {
        self.a_det * self.a_inv.transpose() * tv
    }
    /// Creates rotation transformation.
    pub fn rotation(aixsangle: &Vec3, origin: &Vec3) -> Self {
        Affine::linear(nalgebra::Rotation3::new(*aixsangle).matrix(), origin)
    }
    /// Creates translation transformation.
    pub fn translate(diff: &Vec3) -> Self {
        Affine::new(&Mat3::from_diagonal(&Vec3::new(1.0, 1.0, 1.0)), diff)
    }
    // ToDo: implement composition
}
