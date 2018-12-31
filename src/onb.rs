use crate::aliases::Vec3;
use std::ops::Index;

/// Orthogonal normal basis
/// 0=u, 1=v, 2=w
pub struct Onb {
    axis: [Vec3; 3],
}

impl Onb {
    /// Build a orthonormal basis with w specified.
    /// w_dir is not required to be normalized.
    pub fn build_from_w(w_dir: &Vec3) -> Self {
        let w = w_dir.normalize();
        let another_dir = if w[0].abs() > 0.9 {
            // if w_norm is almost along x-axis,
            Vec3::new(0.0, 1.0, 0.0)
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        };
        let v = w.cross(&another_dir).normalize();
        let u = v.cross(&w);
        Onb { axis: [u, v, w] }
    }
    pub fn u(&self) -> &Vec3 {
        &self.axis[0]
    }
    pub fn v(&self) -> &Vec3 {
        &self.axis[1]
    }
    pub fn w(&self) -> &Vec3 {
        &self.axis[2]
    }
    pub fn local_to_global_vec(&self, uvw: &Vec3) -> Vec3 {
        uvw[0] * self.u() + uvw[1] * self.v() + uvw[2] * self.w()
    }
    pub fn local_to_global_coords(&self, u: f64, v: f64, w: f64) -> Vec3 {
        u * self.u() + v * self.v() + w * self.w()
    }
}

impl Index<usize> for Onb {
    type Output = Vec3;
    fn index(&self, index: usize) -> &Vec3 {
        &self.axis[index]
    }
}
