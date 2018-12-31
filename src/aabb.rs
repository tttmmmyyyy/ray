use crate::affine::Affine;
use crate::aliases::Vec3;
use crate::ray::Ray;
use crate::util::{max_vec3, min_vec3};
use itertools::iproduct;
use std;

/// Axis-Aligned Bounding Box
#[derive(Clone, Copy)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    pub fn new(min: &Vec3, max: &Vec3) -> Self {
        Aabb {
            min: *min,
            max: *max,
        }
    }
    pub fn empty() -> Self {
        Aabb::new(
            &Vec3::new(std::f32::INFINITY, std::f32::INFINITY, std::f32::INFINITY),
            &Vec3::new(
                std::f32::NEG_INFINITY,
                std::f32::NEG_INFINITY,
                std::f32::NEG_INFINITY,
            ),
        )
    }
    pub fn is_empty(&self) -> bool {
        self.min == Vec3::new(std::f32::INFINITY, std::f32::INFINITY, std::f32::INFINITY)
            && self.max
                == Vec3::new(
                    std::f32::NEG_INFINITY,
                    std::f32::NEG_INFINITY,
                    std::f32::NEG_INFINITY,
                )
    }
    pub fn hit(&self, ray: &Ray, t_min: f32, t_max: f32) -> bool {
        let mut t_min_int = t_min; // int = intersection
        let mut t_max_int = t_max;
        for a in 0..3 {
            let inv_d = 1.0 / ray.direction[a];
            let mut t0 = ((self.min)[a] - ray.origin[a]) * inv_d;
            let mut t1 = ((self.max)[a] - ray.origin[a]) * inv_d;
            if inv_d < 0.0 {
                std::mem::swap(&mut t0, &mut t1);
            }
            t_min_int = f32::max(t_min_int, t0);
            t_max_int = f32::min(t_max_int, t1);
            if t_min_int > t_max_int {
                // 厚さ0のAAbbとはヒットする
                return false;
            }
        }
        return true;
    }
    pub fn unite(lhs: &Aabb, rhs: &Aabb) -> Aabb {
        Aabb::new(&min_vec3(&lhs.min, &rhs.min), &max_vec3(&lhs.max, &rhs.max))
    }
    pub fn append_point(&mut self, point: Vec3) {
        self.min = min_vec3(&self.min, &point);
        self.max = max_vec3(&self.max, &point);
    }
    pub fn from_points(pts: Vec<Vec3>) -> Self {
        let mut ret = Self::empty();
        for pt in pts {
            ret.append_point(pt);
        }
        ret
    }
    pub fn get_transformed(&self, aff: &Affine) -> Aabb {
        if self.is_empty() {
            Self::empty()
        } else {
            Self::from_points(
                self.vertices()
                    .into_iter()
                    .map(|ref p| aff.act_point(p))
                    .collect(),
            )
        }
    }
    pub fn vertices(&self) -> Vec<Vec3> {
        let self_min_max: [&Vec3; 2] = [&self.min, &self.max];
        iproduct!(0..2, 0..2, 0..2)
            .map(|(ix, iy, iz)| {
                Vec3::new(
                    self_min_max[ix][0],
                    self_min_max[iy][1],
                    self_min_max[iz][2],
                )
            })
            .collect()
    }
    pub fn area(&self) -> f32 {
        if self.is_empty() {
            return 0.0;
        }
        let diff = self.max - self.min;
        debug_assert!(diff[0] >= 0.0 && diff[1] >= 0.0 && diff[2] >= 0.0);
        2.0 * (diff[0] * diff[1] + diff[1] * diff[2] + diff[2] * diff[0])
    }
    pub fn compare_center(&self, rhs: &Aabb, axis: usize) -> std::cmp::Ordering {
        debug_assert!(!self.is_empty() && !rhs.is_empty());
        let lhs_center: f32 = 0.5 * (self.min + self.max)[axis as usize];
        let rhs_center: f32 = 0.5 * (rhs.min + rhs.max)[axis as usize];
        lhs_center
            .partial_cmp(&rhs_center)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}
