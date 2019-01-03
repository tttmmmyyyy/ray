use crate::affine::Affine;
use crate::aliases::{Vec2, Vec3};
use crate::ray::Ray;
use crate::util::{compare_vec2_le, compare_vec3_le};
use crate::util::{max_vec2, max_vec3, min_vec2, min_vec3};
use itertools::iproduct;
use std;
use std::default;
use std::fmt;

/// Axis-Aligned Bounding Box
#[derive(Clone, Copy)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl fmt::Display for Aabb {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        use crate::util::pretty_print_f32;
        write!(
            f,
            "(min: ({},{},{}), max: ({},{},{}))",
            pretty_print_f32(self.min[0]),
            pretty_print_f32(self.min[1]),
            pretty_print_f32(self.min[2]),
            pretty_print_f32(self.max[0]),
            pretty_print_f32(self.max[1]),
            pretty_print_f32(self.max[2])
        )
    }
}

impl fmt::Debug for Aabb {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        use crate::util::pretty_print_f32;
        write!(
            f,
            "(min: ({},{},{}), max: ({},{},{})), center: ({},{},{})",
            pretty_print_f32(self.min[0]),
            pretty_print_f32(self.min[1]),
            pretty_print_f32(self.min[2]),
            pretty_print_f32(self.max[0]),
            pretty_print_f32(self.max[1]),
            pretty_print_f32(self.max[2]),
            pretty_print_f32(self.center()[0]),
            pretty_print_f32(self.center()[1]),
            pretty_print_f32(self.center()[2])
        )
    }
}

impl default::Default for Aabb {
    fn default() -> Self {
        Aabb::empty()
    }
}

impl Aabb {
    pub fn new(min: &Vec3, max: &Vec3) -> Self {
        debug_assert!(
            compare_vec3_le(&min, &max)
                || (*min == Vec3::new(std::f32::INFINITY, std::f32::INFINITY, std::f32::INFINITY)
                    && *max
                        == Vec3::new(
                            std::f32::NEG_INFINITY,
                            std::f32::NEG_INFINITY,
                            std::f32::NEG_INFINITY,
                        ))
        );
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
    pub fn center(&self) -> Vec3 {
        debug_assert!(!self.is_empty());
        0.5 * (self.min + self.max)
    }
    pub fn intersect(&self, rhs: &Aabb) -> Aabb {
        let mn = max_vec3(&self.min, &rhs.min);
        let mx = min_vec3(&self.max, &rhs.max);
        // Node: operator <= is 'using column-major lexicographic ordering.', so inappropriate
        if compare_vec3_le(&mn, &mx) {
            Aabb::new(&mn, &mx)
        } else {
            Aabb::empty()
        }
    }
    /// Gets an axis (0,1,2) along which this aabb is shortest.
    /// * `return` - if self is empty(), returns 0
    pub fn shortest_axis(&self) -> u8 {
        if self.is_empty() {
            0
        } else {
            let diag = self.max - self.min;
            let mut min_axis = 0u8;
            let mut min_val = std::f32::INFINITY;
            for a in 0..3 {
                if diag[a] < min_val {
                    min_val = diag[a];
                    min_axis = a as u8;
                }
            }
            min_axis
        }
    }
    pub fn most_separating_axis(&self, rhs: &Aabb) -> u8 {
        if self.is_empty() || rhs.is_empty() {
            return 0;
        }
        let int_box = self.intersect(rhs);
        if int_box.is_empty() {
            let mut max_overlap = std::f32::NEG_INFINITY;
            let mut best_axis = 0;
            for a in 0..3 {
                let overlap = self.project(a).intersect(&rhs.project(a)).area();
                if overlap > max_overlap {
                    max_overlap = overlap;
                    best_axis = a;
                }
            }
            best_axis
        } else {
            int_box.shortest_axis()
        }
    }
    pub fn project(&self, axis: u8) -> Aabb2 {
        let mut indices: Vec<usize> = vec![];
        for a in 0..3 {
            if a == axis {
                continue;
            } else {
                indices.push(a as usize);
            }
        }
        Aabb2::new(
            &Vec2::new(self.min[indices[0]], self.min[indices[1]]),
            &Vec2::new(self.max[indices[0]], self.max[indices[1]]),
        )
    }
}

/// Two-dimensional axis-aligned bounding box.
pub struct Aabb2 {
    pub min: Vec2,
    pub max: Vec2,
}

impl Aabb2 {
    pub fn new(min: &Vec2, max: &Vec2) -> Self {
        debug_assert!(
            compare_vec2_le(&min, &max)
                || (*min == Vec2::new(std::f32::INFINITY, std::f32::INFINITY)
                    && *max == Vec2::new(std::f32::NEG_INFINITY, std::f32::NEG_INFINITY))
        );
        Aabb2 {
            min: *min,
            max: *max,
        }
    }
    pub fn empty() -> Self {
        Aabb2::new(
            &Vec2::new(std::f32::INFINITY, std::f32::INFINITY),
            &Vec2::new(std::f32::NEG_INFINITY, std::f32::NEG_INFINITY),
        )
    }
    pub fn is_empty(&self) -> bool {
        self.min == Vec2::new(std::f32::INFINITY, std::f32::INFINITY)
            && self.max == Vec2::new(std::f32::NEG_INFINITY, std::f32::NEG_INFINITY)
    }
    pub fn area(&self) -> f32 {
        if self.is_empty() {
            0.0
        } else {
            let diag = self.max - self.min;
            diag[0] * diag[1]
        }
    }
    pub fn intersect(&self, rhs: &Aabb2) -> Aabb2 {
        let mn = max_vec2(&self.min, &rhs.min);
        let mx = min_vec2(&self.max, &rhs.max);
        if compare_vec2_le(&mn, &mx) {
            Aabb2::new(&mn, &mx)
        } else {
            Aabb2::empty()
        }
    }
}
