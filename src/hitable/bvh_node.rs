use crate::aabb::Aabb;
use crate::hit_record::HitRecord;
use crate::hitable::empty::Empty;
use crate::hitable::hitable_list::HitableList;
use crate::hitable::Hitable;
use crate::ray::Ray;
use rand::StdRng;
use std::sync::Arc;

/// Bouding Volume Hierarchy node
pub struct BvhNode {
    left: Arc<Hitable>,
    right: Arc<Hitable>,
    axis: usize, // the index (0,1,2) of splitting plane
    aabb: Aabb,
}

impl BvhNode {
    /// Constructor
    /// time_0, time_1: used for moving hitables.
    /// list: each element should have non-None bounding box
    pub fn new(mut list: Vec<Arc<Hitable>>, time_0: f64, time_1: f64, rng: &mut StdRng) -> Self {
        if list.is_empty() {
            return BvhNode {
                left: Arc::new(Empty::new()),
                right: Arc::new(Empty::new()),
                axis: 0,
                aabb: Aabb::empty(),
            };
        } else if list.len() == 1 {
            let head = list.pop().unwrap();
            let head_box = head.bounding_box(time_0, time_1).unwrap();
            return BvhNode {
                left: head,
                right: Arc::new(Empty::new()),
                axis: 0,
                aabb: head_box,
            };
        }
        let mut bboxes = list
            .iter()
            .map(|h| h.bounding_box(time_0, time_1).unwrap())
            .collect();
        let (axis, idx) = Self::search_splitting_axis_index(&mut bboxes);
        let (left, right): (Arc<Hitable>, Arc<Hitable>) = if idx == 0 {
            // if (left, right) = (empty, whole) achieves min cost,
            debug_assert!(list.len() >= 2);
            (Arc::new(HitableList::new(list)), Arc::new(Empty::new()))
        } else {
            Self::sort_hitables_center(&mut list, axis, time_0, time_1);
            let former = list.split_off(idx);
            (
                Arc::new(BvhNode::new(former, time_0, time_1, rng)),
                Arc::new(BvhNode::new(list, time_0, time_1, rng)),
            )
        };
        let left_box = left.bounding_box(time_0, time_1).unwrap();
        let right_box = right.bounding_box(time_0, time_1).unwrap();
        let self_box = Aabb::unite(&left_box, &right_box);
        BvhNode {
            left: left,
            right: right,
            aabb: self_box,
            axis: axis,
        }
    }
    /// returns (axis, index).
    fn search_splitting_axis_index(
        bboxes: &mut Vec<Aabb> // sorted along axis after called.
    ) -> (usize, usize) {
        let mut min_axis = 0;
        let mut min_idx = 0;
        let mut min_cost = std::f64::MAX;
        for axis in 0..3 {
            bboxes.sort_unstable_by(|a, b| a.compare_center(b, axis));
            let areas = Self::calc_consecutive_bboxes_united_areas(&bboxes);
            for i in 0..bboxes.len() {
                let cost = Self::cost_sah(areas[i].0, i as i32)
                    + Self::cost_sah(areas[i].1, bboxes.len() as i32 - i as i32);
                if cost < min_cost {
                    min_axis = axis;
                    min_idx = i;
                    min_cost = cost;
                }
            }
        }
        debug_assert!(min_idx != std::usize::MAX);
        debug_assert!(min_idx != bboxes.len()); // in case that this achieves min_cost,
                                                // min_idx should be 0 instead of bboxes.len()
        (min_axis, min_idx)
    }
    /// For example, if list = [a,b,c], then it returns
    /// [(|0|,|a+b+c|), (|a|,|b+c|), (|a+b|,|c|), (|a+b+c|,|0|)]
    /// where + is the unite operator, and |x| is the surface area of x.
    fn calc_consecutive_bboxes_united_areas(list: &Vec<Aabb>) -> Vec<(f64, f64)> {
        let mut areas = Vec::<(f64, f64)>::new();
        areas.resize(list.len() + 1, (0.0, 0.0));
        let mut bbox_f = Aabb::empty();
        let mut bbox_b = Aabb::empty();
        areas[0].0 = 0.0;
        areas[list.len()].1 = 0.0;
        for i in 0..list.len() {
            bbox_f = Aabb::unite(&bbox_f, &list[i]);
            bbox_b = Aabb::unite(&bbox_b, &list[list.len() - i - 1]);
            areas[i + 1].0 = bbox_f.area();
            areas[list.len() - (i + 1)].1 = bbox_b.area();
        }
        areas
    }
    fn sort_hitables_center(list: &mut Vec<Arc<Hitable>>, axis: usize, time_0: f64, time_1: f64) {
        list.sort_unstable_by(|a, b| {
            let a_box = a.bounding_box(time_0, time_1).unwrap(); // expect panic if None
            let b_box = b.bounding_box(time_0, time_1).unwrap();
            a_box.compare_center(&b_box, axis)
        });
    }
    /// Cost of traversing into a group according to SAH = Surface Area Heuristics, i.e.,
    /// (surface area of bbox) * (# of primitives)
    fn cost_sah(box_surface_area: f64, primitives: i32) -> f64 {
        box_surface_area * primitives as f64
    }
}

impl Hitable for BvhNode {
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f64, t_max: f64) -> Option<HitRecord<'s>> {
        if !self.aabb.hit(ray, t_min, t_max) {
            return None;
        }
        // decide the order of traverse.
        let (first, second) = if ray.direction[self.axis] >= 0.0 {
            (&self.left, &self.right)
        } else {
            (&self.right, &self.left)
        };
        if let Some(ref hit_left) = first.hit(ray, t_min, t_max) {
            if let Some(ref hit_right) = second.hit(ray, t_min, hit_left.t) {
                Some(*hit_right)
            } else {
                Some(*hit_left)
            }
        } else {
            second.hit(ray, t_min, t_max)
        }
    }
    fn bounding_box(&self, _t0: f64, _t1: f64) -> Option<Aabb> {
        Some(self.aabb)
    }
}
