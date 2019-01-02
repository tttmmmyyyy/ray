use crate::aabb::Aabb;
use crate::hit_record::HitRecord;
use crate::hitable::empty::Empty;
use crate::hitable::hitable_list::HitableList;
use crate::hitable::Hitable;
use crate::ray::Ray;
use std::sync::Arc;

/// Only used in OBVH::from_bvh_node.
#[derive(Clone)]
pub enum BvhNodeConstructionRecord {
    Inner { ptr: Arc<BvhNode>, bbox: Aabb },
    Leaf { ptr: Arc<dyn Hitable>, bbox: Aabb }, // must not empty
}

impl Default for BvhNodeConstructionRecord {
    fn default() -> Self {
        Self::empty()
    }
}

impl BvhNodeConstructionRecord {
    pub fn empty() -> Self {
        BvhNodeConstructionRecord::Leaf {
            ptr: Arc::new(Empty::new()),
            bbox: Aabb::empty(),
        }
    }
    pub fn leaf(ptr: Arc<Hitable>, time_0: f32, time_1: f32) -> Self {
        BvhNodeConstructionRecord::Leaf {
            bbox: ptr.bounding_box(time_0, time_1).unwrap(),
            ptr: ptr,
        }
    }
    pub fn inner(ptr: Arc<BvhNode>) -> Self {
        BvhNodeConstructionRecord::Inner {
            bbox: ptr.aabb,
            ptr: ptr,
        }
    }
    pub fn bbox(&self) -> Aabb {
        match self {
            BvhNodeConstructionRecord::Inner { ptr: _, bbox: bbox } => *bbox,
            BvhNodeConstructionRecord::Leaf { ptr: _, bbox: bbox } => *bbox,
        }
    }
}

/// Bouding Volume Hierarchy node
pub struct BvhNode {
    pub left: Arc<Hitable>,
    pub right: Arc<Hitable>,
    pub axis: usize, // the index (0,1,2) of splitting plane // ToDo u8にする
    pub aabb: Aabb,
    pub left_node_record: BvhNodeConstructionRecord,
    pub right_node_record: BvhNodeConstructionRecord,
}

impl BvhNode {
    /// Constructor
    /// time_0, time_1: used for moving hitables.
    /// list: each element should have non-None bounding box
    pub fn new(mut list: Vec<Arc<Hitable>>, time_0: f32, time_1: f32) -> Self {
        if list.is_empty() {
            return BvhNode {
                left: Arc::new(Empty::new()),
                left_node_record: BvhNodeConstructionRecord::empty(),
                right: Arc::new(Empty::new()),
                right_node_record: BvhNodeConstructionRecord::empty(),
                axis: 0,
                aabb: Aabb::empty(),
            };
        } else if list.len() == 1 {
            let head = list.pop().unwrap();
            return BvhNode {
                left: head.clone(),
                left_node_record: BvhNodeConstructionRecord::leaf(head.clone(), time_0, time_1),
                right: Arc::new(Empty::new()),
                right_node_record: BvhNodeConstructionRecord::empty(),
                axis: 0,
                aabb: head.bounding_box(time_0, time_1).unwrap(),
            };
        }
        let mut bboxes = list
            .iter()
            .map(|h| h.bounding_box(time_0, time_1).unwrap())
            .collect();
        let (axis, idx) = Self::search_splitting_axis_index(&mut bboxes);
        let (left, left_rec, right, right_rec): (
            Arc<Hitable>,
            BvhNodeConstructionRecord,
            Arc<Hitable>,
            BvhNodeConstructionRecord,
        ) = if idx == 0 {
            // if (left, right) = (empty, whole) achieves min cost,
            debug_assert!(list.len() >= 2);
            let left_node = Arc::new(HitableList::new(list));
            (
                left_node.clone(),
                BvhNodeConstructionRecord::leaf(left_node, time_0, time_1),
                Arc::new(Empty::new()),
                BvhNodeConstructionRecord::empty(),
            )
        } else {
            Self::sort_hitables_center(&mut list, axis, time_0, time_1);
            let right_list = list.split_off(idx);
            let left = Arc::new(BvhNode::new(list, time_0, time_1));
            let right = Arc::new(BvhNode::new(right_list, time_0, time_1));
            (
                left.clone(),
                BvhNodeConstructionRecord::inner(left),
                right.clone(),
                BvhNodeConstructionRecord::inner(right),
            )
        };
        let left_box = left.bounding_box(time_0, time_1).unwrap();
        let right_box = right.bounding_box(time_0, time_1).unwrap();
        let self_box = Aabb::unite(&left_box, &right_box);
        BvhNode {
            left: left,
            left_node_record: left_rec,
            right: right,
            right_node_record: right_rec,
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
        let mut min_cost = std::f32::MAX;
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
    fn calc_consecutive_bboxes_united_areas(list: &Vec<Aabb>) -> Vec<(f32, f32)> {
        let mut areas = Vec::<(f32, f32)>::new();
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
    fn sort_hitables_center(list: &mut Vec<Arc<Hitable>>, axis: usize, time_0: f32, time_1: f32) {
        list.sort_unstable_by(|a, b| {
            let a_box = a.bounding_box(time_0, time_1).unwrap(); // expect panic if None
            let b_box = b.bounding_box(time_0, time_1).unwrap();
            a_box.compare_center(&b_box, axis)
        });
    }
    /// Cost of traversing into a group according to SAH = Surface Area Heuristics, i.e.,
    /// (surface area of bbox) * (# of primitives)
    fn cost_sah(box_surface_area: f32, primitives: i32) -> f32 {
        box_surface_area * primitives as f32
    }
    pub fn is_both_node_empty(&self) -> bool {
        self.left_node_record.bbox().is_empty() && self.right_node_record.bbox().is_empty()
    }
    pub fn singleton_node(&self) -> Option<Arc<Hitable>> {
        if self.is_both_node_empty() {
            return None;
        }
        if !self.left_node_record.bbox().is_empty() && !self.right_node_record.bbox().is_empty() {
            return None;
        }
        if self.left_node_record.bbox().is_empty() {
            Some(self.right.clone())
        } else {
            Some(self.left.clone())
        }
    }
}

impl Hitable for BvhNode {
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, t_max: f32) -> Option<HitRecord<'s>> {
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
    fn bounding_box(&self, _t0: f32, _t1: f32) -> Option<Aabb> {
        Some(self.aabb)
    }
}
