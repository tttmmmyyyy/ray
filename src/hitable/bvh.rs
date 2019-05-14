use crate::aabb::Aabb;
use crate::hit_record::HitRecord;
use crate::hitable::bvh_node::BvhNode;
use crate::hitable::node_pointer::NodePointer;
use crate::hitable::Hitable;
use crate::ray::Ray;

// ToDo: OBVHから参照するためにpubにしている。
pub struct Node {
    pub bboxes: [Aabb; 2],
    pub children: [NodePointer; 2],
    pub axis: u8,
}

impl Default for Node {
    fn default() -> Self {
        Node {
            bboxes: [Aabb::empty(), Aabb::empty()],
            children: [NodePointer::empty_leaf(), NodePointer::empty_leaf()],
            axis: 0,
        }
    }
}

// ToDo: OBVHから参照するためにpubにしている。
pub struct BVH<L> {
    pub leaves: Vec<L>,    // leaf-nodes.
    pub inners: Vec<Node>, // inner nodes. inners[0] is the root node.
    pub bbox: Aabb,
}

impl<L> BVH<L>
where
    L: Hitable,
{
    pub fn new(leaves: Vec<L>, time_0: f32, time_1: f32) -> Self {
        let bboxes: Vec<Aabb> = leaves
            .iter()
            .map(|leaf| leaf.bounding_box(time_0, time_1).unwrap())
            .collect();
        let idx_bboxes: Vec<(usize, Aabb)> = bboxes
            .iter()
            .enumerate()
            .map(|(idx, ref_box)| (idx, *ref_box))
            .collect();
        let bbox = bboxes
            .iter()
            .fold(Aabb::empty(), |accum, aabb| Aabb::unite(&accum, &aabb));
        let mut inners = Vec::<Node>::default();
        Self::construct(idx_bboxes, &mut inners);
        Self {
            leaves,
            inners,
            bbox,
        }
    }
    fn sort_by_center(list: &mut Vec<(usize, Aabb)>, axis: usize) {
        list.sort_unstable_by(|a, b| {
            let a_box = a.1; // expect panic if None
            let b_box = b.1;
            a_box.compare_center(&b_box, axis)
        });
    }
    /// 再帰的にBVH木を構築する
    /// * nodes - 出力変数
    /// * leaves - (葉Hitableのインデックス, そのバウンディングボックス)の配列
    /// * return - このconstructコールで構築された木のルートノードのnodes配列におけるインデックス
    fn construct(mut leaves: Vec<(usize, Aabb)>, nodes: &mut Vec<Node>) -> usize {
        fn bboxes_from_leaves(leaves: &Vec<(usize, Aabb)>) -> Vec<Aabb> {
            leaves.iter().map(|leaf| leaf.1).collect()
        }
        fn bbox_from_leaves(leaves: &Vec<(usize, Aabb)>) -> Aabb {
            bboxes_from_leaves(leaves)
                .iter()
                .fold(Aabb::empty(), |accum, aabb| Aabb::unite(&accum, &aabb))
        }
        nodes.push(Node::default());
        let new_node_idx = nodes.len() - 1;
        if leaves.is_empty() {
            nodes[new_node_idx] = Node {
                bboxes: [Aabb::empty(), Aabb::empty()],
                children: [NodePointer::empty_leaf(), NodePointer::empty_leaf()],
                axis: 0,
            };
        } else if leaves.len() == 1 {
            let head = leaves.pop().unwrap();
            nodes[new_node_idx] = Node {
                bboxes: [head.1, Aabb::empty()],
                children: [NodePointer::new_leaf(head.0), NodePointer::empty_leaf()],
                axis: 0,
            };
        } else if leaves.len() == 2 {
            nodes[new_node_idx] = Node {
                bboxes: [leaves[0].1, leaves[1].1],
                children: [
                    NodePointer::new_leaf(leaves[0].0),
                    NodePointer::new_leaf(leaves[1].0),
                ],
                axis: 0,
            };
        } else {
            let mut bboxes = bboxes_from_leaves(&leaves);
            let (mut axis, mut idx) = BvhNode::search_splitting_axis_index(&mut bboxes);
            if idx == 0 {
                // if (left, right) = (empty, whole) achieves min cost,
                debug_assert!(leaves.len() >= 2);
                idx = leaves.len() / 2;
                // idx = 1でもよい。teapotでのテストもmengerでのテストも同じ時間かかっていた。
                axis = 0;
            }
            Self::sort_by_center(&mut leaves, axis);
            let right_leaves = leaves.split_off(idx);
            let left_bbox = bbox_from_leaves(&leaves);
            let right_bbox = bbox_from_leaves(&right_leaves);
            let left_node_idx = Self::construct(leaves, nodes);
            let right_node_idx = Self::construct(right_leaves, nodes);
            nodes[new_node_idx] = Node {
                bboxes: [left_bbox, right_bbox],
                children: [
                    NodePointer::new_inner(left_node_idx),
                    NodePointer::new_inner(right_node_idx),
                ],
                axis: axis as u8,
            };
        }
        new_node_idx
    }
    fn hit_core<'s, 'r>(
        &'s self,
        bbox: &Aabb,
        node_ptr: NodePointer,
        ray: &'r Ray,
        t_min: f32,
        t_max: f32,
    ) -> Option<HitRecord<'s>> {
        if !bbox.hit(ray, t_min, t_max) {
            None
        } else if node_ptr.is_leaf() {
            if node_ptr.is_empty_leaf() {
                None
            } else {
                self.leaves[node_ptr.index()].hit(ray, t_min, t_max)
            }
        } else {
            let node = &self.inners[node_ptr.index()];
            let prior_idx = (ray.direction[node.axis as usize] < 0.0) as usize;
            if let Some(ref hit_left) = self.hit_core(
                &node.bboxes[prior_idx],
                node.children[prior_idx],
                ray,
                t_min,
                t_max,
            ) {
                if let Some(ref hit_right) = self.hit_core(
                    &node.bboxes[1 - prior_idx],
                    node.children[1 - prior_idx],
                    ray,
                    t_min,
                    hit_left.t,
                ) {
                    Some(*hit_right)
                } else {
                    Some(*hit_left)
                }
            } else {
                self.hit_core(
                    &node.bboxes[1 - prior_idx],
                    node.children[1 - prior_idx],
                    ray,
                    t_min,
                    t_max,
                )
            }
        }
    }
}

impl<L> Hitable for BVH<L>
where
    L: Hitable,
{
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, t_max: f32) -> Option<HitRecord<'s>> {
        self.hit_core(&self.bbox, NodePointer::root(), ray, t_min, t_max)
    }
    // ToDo: 効率的なis_hitを実装する
    fn bounding_box(&self, _t0: f32, _t1: f32) -> Option<Aabb> {
        Some(self.bbox)
    }
}
