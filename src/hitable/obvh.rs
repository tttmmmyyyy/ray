use crate::aabb::Aabb;
use crate::aliases::{RandGen, Vec3};
use crate::hit_record::HitRecord;
use crate::hitable::bvh_node::{BvhNode, BvhNodeConstructionRecord};
use crate::hitable::Hitable;
use crate::ray::Ray;
#[cfg(target_arch = "x86")]
use std::arch::x86::*;
#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::*;
use std::ops::Shl;
use std::ops::Shr;
use std::sync::Arc;
use std::time::{Duration, Instant};

struct RayAVXInfo {
    origin: [__m256; 3],  // origin[axis_idx] = 8 copies of ray.origin[axis_idx]
    inv_dir: [__m256; 3], // origin[axis_idx] = 8 copies of 1.0 / ray.direction[axis_idx]
    dir_sign: [u8; 3], // dir_sign[axis_idx] = 0 (resp. 1) if ray.direction[axis_idx] >= 0 (resp. < 0)
}

impl RayAVXInfo {
    fn from_ray(ray: &Ray) -> Self {
        let calc_sign = |axis| (ray.direction[axis] < 0.0) as u8;
        unsafe {
            Self {
                origin: [
                    _mm256_set1_ps(ray.origin[0]),
                    _mm256_set1_ps(ray.origin[1]),
                    _mm256_set1_ps(ray.origin[2]),
                ],
                inv_dir: [
                    _mm256_set1_ps(1.0 / ray.direction[0]),
                    _mm256_set1_ps(1.0 / ray.direction[1]),
                    _mm256_set1_ps(1.0 / ray.direction[2]),
                ],
                dir_sign: [calc_sign(0), calc_sign(1), calc_sign(2)],
            }
        }
    }
}

/// Octa Bounding Volume Hierarchy
pub struct OBVH {
    leaves: Vec<Arc<Hitable>>, // leaf-nodes.
    inners: Vec<Node>,         // inner nodes. inners[0] is the root node.
}

struct Node {
    // Each of 8 children is associated with a number called 'child_id' which is in [0..8).
    // A Node is of an OBVH tree can be seen as a binary tree of depth 3, and in this perspective
    // child_id & (1 << 0) != 0 <=> "Is this children in the left-half of the whole tree (of depth 3)?"
    // child_id & (1 << 1) != 0 <=> "Is this children in the left-half of the next subtree (of depth 2)?"
    // child_id & (1 << 2) != 0 <=> "Is this children in the left-half of the minimal subtree (of depth 1)?"
    //
    // When identifying __m256 and [f32; 8] (i.e., talking as if bboxes has type [[[f32; 8]; 3]; 2]),
    // then bboxes[min_or_max][axis][child_id] = min[0] or max[1] coordinate value
    // along the axis of the bbox of the child.
    // ToDo: child_id の順番についてはかなり慎重にならなくてはならない。エンディアンとか気にするべきか？
    bboxes: [[__m256; 3]; 2],
    children: [NodePointer; 8],
    axis_top: u8, // ToDo: add an explanation comment
    axis_child: [u8; 2],
    axis_grand_son: [u8; 4],
}

/// A "pointer" to a Node
#[derive(Clone, Copy)]
struct NodePointer {
    info: u32,
}

impl NodePointer {
    pub fn root() -> Self {
        NodePointer { info: 0 }
    }
    pub fn index(&self) -> usize {
        debug_assert!(!self.is_empty_leaf());
        (self.info & !(1u32.shl(31) as u32)) as usize
    }
    pub fn is_leaf(&self) -> bool {
        self.info & 1u32.shl(31) != 0u32
    }
    pub fn is_empty_leaf(&self) -> bool {
        self.info == std::u32::MAX
    }
    pub fn empty_leaf() -> Self {
        Self {
            info: std::u32::MAX,
        }
    }
    #[allow(dead_code)]
    pub fn is_inner(&self) -> bool {
        self.info & 1u32.shl(31) == 0u32
    }
    pub fn new(is_leaf: bool, index: usize) -> Self {
        debug_assert!(index < std::u32::MAX.shr(1) as usize);
        let mut info = index as u32;
        if is_leaf {
            info = info | 1u32.shl(31);
        }
        NodePointer { info: info }
    }
}

fn duration_to_secs(dur: &Duration) -> f64 {
    dur.as_secs() as f64 + dur.subsec_millis() as f64 * 0.001 + dur.subsec_nanos() as f64 * 0.000001
}

// ToDo: 再帰関数はインライン展開されないのではないか。手動インライン展開するとよいかも。
impl Hitable for OBVH {
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, mut t_max: f32) -> Option<HitRecord<'s>> {
        let ray_avx = RayAVXInfo::from_ray(ray);
        let mut node_stack: Vec<NodePointer> = vec![]; // ToDo: reserve.
        const NODE_STACK_UPPER_BOUND: usize = 32;
        node_stack.reserve(NODE_STACK_UPPER_BOUND);
        node_stack.push(NodePointer::root());
        let mut hit_record: Option<HitRecord<'s>> = None;
        while !node_stack.is_empty() {
            let node_ptr = node_stack.pop().unwrap();
            if node_ptr.is_leaf() {
                // if a leaf node,
                if node_ptr.is_empty_leaf() {
                    continue;
                }
                HitRecord::replace_to_some_min(
                    &mut hit_record,
                    &self.leaves[node_ptr.index()].hit(ray, t_min, t_max),
                );
                hit_record.map(|ref hr| {
                    t_max = hr.t;
                });
                debug_assert!(t_min <= t_max);
            } else {
                // if an inner node,
                let node = &self.inners[node_ptr.index()];
                let hit_bits = node.hit(&ray_avx, t_min, t_max);
                node.push_node_stack(&mut node_stack, &ray_avx, hit_bits);
                // println!("{} {}", node_stack.len(), node_ptr.index()); // ToDo: remove
                debug_assert!(node_stack.len() <= NODE_STACK_UPPER_BOUND); // ToDo: debug_assert
            }
        }
        hit_record
    }
    fn bounding_box(&self, _time_0: f32, _time_1: f32) -> Option<Aabb> {
        unimplemented!()
    }
    fn random_direction_from(&self, _origin: &Vec3, _rng: &mut RandGen) -> Vec3 {
        panic!("random_direction_from called for OBHV");
    }
    fn direction_density(&self, _origin: &Vec3, _dir: &Vec3) -> f32 {
        panic!("direction_density called for OBHV");
    }
}

impl Node {
    /// ToDo: write a comment on the return value.
    unsafe fn hit_core(&self, ray: &RayAVXInfo, t_min: f32, t_max: f32) -> i32 {
        let mut t_min = _mm256_set1_ps(t_min);
        let mut t_max = _mm256_set1_ps(t_max);
        for a in 0..3 {
            // a = Axis
            t_min = _mm256_max_ps(
                t_min,
                _mm256_mul_ps(
                    ray.inv_dir[a],
                    _mm256_sub_ps(self.bboxes[ray.dir_sign[a] as usize][a], ray.origin[a]),
                ),
            );
            t_max = _mm256_min_ps(
                t_max,
                _mm256_mul_ps(
                    ray.inv_dir[a],
                    _mm256_sub_ps(self.bboxes[1 - ray.dir_sign[a] as usize][a], ray.origin[a]),
                ),
            );
        }
        _mm256_movemask_ps(_mm256_cmp_ps(
            t_min, t_max, _CMP_LE_OS, /* Ordered, Signaling. */
        ))
    }
    fn hit(&self, ray: &RayAVXInfo, t_min: f32, t_max: f32) -> i32 {
        unsafe { self.hit_core(ray, t_min, t_max) }
    }
    fn push_node_stack(&self, node_stack: &mut Vec<NodePointer>, ray: &RayAVXInfo, hit_bits: i32) {
        self.push_node_stack_core(node_stack, ray, hit_bits, 0, 0)
    }
    fn push_node_stack_core(
        &self,
        node_stack: &mut Vec<NodePointer>,
        ray: &RayAVXInfo,
        hit_bits: i32,
        depth: i32,
        child_id: usize,
    ) {
        debug_assert!(depth <= 3);
        if depth <= 2 {
            let axis = if depth == 0 {
                self.axis_top
            } else if depth == 1 {
                self.axis_child[child_id]
            } else {
                self.axis_grand_son[child_id]
            };
            let (mut fst, mut snd) = (child_id, child_id | 1usize.shl(depth));
            if ray.dir_sign[axis as usize] > 0 {
                // if ray is pointing negative,
                std::mem::swap(&mut fst, &mut snd)
            }
            self.push_node_stack_core(node_stack, ray, hit_bits, depth + 1, fst);
            self.push_node_stack_core(node_stack, ray, hit_bits, depth + 1, snd);
        } else {
            debug_assert!(child_id < 8);
            if hit_bits & (1i32.shl(child_id)) == 0 {
                // if this node does not hit the ray
                return;
            }
            node_stack.push(self.children[child_id]);
        }
    }
    fn empty() -> Self {
        unsafe {
            Node {
                bboxes: [
                    [_mm256_set1_ps(std::f32::MAX); 3],
                    [_mm256_set1_ps(std::f32::MIN); 3],
                ],
                children: [NodePointer::empty_leaf(); 8],
                axis_top: 0u8,
                axis_child: [0u8; 2],
                axis_grand_son: [0u8; 4],
            }
        }
    }
    pub fn from_bvh_node(
        bvh_node: &BvhNodeConstructionRecord,
    ) -> (Self, [BvhNodeConstructionRecord; 8]) {
        let mut children: [BvhNodeConstructionRecord; 8] = Default::default();
        let mut bboxes = [[[std::f32::MAX; 8]; 3], [[std::f32::MIN; 8]; 3]];
        let mut this = Node::empty();
        Self::from_bvh_node_traverse(&mut this, &mut bboxes, &mut children, bvh_node, 0, 0);
        this.bboxes = Self::convert_bboxes(&bboxes);
        (this, children)
    }
    // ToDo-research: 上のfrom_bvh_node関数で let mut bboxes = Self::empty_bboxes_array_layout(); とすると、
    // リリースビルドのときのみSegmentation faultが発生する。
    #[allow(dead_code)]
    fn empty_bboxes_array_layout() -> [[[f32; 8]; 3]; 2] {
        [[[std::f32::MAX; 8]; 3], [[std::f32::MIN; 8]; 3]]
    }
    fn from_bvh_node_traverse(
        &mut self,
        bboxes: &mut [[[f32; 8]; 3]; 2],
        children: &mut [BvhNodeConstructionRecord; 8],
        current: &BvhNodeConstructionRecord,
        depth: usize,
        child_id: u8,
    ) {
        debug_assert!(depth <= 3);
        if depth == 3 {
            Self::set_bboxes_array_layout(bboxes, child_id, &current.bbox());
            children[child_id as usize] = current.clone();
        } else {
            match current {
                BvhNodeConstructionRecord::Leaf { ptr: _, bbox: _ } => {
                    Self::set_bboxes_array_layout(bboxes, child_id, &current.bbox());
                    children[child_id as usize] = current.clone();
                }
                BvhNodeConstructionRecord::Inner { ptr, bbox: _ } => {
                    if depth == 0 {
                        self.axis_top = ptr.axis as u8;
                    } else if depth == 1 {
                        debug_assert!(child_id <= 2);
                        self.axis_child[child_id as usize] = ptr.axis as u8;
                    } else if depth == 2 {
                        debug_assert!(child_id <= 4);
                        self.axis_grand_son[child_id as usize] = ptr.axis as u8;
                    }
                    Self::from_bvh_node_traverse(
                        self,
                        bboxes,
                        children,
                        &ptr.left_node_record,
                        depth + 1,
                        child_id | 1u8.shl(depth),
                    );
                    Self::from_bvh_node_traverse(
                        self,
                        bboxes,
                        children,
                        &ptr.right_node_record,
                        depth + 1,
                        child_id & !1u8.shl(depth),
                    );
                }
            }
        }
    }
    fn set_bboxes_array_layout(bboxes: &mut [[[f32; 8]; 3]; 2], child_id: u8, bbox: &Aabb) {
        for min_max in 0..2 {
            for axis in 0..3 {
                if min_max == 0 {
                    bboxes[min_max][axis][child_id as usize] = bbox.min[axis];
                } else {
                    bboxes[min_max][axis][child_id as usize] = bbox.max[axis];
                }
            }
        }
    }
    fn convert_bboxes(src: &[[[f32; 8]; 3]; 2]) -> [[__m256; 3]; 2] {
        unsafe {
            let mut res = [
                [_mm256_set1_ps(std::f32::MAX); 3],
                [_mm256_set1_ps(std::f32::MIN); 3],
            ];
            for min_max in 0..2 {
                for axis in 0..3 {
                    res[min_max][axis] = _mm256_load_ps(src[min_max][axis].as_ptr());
                }
            }
            res
        }
    }
}

impl OBVH {
    pub fn from_bvh_node(bvh_node: Arc<BvhNode>) -> Self {
        let mut leaves: Vec<Arc<Hitable>> = vec![];
        let mut inners: Vec<Node> = vec![];
        Self::add_inner(
            &BvhNodeConstructionRecord::Inner {
                bbox: bvh_node.aabb,
                ptr: bvh_node,
            },
            &mut inners,
            &mut leaves,
        );
        Self {
            leaves: leaves,
            inners: inners,
        }
    }
    fn add_inner(
        bvh_node: &BvhNodeConstructionRecord,
        inners: &mut Vec<Node>,
        leaves: &mut Vec<Arc<Hitable>>,
    ) -> usize {
        let (node, children) = Node::from_bvh_node(bvh_node);
        let node_idx = inners.len();
        inners.push(node);
        for i in 0..8 {
            match &children[i] {
                BvhNodeConstructionRecord::Leaf { ptr, bbox } => {
                    let node = &mut inners[node_idx];
                    if bbox.is_empty() {
                        node.children[i] = NodePointer::empty_leaf();
                        debug_assert!(node.children[i].is_empty_leaf());
                        debug_assert!(node.children[i].is_leaf());
                    } else {
                        let idx = Self::add_leaf(ptr.clone(), leaves);
                        node.children[i] = NodePointer::new(true, idx);
                    }
                }
                BvhNodeConstructionRecord::Inner { ptr: _, bbox: _ } => {
                    // ToDo: 簡約処理を入れるなら、ここである。
                    // innerを調べて、片方にしかleafがないnodeなら、add_leafの方を呼ぶようにする。
                    // あるいは、BvhNodeでそういう構造ができないように実装するか。どちらだろう。
                    let idx = Self::add_inner(&children[i], inners, leaves);
                    inners[node_idx].children[i] = NodePointer::new(false, idx);
                }
            }
        }
        node_idx
    }
    fn add_leaf(leaf: Arc<Hitable>, leaves: &mut Vec<Arc<Hitable>>) -> usize {
        let idx = leaves.len();
        leaves.push(leaf);
        idx
    }
}

pub fn test_load_movemask_order_compatibility() {
    unsafe {
        let vec_a = _mm256_set1_ps(0.0);
        let b_arr = [4.0, 3.0, 2.0, 1.0, 0.0, -1.0, -2.0, -3.0];
        let vec_b = _mm256_load_ps(b_arr.as_ptr());
        let movemask = _mm256_movemask_ps(_mm256_cmp_ps(
            vec_a, vec_b, _CMP_LE_OS, /* Ordered, Signaling. */
        ));
        assert!(movemask == 0b00011111);
        // assert!(movemask == 0b11111000);
    }
}

// #[cfg(all(
//     any(target_arch = "x86", target_arch = "x86_64"),
//     target_feature = "avx"
// ))]
// fn foo() {
//     #[cfg(target_arch = "x86")]
//     use std::arch::x86::_mm256_add_epi64;
//     #[cfg(target_arch = "x86_64")]
//     use std::arch::x86_64::_mm256_add_epi64;
//     println!("foo");
// }

// pub fn hit(&self, ray: &Ray, t_min: f32, t_max: f32) -> bool {
//         let mut t_min_int = t_min; // int = intersection
//         let mut t_max_int = t_max;
//         for a in 0..3 {
//             let inv_d = 1.0 / ray.direction[a];
//             let mut t0 = ((self.min)[a] - ray.origin[a]) * inv_d;
//             let mut t1 = ((self.max)[a] - ray.origin[a]) * inv_d;
//             if inv_d < 0.0 {
//                 std::mem::swap(&mut t0, &mut t1);
//             }
//             t_min_int = f32::max(t_min_int, t0);
//             t_max_int = f32::min(t_max_int, t1);
//             if t_min_int > t_max_int {
//                 // 厚さ0のAAbbとはヒットする
//                 return false;
//             }
//         }
//         return true;
//     }

// pub fn from_bvh_node(bvh_node: Arc<BvhNode>) -> (Self, [BvhNodeConstructionRecord; 8]) {
//     let mut children: [BvhNodeConstructionRecord; 8] = Default::default();
//     let mut bboxes = [[[std::f32::MAX; 8]; 3], [[std::f32::MIN; 8]; 3]];
//     let mut self = Node::empty();

//     let mut axis_top = 0u8;
//     let mut axis_child = [0u8; 2];
//     let mut axis_grand_son = [0u8; 4];
//     // bboxes: [[__m256; 3]; 2],
//     // children: [NodePointer; 8],
//     // axis_top: u8, // ToDo: add an explanation comment
//     // axis_child: [u8; 2],
//     // axis_grand_son: [u8; 4],

//     for child_id in 0..8 {
//         let mut axises = [0u8; 3];
//         let child = Self::get_great_grandchild(bvh_node.clone(), child_id as u8, &mut axises);
//         axis_top = axises[0];
//         axis_child[child_id % 2] = axises[1];
//         axis_grand_son[child_id % 4] = axises[2];
//         // Note: node.axis_* are set more times than needed,
//         // so this logic is not best, but works well enough.
//         for min_max in 0..2 {
//             for axis in 0..3 {
//                 if min_max == 0 {
//                     bboxes[min_max][axis][child_id] = child.bbox().min[axis];
//                 } else {
//                     bboxes[min_max][axis][child_id] = child.bbox().max[axis];
//                 }
//             }
//         }
//         children[child_id] = child;
//     }
//     (
//         Node {
//             bboxes: Self::convert_bboxes(&bboxes),
//             children: [NodePointer::empty_leaf(); 8],
//             axis_top: axis_top,
//             axis_child: axis_child,
//             axis_grand_son: axis_grand_son,
//         },
//         children,
//     )
// }

// fn get_great_grandchild(
//     bvh_node: Arc<BvhNode>,
//     child_id: u8,
//     axises: &mut [u8; 3], // top, child, grandson
// ) -> BvhNodeConstructionRecord {
//     Self::dig_to_child_node(bvh_node, child_id, 0, axises)
// }
// fn dig_to_child_node(
//     bvh_node: Arc<BvhNode>,
//     child_id: u8, // ToDo: やはり u8 に統一した方が良いか。統一する。
//     depth: u8,
//     axises: &mut [u8; 3], // top, child, grandson
// ) -> BvhNodeConstructionRecord {
//     debug_assert!(depth <= 3);
//     if depth == 3 {
//         BvhNodeConstructionRecord::inner(bvh_node)
//     } else {
//         axises[depth as usize] = bvh_node.axis as u8;
//         let const_rec = if child_id & 1u8.shl(depth) == 0u8 {
//             &bvh_node.left_node_record
//         } else {
//             &bvh_node.right_node_record
//         };
//         match const_rec {
//             BvhNodeConstructionRecord::Inner {
//                 ptr: inner,
//                 bbox: _,
//             } => Self::dig_to_child_node(inner.clone(), child_id, depth + 1, axises),
//             _ => const_rec.clone(),
//         }
//     }
// }

// let mut debug_node = Node::empty();
// let mut bboxes = [[[0.0f32; 8]; 3]; 2];
// Node::set_bboxes_array_layout(
//     &mut bboxes,
//     0,
//     &Aabb::new(&Vec3::new(-1.0, -1.0, -1.0), &Vec3::new(1.0, 1.0, 1.0)),
// );
// debug_node.bboxes = Node::convert_bboxes(&bboxes);
// let debug_ray_avx = RayAVXInfo::from_ray(&Ray::new(
//     &Vec3::new(-2.0, -2.0, -2.0),
//     &Vec3::new(1.0, 1.0, 1.0),
//     0.0,
// ));
// // ToDo: remove
// for min_max in 0..2 {
//     for axis in 0..3 {
//         let mut bboxes_at_child = [0.0f32; 8];
//         unsafe {
//             _mm256_store_ps(
//                 bboxes_at_child.as_mut_ptr(),
//                 debug_node.bboxes[min_max][axis],
//             );
//         }
//         println!(
//             "min_max: {}, axis: {}, src: {:?}, res: {:?}",
//             min_max, axis, bboxes[min_max][axis], bboxes_at_child
//         );
//     }
// }
// println!("{:b}", debug_node.hit(&debug_ray_avx, 0.0, std::f32::MAX));
// // println!("{}, {}", t_min, t_max); // ToDo: たまにt_maxが大きな数になっているので確認する。
