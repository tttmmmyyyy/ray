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
use std::fmt;
use std::ops::Shl;
use std::ops::Shr;
use std::sync::Arc;

#[repr(align(32))]
struct BBoxesArray([[[f32; 8]; 3]; 2]);

impl BBoxesArray {
    fn empty() -> Self {
        Self {
            0: [
                [[std::f32::INFINITY; 8]; 3],
                [[std::f32::NEG_INFINITY; 8]; 3],
            ],
        }
    }
}

struct RayAVXInfo {
    origin: [__m256; 3],  // origin[axis_idx] = 8 copies of ray.origin[axis_idx]
    inv_dir: [__m256; 3], // origin[axis_idx] = 8 copies of 1.0 / ray.direction[axis_idx]
    dir_sign: [usize; 3], // dir_sign[axis_idx] = 1 (resp. 0) if ray.direction[axis_idx] >= 0 (resp. < 0)
}

impl RayAVXInfo {
    fn from_ray(ray: &Ray) -> Self {
        let calc_sign = |axis| (ray.direction[axis] >= 0.0) as usize;
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
    // A Node of an OBVH tree can be seen as a binary tree of depth 3, and in this perspective
    // child_id & (1 << 0) != 0 <=> "Is this children in the left-half of the whole tree (of depth 3)?"
    // child_id & (1 << 1) != 0 <=> "Is this children in the left-half of the next subtree (of depth 2)?"
    // child_id & (1 << 2) != 0 <=> "Is this children in the left-half of the minimal subtree (of depth 1)?"
    // At each inner node, the leaves blow are divided into two groups acccording to
    // the value X=(center-of-bounding-box)[some-axis].
    // The child with lesser X is called 'left' and one with greater X is called 'right'.
    //
    // When identifying __m256 and [f32; 8] (i.e., talking as if bboxes has type [[[f32; 8]; 3]; 2]),
    // then bboxes[min_or_max][axis][child_id] = min[0] or max[1] coordinate value
    // along the axis of the bbox of the child.
    bboxes: [[__m256; 3]; 2],
    children: [NodePointer; 8],
    axis_top: usize,        // Axis dividing left-leaves and right-leaves.
    axis_child: [usize; 2], // axis_child[i] = axis dividing children with child_id % 2 == i
    axis_gson: [usize; 4],  // axis_child[i] = axis dividing children with child_id % 4 == i
}

impl fmt::Debug for Node {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let bboxes_array = Self::store_bboxes(&self.bboxes);
        let mut aabbs: [Aabb; 8] = Default::default();
        Self::eight_aabb_from_array_layout(&mut aabbs, &bboxes_array);
        write!(
            f,
            "axis: {axis_top}
left:
  axis: {axis_child_0}
  left:
    axis: {axis_gson_00}
    left:
      bbox: {bbox_000:?}
      ptr: {ptr_000}
    right:
      bbox: {bbox_100:?}
      ptr: {ptr_100}
  right:
    axis: {axis_gson_10}
    left:
      bbox: {bbox_010:?}
      ptr: {ptr_010}
    right:
      bbox: {bbox_110:?}
      ptr: {ptr_110}
right:
  axis: {axis_child_1}
  left:
    axis: {axis_gson_01}
    left:
      bbox: {bbox_001:?}
      ptr: {ptr_001}
    right:
      bbox: {bbox_101:?}
      ptr: {ptr_101}
  right:
    axis: {axis_gson_11}
    left:
      bbox: {bbox_011:?}
      ptr: {ptr_011}
    right:
      bbox: {bbox_111:?}
      ptr: {ptr_111}",
            axis_top = self.axis_top,
            axis_child_0 = self.axis_child[0],
            axis_child_1 = self.axis_child[1],
            axis_gson_00 = self.axis_gson[0b00usize],
            axis_gson_01 = self.axis_gson[0b01usize],
            axis_gson_10 = self.axis_gson[0b10usize],
            axis_gson_11 = self.axis_gson[0b11usize],
            bbox_000 = aabbs[0b000usize],
            bbox_001 = aabbs[0b001usize],
            bbox_010 = aabbs[0b010usize],
            bbox_011 = aabbs[0b011usize],
            bbox_100 = aabbs[0b100usize],
            bbox_101 = aabbs[0b101usize],
            bbox_110 = aabbs[0b110usize],
            bbox_111 = aabbs[0b111usize],
            ptr_000 = self.children[0b000usize],
            ptr_001 = self.children[0b001usize],
            ptr_010 = self.children[0b010usize],
            ptr_011 = self.children[0b011usize],
            ptr_100 = self.children[0b100usize],
            ptr_101 = self.children[0b101usize],
            ptr_110 = self.children[0b110usize],
            ptr_111 = self.children[0b111usize],
        )
    }
}

/// A "pointer" to a Node
#[derive(Clone, Copy)]
struct NodePointer {
    info: u32,
}

const NODE_STACK_UPPER_BOUND: usize = 32;

struct NodeStack {
    data: [NodePointer; NODE_STACK_UPPER_BOUND],
    end: usize,
}

impl NodeStack {
    fn is_empty(&self) -> bool {
        self.end == 0
    }
    fn push(&mut self, elem: NodePointer) {
        self.data[self.end] = elem;
        self.end += 1;
    }
    fn pop(&mut self) -> NodePointer {
        debug_assert!(!self.is_empty());
        self.end -= 1;
        self.data[self.end]
    }
    fn empty() -> Self {
        NodeStack {
            data: [NodePointer::empty_leaf(); NODE_STACK_UPPER_BOUND],
            end: 0,
        }
    }
    fn len(&self) -> usize {
        self.end
    }
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

impl fmt::Display for NodePointer {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        if self.is_leaf() {
            if self.is_empty_leaf() {
                write!(f, "empty_leaf")
            } else {
                write!(f, "leaf: {}", self.index())
            }
        } else {
            write!(f, "inner: {}", self.index())
        }
    }
}

impl Hitable for OBVH {
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, mut t_max: f32) -> Option<HitRecord<'s>> {
        let ray_avx = RayAVXInfo::from_ray(ray);
        let mut node_stack = NodeStack::empty();
        node_stack.push(NodePointer::root());
        let mut hit_record: Option<HitRecord<'s>> = None;
        while !node_stack.is_empty() {
            let node_ptr = node_stack.pop();
            if node_ptr.is_leaf() {
                if node_ptr.is_empty_leaf() {
                    continue;
                }
                HitRecord::replace_to_some_min(
                    &mut hit_record,
                    &self.leaves[node_ptr.index()].hit(ray, t_min, t_max),
                );
                hit_record.map(|ref hr| {
                    debug_assert!(t_min <= hr.t && hr.t <= t_max);
                    t_max = hr.t;
                });
            } else {
                // if an inner node,
                let node = &self.inners[node_ptr.index()];
                let hit_bits = node.hit(&ray_avx, t_min, t_max);
                debug_assert!(hit_bits <= 255);
                node.push_node_stack(&mut node_stack, &ray_avx, hit_bits);
                debug_assert!(node_stack.len() <= NODE_STACK_UPPER_BOUND);
            }
        }
        hit_record
    }
    fn is_hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, t_max: f32) -> bool {
        let ray_avx = RayAVXInfo::from_ray(ray);
        let mut node_stack = NodeStack::empty();
        node_stack.push(NodePointer::root());
        while !node_stack.is_empty() {
            let node_ptr = node_stack.pop();
            if node_ptr.is_leaf() {
                if node_ptr.is_empty_leaf() {
                    continue;
                }
                if self.leaves[node_ptr.index()].is_hit(ray, t_min, t_max) {
                    return true;
                }
            } else {
                // if an inner node,
                let node = &self.inners[node_ptr.index()];
                let hit_bits = node.hit(&ray_avx, t_min, t_max);
                debug_assert!(hit_bits <= 255);
                for bit in 0..8 {
                    if hit_bits & 1i32.shl(bit) != 0 {
                        node_stack.push(node.children[bit]);
                    }
                }
                debug_assert!(node_stack.len() <= NODE_STACK_UPPER_BOUND);
            }
        }
        false
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
    /// Calculates whether or not ray hits eight bounding boxes (bboxes) with AVX.
    /// * `return` - for i in [0..8), return & (1 << i) != 0 <=> the ray hits the i-th bbox.
    unsafe fn hit_core(&self, ray: &RayAVXInfo, t_min: f32, t_max: f32) -> i32 {
        let mut t_min = _mm256_set1_ps(t_min);
        let mut t_max = _mm256_set1_ps(t_max);
        for a in 0..3 {
            // a = Axis
            t_min = _mm256_max_ps(
                t_min,
                _mm256_mul_ps(
                    ray.inv_dir[a],
                    _mm256_sub_ps(self.bboxes[1 - ray.dir_sign[a]][a], ray.origin[a]),
                ),
            );
            t_max = _mm256_min_ps(
                t_max,
                _mm256_mul_ps(
                    ray.inv_dir[a],
                    _mm256_sub_ps(self.bboxes[ray.dir_sign[a]][a], ray.origin[a]),
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
    #[inline(always)]
    fn push_node_stack(&self, node_stack: &mut NodeStack, ray: &RayAVXInfo, hit_bits: i32) {
        let mut child_id = 0usize;
        child_id ^= 0b001 * ray.dir_sign[self.axis_top];
        child_id ^= 0b010 * ray.dir_sign[self.axis_child[child_id]];
        child_id ^= 0b100 * ray.dir_sign[self.axis_gson[child_id]];
        if hit_bits & (1i32.shl(child_id)) != 0 {
            node_stack.push(self.children[child_id]);
        }
        child_id ^= 0b100;
        if hit_bits & (1i32.shl(child_id)) != 0 {
            node_stack.push(self.children[child_id]);
        }
        child_id &= 0b011;
        child_id ^= 0b010;
        child_id ^= 0b100 * ray.dir_sign[self.axis_gson[child_id]];
        if hit_bits & (1i32.shl(child_id)) != 0 {
            node_stack.push(self.children[child_id]);
        }
        child_id ^= 0b100;
        if hit_bits & (1i32.shl(child_id)) != 0 {
            node_stack.push(self.children[child_id]);
        }
        child_id &= 0b001;
        child_id ^= 0b001;
        child_id ^= 0b010 * ray.dir_sign[self.axis_child[child_id]];
        child_id ^= 0b100 * ray.dir_sign[self.axis_gson[child_id]];
        if hit_bits & (1i32.shl(child_id)) != 0 {
            node_stack.push(self.children[child_id]);
        }
        child_id ^= 0b100;
        if hit_bits & (1i32.shl(child_id)) != 0 {
            node_stack.push(self.children[child_id]);
        }
        child_id &= 0b011;
        child_id ^= 0b010;
        child_id ^= 0b100 * ray.dir_sign[self.axis_gson[child_id]];
        if hit_bits & (1i32.shl(child_id)) != 0 {
            node_stack.push(self.children[child_id]);
        }
        child_id ^= 0b100;
        if hit_bits & (1i32.shl(child_id)) != 0 {
            node_stack.push(self.children[child_id]);
        }
    }
    fn empty() -> Self {
        unsafe {
            Node {
                bboxes: [
                    [_mm256_set1_ps(std::f32::INFINITY); 3],
                    [_mm256_set1_ps(std::f32::NEG_INFINITY); 3],
                ],
                children: [NodePointer::empty_leaf(); 8],
                axis_top: 0,
                axis_child: [0; 2],
                axis_gson: [0; 4],
            }
        }
    }
    pub fn from_bvh_node(
        bvh_node: &BvhNodeConstructionRecord,
    ) -> (Self, [BvhNodeConstructionRecord; 8]) {
        let mut children: [BvhNodeConstructionRecord; 8] = Default::default();
        let mut bboxes = BBoxesArray::empty();
        let mut this = Node::empty();
        Self::from_bvh_node_traverse(&mut this, &mut bboxes, &mut children, bvh_node, 0, 0);
        this.bboxes = Self::load_bboxes(&bboxes);
        (this, children)
    }
    fn from_bvh_node_traverse(
        &mut self,
        bboxes: &mut BBoxesArray,
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
                        debug_assert!(child_id < 1);
                        self.axis_top = ptr.axis;
                    } else if depth == 1 {
                        debug_assert!(child_id < 2);
                        self.axis_child[child_id as usize] = ptr.axis;
                    } else if depth == 2 {
                        debug_assert!(child_id < 4);
                        self.axis_gson[child_id as usize] = ptr.axis;
                    }
                    Self::from_bvh_node_traverse(
                        self,
                        bboxes,
                        children,
                        &ptr.left_node_record,
                        depth + 1,
                        child_id & !1u8.shl(depth),
                    );
                    Self::from_bvh_node_traverse(
                        self,
                        bboxes,
                        children,
                        &ptr.right_node_record,
                        depth + 1,
                        child_id | 1u8.shl(depth),
                    );
                }
            }
        }
    }
    fn set_bboxes_array_layout(bboxes: &mut BBoxesArray, child_id: u8, bbox: &Aabb) {
        for min_max in 0..2 {
            for axis in 0..3 {
                if min_max == 0 {
                    bboxes.0[min_max][axis][child_id as usize] = bbox.min[axis];
                } else {
                    bboxes.0[min_max][axis][child_id as usize] = bbox.max[axis];
                }
            }
        }
    }
    fn eight_aabb_from_array_layout(out: &mut [Aabb; 8], array: &[[[f32; 8]; 3]; 2]) {
        // ToDo: write a test.
        for min_max in 0..2 {
            for axis in 0..3 {
                for child_id in 0..8 {
                    if min_max == 0 {
                        out[child_id].min[axis] = array[0][axis][child_id]
                    } else {
                        out[child_id].max[axis] = array[1][axis][child_id]
                    }
                }
            }
        }
    }
    fn load_bboxes(arr: &BBoxesArray) -> [[__m256; 3]; 2] {
        unsafe {
            let mut res = [
                [_mm256_set1_ps(std::f32::INFINITY); 3],
                [_mm256_set1_ps(std::f32::NEG_INFINITY); 3],
            ];
            for min_max in 0..2 {
                for axis in 0..3 {
                    res[min_max][axis] = _mm256_load_ps(arr.0[min_max][axis].as_ptr());
                }
            }
            res
        }
    }
    fn store_bboxes(src: &[[__m256; 3]; 2]) -> [[[f32; 8]; 3]; 2] {
        unsafe {
            let mut res = [
                [[std::f32::INFINITY; 8]; 3],
                [[std::f32::NEG_INFINITY; 8]; 3],
            ];
            for min_max in 0..2 {
                for axis in 0..3 {
                    _mm256_store_ps(res[min_max][axis].as_mut_ptr(), src[min_max][axis]);
                }
            }
            res
        }
    }
    // ToDo: write a test.
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
                BvhNodeConstructionRecord::Inner { ptr, bbox: _ } => {
                    if let Some(singleton) = ptr.singleton_node() {
                        let idx = Self::add_leaf(singleton, leaves);
                        inners[node_idx].children[i] = NodePointer::new(true, idx);
                    } else {
                        debug_assert!(!ptr.is_both_node_empty());
                        let idx = Self::add_inner(&children[i], inners, leaves);
                        inners[node_idx].children[i] = NodePointer::new(false, idx);
                    }
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

#[cfg(test)]
mod tests {
    use super::Node;
    use super::OBVH;
    use crate::hitable::bvh_node::BvhNode;
    use crate::material::lambertian::Lambertian;
    use crate::obj_file::ObjFile;
    use crate::texture::constant::ConstantTexture;
    #[cfg(target_arch = "x86")]
    use std::arch::x86::*;
    #[cfg(target_arch = "x86_64")]
    use std::arch::x86_64::*;
    use std::path::Path;
    use std::sync::Arc;
    #[test]
    fn load_and_movemask_order_compatibility() {
        unsafe {
            let vec_a = _mm256_set1_ps(0.0);
            let b_arr = [4.0, 3.0, 2.0, 1.0, 0.0, -1.0, -2.0, -3.0];
            let vec_b = _mm256_load_ps(b_arr.as_ptr());
            let movemask = _mm256_movemask_ps(_mm256_cmp_ps(
                vec_a, vec_b, _CMP_LE_OS, /* Ordered, Signaling. */
            ));
            assert!(movemask == 0b00011111);
        }
    }
    #[test]
    fn node_formatting() {
        let node = Node::empty();
        println!("{:?}", node);
    }
    #[test]
    fn obvh_construction() {
        let material = Arc::new(Lambertian::new(Arc::new(ConstantTexture::rgb(
            1.0, 1.0, 1.0,
        ))));
        let obj = &mut ObjFile::from_file(Path::new("res/test.obj"))
            .unwrap()
            .groups[0];
        let bvh_node = Arc::new(BvhNode::new(obj.to_triangles(material.clone()), 0.0, 1.0));
        let obvh = Arc::new(OBVH::from_bvh_node(bvh_node));
        println!("{:?}", obvh);
    }
}

impl fmt::Debug for OBVH {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Num of leaf nodes: {}\n", self.leaves.len()).unwrap();
        write!(f, "Num of inner nodes: {}\n", self.inners.len()).unwrap();
        for i in 0..self.inners.len() {
            let node = &self.inners[i];
            write!(f, "\nnode[{}]\n{:?}\n", i, node).unwrap();
        }
        Ok(())
    }
}
