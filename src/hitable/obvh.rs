use crate::aabb::Aabb;
use crate::aliases::RandGen;
use crate::aliases::Vec3;
use crate::hit_record::HitRecord;
use crate::hitable::bvh::BVH;
use crate::hitable::node_pointer::NodePointer;
use crate::hitable::Hitable;
use crate::ray::Ray;
#[cfg(target_arch = "x86")]
use std::arch::x86::*;
#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::*;
use std::fmt;
use std::ops::Shl;
use std::ops::Shr;

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
pub struct OBVH<L> {
    bbox: Aabb,
    leaves: Vec<L>,    // leaf-nodes.
    inners: Vec<Node>, // inner nodes. inners[0] is the root node.
}

struct Node {
    // OBVHノード。
    // ノード1つが深さ3の2分木で、8個の子を持つ（childrenフィールド）。
    // それぞれの子ノードの配列childrenの位置（インデックス）をchild_id（in [0..8)）と呼ぶ。
    // child_idの第0ビット、第1ビット、第2ビットは、根ノードからどのように進んでその子ノードに到達できるかを表す。
    // 第0ビットが1（resp. 0）<=> 最初に左（resp. 右）に進む。
    // 第1ビットが1（resp. 0）<=> 次に左（resp. 右）に進む。
    // 第2ビットが1（resp. 0）<=> 最後に左（resp. 右）に進む。
    // 分岐点それぞれ（根1つ、子2つ、孫4つの合計7個ある）には、BVH構築の際に用いた分割軸の情報axis（0=x,1=y,2=zのいずれか）
    // が付随している。
    // ある分岐点より左（resp. 右）に格納されている子ノードたちは、軸axisに関して座標値が小さい（resp. 大きい）側である。

    // 各子ノードのバウンディングボックス情報。
    // __m256と[f32; 8]を同一視し、bboxesフィールドの型が[[[f32; 8]; 3]; 2]であると考えるとき、
    // bboxes[min_or_max][axis][child_id]は、子chiild_idのバウンディングボックスの、軸axisに沿った座標値の
    // 最小値（min_or_max=0のとき）か最大値（min_or_max=1のとき）。
    bboxes: [[__m256; 3]; 2],
    children: [NodePointer; 8],
    // 各分岐点における分割軸情報。
    // axis_bit_i（i=0,1）をサイズ8のu8配列とみなすとする（リトルエンディアン）。
    // それぞれのaxis_bit_i[child_id]の値は下位3bitのみが意味を持ち、x{i}でxの第iビットを表すとき、
    // axis_bit_i[child_id]{2} = (根からchild_idに到達する経路における、1番目の分岐点の分割軸){i}
    //   child_idに依存しない。
    // axis_bit_i[child_id]{1} = (根からchild_idに到達する経路における、2番目の分岐点の分割軸){i}
    //   child_idを2で割った余りにだけ依存。
    // axis_bit_i[child_id]{0} = (根からchild_idに到達する経路における、3番目の分岐点の分割軸){i}
    //   child_idを4で割った余りにだけ依存。
    axis_bits_0: u64,
    axis_bits_1: u64,
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
            axis_top = 0,     //self.axis_top, // ToDo: この辺りきちんと実装しておく
            axis_child_0 = 0, //self.axis_child[0],
            axis_child_1 = 0, //self.axis_child[1],
            axis_gson_00 = 0, //self.axis_gson[0b00usize],
            axis_gson_01 = 0, //self.axis_gson[0b01usize],
            axis_gson_10 = 0, //self.axis_gson[0b10usize],
            axis_gson_11 = 0, //self.axis_gson[0b11usize],
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

const NODE_STACK_UPPER_BOUND: usize = 64;

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

impl<L> Hitable for OBVH<L>
where
    L: Hitable,
{
    fn hit<'s, 'r>(&'s self, ray: &'r Ray, t_min: f32, mut t_max: f32) -> Option<HitRecord<'s>> {
        if !self.bbox.hit(ray, t_min, t_max) {
            return None;
        }
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
                debug_assert!(hit_bits < 256);
                let priorities = node.calc_traverse_priority(&ray_avx.dir_sign);
                let mut ordered = [0usize; 8];
                for child_id in 0..8 {
                    ordered[priorities.shr(child_id * 8) as u8 as usize] = child_id;
                }
                for i in 0..8 {
                    let child_id = ordered[i];
                    if hit_bits & (1i32.shl(child_id)) != 0 {
                        node_stack.push(node.children[child_id]);
                    }
                }
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
                        node_stack.push(node.children[bit]); // ToDo: 適当実装？
                    }
                }
                debug_assert!(node_stack.len() <= NODE_STACK_UPPER_BOUND);
            }
        }
        false
    }
    fn bounding_box(&self, _time_0: f32, _time_1: f32) -> Option<Aabb> {
        // ToDo: self.bboxは事前計算しておいたもの。そのときに使ったtime_0とtime_1と一致していないとうまく動かない。
        Some(self.bbox)
    }
    fn random_direction_from(&self, _origin: &Vec3, _rng: &mut RandGen) -> Vec3 {
        panic!("random_direction_from called for OBHV");
    }
    fn direction_density(&self, _origin: &Vec3, _dir: &Vec3) -> f32 {
        panic!("direction_density called for OBHV");
    }
}

impl Node {
    /// レイがぞれぞれのバウンディングボックスにヒットしているか返す。
    /// * `return` - 各i in [0..8)に対し、returnの第iビット =「レイがi番目にヒットしているかどうか（true=1, false=0）」
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
    /// それぞれの子ノードのトラバースにおける優先度を求める。
    /// `return` - リトルエンディアンで[u8; 8]と同一視するとき、return[child_id] = 子child_idの優先度（[0..8)）
    fn calc_traverse_priority(&self, ray_is_pos: &[usize; 3]) -> u64 {
        const CHILD_IDS: u64 = (0b111u64 << 8 * 0)
            | (0b011u64 << 8 * 1)
            | (0b101u64 << 8 * 2)
            | (0b001u64 << 8 * 3)
            | (0b110u64 << 8 * 4)
            | (0b010u64 << 8 * 5)
            | (0b100u64 << 8 * 6)
            | (0b000u64 << 8 * 7);
        // >>
        const MASK: u64 = (0b111u64 << 8 * 0)
            | (0b111u64 << 8 * 1)
            | (0b111u64 << 8 * 2)
            | (0b111u64 << 8 * 3)
            | (0b111u64 << 8 * 4)
            | (0b111u64 << 8 * 5)
            | (0b111u64 << 8 * 6)
            | (0b111u64 << 8 * 7);
        // >>
        let mut mapped: u64 = 0;
        if ray_is_pos[0b00] == 1 {
            mapped |= !self.axis_bits_1 & !self.axis_bits_0 & MASK;
        }
        if ray_is_pos[0b01] == 1 {
            mapped |= !self.axis_bits_1 & self.axis_bits_0;
        }
        if ray_is_pos[0b10] == 1 {
            mapped |= self.axis_bits_1 & !self.axis_bits_0;
        }
        debug_assert!((mapped ^ CHILD_IDS) & MASK == mapped ^ CHILD_IDS);
        mapped ^ CHILD_IDS
    }
    fn empty() -> Self {
        unsafe {
            Node {
                bboxes: [
                    [_mm256_set1_ps(std::f32::INFINITY); 3],
                    [_mm256_set1_ps(std::f32::NEG_INFINITY); 3],
                ],
                children: [NodePointer::empty_leaf(); 8],
                // axis_top: 0,
                // axis_child: [0; 2],
                // axis_gson: [0; 4],
                axis_bits_0: 0,
                axis_bits_1: 0,
            }
        }
    }
    pub fn from_bvh_node<L>(
        bvh: &BVH<L>,
        node_ptr: NodePointer,
        bbox: Aabb,
    ) -> (Self, [(NodePointer, Aabb); 8]) {
        let mut children: [(NodePointer, Aabb); 8] = [(NodePointer::root(), Aabb::empty()); 8];
        let mut bboxes = BBoxesArray::empty();
        let mut this = Node::empty();
        let mut axis_top: usize = 0;
        let mut axis_child = [0usize; 2];
        let mut axis_gson = [0usize; 4];
        this.from_bvh_node_traverse(
            bvh,
            node_ptr,
            bbox,
            &mut bboxes,
            &mut children,
            0,
            0,
            &mut axis_top,
            &mut axis_child,
            &mut axis_gson,
        );
        this.calc_axis_bits(axis_top, axis_child, axis_gson);
        this.bboxes = Self::load_bboxes(&bboxes);
        (this, children)
    }
    fn from_bvh_node_traverse<L>(
        &mut self,
        bvh: &BVH<L>,
        current: NodePointer,
        bbox: Aabb,
        bboxes: &mut BBoxesArray,
        children: &mut [(NodePointer, Aabb); 8],
        depth: usize,
        child_id: u8,
        axis_top: &mut usize, // ToDo: u8にする
        axis_child: &mut [usize; 2],
        axis_gson: &mut [usize; 4],
    ) {
        debug_assert!(depth <= 3);
        if depth == 3 {
            Self::set_bboxes_array_layout(bboxes, child_id, &bbox);
            children[child_id as usize] = (current, bbox);
        } else {
            if current.is_leaf() {
                Self::set_bboxes_array_layout(bboxes, child_id, &bbox);
                children[child_id as usize] = (current, bbox);
            } else {
                let bvh_node = &bvh.inners[current.index()];
                let axis: usize = bvh_node.axis as usize;
                if depth == 0 {
                    debug_assert!(child_id < 1);
                    *axis_top = axis;
                } else if depth == 1 {
                    debug_assert!(child_id < 2);
                    (*axis_child)[child_id as usize] = axis;
                } else if depth == 2 {
                    debug_assert!(child_id < 4);
                    (*axis_gson)[child_id as usize] = axis;
                }
                self.from_bvh_node_traverse(
                    bvh,
                    bvh_node.children[0],
                    bvh_node.bboxes[0],
                    bboxes,
                    children,
                    depth + 1,
                    child_id | 1u8.shl(depth),
                    axis_top,
                    axis_child,
                    axis_gson,
                );
                self.from_bvh_node_traverse(
                    bvh,
                    bvh_node.children[1],
                    bvh_node.bboxes[1],
                    bboxes,
                    children,
                    depth + 1,
                    child_id,
                    axis_top,
                    axis_child,
                    axis_gson,
                );
            }
        }
    }
    fn calc_axis_bits(&mut self, axis_top: usize, axis_child: [usize; 2], axis_gson: [usize; 4]) {
        let mut axis_bits_0: u64 = 0;
        let mut axis_bits_1: u64 = 0;
        let axis_bits_ref = [&mut axis_bits_0, &mut axis_bits_1];
        for child_id in 0..8 {
            let base: u64 = 1u64.shl(child_id * 8);
            for i in 0..2 {
                if axis_top & 1usize.shl(i) != 0usize {
                    *axis_bits_ref[i] |= base.shl(2);
                }
                if axis_child[child_id % 2] & 1usize.shl(i) != 0usize {
                    *axis_bits_ref[i] |= base.shl(1);
                }
                if axis_gson[child_id % 4] & 1usize.shl(i) != 0usize {
                    *axis_bits_ref[i] |= base.shl(0);
                }
            }
        }
        self.axis_bits_0 = axis_bits_0;
        self.axis_bits_1 = axis_bits_1;
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
}

impl<L> OBVH<L>
where
    L: Hitable,
{
    pub fn from_bvh(bvh: BVH<L>) -> Self {
        let mut inners: Vec<Node> = Vec::default();
        Self::add_inner(&bvh, NodePointer::root(), bvh.bbox, &mut inners);
        Self {
            bbox: bvh.bbox, // ToDo: 数字は適当。
            inners: inners,
            leaves: bvh.leaves,
        }
    }
    // ToDo: NodePointerがBVHとOBVHの両方にあってわかりづらいのでnewtypeパターンする
    /// BVHを平坦化して、OBVHノードを構築する。
    /// * return - 追加したOBVHノードのうちもっとも根に近いもののインデックス
    fn add_inner(bvh: &BVH<L>, node_ptr: NodePointer, bbox: Aabb, inners: &mut Vec<Node>) -> usize {
        let (node, children) = Node::from_bvh_node(bvh, node_ptr, bbox);
        let node_idx = inners.len();
        inners.push(node);
        for i in 0..8 {
            if children[i].0.is_leaf() {
                let node = &mut inners[node_idx];
                node.children[i] = children[i].0;
            } else {
                let idx = Self::add_inner(bvh, children[i].0, children[i].1, inners);
                inners[node_idx].children[i] = NodePointer::new(false, idx);
            }
        }
        node_idx
    }
}

#[cfg(test)]
mod tests {
    use super::BBoxesArray;
    use super::Node;
    use super::NodePointer;
    use super::BVH;
    use super::OBVH;
    use crate::material::lambertian::Lambertian;
    use crate::obj_file::ObjFile;
    use crate::texture::constant::ConstantTexture;
    use itertools::iproduct;
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
        let bvh = BVH::new(obj.to_triangles(material.clone()), 0.0, 1.0);
        let obvh = Arc::new(OBVH::from_bvh(bvh));
        assert_eq!(obvh.inners.len(), 1);
        assert_eq!(obvh.leaves.len(), 8);
        let node = &obvh.inners[0];
        assert_eq!(
            node.axis_bits_0,
            0b0000000000000000000000000000000000000000000000000000000000000000
        );
        assert_eq!(
            node.axis_bits_1,
            0b0000001000000010000000100000001000000010000000100000001000000010
        );
    }
    fn traverse_priority_answer(
        top_axis: usize,
        child_axis: &[usize; 2],
        gson_axis: &[usize; 4],
        ray_is_pos: &[usize; 3],
        child_id: u8,
        depth: i32,
        priorities: &mut [u8; 8],
        count: &mut u8,
    ) {
        if depth == 3 {
            priorities[child_id as usize] = 7 - *count;
            *count += 1;
        } else {
            let left_first = if depth == 0 {
                ray_is_pos[top_axis] == 1
            } else if depth == 1 {
                ray_is_pos[child_axis[(child_id % 2) as usize]] == 1
            } else {
                assert_eq!(depth, 2);
                ray_is_pos[gson_axis[(child_id % 4) as usize]] == 1
            };
            let mut first_child_id = child_id | (1u8 << depth); // >>
            let mut second_child_id = child_id;
            if !left_first {
                std::mem::swap(&mut first_child_id, &mut second_child_id);
            }
            traverse_priority_answer(
                top_axis,
                child_axis,
                gson_axis,
                ray_is_pos,
                first_child_id,
                depth + 1,
                priorities,
                count,
            );
            traverse_priority_answer(
                top_axis,
                child_axis,
                gson_axis,
                ray_is_pos,
                second_child_id,
                depth + 1,
                priorities,
                count,
            );
        }
    }
    #[test]
    fn traverse_priority() {
        for (
            top_axis_0,
            child_axis_0,
            child_axis_1,
            gson_axis_0,
            gson_axis_1,
            gson_axis_2,
            gson_axis_3,
        ) in iproduct!(0..3, 0..3, 0..3, 0..3, 0..3, 0..3, 0..3)
        {
            for (ray_is_pos_x, ray_is_pos_y, ray_is_pos_z) in iproduct!(0..2, 0..2, 0..2) {
                let mut node = Node {
                    bboxes: Node::load_bboxes(&BBoxesArray::empty()),
                    children: [NodePointer::empty_leaf(); 8],
                    axis_bits_0: 0,
                    axis_bits_1: 0,
                };
                node.calc_axis_bits(
                    top_axis_0,
                    [child_axis_0, child_axis_1],
                    [gson_axis_0, gson_axis_1, gson_axis_2, gson_axis_3],
                );
                let res = node.calc_traverse_priority(&[ray_is_pos_x, ray_is_pos_y, ray_is_pos_z]);
                let mut ans = [0u8; 8];
                traverse_priority_answer(
                    top_axis_0,
                    &[child_axis_0, child_axis_1],
                    &[gson_axis_0, gson_axis_1, gson_axis_2, gson_axis_3],
                    &[ray_is_pos_x, ray_is_pos_y, ray_is_pos_z],
                    0,
                    0,
                    &mut ans,
                    &mut 0,
                );
                for i in 0..8 {
                    assert_eq!(ans[i], (res >> (i * 8)) as u8);
                }
            }
        }
    }
}

impl<L> fmt::Debug for OBVH<L> {
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
