// ToDo: OBVHとBVHをサブモジュールにする

use std::ops::Shl;
use std::ops::Shr;

/// A "pointer" to a Node
#[derive(Clone, Copy)]
pub struct NodePointer {
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
    pub fn empty_leaf() -> Self {
        Self {
            info: std::u32::MAX,
        }
    }
    pub fn new_leaf(index: usize) -> Self {
        Self::new(true, index)
    }
    pub fn new_inner(index: usize) -> Self {
        Self::new(false, index)
    }
}
