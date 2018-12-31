use crate::aliases::Vec3;
use crate::ray::Ray;

pub trait Background: Send + Sync {
    fn color(&self, ray: &Ray) -> Vec3;
}

pub struct DirectionalLight {
    direction: Vec3, // normalized
    color: Vec3,
}

impl DirectionalLight {
    pub fn new(direction: &Vec3, color: &Vec3) -> Self {
        Self {
            direction: direction.normalize(),
            color: *color,
        }
    }
}

impl Background for DirectionalLight {
    fn color(&self, ray: &Ray) -> Vec3 {
        ray.direction.normalize().dot(&self.direction).max(0.0) * self.color
    }
}

pub struct AmbientLight {
    color: Vec3,
}

impl AmbientLight {
    pub fn new(color: &Vec3) -> Self {
        Self { color: *color }
    }
}

impl Background for AmbientLight {
    fn color(&self, _ray: &Ray) -> Vec3 {
        self.color
    }
}

pub struct WeightedBg {
    a_weight: f64,
    b_weight: f64,
    a: Box<Background>,
    b: Box<Background>,
}

impl WeightedBg {
    pub fn new(a_weight: f64, a: Box<Background>, b_weight: f64, b: Box<Background>) -> Self {
        Self {
            a_weight,
            a,
            b_weight,
            b,
        }
    }
}

impl Background for WeightedBg {
    fn color(&self, ray: &Ray) -> Vec3 {
        self.a_weight * self.a.color(ray) + self.b_weight * self.b.color(ray)
    }
}
