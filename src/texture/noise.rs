use crate::aliases::{RandGen, Vec2, Vec3};
use crate::pdf::rnd_in_unit_sphere;
use crate::texture::Texture;
use rand::Rng;
use std::sync::Arc;

pub struct Perlin {
    perm_x: Box<[u8; 256]>,   // permutation of {0,1,...,256}
    perm_y: Box<[u8; 256]>,   // permutation of {0,1,...,256}
    perm_z: Box<[u8; 256]>,   // permutation of {0,1,...,256}
    ranvec: Box<[Vec3; 256]>, // random unit vectors
}

impl Perlin {
    pub fn new(rng: &mut RandGen) -> Self {
        Perlin {
            perm_x: Self::generate_perm(rng),
            perm_y: Self::generate_perm(rng),
            perm_z: Self::generate_perm(rng),
            ranvec: Self::generate_ranvec(rng),
        }
    }
    pub fn generate_perm(rng: &mut RandGen) -> Box<[u8; 256]> {
        let mut res = Box::<[u8; 256]>::new([0; 256]);
        for i in 0..256 {
            res[i] = i as u8;
        }
        for i in (1..256).rev() {
            let j = (rng.gen::<f64>() * ((i + 1) as f64)) as usize;
            res.swap(i as usize, j);
        }
        res
    }
    pub fn generate_ranvec(rng: &mut RandGen) -> Box<[Vec3; 256]> {
        let zero = Vec3::new(0.0, 0.0, 0.0);
        let mut res = Box::<[Vec3; 256]>::new([zero; 256]);
        for i in 0..256 {
            res[i] = rnd_in_unit_sphere(rng);
        }
        res
    }
    pub fn perlin_interpolate(c: &[[[Vec3; 2]; 2]; 2], point: &Vec3) -> f64 {
        let mut herm_cubic = Vec3::new(0.0, 0.0, 0.0);
        for i in 0..3 {
            herm_cubic[i] = point[i] * point[i] * (3.0 - 2.0 * point[i]);
        }
        let mut accum = 0.0;
        for i in 0..2 {
            for j in 0..2 {
                for k in 0..2 {
                    let ivec = Vec3::new(i as f64, j as f64, k as f64);
                    let weight = point - ivec;
                    accum += (i as f64 * herm_cubic[0] + (1 - i) as f64 * (1.0 - herm_cubic[0]))
                        * (j as f64 * herm_cubic[1] + (1 - j) as f64 * (1.0 - herm_cubic[1]))
                        * (k as f64 * herm_cubic[2] + (1 - k) as f64 * (1.0 - herm_cubic[2]))
                        * c[i][j][k].dot(&weight);
                }
            }
        }
        accum
    }
    /// returns values in [-1,1]
    pub fn noise(&self, p: &Vec3) -> f64 {
        let mut fract = Vec3::new(0.0, 0.0, 0.0);
        for i in 0..3 {
            fract[i] = p[i] - p[i].floor();
        }
        let i = p[0].floor() as u8;
        let j = p[1].floor() as u8;
        let k = p[2].floor() as u8;
        let mut c: [[[Vec3; 2]; 2]; 2] = [[[Vec3::new(0.0, 0.0, 0.0); 2]; 2]; 2];
        for di in 0..2 {
            for dj in 0..2 {
                for dk in 0..2 {
                    let idx = self.perm_x[(i + di) as u8 as usize]
                        ^ self.perm_y[(j + dj) as u8 as usize]
                        ^ self.perm_z[(k + dk) as u8 as usize];
                    c[di as usize][dj as usize][dk as usize] = self.ranvec[idx as u8 as usize];
                }
            }
        }
        Self::perlin_interpolate(&c, &fract) // [-1, 1]
    }
    pub fn turb(&self, p: &Vec3) -> f64 {
        const DEPTH: usize = 7;
        let mut accum = 0.0;
        let mut p_m = *p;
        let mut weight = 1.0;
        for _ in 0..DEPTH {
            accum += weight * self.noise(&p_m);
            weight *= 0.5;
            p_m *= 2.0;
        }
        accum.abs()
    }
}

pub struct NoiseTexture {
    perlin: Arc<Perlin>, // change to arc
    scale: f64,
}

impl NoiseTexture {
    pub fn new(scale: f64, rng: &mut RandGen) -> Self {
        NoiseTexture {
            perlin: Arc::new(Perlin::new(rng)),
            scale: scale,
        }
    }
}

impl Texture for NoiseTexture {
    fn value(&self, _uv: &Vec2, p: &Vec3) -> Vec3 {
        Vec3::new(1.0, 1.0, 1.0)
            * 0.5
            * (1.0 + f64::sin(self.scale * p[2] + 10.0 * self.perlin.turb(p)))
    }
}
