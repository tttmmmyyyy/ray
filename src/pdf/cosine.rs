use crate::aliases::{RandGen, Vec3};
use crate::onb::Onb;
use crate::pdf::random_cosine_direction;
use crate::pdf::random_cosine_n_direction;
use crate::pdf::Pdf;
use std::f32::consts::PI;

/// Pdf such that pdf(d) ~= max(cos(t), 0) in a specified local coordinate (u,v,w)
pub struct CosinePdf(Onb);

impl CosinePdf {
    pub fn new(w: &Vec3) -> Self {
        CosinePdf {
            0: Onb::build_from_w(w),
        }
    }
}

impl Pdf for CosinePdf {
    fn density(&self, dir: &Vec3) -> f32 {
        let cosine = dir.normalize().dot(self.0.w());
        (cosine / PI).max(0.0) // since integral of cos(theta) on a unit hemisphere = pi.
    }
    fn generate(&self, rng: &mut RandGen) -> Vec3 {
        self.0.local_to_global_vec(&random_cosine_direction(rng))
    }
}

/// Pdf such that pdf(d) ~= max(cos^n(t), 0) in a specified local coordinate (u,v,w)
pub struct CosineNPdf {
    pub basis: Onb,
    pub exponent: i32,
}

impl CosineNPdf {
    pub fn new(w: &Vec3, exponent: i32) -> Self {
        CosineNPdf {
            basis: Onb::build_from_w(w),
            exponent: exponent,
        }
    }
}

impl Pdf for CosineNPdf {
    fn density(&self, dir: &Vec3) -> f32 {
        let cosine = dir.normalize().dot(self.basis.w());
        if cosine < 0.0 {
            0.0
        } else {
            let normalizer = 2.0 * PI / (self.exponent as f32 + 1.0);
            cosine.powf(self.exponent as f32) / normalizer
        }
    }
    fn generate(&self, rng: &mut RandGen) -> Vec3 {
        let ret = self
            .basis
            .local_to_global_vec(&random_cosine_n_direction(self.exponent, rng));
        debug_assert!(ret.norm().is_finite() && ret.norm() > 0.0);
        ret
    }
}

#[cfg(test)]
mod tests {
    use crate::aliases::Vec3;
    use crate::pdf::cosine::CosineNPdf;
    use crate::pdf::cosine::CosinePdf;
    use crate::pdf::random_in_cone;
    use crate::pdf::Pdf;
    use std::f32::consts::PI;
    #[test]
    fn cosine_pdf_density() {
        const SAMPLE_CNT: usize = 10000;
        let mut rng = rand::thread_rng();
        let pdf = CosinePdf::new(&Vec3::new(0.0, 0.0, 1.0));
        let mut integral = 0.0f32;
        for _ in 0..SAMPLE_CNT {
            let ray_out = random_in_cone(-1.0, &mut rng);
            let density = pdf.density(&ray_out);
            integral += density;
        }
        integral *= 4.0 * PI / SAMPLE_CNT as f32;
        println!("[cosine_pdf_density] integral: {}", integral);
        assert!((integral - 1.0).abs() < 0.05);
    }
    #[test]
    fn cosn_pdf_density() {
        const SAMPLE_CNT: usize = 10000;
        const MAX_EXPONENT: usize = 10;
        let mut rng = rand::thread_rng();
        for e in 1..=MAX_EXPONENT {
            let pdf = CosineNPdf::new(&Vec3::new(0.0, 0.0, 1.0), e as i32);
            let mut integral = 0.0f32;
            for _ in 0..SAMPLE_CNT {
                let ray_out = random_in_cone(-1.0, &mut rng);
                let density = pdf.density(&ray_out);
                integral += density;
            }
            integral *= 4.0 * PI / SAMPLE_CNT as f32;
            println!(
                "[cosnine_pdf_density] exponent: {}, integral: {}",
                e, integral
            );
            assert!((integral - 1.0).abs() < 0.05);
        }
    }
}
