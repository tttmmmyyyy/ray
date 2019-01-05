use crate::aliases::RandGen;
use crate::aliases::Vec3;
use crate::material::glass::reflect;
use crate::pdf::cosine::CosineNPdf;
use crate::pdf::Pdf;

/// A pdf used for sampling an outgoing direction in Blinn-Phong model.
pub struct BlinnPhongPdf {
    in_ray_dir: Vec3, // normalized
    cosn_pdf: CosineNPdf,
}

impl BlinnPhongPdf {
    /// * `normal` - need not be normalized.
    /// * `in_ray_dir` - need not be normalized.
    pub fn new(exponent: i32, normal: &Vec3, in_ray_dir: &Vec3) -> Self {
        Self {
            in_ray_dir: in_ray_dir.normalize(),
            cosn_pdf: CosineNPdf::new(normal, exponent),
        }
    }
    pub fn normal(&self) -> Vec3 {
        *(self.cosn_pdf.basis.w())
    }
}

impl Pdf for BlinnPhongPdf {
    fn density(&self, dir: &Vec3) -> f32 {
        // The following is also mathematically correct, but I didn't use it because
        // the test `blinn_phong_pdf_density` succeeds only for high SAMPLE_CNT
        // in case of in_ray_dir is far from normal.
        let dir_norm = dir.normalize();
        let half_vec = (-self.in_ray_dir + dir_norm).normalize();
        self.cosn_pdf.density(&half_vec) * (1.0 / (4.0 * dir_norm.dot(&half_vec))) // Jacobian
    }
    fn generate(&self, rng: &mut RandGen) -> Vec3 {
        let half_vec = self.cosn_pdf.generate(rng);
        let ret = reflect(&self.in_ray_dir, &half_vec);
        debug_assert!(ret.norm().is_finite() && ret.norm() > 0.0);
        ret
    }
}

#[cfg(test)]
mod tests {
    use crate::aliases::Vec3;
    use crate::pdf::blinnphong::BlinnPhongPdf;
    use crate::pdf::random_in_cone;
    use crate::pdf::Pdf;
    use std::f32::consts::PI;
    #[test]
    fn blinn_phong_pdf_density() {
        const SAMPLE_CNT: usize = 10000;
        const EXPONENT: i32 = 100;
        let mut rng = rand::thread_rng();
        let ray_in = Vec3::new(-1.0, 0.0, -1.0);
        let pdf = BlinnPhongPdf::new(EXPONENT, &Vec3::new(0.0, 0.0, 1.0), &ray_in);
        let mut integral = 0.0f32;
        for _ in 0..SAMPLE_CNT {
            let ray_out = random_in_cone(-1.0, &mut rng);
            let density = pdf.density(&ray_out);
            integral += density;
        }
        integral *= 4.0 * PI / SAMPLE_CNT as f32;
        println!("[blinn_phong_pdf_density] integral: {}", integral);
        assert!((integral - 1.0).abs() < 0.05);
    }
}
