use crate::aliases::{RandGen, Vec3};
use crate::pdf::Pdf;
use rand::Rng;

pub struct MixturePdf<'a, 'b> {
    mix: f32,
    a_pdf: Option<&'a Pdf>,
    b_pdf: &'b Pdf, // None when mix == 0.0
}

impl<'a, 'b> MixturePdf<'a, 'b> {
    pub fn new(mix: f32, a_pdf: &'a Pdf, b_pdf: &'b Pdf) -> Self {
        debug_assert!(0.0 <= mix && mix <= 1.0);
        if mix == 0.0 {
            Self::zero(b_pdf)
        } else {
            MixturePdf {
                mix: mix,
                a_pdf: Some(a_pdf),
                b_pdf: b_pdf,
            }
        }
    }
    pub fn zero(b_pdf: &'b Pdf) -> Self {
        MixturePdf {
            mix: 0.0,
            a_pdf: None,
            b_pdf: b_pdf,
        }
    }
}

impl<'a, 'b> Pdf for MixturePdf<'a, 'b> {
    fn density(&self, dir: &Vec3) -> f32 {
        match self.a_pdf {
            Some(a_pdf) => {
                self.mix * a_pdf.density(dir) + (1.0 - self.mix) * self.b_pdf.density(dir)
            }
            None => self.b_pdf.density(dir),
        }
    }
    fn generate(&self, rng: &mut RandGen) -> Vec3 {
        match self.a_pdf {
            Some(a_pdf) => {
                if rng.gen::<f32>() < self.mix {
                    a_pdf.generate(rng)
                } else {
                    self.b_pdf.generate(rng)
                }
            }
            None => self.b_pdf.generate(rng),
        }
    }
}

pub struct MixturePdfBox {
    pub mix: f32,
    pub a_pdf: Box<Pdf>,
    pub b_pdf: Box<Pdf>,
}

impl Pdf for MixturePdfBox {
    fn density(&self, dir: &Vec3) -> f32 {
        self.mix * self.a_pdf.density(dir) + (1.0 - self.mix) * self.b_pdf.density(dir)
    }
    fn generate(&self, rng: &mut RandGen) -> Vec3 {
        if rng.gen::<f32>() < self.mix {
            self.a_pdf.generate(rng)
        } else {
            self.b_pdf.generate(rng)
        }
    }
}
