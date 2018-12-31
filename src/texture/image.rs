use crate::aliases::{Vec2, Vec3};
use crate::texture::Texture;
use image;
use image::GenericImage;
use std::path::Path;

pub struct ImageTexture {
    data: Vec<u8>, // RGBRGBRGB...
    width: usize,
    height: usize,
}

impl ImageTexture {
    pub fn new(path: &Path) -> Self {
        let img = image::open(path).unwrap();
        let width = img.width() as usize;
        let height = img.height() as usize;
        let mut data = vec![0; width * height * 3];
        for i in 0..width {
            for j in 0..height {
                let col: image::Rgba<u8> = img.get_pixel(i as u32, j as u32);
                for c in 0..3 {
                    data[3 * (i + (height - j - 1) * width) + c] =
                        ((col[c] as u32) * (col[3] as u32) / 255u32) as u8; // alpha blend to black background
                }
            }
        }
        ImageTexture {
            data: data,
            width: width,
            height: height,
        }
    }
}

impl Texture for ImageTexture {
    fn value(&self, uv: &Vec2, _p: &Vec3) -> Vec3 {
        let i = ((uv[0] * (self.width as f32)) as usize)
            .min(self.width - 1)
            .max(0);
        let j = ((uv[1] * (self.height as f32)) as usize)
            .min(self.height - 1)
            .max(0);
        let mut col = Vec3::new(0.0, 0.0, 0.0);
        // ToDo: implement linear (or trilinear) interpolated sampling
        for c in 0..3 {
            col[c] = self.data[3 * (i + j * self.width) + c] as f32 / 255.0;
        }
        col
    }
}
