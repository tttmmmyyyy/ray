use rand;
use rand::prelude::Rng;
use rand::{SeedableRng, StdRng};
use ray::aliases::Vec3;
use ray::background::AmbientLight;
use ray::camera::Camera;
use ray::hitable::bvh_node::BvhNode;
use ray::hitable::empty::Empty;
use ray::hitable::hitable_list::HitableList;
use ray::hitable::rectangle::Rectangle;
use ray::hitable::sphere::Sphere;
use ray::hitable::Hitable;
use ray::material::diffuse_light::DiffuseLight;
use ray::material::glass::Glass;
use ray::material::lambertian::Lambertian;
use ray::material::metal::Metal;
use ray::scene::Scene;
use ray::texture::constant::ConstantTexture;
use std::sync::Arc;

pub fn scene(aspect_ratio: f32) -> Scene {
    let seed: [u8; 32] = [3; 32];
    let mut rng: StdRng = SeedableRng::from_seed(seed);
    let mut objs = Vec::<Arc<Hitable>>::new();
    const L: f32 = 10.0;
    objs.push(Arc::new(Rectangle::new(
        &Vec3::new(-L, 0.0, -L),
        &Vec3::new(0.0, 0.0, 2.0 * L),
        &Vec3::new(2.0 * L, 0.0, 0.0),
        Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
            0.5, 0.5, 0.5,
        ))))),
        0.1,
    ))); // floor
    objs.push(Arc::new(Sphere::new(
        &Vec3::new(0.0, 16.0, 0.0),
        4.0,
        Arc::new(DiffuseLight::new(Arc::new(ConstantTexture::new(
            &Vec3::new(5.0, 5.0, 5.0),
        )))),
    ))); // light
    objs.push(Arc::new(Sphere::new(
        &Vec3::new(0.0, 1.0, 0.0),
        1.0,
        Arc::new(Glass::new(1.5, 0.0)),
    ))); // glass big sphere
    objs.push(Arc::new(Sphere::new(
        &Vec3::new(-4.0, 1.0, 0.0),
        1.0,
        Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
            0.4, 0.2, 0.1,
        ))))),
    ))); // lambetian big sphere
    objs.push(Arc::new(Sphere::new(
        &Vec3::new(4.0, 1.0, 0.0),
        1.0,
        Arc::new(Metal::new(&Vec3::new(0.7, 0.8, 0.9), 0.0)),
    ))); // metal big sphere
    let mut small_spheres: Vec<Arc<Hitable>> = Vec::new();
    for a in -10..10 {
        for b in -10..10 {
            let center = Vec3::new(
                a as f32 + 0.9 * rng.gen::<f32>(),
                0.2,
                b as f32 + 0.9 * rng.gen::<f32>(),
            );
            let c = Vec3::new(4.0, 2.0, 0.0);
            if (center - c).norm() > 0.9 {
                let mat_rnd = rng.gen::<f32>();
                if mat_rnd < 0.8 {
                    small_spheres.push(Arc::new(Sphere::new(
                        &center,
                        0.2,
                        Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
                            rng.gen::<f32>() * rng.gen::<f32>(),
                            rng.gen::<f32>() * rng.gen::<f32>(),
                            rng.gen::<f32>() * rng.gen::<f32>(),
                        ))))),
                    )));
                } else if mat_rnd < 0.9 {
                    small_spheres.push(Arc::new(Sphere::new(
                        &center,
                        0.2,
                        Arc::new(Metal::new(
                            &Vec3::new(
                                0.5 * (1.0 + rng.gen::<f32>()),
                                0.5 * (1.0 + rng.gen::<f32>()),
                                0.5 * (1.0 + rng.gen::<f32>()),
                            ),
                            0.5 * rng.gen::<f32>(),
                        )),
                    )));
                } else {
                    small_spheres.push(Arc::new(Sphere::new(
                        &center,
                        0.2,
                        Arc::new(Glass::new(1.33 * 1.1 * rng.gen::<f32>(), 0.0)),
                    )));
                }
            }
        }
    }
    println!("smallsphres.len() = {}", small_spheres.len());
    objs.push(Arc::new(BvhNode::new(small_spheres, 0.0, 1.0)));
    let objs = Arc::new(HitableList::new(objs));
    let look_from = Vec3::new(10.0, 1.0, 10.0);
    let look_at = Vec3::new(0.0, 1.0, 0.0);
    let dist_to_focus = 10.0;
    let vfov = 40.0;
    let camera = Camera::new_time(
        &look_from,
        &look_at,
        &Vec3::new(0.0, 1.0, 0.0),
        vfov,
        aspect_ratio,
        0.0, // lens_radius
        dist_to_focus,
        0.0, // time_0
        1.0, // time_1
    );
    Scene {
        hitables: objs,
        light: None,
        camera: camera,
        bg: Arc::new(AmbientLight::new(&Vec3::new(0.75, 0.85, 1.0))),
    }
}
