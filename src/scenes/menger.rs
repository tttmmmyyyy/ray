use itertools::iproduct;
use nalgebra as na;
use ray::affine::Affine;
use ray::aliases::Vec3;
use ray::background::AmbientLight;
use ray::camera::Camera;
use ray::hitable::bvh::BVH;
use ray::hitable::bvh_node::BvhNode;
use ray::hitable::hitable_list::HitableList;
use ray::hitable::obvh::OBVH;
use ray::hitable::rectangle::Rectangle;
use ray::hitable::sphere::Sphere;
use ray::hitable::transform::Transform;
use ray::hitable::Hitable;
use ray::material::diffuse_light::DiffuseLight;
use ray::material::glass::Glass;
use ray::material::lambertian::Lambertian;
use ray::material::Material;
use ray::scene::Scene;
use ray::texture::checker::CheckerTexture;
use ray::texture::constant::ConstantTexture;
use std::ops::Shl;
use std::sync::Arc;

pub fn scene(aspect_ratio: f32) -> Scene {
    let mut objs = Vec::<Arc<Hitable>>::new();
    const L: f32 = 20.0;
    let checker = Arc::new(CheckerTexture::new(
        Arc::new(ConstantTexture::new(&Vec3::new(1.0, 1.0, 1.0))),
        Arc::new(ConstantTexture::new(&Vec3::new(0.8, 0.8, 0.8))),
        0.8,
        &Vec3::new(0.0, 0.1, 0.1),
    ));
    let lambert_checker = Arc::new(Lambertian::new(checker.clone()));
    let wall_mat = lambert_checker.clone();
    objs.push(Arc::new(Rectangle::new(
        &Vec3::new(-L, -0.0, -L),
        &Vec3::new(0.0, -0.0, 2.0 * L),
        &Vec3::new(2.0 * L, -0.01, 0.0),
        wall_mat.clone(),
        0.1,
    ))); // floor
    let wall_z = -3.0;
    objs.push(Arc::new(Rectangle::new(
        &Vec3::new(-L, -L, wall_z),
        &Vec3::new(2.0 * L, 0.0, 0.0),
        &Vec3::new(0.0, 2.0 * L, 0.0),
        wall_mat.clone(),
        0.1,
    ))); // wall
    let light_power = 10.0;
    let light_radius = 6.0;
    let light = Arc::new(Sphere::new(
        &Vec3::new(5.0, 22.5, light_radius),
        light_radius,
        Arc::new(DiffuseLight::new(Arc::new(ConstantTexture::new(
            &Vec3::new(light_power, light_power, light_power),
        )))),
    ));
    objs.push(light.clone()); // light
    let lambert = Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
        1.0, 0.8, 0.8,
    )))));
    let _glass = Arc::new(Glass::new(2.0, 0.0));
    let cube_recs = menger_rectangles(
        &Vec3::new(-2.0, 0.0, -2.0),
        &Vec3::new(4.0, 4.0, 4.0),
        1,
        lambert.clone(),
        0,
    );
    let cube = Arc::new(BVH::new(cube_recs, 0.0, 1.0));
    // let cube = Arc::new(OBVH::from_bvh_node(cube));
    let cube = Arc::new(Transform::new(
        cube,
        &Affine::rotation(
            &Vec3::new(0.0, std::f32::consts::FRAC_PI_6, 0.0),
            &na::zero(),
        ),
        0.0,
        1.0,
    ));
    objs.push(cube);
    let objs = Arc::new(HitableList::new(objs));
    let look_from = Vec3::new(0.0, 7.5, 10.0);
    let look_at = Vec3::new(0.0, 2.0, 0.0);
    // let look_from = Vec3::new(0.0, 2.0, 0.0);
    // let look_at = Vec3::new(1.0, 2.0, 0.0);
    let dist_to_focus = 5.0;
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
    let bg = Arc::new(AmbientLight::new(&Vec3::new(0.0, 0.0, 0.0)));
    Scene {
        hitables: objs,
        light: Some(light),
        camera: camera,
        bg: bg,
    }
}

/// cubeを構成する矩形リストを返す
/// * mask - 不要な面のマスク。cubes_rectangles関数の同名の引数と同じ意味。
fn menger_rectangles(
    pos: &Vec3,
    size: &Vec3,
    depth: usize,
    texture: Arc<Material>,
    mask: u8,
) -> Vec<Rectangle> {
    if depth == 0 {
        return ray::hitable::cube_rectangles(pos, size, texture.clone(), mask);
    }
    iproduct!(0..3, 0..3, 0..3)
        .filter(|(x, y, z): &(i32, i32, i32)| {
            (*x == 1) as i32 + (*y == 1) as i32 + (*z == 1) as i32 <= 1
        })
        .map(|(x, y, z)| {
            let inner_size = size / 3.0;
            let inner_pos =
                pos + inner_size.component_mul(&Vec3::new(x as f32, y as f32, z as f32));
            let mut next_mask: u8 = 0;
            for i in 0..3 {
                let a = [x, y, z][i];
                let b = [x, y, z][(i + 1) % 3];
                let c = [x, y, z][(i + 2) % 3];
                if a % 2 == 0 {
                    next_mask = next_mask | (mask & 0b01u8.shl(2 * i + a as usize / 2));
                }
                if b % 2 == 0 && c % 2 == 0 {
                    match a {
                        0 => next_mask = next_mask | 0b10u8.shl(2 * i),
                        1 => next_mask = next_mask | 0b11u8.shl(2 * i),
                        2 => next_mask = next_mask | 0b01u8.shl(2 * i),
                        _ => unreachable!(),
                    }
                }
            }
            menger_rectangles(
                &inner_pos,
                &inner_size,
                depth - 1,
                texture.clone(),
                next_mask,
            )
        })
        .flatten()
        .collect()
}
