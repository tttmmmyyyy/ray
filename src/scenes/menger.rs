use rand;
use ray::aliases::Vec3;
use ray::background::AmbientLight;
use ray::camera::Camera;
use ray::hitable::bvh_node::BvhNode;
use ray::hitable::hitable_list::HitableList;
use ray::hitable::obvh::OBVH;
use ray::hitable::rectangle::Rectangle;
use ray::hitable::sphere::Sphere;
use ray::hitable::Hitable;
use ray::material::diffuse_light::DiffuseLight;
use ray::material::glass::Glass;
use ray::material::lambertian::Lambertian;
use ray::material::phong::Phong;
use ray::scene::Scene;
use ray::texture::checker::CheckerTexture;
use ray::texture::constant::ConstantTexture;
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
    let _phong_checker = Arc::new(Phong::new(checker.clone(), 0.5, 0.5, 50, 0.5));
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
        &Vec3::new(0.0, 22.5, light_radius),
        light_radius,
        Arc::new(DiffuseLight::new(Arc::new(ConstantTexture::new(
            &Vec3::new(light_power, light_power, light_power),
        )))),
    ));
    objs.push(light.clone()); // light
    let _lambert = Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
        232.0 / 255.0,
        200.0 / 255.0,
        0.5,
    )))));
    let texture = Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
        1.0, 0.0, 0.0,
    )))));
    let mut cube_recs = Vec::new();
    cube_recs.append(&mut ray::hitable::cube_rectangles(
        &Vec3::new(-0.5, 0.0, -0.5),
        &Vec3::new(1.0, 1.0, 1.0),
        texture.clone(),
    ));
    cube_recs.append(&mut ray::hitable::cube_rectangles(
        &Vec3::new(0.5, 0.0, -0.5),
        &Vec3::new(1.0, 1.0, 1.0),
        texture.clone(),
    ));
    cube_recs.append(&mut ray::hitable::cube_rectangles(
        &Vec3::new(-0.5, 1.0, -0.5),
        &Vec3::new(1.0, 1.0, 1.0),
        texture.clone(),
    ));
    let cube = Arc::new(BvhNode::new(cube_recs, 0.0, 1.0));
    let cube = Arc::new(OBVH::from_bvh_node(cube));
    objs.push(cube);
    let objs = Arc::new(HitableList::new(objs));
    let look_from = Vec3::new(0.0, 3.0, 10.0);
    let look_at = Vec3::new(0.0, 1.0, 0.0);
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
