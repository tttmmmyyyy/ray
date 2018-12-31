use rand;
use rand::SeedableRng;
use ray::aliases::Vec3;
use ray::background::{AmbientLight};
use ray::camera::Camera;
use ray::hitable::bvh_node::BvhNode;
use ray::hitable::empty::Empty;
use ray::hitable::hitable_list::HitableList;
use ray::hitable::rectangle::Rectangle;
use ray::hitable::sphere::Sphere;
use ray::hitable::Hitable;
use ray::material::diffuse_light::DiffuseLight;
use ray::material::lambertian::Lambertian;
use ray::obj_file::ObjFile;
use ray::scene::Scene;
use ray::texture::checker::CheckerTexture;
use ray::texture::constant::ConstantTexture;
use std::path::Path;
use std::sync::Arc;

pub fn scene(aspect_ratio: f64) -> Scene {
    let mut objs = Vec::<Arc<Hitable>>::new();
    const L: f64 = 20.0;
    objs.push(Arc::new(Rectangle::new(
        &Vec3::new(-L, -0.01, -L),
        &Vec3::new(0.0, -0.01, 2.0 * L),
        &Vec3::new(2.0 * L, -0.01, 0.0),
        Arc::new(Lambertian::new(Arc::new(CheckerTexture::new(
            Arc::new(ConstantTexture::new(&Vec3::new(0.9, 0.9, 0.9))),
            Arc::new(ConstantTexture::new(&Vec3::new(0.15, 0.15, 0.50))),
            0.8,
        )))),
        // Lambertian::boxed(ConstantTexture::boxed(&Vec3::new(0.9, 0.9, 0.9))),
        0.1,
    ))); // floor
    objs.push(Arc::new(Sphere::new(
        &Vec3::new(0.0, 16.0, 0.0),
        4.0,
        Arc::new(DiffuseLight::new(Arc::new(ConstantTexture::new(
            &Vec3::new(2.5, 2.5, 2.5),
        )))),
    ))); // light
         // objs.push(Sphere::boxed(
         //     &Vec3::new(0.0, 1.0, 0.0),
         //     1.0,
         //     Glass::boxed(1.5, 0.0),
         // )); // glass sphere
         // objs.push(Transform::boxed(
         //     BvhNode::boxed(
         //         ObjFile::from_file(Path::new("res/cube.obj"))
         //             .unwrap()
         //             .groups[0]
         //             .to_triangles(),
         //         0.0,
         //         1.0,
         //         &mut rng,
         //     ),
         //     &Affine::scale(1.0 / 10.0, &Vec3::new(0.0, 0.0, 0.0)),
         //     0.0,
         //     1.0,
         // ));
    let teapot = &mut ObjFile::from_file(Path::new("res/teapot.obj"))
        .unwrap()
        .groups[0];
    teapot.set_smooth_normals();
    let material = Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
        232.0 / 255.0,
        200.0 / 255.0,
        0.5,
    )))));
    objs.push(Arc::new(BvhNode::new(
        teapot.to_triangles(material),
        0.0,
        1.0,
        &mut SeedableRng::from_seed([0; 32]),
    )));
    // objs.push(Triangle::boxed(
    //     &Vec3::new(0.0, 1.0, 0.0),
    //     &Vec3::new(1.0, 1.0, 0.0),
    //     &Vec3::new(1.0, 2.0, 1.0),
    //     Lambertian::boxed(ConstantTexture::boxed(&Vec3::new(0.0, 0.0, 0.0))),
    // ));
    let objs = Arc::new(HitableList::new(objs));
    let look_from = Vec3::new(-5.0, 6.0, -5.0);
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
    let bg = Arc::new(AmbientLight::new(&Vec3::new(0.75, 0.85, 1.0)));
    Scene {
        hitables: objs,
        importance: Arc::new(Empty::new()),
        importance_weight: 0.0,
        camera: camera,
        bg: bg,
    }
}
