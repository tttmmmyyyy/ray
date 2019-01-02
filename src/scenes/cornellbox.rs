use rand;
use ray;
use ray::affine::Affine;
use ray::aliases::Vec3;
use ray::background::AmbientLight;
use ray::camera::Camera;
use ray::hitable::bvh_node::BvhNode;
use ray::hitable::cube;
use ray::hitable::hitable_list::HitableList;
use ray::hitable::obvh::OBVH;
use ray::hitable::rectangle::Rectangle;
use ray::hitable::sphere::Sphere;
use ray::hitable::transform::Transform;
use ray::hitable::Hitable;
use ray::material::diffuse_light::DiffuseLight;
use ray::material::glass::Glass;
use ray::material::lambertian::Lambertian;
use ray::material::metal::Metal;
use ray::scene::Scene;
use ray::texture::checker::CheckerTexture;
use ray::texture::constant::ConstantTexture;
use std::sync::Arc;

pub fn scene(aspect_ratio: f32) -> Scene {
    let mut objs = Vec::<Arc<Hitable>>::new();
    objs.push(Arc::new(Rectangle::new(
        &Vec3::new(0.0, 0.0, 555.0),
        &Vec3::new(0.0, 555.0, 0.0),
        &Vec3::new(555.0, 0.0, 0.0),
        Arc::new(Lambertian::new(Arc::new(CheckerTexture::new(
            Arc::new(ConstantTexture::new(&Vec3::new(0.73, 0.73, 0.73))),
            Arc::new(ConstantTexture::new(&Vec3::new(0.15, 0.15, 0.50))),
            55.5,
        )))),
        0.000,
    ))); // far
    objs.push(Arc::new(Rectangle::new(
        &Vec3::new(0.0, 0.0, 0.0),
        &Vec3::new(0.0, 555.0, 0.0),
        &Vec3::new(0.0, 0.0, 555.0),
        Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
            0.65, 0.05, 0.05,
        ))))),
        0.000,
    ))); // right
    objs.push(Arc::new(Rectangle::new(
        &Vec3::new(555.0, 0.0, 0.0),
        &Vec3::new(0.0, 0.0, 555.0),
        &Vec3::new(0.0, 555.0, 0.0),
        Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
            0.12, 0.45, 0.15,
        ))))),
        0.000,
    ))); // left
    objs.push(Arc::new(Rectangle::new(
        &Vec3::new(0.0, 555.0, 0.0),
        &Vec3::new(555.0, 0.0, 0.0),
        &Vec3::new(0.0, 0.0, 555.0),
        Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
            0.73, 0.73, 0.73,
        ))))),
        0.000,
    ))); // top
    objs.push(Arc::new(Rectangle::new(
        &Vec3::new(0.0, 0.0, 0.0),
        &Vec3::new(0.0, 0.0, 555.0),
        &Vec3::new(555.0, 0.0, 0.0),
        // Metal::boxed(&Vec3::new(1.0, 1.0, 1.0), 0.2),
        Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
            0.73, 0.73, 0.73,
        ))))),
        0.000,
    ))); // bottom
    let light = Arc::new(Rectangle::new(
        &Vec3::new(185.0, 554.0, 185.0),
        &Vec3::new(185.0, 0.0, 0.0),
        &Vec3::new(0.0, 0.0, 185.0),
        Arc::new(DiffuseLight::new(Arc::new(ConstantTexture::new(
            &Vec3::new(15.0, 15.0, 15.0),
        )))),
        0.001,
    ));
    objs.push(light.clone()); // light
    let lambert_white = Arc::new(Lambertian::new(Arc::new(ConstantTexture::new(&Vec3::new(
        1.0, 1.0, 1.0,
    )))));
    objs.push(Arc::new(Transform::new(
        Arc::new(Transform::new(
            Arc::new(cube(&Vec3::new(165.0, 165.0, 165.0), lambert_white.clone())),
            &Affine::translate(&Vec3::new(130.0, 0.0, 65.0)),
            0.0,
            1.0,
        )),
        &Affine::rotation(
            &Vec3::new(0.0, -0.4, 0.0),
            &Vec3::new(130.0 + 165.0 * 0.5, 0.0, 65.0 + 165.0 * 0.5),
        ),
        0.0,
        1.0,
    ))); // tall cube
    objs.push(Arc::new(Transform::new(
        Arc::new(Transform::new(
            Arc::new(cube(&Vec3::new(165.0, 330.0, 165.0), lambert_white.clone())),
            &Affine::translate(&Vec3::new(265.0, 0.0, 295.0)),
            0.0,
            1.0,
        )),
        &Affine::rotation(
            &Vec3::new(0.0, 0.5, 0.0),
            &Vec3::new(265.0 + 165.0 * 0.5, 0.0, 295.0 + 165.0 * 0.5),
        ),
        0.0,
        1.0,
    ))); // short cube
    let glass_sphere = Arc::new(Sphere::new(
        &Vec3::new(130.0 + 165.0 * 0.5, 165.0 + 82.5, 65.0 + 165.0 * 0.5),
        82.5,
        Arc::new(Glass::new(2.2, 0.0)),
    ));
    objs.push(glass_sphere.clone()); // glass sphere
    let metal_sphere = Arc::new(Sphere::new(
        &Vec3::new(265.0 + 165.0 * 0.5, 333.0 + 82.5, 295.0 + 165.0 * 0.5),
        82.5,
        Arc::new(Metal::new(&Vec3::new(1.0, 1.0, 1.0), 0.3)),
    ));
    objs.push(metal_sphere.clone()); // metal sphere
                                     // let objs = Arc::new(HitableList::new(objs));
    let objs = Arc::new(OBVH::from_bvh_node(Arc::new(BvhNode::new(objs, 0.0, 1.0))));
    let importance = light.clone();
    let look_from = Vec3::new(278.0, 278.0, -800.0);
    let look_at = Vec3::new(278.0, 278.0, 0.0);
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
        importance: importance,
        importance_weight: 0.4,
        camera: camera,
        bg: Arc::new(AmbientLight::new(&Vec3::new(0.0, 0.0, 0.0))),
    }
}
