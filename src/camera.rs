use crate::aliases::{RandGen, Vec3};
use crate::pdf::rnd_in_unit_disc;
use crate::ray::Ray;
use rand::Rng;
use std::f64::consts::PI;

pub struct Camera {
    lower_left_corner: Vec3,
    horizontal: Vec3,
    vertical: Vec3,
    origin: Vec3,
    lens_radius: f64,
    u: Vec3, // a unit vector directing right
    v: Vec3, // a unit vector directing up
    #[allow(dead_code)]
    w: Vec3, // u.cross(v)
    time_0: f64, // shutter open
    time_1: f64, // shutter close
}

impl Camera {
    pub fn new_time(
        look_from: &Vec3,
        look_at: &Vec3,
        view_up: &Vec3,
        vfov: f64,   // vertical field of view
        aspect: f64, // width over height
        lens_radius: f64,
        focus_dist: f64,
        time_0: f64, // shutter open
        time_1: f64, // shutter close
    ) -> Self {
        let theta: f64 = vfov * PI / 180.0;
        let half_height: f64 = (theta * 0.5).tan();
        let half_width: f64 = aspect * half_height;
        let origin: Vec3 = *look_from;
        let w: Vec3 = (look_from - look_at).normalize();
        let u: Vec3 = view_up.cross(&w).normalize();
        let v: Vec3 = w.cross(&u);
        let lower_left_corner = origin - focus_dist * (half_width * u + half_height * v + w);
        let horizontal = u * 2.0 * focus_dist * half_width;
        let vertical = v * 2.0 * focus_dist * half_height;
        Camera {
            lower_left_corner: lower_left_corner,
            horizontal: horizontal,
            vertical: vertical,
            origin: origin,
            lens_radius: lens_radius,
            u: u,
            v: v,
            w: w,
            time_0: time_0,
            time_1: time_1,
        }
    }
    pub fn new(
        look_from: &Vec3,
        look_at: &Vec3,
        view_up: &Vec3,
        vfov: f64,   // vertical field of view
        aspect: f64, // width over height
        lens_radius: f64,
        focus_dist: f64,
    ) -> Self {
        Camera::new_time(
            look_from,
            look_at,
            view_up,
            vfov,
            aspect,
            lens_radius,
            focus_dist,
            0.0,
            0.0,
        )
    }
    pub fn get_ray(&self, u: f64, v: f64, rng: &mut RandGen) -> Ray {
        let r = self.lens_radius * rnd_in_unit_disc(rng);
        let offset = r.x * self.u + r.y * self.v;
        let time = self.time_0 + rng.gen::<f64>() * (self.time_1 - self.time_0);
        Ray::new(
            &(self.origin + offset),
            &(self.lower_left_corner + u * self.horizontal + v * self.vertical
                - self.origin
                - offset),
            time,
        )
    }
}
