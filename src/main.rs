mod scenes;

use crate::scenes::ScenesType;
use rand::prelude::Rng;
use ray::aliases::Vec3;
use ray::scene::Scene;
#[cfg(target_arch = "x86")]
use std::arch::x86::*;
#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::*;
use std::path::Path;
use std::sync::mpsc::channel;
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

struct ColorSum {
    nx: i32,
    ny: i32,
    pub count: i32,
    pub sum: Vec<Vec3>,
}

impl ColorSum {
    pub fn zero(nx: i32, ny: i32) -> Self {
        ColorSum {
            nx: nx,
            ny: ny,
            count: 0,
            sum: vec![Vec3::new(0.0, 0.0, 0.0); (nx as usize) * (ny as usize)],
        }
    }
    pub fn replace_zero(&mut self) -> ColorSum {
        let (x, y) = (self.nx, self.ny);
        std::mem::replace(self, ColorSum::zero(x, y))
    }
    pub fn add(&mut self, rhs: ColorSum) {
        debug_assert_eq!((self.nx, self.ny), (rhs.nx, rhs.ny));
        self.count += rhs.count;
        for i in 0..((self.nx as usize) * (self.ny as usize)) {
            self.sum[i] += rhs.sum[i];
        }
    }
    pub fn save_png(&self, name: &str, elapsed_time: &Duration) {
        debug_assert!(self.count > 0);
        let mut buffer: Vec<u8> = vec![0; (self.nx as usize) * (self.ny as usize) * 4];
        for i in 0..self.nx {
            for j in 0..self.ny {
                let idx: usize = i as usize + j as usize * self.nx as usize;
                let mut col = self.sum[idx];
                col /= self.count as f32;
                buffer[idx * 4 + 0] = (255.99 * col[0].min(1.0).max(0.0)) as u8;
                buffer[idx * 4 + 1] = (255.99 * col[1].min(1.0).max(0.0)) as u8;
                buffer[idx * 4 + 2] = (255.99 * col[2].min(1.0).max(0.0)) as u8;
                buffer[idx * 4 + 3] = (255.99 * 1.0) as u8;
            }
        }
        let _ = image::save_buffer(
            &Path::new(&format!(
                "{}{}rays{}secs.png",
                name,
                self.count,
                elapsed_time.as_secs()
            )),
            buffer.as_slice(),
            self.nx as u32,
            self.ny as u32,
            image::RGBA(8),
        );
    }
    #[allow(dead_code)]
    pub fn save_raw(_name: &str) {
        unimplemented!()
    }
}

fn trace_rays(
    nx: i32,
    ny: i32,
    ns: i32,
    scene: &Scene,
    report_interval: i32,
    tx: Sender<ColorSum>,
) {
    let mut rng = rand::prelude::thread_rng();
    let mut color_sum = ColorSum::zero(nx, ny);
    let report = |result: &mut ColorSum| {
        tx.send(result.replace_zero()).unwrap();
    };
    for _ in 0..ns {
        for i in 0..nx {
            for j in 0..ny {
                let u = (i as f32 + rng.gen::<f32>()) / nx as f32;
                let v = (j as f32 + rng.gen::<f32>()) / ny as f32;
                let ray = scene.camera.get_ray(u, v, &mut rng);
                let col = ray::calc_color(&ray, scene, &mut rng, 50 /* depth */);
                let idx = (i + (ny - j - 1) * nx) as usize;
                color_sum.sum[idx] += col;
            }
        }
        color_sum.count += 1;
        if color_sum.count % report_interval == 0 {
            report(&mut color_sum);
        }
    }
    report(&mut color_sum);
}

fn main() {
    let start_time = Instant::now();
    const IMAGE_WIDTH: i32 = 200;
    const IMAGE_HEIGHT: i32 = 200;
    let aspect = IMAGE_WIDTH as f32 / IMAGE_HEIGHT as f32;
    const RAYS_PER_PIXEL: i32 = 1000;
    const THREAD_CNT: i32 = 4;
    const REPORT_INTERVAL: i32 = 100;
    const FILE_PATH_PREFIX: &'static str = "debug_images/image_";
    if get_output_dir_if_exists(Path::new(FILE_PATH_PREFIX)).is_none() {
        println!(
            "Wrong FILE_NAME (directory not exist): {}",
            FILE_PATH_PREFIX
        );
        std::process::exit(1);
    }
    println!(
        "FILE_PATH_PREFIX: {}, IMAGE_WIDTH: {}, IMAGE_HEIGHT: {}, RAYS_PER_PIXEL: {}, THREAD_CNT: {}",
        FILE_PATH_PREFIX, IMAGE_WIDTH, IMAGE_HEIGHT, RAYS_PER_PIXEL, THREAD_CNT
    );
    if RAYS_PER_PIXEL % THREAD_CNT != 0 {
        println!("RAYS_PER_PIXEL must divide THREAD_CNT.");
        std::process::exit(1);
    }
    if REPORT_INTERVAL % THREAD_CNT != 0 {
        println!("REPORT_INTERVAL must divide THREAD_CNT.");
        std::process::exit(1);
    }
    // let scene = scenes::get(ScenesType::CornellBox, aspect);
    // let scene = scenes::get(ScenesType::ManySpheres, aspect);
    let scene = scenes::get(ScenesType::Teapot, aspect);
    println!(
        "Scene constructed. ({:.3} secs elapsed)",
        duration_to_secs(&start_time.elapsed())
    );
    let rays_per_thread = RAYS_PER_PIXEL / THREAD_CNT;

    // ToDo: remove
    // {
    //     let mut rng = rand::prelude::thread_rng();
    //     for _ in 0..rays_per_thread {
    //         for i in 0..IMAGE_WIDTH {
    //             for j in 0..IMAGE_HEIGHT {
    //                 let u = (i as f32 + rng.gen::<f32>()) / IMAGE_WIDTH as f32;
    //                 let v = (j as f32 + rng.gen::<f32>()) / IMAGE_HEIGHT as f32;
    //                 let ray = scene.camera.get_ray(u, v, &mut rng);
    //                 let col = ray::calc_color(&ray, &scene, &mut rng, 50 /* depth */);
    //                 // println!("end calc_color");
    //             }
    //         }
    //     }
    // }

    crossbeam::scope(|scope| {
        let (tx, cx) = channel::<ColorSum>();
        let mut opt_tx = Some(tx);
        let mut threads: Vec<crossbeam::thread::ScopedJoinHandle<()>> = Vec::new();
        for _ in 0..THREAD_CNT {
            let tx = opt_tx.as_ref().unwrap().clone();
            let th = scope.spawn(|_| {
                trace_rays(
                    IMAGE_WIDTH,
                    IMAGE_HEIGHT,
                    rays_per_thread,
                    &scene,
                    REPORT_INTERVAL / THREAD_CNT,
                    tx,
                );
            });
            threads.push(th);
        }
        opt_tx.take(); // without this, the destruction of the tx in opt_tx is later
                       // than save_thread.join() while save_thread waits until every tx is destructed,
                       // and therefore causes deadlock.
        let save_thread = scope.spawn(move |_| {
            let mut current = ColorSum::zero(IMAGE_WIDTH, IMAGE_HEIGHT);
            let mut cnt = 0;
            loop {
                if let Ok(res) = cx.recv() {
                    current.add(res);
                    cnt += 1;
                    if cnt % THREAD_CNT == 0 {
                        let elapsed_time = start_time.elapsed();
                        current.save_png(FILE_PATH_PREFIX, &elapsed_time);
                    }
                } else {
                    break;
                }
            }
        });
        for th in threads {
            th.join().unwrap();
        }
        save_thread.join().unwrap();
    })
    .unwrap();
    println!(
        "Completed. ({:.3} secs elapsed)",
        duration_to_secs(&start_time.elapsed()),
    );
}

fn get_output_dir_if_exists(path: &Path) -> Option<&Path> {
    path.parent().and_then(|dir| {
        if dir.is_dir() && dir.exists() {
            Some(dir)
        } else {
            None
        }
    })
}

fn duration_to_secs(dur: &Duration) -> f32 {
    dur.as_secs() as f32 + dur.subsec_millis() as f32 * 0.001
}
