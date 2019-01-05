// 参考：
// https://www.hiramine.com/programming/3dmodelfileformat/objfileformat.html

use crate::aliases::{Vec2, Vec3};
use crate::hitable::triangle::Triangle;
use crate::hitable::Hitable;
use crate::material::Material;
use crate::util::HashVec3;
use std::collections::HashMap;
use std::collections::VecDeque;
use std::f32;
use std::fs::File;
use std::io;
use std::io::BufRead;
use std::io::BufReader;
use std::num;
use std::path::Path;
use std::str::FromStr;
use std::sync::Arc;

pub struct ObjMaterialFile {
    #[allow(dead_code)]
    name: String,
}

pub struct ObjFile {
    pub material_file: Option<ObjMaterialFile>,
    pub groups: Vec<ObjGroup>,
}

pub struct ObjGroup {
    name: Option<String>,
    #[allow(dead_code)]
    material: Option<ObjMaterial>,
    vertices: Vec<Vec3>,
    tex_coords: Vec<Vec2>,
    normals: Vec<Vec3>,
    faces: Vec<ObjFace>,
}

pub struct ObjMaterial {
    #[allow(dead_code)]
    name: String,
}

pub struct ObjVertex {
    vertex: usize,
    #[allow(dead_code)]
    tex_coord: Option<usize>,
    normal: Option<usize>,
}

pub struct ObjFace(Vec<ObjVertex>);

impl ObjFace {
    pub fn vertex_indices_for_triangle(&self, triangle_idx: usize) -> (usize, usize, usize) {
        (
            self.0[0].vertex,
            self.0[triangle_idx + 1].vertex,
            self.0[triangle_idx + 2].vertex,
        )
    }
}

// ToDo: implement Error trait. use custom_error! macro.
#[derive(Debug)]
pub enum Error {
    IO(io::Error),
    Parse(String),
    Unsupported(String),
    ParseFloat(num::ParseFloatError),
    ParseInt(num::ParseIntError),
}

impl From<std::io::Error> for Error {
    fn from(io_e: io::Error) -> Self {
        Error::IO(io_e)
    }
}

impl From<num::ParseFloatError> for Error {
    fn from(pfe: num::ParseFloatError) -> Self {
        Error::ParseFloat(pfe)
    }
}

impl From<num::ParseIntError> for Error {
    fn from(pie: num::ParseIntError) -> Self {
        Error::ParseInt(pie)
    }
}

impl ObjFile {
    pub fn from_file(path: &Path) -> Result<Self, Error> {
        Self::from_buf_reader(BufReader::new(File::open(path)?))
    }
    pub fn from_buf_reader(reader: impl BufRead) -> Result<Self, Error> {
        let mut obj_file = ObjFile {
            material_file: None,
            groups: Vec::new(),
        };
        let mut lines: VecDeque<String> = VecDeque::new();
        for line in reader.lines() {
            lines.push_back(line?);
        }
        loop {
            if lines.is_empty() {
                return Ok(obj_file);
            }
            let line = lines.front().unwrap().clone();
            if line.is_empty() {
                lines.pop_front();
            } else if line.starts_with("#") {
                lines.pop_front();
            } else if line.starts_with("mtllib ") {
                if obj_file.material_file.is_some() {
                    return Err(Error::Parse("Found two mtllib definitions.".to_string()));
                } else {
                    obj_file.material_file = Some(ObjMaterialFile {
                        name: Self::parse_matlib_line(line)?,
                    });
                    lines.pop_front();
                }
            } else if line.starts_with("g ")
                || line.starts_with("usemtl ")
                || line.starts_with("v ")
                || line.starts_with("vt ")
                || line.starts_with("vn ")
                || line.starts_with("f ")
            {
                obj_file.groups.push(ObjGroup::parse(&mut lines)?);
            } else {
                return Err(Error::Parse(format!("Unexpected line:\n{}.", line)));
            }
        }
    }
    fn parse_matlib_line(line: String) -> Result<String, Error> {
        let columns: Vec<&str> = line.split_whitespace().collect();
        if columns.len() != 2 {
            Err(Error::Parse(format!(
                "Invalid matlib definition:\n{}.",
                line
            )))
        } else {
            Ok(columns[1].to_string())
        }
    }
}

impl ObjGroup {
    fn empty() -> Self {
        ObjGroup {
            name: None,
            material: None,
            vertices: Vec::new(),
            tex_coords: Vec::new(),
            normals: Vec::new(),
            faces: Vec::new(),
        }
    }
    pub fn unify_vertex(&mut self) {
        let mut new_vertices: Vec<Vec3> = vec![];
        new_vertices.reserve(self.vertices.len());
        let mut vec_to_idx = HashMap::<HashVec3, usize>::new();
        vec_to_idx.reserve(self.vertices.len());
        for v in &self.vertices {
            if vec_to_idx.contains_key(&HashVec3 { 0: *v }) {
                continue;
            }
            let idx = new_vertices.len();
            new_vertices.push(*v);
            vec_to_idx.insert(HashVec3 { 0: *v }, idx);
        }
        for f in &mut self.faces {
            for v in &mut f.0 {
                let new_idx = vec_to_idx[&HashVec3 {
                    0: self.vertices[v.vertex],
                }];
                v.vertex = new_idx;
            }
        }
        self.vertices = new_vertices;
    }
    fn parse(lines: &mut VecDeque<String>) -> Result<Self, Error> {
        let mut grp = Self::empty();
        if lines.is_empty() {
            return Ok(grp);
        }
        if lines.front().unwrap().starts_with("g ") {
            grp.name = Some(Self::parse_group_line(&lines.front().unwrap())?);
            lines.pop_front();
        }
        loop {
            if lines.is_empty() {
                return Ok(grp);
            }
            let line = lines.front().unwrap();
            if line.is_empty() {
                lines.pop_front();
            } else if line.starts_with("# ") {
                lines.pop_front();
            } else if line.starts_with("g ") {
                return Ok(grp);
            } else if line.starts_with("v ") {
                grp.vertices.push(Self::parse_vec3_line(line)?);
                lines.pop_front();
            } else if line.starts_with("vt ") {
                grp.tex_coords.push(Self::parse_vec2_line(line)?);
                lines.pop_front();
            } else if line.starts_with("vn ") {
                grp.normals.push(Self::parse_vec3_line(line)?);
                lines.pop_front();
            } else if line.starts_with("f ") {
                grp.faces.push(Self::parse_face_line(line)?);
                lines.pop_front();
            } else {
                return Err(Error::Parse(format!("Unexpected line:\n{}.", line)));
            }
        }
    }
    fn parse_group_line(line: &String) -> Result<String, Error> {
        let columns: Vec<&str> = line.split_whitespace().collect();
        if columns.len() != 2 {
            Err(Error::Parse(format!(
                "Invalid group definition:\n{}.",
                line
            )))
        } else {
            Ok(columns[1].to_string())
        }
    }
    fn parse_vec3_line(line: &String) -> Result<Vec3, Error> {
        let columns: Vec<&str> = line.split_whitespace().collect();
        if columns.len() != 4 {
            Err(Error::Parse(format!("Invalid line:\n{}.", line)))
        } else {
            Ok(Vec3::new(
                f32::from_str(columns[1])?,
                f32::from_str(columns[2])?,
                f32::from_str(columns[3])?,
            ))
        }
    }
    fn parse_vec2_line(line: &String) -> Result<Vec2, Error> {
        let columns: Vec<&str> = line.split_whitespace().collect();
        if columns.len() != 3 {
            Err(Error::Parse(format!("Invalid line:\n{}.", line)))
        } else {
            Ok(Vec2::new(
                f32::from_str(columns[1])?,
                f32::from_str(columns[2])?,
            ))
        }
    }
    fn parse_face_line(line: &String) -> Result<ObjFace, Error> {
        let columns: Vec<&str> = line.split_whitespace().collect();
        if columns.len() <= 1 {
            return Err(Error::Parse(format!("Invalid line:\n{}.", line)));
        }
        let mut face = ObjFace { 0: Vec::new() };
        for i in 1..columns.len() {
            face.0
                .push(Self::parse_vertex_column(columns[i].to_string())?);
        }
        Ok(face)
    }
    fn parse_vertex_column(column: String) -> Result<ObjVertex, Error> {
        let indices: Vec<&str> = column.split('/').collect();
        if indices.len() == 1 {
            Ok(ObjVertex {
                vertex: usize::from_str(indices[0])? - 1, // .obj format indexes vertices starting from 1
                tex_coord: None,
                normal: None,
            })
        } else if indices.len() == 2 {
            Ok(ObjVertex {
                vertex: usize::from_str(indices[0])? - 1,
                tex_coord: Some(usize::from_str(indices[1])? - 1),
                normal: None,
            })
        } else if indices.len() == 3 {
            if indices[1].is_empty() {
                Ok(ObjVertex {
                    vertex: usize::from_str(indices[0])? - 1,
                    tex_coord: None,
                    normal: Some(usize::from_str(indices[2])? - 1),
                })
            } else {
                Ok(ObjVertex {
                    vertex: usize::from_str(indices[0])? - 1,
                    tex_coord: Some(usize::from_str(indices[1])? - 1),
                    normal: Some(usize::from_str(indices[2])? - 1),
                })
            }
        } else {
            Err(Error::Parse(format!(
                "Invalid vertex definition: {}.",
                column
            )))
        }
    }
    pub fn to_triangles(&self, material: Arc<Material>) -> Vec<Arc<Hitable>> {
        let mut tris: Vec<Arc<Hitable>> = Vec::new();
        for face in &self.faces {
            if face.0.len() != 3 {
                // ToDo: decompose to triangle.
                panic!("This group has a polygon which is not a triangle.");
            }
            let vertices = [
                self.vertices[face.0[0].vertex],
                self.vertices[face.0[1].vertex],
                self.vertices[face.0[2].vertex],
            ];
            let normals = (|| -> Option<[Vec3; 3]> {
                Some([
                    self.normals[face.0[0].normal?],
                    self.normals[face.0[1].normal?],
                    self.normals[face.0[2].normal?],
                ])
            })();
            tris.push(Arc::new(Triangle::new(
                &vertices,
                &normals,
                material.clone(),
                // Glass::boxed(2.0, 0.0),
            )));
        }
        tris
    }
    pub fn set_smooth_normals(&mut self) {
        let mut normals_at_vtx: Vec<Vec<Vec3>> = vec![];
        normals_at_vtx.reserve(self.vertices.len());
        for _ in 0..self.vertices.len() {
            normals_at_vtx.push(vec![]);
        }
        for face in &self.faces {
            for triangle_idx in 0..face.0.len() - 2 {
                let normal = self.calc_normal_to_face(&face, triangle_idx);
                let (ai, bi, ci) = face.vertex_indices_for_triangle(triangle_idx);
                for vi in &[ai, bi, ci] {
                    normals_at_vtx[*vi].push(normal);
                }
            }
        }
        let mut normal_at_vtx: Vec<Vec3> = normals_at_vtx
            .iter()
            .map(|vecs| {
                if vecs.len() == 0 {
                    Vec3::new(0.0, 0.0, 0.0)
                } else {
                    (vecs.iter().sum::<Vec3>() / vecs.len() as f32).normalize()
                }
            })
            .collect();
        let new_idx_begin = self.normals.len();
        self.normals.append(&mut normal_at_vtx);
        for face in &mut self.faces {
            for vtx in &mut face.0 {
                vtx.normal = Some(new_idx_begin + vtx.vertex);
            }
        }
    }
    fn calc_normal_to_face(&self, face: &ObjFace, triangle_idx: usize) -> Vec3 {
        debug_assert!(triangle_idx < face.0.len() - 2);
        let (ai, bi, ci) = face.vertex_indices_for_triangle(triangle_idx);
        let a = self.vertices[ai];
        let b = self.vertices[bi];
        let c = self.vertices[ci];
        (b - a).cross(&(c - a)).normalize()
    }
}
