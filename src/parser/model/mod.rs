use std::fmt::Debug;
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct FactorGraphModel {
    vertices: Vec<Vertex>,
    edges: Vec<Edge>,
}

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct Vertex {
    id: usize,
    #[serde(rename = "type")]
    vertex_type: String,
    position: [f64; 2],
    rotation: [f64; 1],
}

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct Edge {
    #[serde(rename = "type")]
    edge_type: String,
    vertices: Vec<usize>,
    restriction: [f64; 3],
    #[serde(rename = "informationMatrix")]
    information_matrix: [f64; 9],
}

impl FactorGraphModel {
    pub fn new(vertices: Vec<Vertex>, edges: Vec<Edge>) -> Self {
        FactorGraphModel { vertices, edges }
    }
}

impl Vertex {
    pub fn new(id: usize, vertex_type: String, position: [f64; 2], rotation: [f64; 1]) -> Self {
        Vertex { id, vertex_type, position, rotation }
    }
}

impl Edge {
    pub fn new(edge_type: String, vertices: Vec<usize>, restriction: [f64; 3], information_matrix: [f64; 9]) -> Self {
        Edge { edge_type, vertices, restriction, information_matrix }
    }
}