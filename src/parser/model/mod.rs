//! Structures and functions for an intermediate step when converting between factor graphs and serialized files.

use serde::{Deserialize, Serialize};
use std::fmt::Debug;

mod converter;

/// Structure containing the serializable model of a factor graph.
#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct FactorGraphModel {
    pub vertices: Vec<Vertex>,
    pub edges: Vec<Edge>,
}

/// Structure containing a factor graph model's vertex, representing a variable.
#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct Vertex {
    pub id: usize,
    #[serde(rename = "type")]
    pub vertex_type: String,
    pub position: [f64; 2],
    pub rotation: [f64; 1],
}

/// Structure containing a factor graph model's edge, representing a factor.
#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct Edge {
    #[serde(rename = "type")]
    pub edge_type: String,
    pub vertices: Vec<usize>,
    pub restriction: [f64; 3],
    #[serde(rename = "informationMatrix")]
    pub information_matrix: [f64; 9],
}



// TODO make constructors below deprecated and remove once tests run without it?

impl FactorGraphModel {
    pub fn new(vertices: Vec<Vertex>, edges: Vec<Edge>) -> Self {
        FactorGraphModel { vertices, edges }
    }
}

impl Vertex {
    pub fn new(id: usize, vertex_type: String, position: [f64; 2], rotation: [f64; 1]) -> Self {
        Vertex {
            id,
            vertex_type,
            position,
            rotation,
        }
    }
}

impl Edge {
    pub fn new(
        edge_type: String,
        vertices: Vec<usize>,
        restriction: [f64; 3],
        information_matrix: [f64; 9],
    ) -> Self {
        Edge {
            edge_type,
            vertices,
            restriction,
            information_matrix,
        }
    }
}
