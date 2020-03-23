use std::fmt::Debug;
use serde::{Serialize, Deserialize};
/*
use crate::factor_graph::FactorGraph;
use crate::factor_graph::variable::Variable;
use crate::factor_graph::variable::vehicle_variable_2d::VehicleVariable2D;
*/

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct FactorGraphModel {
    pub vertices: Vec<Vertex>,
    pub edges: Vec<Edge>,
}

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct Vertex {
    pub id: usize,
    #[serde(rename = "type")]
    pub vertex_type: String,
    pub position: [f64; 2],
    pub rotation: [f64; 1],
}

#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct Edge {
    #[serde(rename = "type")]
    pub edge_type: String,
    pub vertices: Vec<usize>,
    pub restriction: [f64; 3],
    #[serde(rename = "informationMatrix")]
    pub information_matrix: [f64; 9],
}



// TODO code code below once tests run without it

impl FactorGraphModel {
    #[deprecated(note="use direct initialization instead")]
    pub fn new(vertices: Vec<Vertex>, edges: Vec<Edge>) -> Self {
        FactorGraphModel { vertices, edges }
    }
}

impl Vertex {
    #[deprecated(note="use direct initialization instead")]
    pub fn new(id: usize, vertex_type: String, position: [f64; 2], rotation: [f64; 1]) -> Self {
        Vertex { id, vertex_type, position, rotation }
    }
}

impl Edge {
    #[deprecated(note="use direct initialization instead")]
    pub fn new(edge_type: String, vertices: Vec<usize>, restriction: [f64; 3], information_matrix: [f64; 9]) -> Self {
        Edge { edge_type, vertices, restriction, information_matrix }
    }
}