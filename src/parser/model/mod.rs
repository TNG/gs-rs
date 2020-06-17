//! Structures and functions for an intermediate step when converting between factor graphs and serialized files.

use serde::{Deserialize, Serialize};
use std::fmt::Debug;
use std::collections::BTreeSet;

mod converter;

/// Structure containing the serializable model of a factor graph.
#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct FactorGraphModel {
    /// All vertices in the factor graph.
    pub vertices: Vec<Vertex>,
    /// All edges in the factor graph.
    pub edges: Vec<Edge>,
    /// The IDs of all fixed vertices, i.e. vertices which will not be changed during optimization.
    pub fixed_vertices: BTreeSet<usize>,
}

/// Structure containing a factor graph model's vertex, representing a variable.
#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct Vertex {
    /// The vertex's ID. Should be unique within the factor graph.
    pub id: usize,
    /// The vertex's type. Supported types: "Vehicle2D", "Landmark2D"
    #[serde(rename = "type")]
    pub vertex_type: String,
    /// The vertex's content. The structure depends on the vertex's type:
    ///
    /// Content for "Vehicle2D": vec![position_x, position_y, rotation]
    ///
    /// Content for "Landmark2D": vec![position_x, position_y]
    ///
    /// Content for "Vehicle3D": vec![position_x, position_y, position_z, quaternion_x, quaternion_y, quaternion_z, quaternion_w]
    ///
    /// Content for "Landmark3D": vec![position_x, position_y, position_z]
    pub content: Vec<f64>,
}

/// Structure containing a factor graph model's edge, representing a factor.
#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct Edge {
    /// The edge's type. Supported types: "Position2D", "Odometry2D", "Observation2D"
    #[serde(rename = "type")]
    pub edge_type: String,
    /// The IDs of this edge's vertices. The structure depends on the edge's type:
    ///
    /// Content for "Position2D": vec![Vehicle2D_vertex]
    ///
    /// Content for "Odometry2D": vec![Vehicle2D_vertex, Vehicle2D_vertex]
    ///
    /// Content for "Observation2D": vec![Vehicle2D_vertex, Landmark2D_vertex]
    ///
    /// Content for "Position3D": vec![Vehicle3D_vertex]
    ///
    /// Content for "Odometry3D": vec![Vehicle3D_vertex, Vehicle3D_vertex]
    ///
    /// Content for "Observation3D": vec![Vehicle3D_vertex, Landmark3D_vertex]
    pub vertices: Vec<usize>,
    /// The edge's restriction, representing a measurement. The structure depends on the edge's type:
    ///
    /// Content for "Position2D": vec![position_x, position_y, rotation]
    ///
    /// Content for "Odometry2D": vec![delta_position_x, delta_position_y, delta_rotation]
    ///
    /// Content for "Observation2D": vec![delta_position_x, delta_position_y]
    ///
    /// Content for "Position3D": vec![position_x, position_y, position_z, quaternion_x, quaternion_y, quaternion_z, quaternion_w]
    ///
    /// Content for "Odometry3D": vec![delta_position_x, delta_position_y, delta_position_z, quaternion_x, quaternion_y, quaternion_z, quaternion_w]
    ///
    /// Content for "Observation3D": vec![delta_position_x, delta_position_y, delta_position_z]
    pub restriction: Vec<f64>,
    /// The edge's entire information matrix. It is expected to be symmetric, hence having identical row- and column-major representations.
    #[serde(rename = "informationMatrix")]
    pub information_matrix: Vec<f64>,
}
