//! Structures and functions for an intermediate step when converting between factor graphs and serialized files.

// TODO @Samuel: rename vertex/edge types

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
    /// The vertex's type. Supported types: "POSE2D_ANGLE", "LANDMARK2D_ANGLE"
    #[serde(rename = "type")]
    pub vertex_type: String,
    /// The vertex's content. The structure depends on the vertex's type:
    ///
    /// Content for "POSE2D_ANGLE": vec![position_x, position_y, rotation]
    ///
    /// Content for "LANDMARK2D_ANGLE": vec![position_x, position_y]
    pub content: Vec<f64>,
}

/// Structure containing a factor graph model's edge, representing a factor.
#[derive(Debug, Serialize, Deserialize, PartialEq)]
pub struct Edge {
    /// The edge's type. Supported types: "PRIOR2D_ANGLE", "ODOMETRY2D_ANGLE", "OBSERVATION2D_ANGLE"
    #[serde(rename = "type")]
    pub edge_type: String,
    /// The IDs of this edge's vertices. The structure depends on the edge's type:
    ///
    /// Content for "PRIOR2D_ANGLE": vec![pose2d_angle_vertex]
    ///
    /// Content for "ODOMETRY2D_ANGLE": vec![pose2d_angle_vertex, pose2d_angle_vertex]
    ///
    /// Content for "OBSERVATION2D_ANGLE": vec![pose2d_angle_vertex, landmark2d_angle_vertex]
    pub vertices: Vec<usize>,
    /// The edge's restriction, representing a measurement. The structure depends on the edge's type:
    ///
    /// Content for "PRIOR2D_ANGLE": vec![measured_position_x, measured_position_y, measured_rotation]
    ///
    /// Content for "ODOMETRY2D_ANGLE": vec![measured_delta_position_x, measured_delta_position_y, measured_delta_rotation]
    ///
    /// Content for "OBSERVATION2D_ANGLE": vec![measured_delta_position_x, measured_delta_position_y]
    pub restriction: Vec<f64>,
    /// The edge's entire information matrix. It is expected to be symmetric, hence having identical row- and column-major representations.
    #[serde(rename = "informationMatrix")]
    pub information_matrix: Vec<f64>,
}
