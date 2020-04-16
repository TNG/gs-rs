use petgraph::csr::Csr;

use crate::factor_graph::factor::{Factor, FactorType::*};
use crate::factor_graph::variable::{
    VariableType::*, landmark_variable_2d::LandmarkVariable2D, vehicle_variable_2d::VehicleVariable2D, vehicle_variable_3d::VehicleVariable3D,
};
use crate::factor_graph::FactorGraph;
use crate::parser::model::{Edge, FactorGraphModel, Vertex};
use std::ops::{Index, Range};
use petgraph::visit::EdgeRef;
use std::collections::{BTreeSet, HashMap};

impl From<FactorGraphModel> for FactorGraph<'_> {
    fn from(model: FactorGraphModel) -> Self {
        let mut factor_graph = FactorGraph {
            csr: Csr::new(),
            node_indices: vec![],
            matrix_dim: 0,
            custom_to_csr_id_map: HashMap::new(),
        };

        model.vertices.iter()
            .for_each(|v| add_vertex(&mut factor_graph, v, model.fixed_vertices.contains(&v.id)));

        model.edges.iter()
            .for_each(|e| add_edge(&mut factor_graph, e));

        factor_graph
    }
}

impl From<&FactorGraph<'_>> for FactorGraphModel {
    fn from(factor_graph: &FactorGraph) -> Self {
        let mut model = FactorGraphModel { vertices: vec![], edges: vec![], fixed_vertices: BTreeSet::new() };
        for node_index in &factor_graph.node_indices {
            let node = factor_graph.csr.index(*node_index);
            model.vertices.push(Vertex {
                id: node.get_id(),
                vertex_type: match node.get_type() {
                    Vehicle2D => String::from("POSE2D_ANGLE"),
                    Landmark2D => String::from("LANDMARK2D_ANGLE"),
                    Vehicle3D => String::from("POSE3D_ANGLE"),
                },
                content: node.get_content(),
            });
            for edge in factor_graph.csr.edges(*node_index) {
                let factor: &Factor = edge.weight();
                let mut edge_vertices = vec![node.get_id()];
                if edge.target() != *node_index {
                    edge_vertices.push(factor_graph.csr.index(edge.target()).get_id());
                }
                model.edges.push(Edge {
                    edge_type: match factor.factor_type {
                        Position2D => String::from("PRIOR2D_ANGLE"),
                        Odometry2D => String::from("ODOMETRY2D_ANGLE"),
                        Observation2D => String::from("OBSERVATION2D_ANGLE"),
                        Odometry3D => String::from("ODOMETRY3D_ANGLE"),
                    },
                    vertices: edge_vertices,
                    restriction: factor.constraint.clone(),
                    information_matrix: factor.information_matrix.content.as_slice().to_owned(),
                });
            }
            if node.is_fixed() {
                model.fixed_vertices.insert(node.get_id());
            }
        }
        model
    }
}

fn add_edge(factor_graph: &mut FactorGraph, edge: &Edge) {
    let (target_index, factor_type) = match edge.edge_type.as_str() {
        "PRIOR2D_ANGLE" => (0, Position2D),
        "ODOMETRY2D_ANGLE" => (1, Odometry2D),
        "OBSERVATION2D_ANGLE" => (1, Observation2D),
        "ODOMETRY3D_ANGLE" => (1, Odometry3D),
        other_type => panic!("Unsupported edge type in the model: {}", other_type),
    };
    factor_graph.csr.add_edge(
        factor_graph.custom_to_csr_id_map[&edge.vertices[0]],
        factor_graph.custom_to_csr_id_map[&edge.vertices[target_index]],
        Factor {
            factor_type,
            constraint: edge.restriction.to_vec(),
            information_matrix: edge.information_matrix.to_vec().into(),
        },
    );
}

fn add_vertex(factor_graph: &mut FactorGraph, vertex: &Vertex, fixed: bool) {
    match vertex.vertex_type.as_str() {
        "POSE2D_ANGLE" => factor_graph.node_indices.push(
            factor_graph.csr
                .add_node(Box::new(VehicleVariable2D::new(
                    vertex.id,
                    vertex.content[0],
                    vertex.content[1],
                    vertex.content[2],
                    fixed,
                    add_var_to_matrix(&mut factor_graph.matrix_dim, 3, fixed),
                ))),
        ),
        "LANDMARK2D_ANGLE" => factor_graph.node_indices.push(
            factor_graph.csr
                .add_node(Box::new(LandmarkVariable2D::new(
                    vertex.id,
                    vertex.content[0],
                    vertex.content[1],
                    fixed,
                    add_var_to_matrix(&mut factor_graph.matrix_dim, 2, fixed),
                ))),
        ),
        "POSE3D_ANGLE" => factor_graph.node_indices.push(
            factor_graph.csr
                .add_node(Box::new(VehicleVariable3D::new(
                    vertex.id,
                    vertex.content[0],
                    vertex.content[1],
                    vertex.content[2],
                    vertex.content[3],
                    vertex.content[4],
                    vertex.content[5],
                    vertex.content[6],
                    fixed,
                    add_var_to_matrix(&mut factor_graph.matrix_dim, 7, fixed),
                ))),
        ),
        other_type => panic!("Unsupported vertex type in the model: {}", other_type),
    };
    factor_graph.custom_to_csr_id_map.insert(vertex.id, *factor_graph.node_indices.last().unwrap());
}

fn add_var_to_matrix(dim: &mut usize, added_dim: usize, fixed: bool) -> Option<Range<usize>> {
    if fixed {
        None
    } else {
        *dim += added_dim;
        Some(*dim-added_dim..*dim)
    }
}