use petgraph::csr::Csr;

use crate::factor_graph::factor::{Factor, FactorType::*};
use crate::factor_graph::variable::{
    VariableType::*, landmark_variable_2d::LandmarkVariable2D, vehicle_variable_2d::VehicleVariable2D,
};
use crate::factor_graph::FactorGraph;
use crate::parser::model::{Edge, FactorGraphModel, Vertex};
use std::ops::Index;
use petgraph::visit::EdgeRef;
use std::collections::HashSet;

impl From<FactorGraphModel> for FactorGraph<'_> {
    fn from(model: FactorGraphModel) -> Self {
        let mut factor_graph = FactorGraph {
            csr: Csr::new(),
            node_indices: vec![],
            number_of_dynamic_nodes: 0,
        };

        // TODO replace by some kind of for_each
        model
            .vertices
            .iter()
            .for_each(|v| add_vertex(&mut factor_graph, v, model.fixed_vertices.contains(&v.id)));

        // TODO replace by some kind of for_each
        model
            .edges
            .iter()
            .for_each(|e| add_edge(&mut factor_graph, e));

        factor_graph
    }
}

impl From<&FactorGraph<'_>> for FactorGraphModel {
    fn from(factor_graph: &FactorGraph) -> Self {
        let mut model = FactorGraphModel { vertices: vec![], edges: vec![], fixed_vertices: HashSet::new() };
        for node_index in &factor_graph.node_indices {
            let node = factor_graph.csr.index(*node_index);
            model.vertices.push(Vertex {
                id: node.get_id().as_u128() as usize,
                vertex_type: match node.get_type() {
                    Vehicle2D => String::from("POSE2D_ANGLE"),
                    Landmark2D => String::from("LANDMARK2D_ANGLE"),
                },
                position: [node.get_pose()[0], node.get_pose()[1]],
                rotation: [node.get_pose()[2]],
            });
            for edge in factor_graph.csr.edges(*node_index) {
                let factor: &Factor = edge.weight();
                let mut edge_vertices = vec![node.get_id().as_u128() as usize];
                if edge.target() != *node_index {
                    edge_vertices.push(factor_graph.csr.index(edge.target()).get_id().as_u128() as usize);
                }
                let mut restriction_arr = [0.0; 3];
                restriction_arr.copy_from_slice(&factor.constraint.as_slice());
                let mut information_matrix_arr = [0.0; 9];
                information_matrix_arr.copy_from_slice(&factor.information_matrix.content.as_slice());
                model.edges.push(Edge {
                    edge_type: match factor.factor_type {
                        Position2D => String::from("PRIOR2D_ANGLE"),
                        Odometry2D => String::from("ODOMETRY2D_ANGLE"),
                        Observation2D => String::from("OBSERVATION2D_ANGLE"),
                    },
                    vertices: edge_vertices,
                    restriction: [factor.constraint[0], factor.constraint[1], factor.constraint[2]],
                    information_matrix: information_matrix_arr,
                });
            }
            if node.is_fixed() {
                model.fixed_vertices.insert(node.get_id().as_u128() as usize);
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
        _ => {
            error!("Could not add edge {:?}", edge);
            return;
        }
    };
    factor_graph.csr.add_edge(
        edge.vertices[0],
        edge.vertices[target_index],
        Factor {
            factor_type,
            constraint: edge.restriction.to_vec(),
            information_matrix: edge.information_matrix.to_vec().into(),
        },
    );
}

fn add_vertex(factor_graph: &mut FactorGraph, vertex: &Vertex, fixed: bool) {
    match vertex.vertex_type.as_str() {
        "POSE2D_ANGLE" => factor_graph
            .node_indices
            .push(
                factor_graph
                    .csr
                    .add_node(Box::new(VehicleVariable2D::from_full_config(
                        vertex.id,
                        vertex.position[0],
                        vertex.position[1],
                        vertex.rotation[0],
                        fixed,
                        factor_graph.number_of_dynamic_nodes
                    ))),
            ),
        "LANDMARK2D_ANGLE" => factor_graph
            .node_indices
            .push(
                factor_graph
                    .csr
                    .add_node(Box::new(LandmarkVariable2D::from_full_config(
                        vertex.id,
                        vertex.position[0],
                        vertex.position[1],
                        vertex.rotation[0],
                        fixed,
                        factor_graph.number_of_dynamic_nodes
                    ))),
            ),
        _ => {
            error!("Could not add vertex {:?}", vertex);
        }
    };
    if !fixed {
        factor_graph.number_of_dynamic_nodes += 1;
    }
}
