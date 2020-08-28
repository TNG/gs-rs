use petgraph::csr::Csr;

use crate::factor_graph::factor::{Factor, FactorType::*};

use crate::factor_graph::{
    variable::{FixedType, LandmarkVariable2D, LandmarkVariable3D, Variable, VehicleVariable2D, VehicleVariable3D},
    FactorGraph,
};
use crate::parser::model::{Edge, FactorGraphModel, Vertex};
use petgraph::visit::EdgeRef;
use std::collections::{BTreeSet, HashMap};
use std::ops::Index;

impl From<FactorGraphModel> for FactorGraph {
    fn from(model: FactorGraphModel) -> Self {
        let mut factor_graph = FactorGraph {
            csr: Csr::new(),
            node_indices: vec![],
            matrix_dim: 0,
            custom_to_csr_id_map: HashMap::new(),
        };

        model
            .vertices
            .iter()
            .for_each(|v| add_vertex(&mut factor_graph, v, model.fixed_vertices.contains(&v.id)));

        model.edges.iter().for_each(|e| add_edge(&mut factor_graph, e));

        factor_graph
    }
}

impl From<&FactorGraph> for FactorGraphModel {
    fn from(factor_graph: &FactorGraph) -> Self {
        let mut model = FactorGraphModel {
            vertices: vec![],
            edges: vec![],
            fixed_vertices: BTreeSet::new(),
        };
        for node_index in &factor_graph.node_indices {
            let node = factor_graph.csr.index(*node_index);
            model.vertices.push(Vertex {
                id: node.get_id(),
                vertex_type: match node {
                    Variable::Vehicle2D(_) => String::from("Vehicle2D"),
                    Variable::Landmark2D(_) => String::from("Landmark2D"),
                    Variable::Vehicle3D(_) => String::from("Vehicle3D"),
                    Variable::Landmark3D(_) => String::from("Landmark3D"),
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
                        Position2D => String::from("Position2D"),
                        Odometry2D => String::from("Odometry2D"),
                        Observation2D => String::from("Observation2D"),
                        Position3D => String::from("Position3D"),
                        Odometry3D => String::from("Odometry3D"),
                        Observation3D => String::from("Observation3D"),
                    },
                    vertices: edge_vertices,
                    restriction: factor.constraint.clone(),
                    information_matrix: factor.information_matrix.content.as_slice().to_owned(),
                });
            }
            if node.get_fixed_type() == &FixedType::Fixed {
                model.fixed_vertices.insert(node.get_id());
            }
        }
        model
    }
}

fn add_edge(factor_graph: &mut FactorGraph, edge: &Edge) {
    let (target_index, factor_type) = match edge.edge_type.as_str() {
        "Position2D" => (0, Position2D),
        "Odometry2D" => (1, Odometry2D),
        "Observation2D" => (1, Observation2D),
        "Position3D" => (0, Position3D),
        "Odometry3D" => (1, Odometry3D),
        "Observation3D" => (1, Observation3D),
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
        "Vehicle2D" => factor_graph
            .node_indices
            .push(factor_graph.csr.add_node(Variable::Vehicle2D(VehicleVariable2D::new(
                vertex.id,
                vertex.content[0],
                vertex.content[1],
                vertex.content[2],
                add_var_to_matrix(&mut factor_graph.matrix_dim, 3, fixed),
            )))),
        "Landmark2D" => factor_graph
            .node_indices
            .push(factor_graph.csr.add_node(Variable::Landmark2D(LandmarkVariable2D::new(
                vertex.id,
                vertex.content[0],
                vertex.content[1],
                add_var_to_matrix(&mut factor_graph.matrix_dim, 2, fixed),
            )))),
        "Vehicle3D" => factor_graph
            .node_indices
            .push(factor_graph.csr.add_node(Variable::Vehicle3D(VehicleVariable3D::new(
                vertex.id,
                vertex.content[0],
                vertex.content[1],
                vertex.content[2],
                vertex.content[3],
                vertex.content[4],
                vertex.content[5],
                vertex.content[6],
                add_var_to_matrix(&mut factor_graph.matrix_dim, 6, fixed),
            )))),
        "Landmark3D" => factor_graph
            .node_indices
            .push(factor_graph.csr.add_node(Variable::Landmark3D(LandmarkVariable3D::new(
                vertex.id,
                vertex.content[0],
                vertex.content[1],
                vertex.content[2],
                add_var_to_matrix(&mut factor_graph.matrix_dim, 3, fixed),
            )))),
        other_type => panic!("Unsupported vertex type in the model: {}", other_type),
    };
    factor_graph
        .custom_to_csr_id_map
        .insert(vertex.id, *factor_graph.node_indices.last().unwrap());
}

fn add_var_to_matrix(dim: &mut usize, added_dim: usize, fixed: bool) -> FixedType {
    if fixed {
        FixedType::Fixed
    } else {
        *dim += added_dim;
        FixedType::NonFixed(*dim - added_dim..*dim)
    }
}
