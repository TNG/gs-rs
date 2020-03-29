use petgraph::csr::Csr;

use crate::factor_graph::factor::{Factor, FactorType::*};
use crate::factor_graph::variable::{
    landmark_variable_2d::LandmarkVariable2D, vehicle_variable_2d::VehicleVariable2D,
};
use crate::factor_graph::FactorGraph;
use crate::parser::model::{Edge, FactorGraphModel, Vertex};

impl From<FactorGraphModel> for FactorGraph<'_> {
    fn from(model: FactorGraphModel) -> Self {
        let mut factor_graph = FactorGraph {
            csr: Csr::new(),
            node_indices: vec![],
        };

        // TODO replace by some kind of for_each
        model
            .vertices
            .iter()
            .for_each(|x| add_vertex(&mut factor_graph, x));

        // TODO replace by some kind of for_each
        model
            .edges
            .iter()
            .for_each(|x| add_edge(&mut factor_graph, x));

        factor_graph
    }
}

// TODO implement to be able to compose JSON files
impl From<FactorGraph<'_>> for FactorGraphModel {
    fn from(_: FactorGraph) -> Self {
        unimplemented!();
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

fn add_vertex(factor_graph: &mut FactorGraph, vertex: &Vertex) {
    match vertex.vertex_type.as_str() {
        "POSE2D_ANGLE" => factor_graph
            .node_indices
            .push(
                factor_graph
                    .csr
                    .add_node(Box::new(VehicleVariable2D::from_pose_and_id(
                        vertex.id,
                        vertex.position[0],
                        vertex.position[1],
                        vertex.rotation[0],
                    ))),
            ),
        "LANDMARK2D_ANGLE" => factor_graph
            .node_indices
            .push(
                factor_graph
                    .csr
                    .add_node(Box::new(LandmarkVariable2D::from_pose_and_id(
                        vertex.id,
                        vertex.position[0],
                        vertex.position[1],
                        vertex.rotation[0],
                    ))),
            ),
        _ => {
            error!("Could not add vertex {:?}", vertex);
        }
    }
}
