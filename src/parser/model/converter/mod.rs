use petgraph::csr::Csr;
use crate::parser::model::{FactorGraphModel, Edge, Vertex};
use crate::factor_graph::FactorGraph;
use crate::factor_graph::factor::Factor::{PositionFactor2D, OdometryFactor2D, ObservationFactor2D};
use crate::factor_graph::variable::{vehicle_variable_2d::VehicleVariable2D, landmark_variable_2d::LandmarkVariable2D};

impl From<FactorGraphModel> for FactorGraph<'_> {
    fn from(model: FactorGraphModel) -> Self {
        let mut csr: FactorGraph = Csr::new();

        // TODO replace by some kind of for_each
        model.vertices.iter()
            .map(|x| add_vertex(&mut csr, x))
            .filter_map(Result::ok)
            .for_each(drop);

        // TODO replace by some kind of for_each
        model.edges.iter()
            .map(|x| add_edge(&mut csr, x))
            .filter_map(Result::ok)
            .filter(|x| *x == true)
            .for_each(drop);

        csr
    }
}

// TODO implement to be able to compose JSON files
impl From<FactorGraph<'_>> for FactorGraphModel {
    fn from(_: FactorGraph) -> Self {
        unimplemented!()
    }
}


// auxiliary functions

fn add_edge(csr: &mut FactorGraph, edge: &Edge) -> Result<bool, String> {
    match edge.edge_type.as_str() {
        "PRIOR2D_ANGLE" => Ok(csr.add_edge(edge.vertices[0], edge.vertices[0], PositionFactor2D(edge.restriction[0], edge.restriction[1], edge.restriction[2], edge.information_matrix.to_vec().into()))),
        "ODOMETRY2D_ANGLE" => Ok(csr.add_edge(edge.vertices[0], edge.vertices[1], OdometryFactor2D(edge.restriction[0], edge.restriction[1], edge.restriction[2], edge.information_matrix.to_vec().into()))),
        "OBSERVATION2D_ANGLE" => Ok(csr.add_edge(edge.vertices[0], edge.vertices[1], ObservationFactor2D(edge.restriction[0], edge.restriction[1], edge.restriction[2], edge.information_matrix.to_vec().into()))),
        _ => {
            error!("Could not add edge {:?}", edge);
            Err(format!("Could not add edge {:?}", edge))
        }
    }
}

fn add_vertex(csr: &mut FactorGraph, vertex: &Vertex) -> Result<usize, String> {
    match vertex.vertex_type.as_str() {
        "POSE2D_ANGLE" => Ok(csr.add_node(Box::new(VehicleVariable2D::from_pose_and_id(vertex.id, vertex.position[0], vertex.position[1], vertex.rotation[0])))),
        "LANDMARK2D_ANGLE" => Ok(csr.add_node(Box::new(LandmarkVariable2D::from_pose_and_id(vertex.id, vertex.position[0], vertex.position[1], vertex.rotation[0])))),
        _ => {
            error!("Could not add vertex {:?}", vertex);
            Err(format!("Could not add vertex {:?}", vertex))
        }
    }
}