use petgraph::csr::Csr;
use petgraph::Undirected;

// TODO reorganize imports after adding all factors and variables
use crate::factor_graph::factor::Factor;
use crate::factor_graph::factor::FactorType::Odometry2D;
use crate::factor_graph::factor::odometry_factor_2d::OdometryFactor2D;
use crate::factor_graph::variable::Variable;
use crate::factor_graph::variable::vehicle_variable_2d::VehicleVariable2D;
use crate::factor_graph::FactorTest::{OdoFactor, PositionFactor};
use crate::parser::model::{Edge, FactorGraphModel, Vertex};
use nalgebra::{Dynamic, VecStorage, DMatrix};

pub mod factor;
pub mod variable;

#[derive(Debug, Clone)]
pub struct InformationMatrix {
    content: DMatrix<f64>,
}

impl  From<Vec<f64>> for InformationMatrix {
    fn from(content: Vec<f64>) -> Self {
        let dim = (content.len() as f64).sqrt() as usize;
        InformationMatrix {
            content: DMatrix::from_data(VecStorage::new(Dynamic::new(dim), Dynamic::new(dim), content.into()))
        }
    }
}

#[derive(Debug, Clone)]
pub enum FactorTest {
    OdoFactor(f64, f64, f64, InformationMatrix),
    PositionFactor(f64, f64, f64, InformationMatrix),
}

pub type FactorGraph<'a> = Csr<Box<dyn Variable<'a>>, FactorTest, Undirected, usize>;

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

fn add_edge(csr: &mut FactorGraph, edge: &Edge) -> Result<bool, String> {
    match edge.edge_type.as_str() {
        "PRIOR2D_ANGLE" => Ok(csr.add_edge(edge.vertices[0], edge.vertices[0], PositionFactor(edge.restriction[0], edge.restriction[1], edge.restriction[2], edge.information_matrix.to_vec().into()))),
        "ODOMETRY2D_ANGLE" => Ok(csr.add_edge(edge.vertices[0], edge.vertices[1], OdoFactor(edge.restriction[0], edge.restriction[1], edge.restriction[2], edge.information_matrix.to_vec().into()))),
        _ => {
            error!("Could not add edge {:?}", edge);
            Err(format!("Could not add edge {:?}", edge))
        }
    }
}

fn add_vertex(csr: &mut FactorGraph, vertex: &Vertex) -> Result<usize, String> {
    match vertex.vertex_type.as_str() {
        "POSE2D_ANGLE" => Ok(csr
            .add_node(Box::new(VehicleVariable2D::from_pose_and_id(vertex.id, vertex.position[0], vertex.position[1], vertex.rotation[0])))),
        _ => {
            error!("Could not add vertex {:?}", vertex);
            Err(format!("Could not add vertex {:?}", vertex))
        }
    }
}

#[cfg(test)]
mod tests {
    use log::LevelFilter;

    use crate::parser::json::JsonParser;
    use crate::parser::Parser;

    use super::*;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn factor_graph_from_json() {
        let parsed_model: FactorGraph = match JsonParser::parse_file("test_files/testTrajectory2DAngle.json") {
            Ok(x) => x,
            Err(str) => panic!(str),
        };
        dbg!(&parsed_model);
    }

    #[test]
    #[should_panic]
    fn factor_graph_from_broken_json() {
        init();

        let parsed_model: FactorGraph = match JsonParser::parse_file("test_files/testBrokenTrajectory2DAngle.json") {
            Ok(x) => x,
            Err(str) => panic!(str),
        };
        error!("Test should panic!");
    }
}