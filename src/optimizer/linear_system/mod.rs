use crate::factor_graph::FactorGraph;
use nalgebra::{DMatrix, DVector};
use petgraph::csr::EdgeReference;
use crate::factor_graph::factor::{Factor, FactorType::*};
use petgraph::Directed;
use petgraph::visit::EdgeRef;

mod pos2d_handler;
mod odo2d_handler;
mod obs2d_handler;

pub mod iso3d_gradients;
mod pos3d_handler;
mod odo3d_handler;

pub fn calculate_H_b(factor_graph: &FactorGraph) -> (DMatrix<f64>, DVector<f64>) {
    let dim = factor_graph.matrix_dim;
    let mut H = DMatrix::from_vec(dim, dim, vec![0.0; dim * dim]);
    let mut b = DVector::from_vec(vec![0.0; dim]);

    factor_graph.node_indices.iter()
        .map(|i| factor_graph.csr.edges(*i))
        .for_each(|edges| edges.into_iter()
            .for_each(|edge| update_H_b(factor_graph, &mut H, &mut b, edge)));

    (H, b)
}

fn update_H_b(factor_graph: &FactorGraph, H: &mut DMatrix<f64>, b: &mut DVector<f64>, edge: EdgeReference<Factor, Directed, usize>) {
    let factor = edge.weight();
    match factor.factor_type {
        Position2D => pos2d_handler::update_H_b(H, b, factor, &factor_graph.get_var(edge.source())),
        Odometry2D => odo2d_handler::update_H_b(H, b, factor, &factor_graph.get_var(edge.source()), &factor_graph.get_var(edge.target())),
        Observation2D => obs2d_handler::update_H_b(H, b, factor, &factor_graph.get_var(edge.source()), &factor_graph.get_var(edge.target())),
        Position3D => pos3d_handler::update_H_b(H, b, factor, &factor_graph.get_var(edge.source())),
        Odometry3D => odo3d_handler::update_H_b(H, b, factor, &factor_graph.get_var(edge.source()), &factor_graph.get_var(edge.target())),
        Observation3D => unimplemented!(),
    };
}
