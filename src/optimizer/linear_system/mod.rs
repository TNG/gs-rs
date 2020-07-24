use crate::factor_graph::factor::{Factor, FactorType::*};
use crate::factor_graph::FactorGraph;
use nalgebra::{DMatrix, DVector};
use petgraph::csr::EdgeReference;
use petgraph::visit::EdgeRef;
use petgraph::Directed;

mod obs2d_handler;
mod odo2d_handler;
mod pos2d_handler;

pub mod iso3d_gradients;
mod obs3d_handler;
mod odo3d_handler;
mod pos3d_handler;

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
    use crate::factor_graph::variable::Variable::*;
    let factor = edge.weight();
    let var_i = &factor_graph.get_var(edge.source());
    let var_j = &factor_graph.get_var(edge.target());

    match (var_i, var_j) {
        (Vehicle2D(_), Vehicle2D(_)) => {}
        (Vehicle2D(_), Landmark2D(_)) => {}
        (Vehicle2D(_), Vehicle3D(_)) => {}
        (Vehicle2D(_), Landmark3D(_)) => {}
        (Landmark2D(_), Vehicle2D(_)) => {}
        (Landmark2D(_), Landmark2D(_)) => {}
        (Landmark2D(_), Vehicle3D(_)) => {}
        (Landmark2D(_), Landmark3D(_)) => {}
        (Vehicle3D(_), Vehicle2D(_)) => {}
        (Vehicle3D(_), Landmark2D(_)) => {}
        (Vehicle3D(_), Vehicle3D(_)) => {}
        (Vehicle3D(_), Landmark3D(_)) => {}
        (Landmark3D(_), Vehicle2D(_)) => {}
        (Landmark3D(_), Landmark2D(_)) => {}
        (Landmark3D(_), Vehicle3D(_)) => {}
        (Landmark3D(_), Landmark3D(_)) => {}
    }

    match factor.factor_type {
        Position2D    => pos2d_handler::update_H_b(H, b, factor, var_i),
        Odometry2D    => odo2d_handler::update_H_b(H, b, factor, var_i, var_j),
        Observation2D => obs2d_handler::update_H_b(H, b, factor, var_i, var_j),
        Position3D    => pos3d_handler::update_H_b(H, b, factor, var_i),
        Odometry3D    => odo3d_handler::update_H_b(H, b, factor, var_i, var_j),
        Observation3D => obs3d_handler::update_H_b(H, b, factor, var_i, var_j),
    };
}
