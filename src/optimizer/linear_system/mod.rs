// -----------------------------------------------------------------------------------------------------
//                                      gs-rs - Graph SLAM in Rust
// -----------------------------------------------------------------------------------------------------
//
// SPDX-FileCopyrightText:      © 2020 Samuel Valenzuela (samuel.valenzuela@tngtech.com)
//                              © 2020 Florian Rohm (florian.rohm@tngtech.com)
//                              © 2020 Daniel Pape (daniel.pape@tngtech.com)
// SPDX-License-Identifier:     MIT OR Apache-2.0
//
// This product includes software developed at TNG Technology Consulting GmbH (https://www.tngtech.com/).
//


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

    factor_graph
        .node_indices
        .iter()
        .map(|i| factor_graph.csr.edges(*i))
        .for_each(|edges| {
            edges
                .into_iter()
                .for_each(|edge| update_H_b(factor_graph, &mut H, &mut b, edge))
        });

    (H, b)
}

fn update_H_b(
    factor_graph: &FactorGraph,
    H: &mut DMatrix<f64>,
    b: &mut DVector<f64>,
    edge: EdgeReference<Factor, Directed, usize>,
) {
    use crate::factor_graph::variable::Variable::*;
    let factor = edge.weight();
    let var_i = &factor_graph.get_var(edge.source());
    let var_j = &factor_graph.get_var(edge.target());

    match (&factor.factor_type, var_i, var_j) {
        (Position2D, Vehicle2D(var_i), _) => pos2d_handler::update_H_b(H, b, factor, var_i),
        (Odometry2D, Vehicle2D(var_i), Vehicle2D(var_j)) => odo2d_handler::update_H_b(H, b, factor, var_i, var_j),
        (Observation2D, Vehicle2D(var_i), Landmark2D(var_j)) => obs2d_handler::update_H_b(H, b, factor, var_i, var_j),
        (Position3D, Vehicle3D(var_i), _) => pos3d_handler::update_H_b(H, b, factor, var_i),
        (Odometry3D, Vehicle3D(var_i), Vehicle3D(var_j)) => odo3d_handler::update_H_b(H, b, factor, var_i, var_j),
        (Observation3D, Vehicle3D(var_i), Landmark3D(var_j)) => obs3d_handler::update_H_b(H, b, factor, var_i, var_j),
        _ => unreachable!("No valid edge."),
    }
}
