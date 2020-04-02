//! This file should replace linear_system.rs once completed.

use crate::factor_graph::FactorGraph;
use nalgebra::{DMatrix, DVector};
use crate::factor_graph::variable::Variable;
use petgraph::csr::Edges;
use crate::factor_graph::factor::Factor;
use petgraph::Directed;

pub fn calculate_H_b(factor_graph: &FactorGraph) -> (DMatrix<f64>, DVector<f64>) {
    let dim = factor_graph.number_of_dynamic_nodes * 3;
    let mut H = DMatrix::from_vec(dim, dim, vec![0.0; dim * dim]);
    let mut b = DVector::from_vec(vec![0.0; dim]);

    factor_graph.node_indices.iter()
        .map(|i| (factor_graph.get_var_at_csr_index(*i), factor_graph.csr.edges(*i)))
        .filter(|(var, edges)| !var.is_fixed())
        .for_each(|(var, edges)| update_H_b(&mut H, &mut b, var, edges));

    (H, b)
}

fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, var: &Box<dyn Variable>, edges: Edges<Factor, Directed, usize>) {
    // TODO implement
}
