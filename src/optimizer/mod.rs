//! Improves variable estimates to better fit measurements.

#![allow(non_snake_case)]

use std::borrow::BorrowMut;
use std::ops::{Deref, Index};

use itertools::Itertools;

use crate::factor_graph::FactorGraph;
use crate::optimizer::linear_system::calculate_H_b;
use crate::solver::sparse_cholesky::SparseCholeskySolver;
use crate::solver::Solver;
use std::f64::consts::PI;
use crate::factor_graph::variable::Variable;

mod linear_system;

/// Optimizes a factor graph with the given number of iterations.
pub fn optimize(graph: &FactorGraph, iterations: usize) {
    for _i in 0..iterations {
        update_once(graph);
    }
}

fn update_once(factor_graph: &FactorGraph) {
    let (H, b) = calculate_H_b(&factor_graph);
    // TODO clumsy, since the solver transforms the arguments back to nalgebra matrices
    let solution = SparseCholeskySolver::solve(H, &b);
    factor_graph.node_indices.iter()
        .map(|i| factor_graph.get_var_at_csr_index(*i))
        .for_each(|var| update_var(var, solution.as_ref().unwrap().as_slice()));
}

fn update_var(var: &Box<dyn Variable>, solution: &[f64]) {
    if var.is_fixed() {
        return;
    }
    let old_pose = var.get_pose();
    let index = var.get_index().unwrap();
    let correction = &solution[3*index..3*index+3];
    let updated_pose = vec![old_pose[0] + correction[0],
                            old_pose[1] + correction[1],
                            (old_pose[2] + correction[2]) % (2.0 * PI) - PI];
    var.update_pose(updated_pose);
}

#[cfg(test)]
mod tests {
    use crate::optimizer::optimize;
    use crate::parser::json::JsonParser;
    use crate::parser::Parser;

    use log::LevelFilter;
    use std::time::SystemTime;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn iter_once() {
        init();
        let graph = JsonParser::parse_file_to_model("test_files/graphSLAM2dExample.json")
            .unwrap()
            .into();
        info!("{:?}", SystemTime::now());
        optimize(&graph, 1);
        info!("{:?}", SystemTime::now());
    }

    #[test]
    fn iter_ten_times() {
        init();
        let graph = JsonParser::parse_file_to_model("test_files/graphSLAM2dExample.json")
            .unwrap()
            .into();
        info!("{:?}", SystemTime::now());
        optimize(&graph, 10);
        info!("{:?}", SystemTime::now());
    }
}
