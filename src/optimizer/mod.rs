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
    let sol = SparseCholeskySolver::solve(H, &(b * -1.0)).unwrap();
    factor_graph.node_indices.iter()
        .map(|i| factor_graph.get_var_at_csr_index(*i))
        .for_each(|var| update_var(var, sol.as_slice()));
}

fn update_var(var: &Box<dyn Variable>, solution: &[f64]) {
    if var.is_fixed() {
        return;
    }
    let old_pose = var.get_content();
    let range = var.get_range().unwrap();
    let correction = &solution[range];
    let mut updated_rot = (old_pose[2] + correction[2]) % (2.0 * PI);
    if updated_rot > PI {
        updated_rot -= 2.0 * PI;
    } else if updated_rot < -PI {
        updated_rot += 2.0 * PI;
    }
    let updated_pose = vec![old_pose[0] + correction[0], old_pose[1] + correction[1], updated_rot];
    var.set_content(updated_pose);
}

#[cfg(test)]
mod tests {
    use crate::optimizer::optimize;
    use crate::parser::g2o::G2oParser;
    use crate::parser::Parser;
    use crate::parser::model::FactorGraphModel;

    use log::LevelFilter;
    use std::time::SystemTime;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    fn test_valid_optimization(file_name: &str, iterations: usize) {
        init();
        let test_factor_graph = G2oParser::parse_file(&["test_files/optimizer_tests/", file_name, "_0.g2o"].concat()).unwrap();
        optimize(&test_factor_graph, iterations);
        let test_model = FactorGraphModel::from(&test_factor_graph);
        let expected_model = G2oParser::parse_file_to_model(&["test_files/optimizer_tests/", file_name, "_", &iterations.to_string(), ".g2o"].concat()).unwrap();
        assert_eq!(test_model, expected_model);
    }

    #[test]
    fn test_testing_function() {
        test_valid_optimization("pos2d_and_odo2d", 0);
    }

    #[test]
    fn test_only_pos2d_factors() {
        test_valid_optimization("pos2d_only", 1);
    }

    #[test]
    fn test_only_odo2d_factors() {
        test_valid_optimization("odo2d_only", 1);
   }

    #[test]
    fn test_pos2d_and_odo2d_factors() {
        test_valid_optimization("pos2d_and_odo2d", 1);
    }

    #[test]
    fn test_multiple_iterations() {
        test_valid_optimization("pos2d_and_odo2d", 25);
    }
}
