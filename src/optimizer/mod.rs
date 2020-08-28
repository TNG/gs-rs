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


//! Improves variable estimates to better fit measurements.

#![allow(non_snake_case)]

use crate::factor_graph::variable::{FixedType, Variable};
use crate::factor_graph::FactorGraph;
use crate::optimizer::linear_system::calculate_H_b;
use crate::optimizer::linear_system::iso3d_gradients::{get_isometry, get_isometry_normalized};
use crate::optimizer::solver::sparse_cholesky::SparseCholeskySolver;
use crate::optimizer::solver::Solver;
use std::f64::consts::PI;

mod linear_system;
mod solver;

/// Optimizes a factor graph with the given number of iterations.
pub fn optimize(graph: &FactorGraph, iterations: usize) {
    for _i in 0..iterations {
        update_once(graph);
    }
}

fn update_once(factor_graph: &FactorGraph) {
    let (H, b) = calculate_H_b(&factor_graph);
    // TODO @Daniel: clumsy, since the solver transforms the arguments back to nalgebra matrices
    let sol = SparseCholeskySolver::solve(H, &(b * -1.0)).unwrap();
    factor_graph
        .node_indices
        .iter()
        .map(|i| factor_graph.get_var(*i))
        .for_each(|var| update_var(var, sol.as_slice()));
}

fn update_var(var: &Variable, solution: &[f64]) {
    let correction = if let FixedType::NonFixed(range) = var.get_fixed_type() {
        &solution[range.to_owned()]
    } else {
        return;
    };

    let updated_content = match var {
        Variable::Vehicle2D(var) => {
            let mut updated_content: Vec<f64> = var
                .pose
                .borrow()
                .iter()
                .zip(correction.iter())
                .map(|(old, cor)| old + cor)
                .collect();
            updated_content[2] %= 2.0 * PI;
            if updated_content[2] > PI {
                updated_content[2] -= 2.0 * PI;
            } else if updated_content[2] < -PI {
                updated_content[2] += 2.0 * PI;
            }
            updated_content
        }
        Variable::Landmark2D(var) => var
            .position
            .borrow()
            .iter()
            .zip(correction.iter())
            .map(|(old, cor)| old + cor)
            .collect(),
        Variable::Vehicle3D(var) => {
            let old_iso = get_isometry(&*var.pose.borrow());
            let cor_iso = get_isometry_normalized(correction);
            let new_iso = old_iso * cor_iso;
            let mut updated_content = new_iso.translation.vector.data.to_vec();
            updated_content.extend(&new_iso.rotation.quaternion().coords.data.to_vec());
            updated_content
        }
        Variable::Landmark3D(var) => var
            .position
            .borrow()
            .iter()
            .zip(correction.iter())
            .map(|(old, cor)| old + cor)
            .collect(),
    };
    var.set_content(updated_content);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::g2o::G2oParser;
    use crate::parser::model::FactorGraphModel;
    use crate::parser::Parser;

    use log::LevelFilter;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    fn test_valid_optimization(file_name: &str, iterations: usize) {
        init();
        let test_factor_graph =
            G2oParser::parse_file(&["data_files/optimizer_tests/", file_name, "_0.g2o"].concat()).unwrap();
        optimize(&test_factor_graph, iterations);
        let test_model = FactorGraphModel::from(&test_factor_graph);
        let expected_model = G2oParser::parse_file_to_model(
            &[
                "data_files/optimizer_tests/",
                file_name,
                "_",
                &iterations.to_string(),
                ".g2o",
            ]
            .concat(),
        )
        .unwrap();
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
    fn test_mainly_obs2d_factors() {
        test_valid_optimization("obs2d_mainly", 1);
    }

    #[test]
    fn test_all_2d_factors() {
        test_valid_optimization("full2d", 1);
    }

    #[test]
    fn test_multiple_iterations_2d() {
        test_valid_optimization("full2d", 25);
    }

    #[test]
    #[ignore] // floating point rounding makes results not equal
    fn test_only_pos3d_factors() {
        test_valid_optimization("pos3d_only", 1);
    }

    #[test]
    fn test_only_odo3d_factors() {
        test_valid_optimization("odo3d_only", 1);
    }

    #[test]
    fn test_mainly_obs3d_factors() {
        test_valid_optimization("obs3d_mainly", 1);
    }
}