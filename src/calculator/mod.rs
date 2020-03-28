#![allow(non_snake_case)]

use std::borrow::BorrowMut;
use std::ops::{Deref, Index};

use itertools::Itertools;

use crate::calculator::linear_system::calculate_H_b;
use crate::factor_graph::FactorGraph;
use crate::factor_graph::variable::Variable;
use crate::solver::cholesky::CholeskySolver;
use crate::solver::Solver;

pub mod linear_system;


pub fn optimize(graph: &FactorGraph, iterations: usize) {
    for i in 0..iterations {
        update_once(graph);
    }
}

fn update_once(graph: &FactorGraph) {
    let (H, b) = calculate_H_b(&graph);
    // TODO clumsy, since the solver transforms the arguments back to nalgebra matrices
    let bla = CholeskySolver::solve(b.data.as_vec().as_slice(), H.as_slice());
    izip!(&graph.node_indices, bla.unwrap().chunks(3).collect_vec())
        .for_each(|(node_index, value)| update_single_variable(&graph, *node_index, value));
}

fn update_single_variable(graph: &FactorGraph, node_index: usize, value: &[f64]) {
    let mut var = graph.csr.index(node_index);
    let bla = var.borrow_mut().deref().get_pose();
    dbg!(&bla);
    var.update_pose(izip!(&bla, value).map(|(x, y)| x + y).collect_vec());
    dbg!(&bla);
}


#[cfg(test)]
mod tests {
    use crate::calculator::{update_once, optimize};
    use crate::factor_graph::FactorGraph;
    use crate::parser::json::JsonParser;
    use crate::parser::model::FactorGraphModel;
    use crate::parser::Parser;
    use crate::visualization::visualize;

    #[test]
    fn iter_once() {
        let graph = JsonParser::parse_file_to_model("test_files/testTrajectory2DAngle.json").unwrap().into();
       // dbg!(&graph);
        optimize(&graph, 1);
        visualize(&graph);
    }

    #[test]
    fn iter_ten_times() {
        let graph = JsonParser::parse_file_to_model("test_files/testTrajectory2DAngle.json").unwrap().into();
        dbg!(&graph);
        optimize(&graph, 10);
        visualize(&graph);
    }

}