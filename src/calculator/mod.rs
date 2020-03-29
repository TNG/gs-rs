#![allow(non_snake_case)]

use std::borrow::BorrowMut;
use std::ops::{Deref, Index};

use itertools::Itertools;

use crate::calculator::linear_system::calculate_H_b;
use crate::factor_graph::FactorGraph;
use crate::solver::sparse_cholesky::SparseCholeskySolver;
use crate::solver::Solver;

pub mod linear_system;


pub fn optimize(graph: &FactorGraph, iterations: usize) {
    for _i in 0..iterations {
        update_once(graph);
    }
}

fn update_once(graph: &FactorGraph) {
    let (H, b) = calculate_H_b(&graph);
    // TODO clumsy, since the solver transforms the arguments back to nalgebra matrices
    let bla = SparseCholeskySolver::solve(&b, H);
    izip!(&graph.node_indices, bla.unwrap().chunks(3).collect_vec())
        .for_each(|(node_index, value)| update_single_variable(&graph, *node_index, value));
}

fn update_single_variable(graph: &FactorGraph, node_index: usize, value: &[f64]) {
    let mut var = graph.csr.index(node_index);
    let bla = var.borrow_mut().deref().get_pose();
    var.update_pose(izip!(&bla, value).map(|(x, y)| x + y).collect_vec());
}


#[cfg(test)]
mod tests {
    use crate::calculator::optimize;
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
        let graph = JsonParser::parse_file_to_model("test_files/graphSLAM2dExample.json").unwrap().into();
        info!("{:?}", SystemTime::now());
        optimize(&graph, 1);
        info!("{:?}", SystemTime::now());
    }

    #[test]
    fn iter_ten_times() {
        init();
        let graph = JsonParser::parse_file_to_model("test_files/graphSLAM2dExample.json").unwrap().into();
        info!("{:?}", SystemTime::now());
        optimize(&graph, 10);
        info!("{:?}", SystemTime::now());
    }

}