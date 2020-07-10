#[cfg(test)]
#[macro_use]
extern crate approx;
extern crate itertools;
extern crate kiss3d;
#[macro_use]
extern crate log;
extern crate nalgebra;
extern crate petgraph;

use crate::parser::g2o::G2oParser;
use crate::parser::Parser;

pub mod factor_graph;
pub mod optimizer;
pub mod parser;
pub mod visualizer;

/// Example for the usage of the library
pub fn optimize(in_file: &str, out_file: &str, iterations: usize) {
    let factor_graph = G2oParser::parse_file(in_file).unwrap();

    optimizer::optimize(&factor_graph, iterations);

    G2oParser::compose_file(&factor_graph, out_file).unwrap();
}
