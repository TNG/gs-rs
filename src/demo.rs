// TODO delete this file once done testing with this

use crate::optimizer::optimize;
use crate::parser::json::JsonParser;
use crate::parser::Parser;
use crate::visualizer::visualize;

#[test]
fn main() {
    let factor_graph = match JsonParser::parse_file("test_files/fullTestGraph.json") {
        Ok(factor_graph) => factor_graph,
        Err(str) => panic!(str),
    };
    optimize(&factor_graph, 0);
    visualize(&factor_graph);
}

use crate::parser::g2o::G2oParser;
use std::collections::HashSet;

#[test]
fn rotation_change_test() {
    let factor_graph = match G2oParser::parse_file("test_files/MIT_simple2D.g2o") {
        Ok(factor_graph) => factor_graph,
        Err(str) => panic!(str),
    };
    optimize(&factor_graph, 1);
    G2oParser::compose_file(&factor_graph, "test_files/MIT_simple2D_out.g2o");
}
