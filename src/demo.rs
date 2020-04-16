// TODO delete this file once done testing with this

use crate::optimizer::optimize;
use crate::parser::json::JsonParser;
use crate::parser::Parser;
use crate::visualizer::visualize;
use crate::parser::g2o::G2oParser;
use std::collections::BTreeSet;

#[test]
fn current_test() {
    let factor_graph = match G2oParser::parse_file("data_files/optimizer_tests/full2d_0.g2o") {
        Ok(factor_graph) => factor_graph,
        Err(str) => panic!(str),
    };
    optimize(&factor_graph, 25);
    // G2oParser::compose_file(&factor_graph, "data_files/optimizer_tests/full2d_25.g2o");
}