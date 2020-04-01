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
    optimize(&factor_graph, 5);
    visualize(&factor_graph);
}

use crate::parser::g2o::G2oParser;

#[test]
fn g2o_test() {
    let factor_graph = match G2oParser::parse_file("test_files/dumb.g2o") {
        Ok(factor_graph) => factor_graph,
        Err(str) => panic!(str),
    };
    JsonParser::compose_file(&factor_graph, "test_files/dumb.json");
    optimize(&factor_graph, 1);
    JsonParser::compose_file(&factor_graph, "test_files/dumb_out.json");
}
