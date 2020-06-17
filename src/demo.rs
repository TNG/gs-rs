use crate::optimizer::optimize;
use crate::parser::g2o::G2oParser;
use crate::parser::json::JsonParser;
use crate::parser::Parser;
use crate::visualizer::visualize;

// TODO @Samuel: go through ignored tests

// TODO @Samuel: remove all compiler warnings

#[test]
fn current_test() {
    // TODO @Samuel: delete this file if not used anymore
    let factor_graph = JsonParser::parse_file("data_files/bad3d.json").unwrap();
    optimize(&factor_graph, 1);
    G2oParser::compose_file(&factor_graph, "data_files/bad3d.g2o");

}
