use crate::optimizer::optimize;
use crate::parser::g2o::G2oParser;
use crate::parser::json::JsonParser;
use crate::parser::Parser;
use crate::visualizer::visualize;

// TODO @Samuel: filter data_files

// TODO @Samuel: remove all compiler warnings

#[test]
fn current_test() {
    // TODO @Samuel: delete this file if not used anymore
    // let factor_graph = JsonParser::parse_file("data_files/full_demos/tiny_vehicle_only_2d.json").unwrap();
    // visualize(&factor_graph);
    // optimize(&factor_graph, 1);
    // G2oParser::compose_file(&factor_graph, "data_files/full_demos/tiny_vehicle_only_2d.g2o");
}
