use crate::optimizer::optimize;
use crate::parser::g2o::G2oParser;
use crate::parser::json::JsonParser;
use crate::parser::Parser;
use crate::visualizer::visualize;

#[test]
fn current_test() {
    // let factor_graph = G2oParser::parse_file("data_files/odo3d_only_0.g2o").unwrap();
    // optimize(&factor_graph, 1);
    // G2oParser::compose_file(&factor_graph, "data_files/odo3d_only_1.g2o");
}
