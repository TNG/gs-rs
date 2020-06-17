use crate::optimizer::optimize;
use crate::parser::g2o::G2oParser;
use crate::parser::json::JsonParser;
use crate::parser::Parser;
use crate::visualizer::visualize;

// TODO @Samuel: remove all compiler warnings

#[test]
fn current_test() {
    // TODO @Samuel: delete this file if not used anymore
    // let factor_graph = G2oParser::parse_file("data_files/obs3d.g2o").unwrap();
    // optimize(&factor_graph, 1);
    // G2oParser::compose_file(&factor_graph, "data_files/obs3d_1.g2o");

}
