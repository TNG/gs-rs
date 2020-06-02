use crate::optimizer::optimize;
use crate::parser::g2o::G2oParser;
use crate::parser::json::JsonParser;
use crate::parser::Parser;
use crate::visualizer::visualize;

#[test]
fn current_test() {
    let factor_graph = G2oParser::parse_file_to_model("data_files/pos3d.g2o").unwrap();
    // optimize(&factor_graph, 1);
    G2oParser::compose_model_to_file(factor_graph, "data_files/pos3d_0.g2o");
}
