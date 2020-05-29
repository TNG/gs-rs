// TODO delete this file once done testing with this

use crate::optimizer::optimize;
use crate::parser::g2o::G2oParser;
use crate::parser::json::JsonParser;
use crate::parser::Parser;
use crate::visualizer::visualize;
use std::collections::BTreeSet;

#[test]
fn current_test() {
    let factor_graph = G2oParser::parse_file("data_files/bad3d.g2o").unwrap();
    optimize(&factor_graph, 2);
    G2oParser::compose_file(&factor_graph, "data_files/bad3d_sol1.g2o");
}
