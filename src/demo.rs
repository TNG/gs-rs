use crate::optimizer::optimize;
use crate::parser::g2o::G2oParser;
use crate::parser::json::JsonParser;
use crate::parser::Parser;
use crate::visualizer::visualize;
use crate::parser::model::FactorGraphModel;
use std::collections::BTreeSet;

// TODO @Samuel: go through ignored tests

// TODO @Samuel: remove all compiler warnings

#[test]
fn current_test() {
    // TODO @Samuel: delete this file if not used anymore
    let factor_graph = G2oParser::parse_file("data_files/bad3d.g2o").unwrap();
    visualize(&factor_graph);
    // optimize(&factor_graph, 1);
    // G2oParser::compose_file(&factor_graph, "data_files/bad3d.g2o");
    let empty_model = FactorGraphModel {
        edges: vec![],
        vertices: vec![],
        fixed_vertices: BTreeSet::new(),
    };
    // G2oParser::compose_file(&empty_model.into(), "data_files/asdf.g2o");
    // optimize(&empty_model.into(), 10);
    // visualize(&empty_model.into());
}
