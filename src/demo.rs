// TODO delete this file once done testing with this

use crate::optimizer::optimize;
use crate::parser::json::JsonParser;
use crate::parser::Parser;
use crate::visualizer::visualize;
use crate::parser::g2o::G2oParser;
use std::collections::BTreeSet;

#[test]
fn current_test() {
    let g2o_graph = match G2oParser::parse_file("data_files/sphere_3D.g2o") {
        Ok(model) => model,
        Err(str) => panic!(str),
    };
    let json_graph = match JsonParser::parse_file("data_files/sphere_3D.json") {
        Ok(model) => model,
        Err(str) => panic!(str),
    };
}