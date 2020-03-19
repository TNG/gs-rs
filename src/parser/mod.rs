use crate::parser::model::FactorGraphModel;
use crate::factor_graph::FactorGraph;

use std::fs;

pub mod model;
pub mod json;
pub mod g2o;

pub trait Parser {

    // TODO correct lifetime usage?
    fn parse_file<'a>(/*file path*/) -> Result<FactorGraph<'a>, String> {
        unimplemented!()
    }

    fn parse_file_to_model(file_path: &str) -> Result<FactorGraphModel, String> {
        let file_string = match fs::read_to_string(file_path) {
            Ok(s) => s,
            Err(e) => return Err(format!("File could not be parsed: {}", file_path)),
        };
        Self::parse_string_to_model(&file_string)
    }

    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String>;

    fn compose_file(factor_graph: FactorGraph, file_path: &str){
        unimplemented!()
    }

    fn compose_model_to_file(model: FactorGraphModel, file_path: &str) {
        unimplemented!()
    }

    fn compose_model_to_string(model: FactorGraphModel) -> String;
}