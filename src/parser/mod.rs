pub mod model;

use crate::parser::model::FactorGraphModel;

use crate::factor_graph::FactorGraph;
use std::fs;

pub mod g2o;
pub mod json;

pub trait Parser {
    fn parse_file<'a>(file_path: &str) -> Result<FactorGraph<'a>, String> {
        match Self::parse_file_to_model(file_path) {
            Ok(model) => Ok(model.into()),
            Err(s) => Err(s),
        }
    }

    fn parse_file_to_model(file_path: &str) -> Result<FactorGraphModel, String> {
        let file_string = match fs::read_to_string(file_path) {
            Ok(s) => s,
            Err(_e) => return Err(format!("File could not be parsed: {}", file_path)),
        };
        Self::parse_string_to_model(&file_string)
    }

    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String>;

    // TODO implement after implementing conversion from FactorGraph to FactorGraphModel
    fn compose_file(_factor_graph: FactorGraph, _file_path: &str) {
        unimplemented!()
    }

    fn compose_model_to_file(model: FactorGraphModel, file_path: &str) -> Result<(), String> {
        let s = Self::compose_model_to_string(model)?;
        fs::write(file_path, s)
            .or_else(|_| Err(format!("File could not be written to: {}", file_path)))
    }

    fn compose_model_to_string(model: FactorGraphModel) -> Result<String, String>;
}
