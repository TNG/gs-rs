pub mod model;

use crate::parser::model::FactorGraphModel;

use std::fs;

pub mod json;
pub mod g2o;
pub mod converter;

pub trait Parser {
    fn parse_file_to_model(file_path: &str) -> Result<FactorGraphModel, String> {
        let file_string = match fs::read_to_string(file_path) {
            Ok(s) => s,
            Err(e) => return Err(format!("File could not be parsed: {}", file_path)),
        };
        Self::parse_string_to_model(&file_string)
    }

    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String>;
}
