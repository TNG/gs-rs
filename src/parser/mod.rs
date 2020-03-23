pub mod model;

use crate::parser::model::FactorGraphModel;

use std::fs;
use crate::factor_graph::FactorGraph;

pub mod json;
pub mod g2o;

pub trait Parser {

    // TODO try to implement for new FactorGraph type (otherwise let interface user take care of it)
    // fn parse_file<'a>(/*file path*/) -> Result<FactorGraph<'a>, String> {
    //     unimplemented!()
    // }

    fn parse_file_to_model(file_path: &str) -> Result<FactorGraphModel, String> {
        let file_string = match fs::read_to_string(file_path) {
            Ok(s) => s,
            Err(_e) => return Err(format!("File could not be parsed: {}", file_path)),
        };
        Self::parse_string_to_model(&file_string)
    }

    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String>;

    // TODO try to implement for new FactorGraph type (otherwise let interface user take care of it)
    fn compose_file(factor_graph: FactorGraph, file_path: &str){
        unimplemented!()
    }

    fn compose_model_to_file(model: FactorGraphModel, file_path: &str) -> Result<(), String>{
        let s = Self::compose_model_to_string(model)?;
        match fs::write(file_path, s) {
            Ok(x) => Ok(x),
            Err(_e) => Err(format!("File could not be written to: {}", file_path))
        }
    }

    fn compose_model_to_string(model: FactorGraphModel) -> Result<String, String>;
}