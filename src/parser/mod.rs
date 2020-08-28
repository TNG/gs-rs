// gs-rs - Graph SLAM in Rust
// --------------
//
// © 2020 Samuel Valenzuela (samuel.valenzuela@tngtech.com)
// © 2020 Florian Rohm (florian.rohm@tngtech.com)
// © 2020 Daniel Pape (daniel.pape@tngtech.com)
//
// This product includes software developed at
// TNG Technology Consulting GmbH (https://www.tngtech.com/).
//
// gs-rs is licensed under the Apache License, Version 2.0 (LICENSE-APACHE.md or
// http://www.apache.org/licenses/LICENSE-2.0) or the MIT license (LICENSE-MIT.md
// or http://opensource.org/licenses/MIT), at your option.
//! Conversion between factor graph structures and files.

use crate::factor_graph::FactorGraph;
use crate::parser::model::FactorGraphModel;
use std::fs;

pub mod g2o;
pub mod json;
pub mod model;

/// Trait to be used by all parsers with the basic file parsing and composition functionality.
pub trait Parser {
    /// Tries to parse a file at the given path to the internal factor graph representation.
    fn parse_file<'a>(file_path: &str) -> Result<FactorGraph, String> {
        match Self::parse_file_to_model(file_path) {
            Ok(model) => Ok(model.into()),
            Err(s) => Err(s),
        }
    }

    /// Tries to parse a file at the given path to the factor graph model used in the context with files.
    fn parse_file_to_model(file_path: &str) -> Result<FactorGraphModel, String> {
        let file_string = match fs::read_to_string(file_path) {
            Ok(s) => s,
            Err(_e) => return Err(format!("File could not be parsed: {}", file_path)),
        };
        Self::parse_string_to_model(&file_string)
    }

    /// Tries to parse a string to the factor graph model used in the context with files.
    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String>;

    /// Tries to compose a file at the given path containing the serialized factor graph.
    fn compose_file(factor_graph: &FactorGraph, file_path: &str) -> Result<(), String> {
        Self::compose_model_to_file(factor_graph.into(), file_path)
    }

    /// Tries to compose a file at the given path containing the factor graph model's serialization.
    fn compose_model_to_file(model: FactorGraphModel, file_path: &str) -> Result<(), String> {
        let s = Self::compose_model_to_string(model)?;
        fs::write(file_path, s).or_else(|_| Err(format!("File could not be written to: {}", file_path)))
    }

    /// Tries to compose a string containing the factor graph model's serialization.
    fn compose_model_to_string(model: FactorGraphModel) -> Result<String, String>;
}
