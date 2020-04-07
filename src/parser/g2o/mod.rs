//! Conversion between factor graph structures and G2O files.

use crate::parser::model::{FactorGraphModel, Vertex, Edge};
use crate::parser::Parser;
use std::collections::HashSet;

/// Implements G2O specific functions for parsing and composing files.
pub struct G2oParser;

impl Parser for G2oParser {
    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String> {
        let lines = s.split("\n");
        // lines.into_iter();
        Err(String::from("nix implemented n' stuff"))
    }

    fn compose_model_to_string(model: FactorGraphModel) -> Result<String, String> {
        let mut str_vec: Vec<String> = model.vertices.iter().map(|v| Self::vertex_to_string(v, &model.fixed_vertices)).collect();
        str_vec.extend::<Vec<String>>(model.edges.iter().map(Self::edge_to_string).collect());
        Ok(str_vec.join("\n"))
    }
}

impl G2oParser {
    fn vertex_to_string(v: &Vertex, fixed_vertices: &HashSet<usize>) -> String {
        let mut tokens: Vec<String> = vec![];
        match v.vertex_type.as_str() {
            "POSE2D_ANGLE" => tokens.push(String::from("VERTEX_SE2")),
            other_type => panic!(format!("Vertex type unsupported to be composed to G2O format: {}", other_type)),
        }
        tokens.push(v.id.to_string());
        Self::append_f64_slice_to_string_vec(&mut tokens, &v.position);
        Self::append_f64_slice_to_string_vec(&mut tokens, &v.rotation);
        let mut vertex_string = tokens.join(" ");
        if fixed_vertices.contains(&v.id) {
            vertex_string.push_str(&format!("\nFIX {}", v.id));
        }
        vertex_string
    }

    fn edge_to_string(e: &Edge) -> String {
        let mut tokens: Vec<String> = vec![];
        match e.edge_type.as_str() {
            "ODOMETRY2D_ANGLE" => tokens.push(String::from("EDGE_SE2")),
            other_type => panic!(format!("Edge type unsupported to be composed to G2O format: {}", other_type)),
        }
        Self::append_usize_slice_to_string_vec(&mut tokens, e.vertices.as_slice());
        Self::append_f64_slice_to_string_vec(&mut tokens, &e.restriction);
        let lower_triangle: [usize; 6] = [0, 3, 4, 6, 7, 8];
        Self::append_f64_slice_elements_to_string_vec(&mut tokens, &e.information_matrix, &lower_triangle);
        tokens.join(" ")
    }

    fn append_f64_slice_to_string_vec(tokens: &mut Vec<String>, f64_slice: &[f64]) {
        tokens.extend::<Vec<String>>(f64_slice.iter().map(|val| format!("{:?}", val)).collect());
    }

    fn append_usize_slice_to_string_vec(tokens: &mut Vec<String>, usize_slice: &[usize]) {
        tokens.extend::<Vec<String>>(usize_slice.iter().map(|val| format!("{:?}", val)).collect());
    }

    fn append_f64_slice_elements_to_string_vec(tokens: &mut Vec<String>, f64_slice: &[f64], indices: &[usize]) {
        tokens.extend::<Vec<String>>(indices.iter().map(|i| format!("{:?}", f64_slice[*i])).collect());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use log::LevelFilter;
    use std::fs;
    use crate::parser::json::JsonParser;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn test_parse_minimal_file() {
        init();

        let parsed_factor_graph =
            match G2oParser::parse_file("test_files/minimal_size_and_types.g2o") {
                Ok(x) => x,
                Err(str) => panic!(str),
            };
        dbg!("{:?}", &parsed_factor_graph);
    }

    #[test]
    fn test_dumb_compose_model_to_string() {
        init();

        let model = JsonParser::parse_file_to_model("test_files/dumb.json").unwrap();
        // dbg!(&model);
        let g2o_string = G2oParser::compose_model_to_string(model).unwrap();
        info!("\n{}", &g2o_string);
    }

    // TODO Should compose_model_to_string() be tested with unit tests?
}