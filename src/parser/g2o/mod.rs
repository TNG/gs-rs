//! Conversion between factor graph structures and G2O files.

use crate::parser::model::{FactorGraphModel, Vertex, Edge};
use crate::parser::Parser;
use std::collections::HashSet;

/// Implements G2O specific functions for parsing and composing files.
/// More information on the G2O file format: https://github.com/RainerKuemmerle/g2o/wiki/File-Format
///
/// Currently supported G2O vertices: VERTEX_SE2, VERTEX_XY
/// Currently supported G2O edges: EDGE_PRIOR_SE2, EDGE_SE2, EDGE_SE2_XY
///
/// Other limitations:
/// - Comments are not supported. Files that include lines starting with a "#" can not be parsed.
/// - Lines starting with "FIX" only expect one fixed vertex. If more than one vertex is fixed, multiple lines starting with "FIX" need to be used.
pub struct G2oParser;

impl Parser for G2oParser {
    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String> {
        let mut model = FactorGraphModel {
            vertices: vec![],
            edges: vec![],
            fixed_vertices: HashSet::new(),
        };
        let lines = s.split("\n");
        lines.enumerate()
            .for_each(|(i, line)| Self::parse_line(&mut model, line, i+1));
        Ok(model)
    }

    fn compose_model_to_string(model: FactorGraphModel) -> Result<String, String> {
        let mut str_vec: Vec<String> = model.vertices.iter().map(|v| Self::vertex_to_string(v, &model.fixed_vertices)).collect();
        str_vec.extend::<Vec<String>>(model.edges.iter().map(Self::edge_to_string).collect());
        Ok(str_vec.join("\n"))
    }
}

impl G2oParser {
    fn parse_line(model: &mut FactorGraphModel, line: &str, line_number: usize) {
        let tokens: Vec<&str> = line.split_whitespace().collect();
        if tokens.len() == 0 {
            return;
        }
        match tokens[0] {
            "VERTEX_SE2" | "VERTEX_XY" => model.vertices.push(Self::parse_vertex(&tokens, line_number)),
            "EDGE_PRIOR_SE2" | "EDGE_SE2" | "EDGE_SE2_XY" => model.edges.push(Self::parse_edge(&tokens, line_number)),
            "FIX" => {model.fixed_vertices.insert(Self::parse_fix(&tokens, line_number));},
            _ => panic!("Unknown keyword at beginning of line {}: {}", line_number, tokens[0]),
        };
    }

    fn parse_vertex(tokens: &[&str], line_number: usize) -> Vertex {
        let (type_str, c_len) = match tokens[0] {
            "VERTEX_SE2" => ("POSE2D_ANGLE", 3),
            "VERTEX_XY" => ("LANDMARK2D_ANGLE", 2),
            _ => panic!("Unknown keyword at beginning of line {}: {}", line_number, tokens[0]),
        };
        let expected_length = 2 + c_len;
        Self::assert_tokens(expected_length, tokens.len(), line_number);
        Vertex {
            id: Self::parse_val(tokens[1], line_number),
            vertex_type: String::from(type_str),
            content: tokens[2..].iter()
                .map(|s| Self::parse_val(s, line_number)).collect(),
        }
    }

    fn parse_edge(tokens: &[&str], line_number: usize) -> Edge {
        let (type_str, v_num, c_len, index_mapping, upper_t_len) = match tokens[0] {
            "EDGE_PRIOR_SE2" => ("PRIOR2D_ANGLE", 1, 3, vec![0, 1, 2, 1, 3, 4, 2, 4, 5], 6),
            "EDGE_SE2" => ("ODOMETRY2D_ANGLE", 2, 3, vec![0, 1, 2, 1, 3, 4, 2, 4, 5], 6),
            "EDGE_SE2_XY" => ("OBSERVATION2D_ANGLE", 2, 2, vec![0, 1, 1, 2], 3),
            _ => panic!("Unknown keyword at beginning of line {}: {}", line_number, tokens[0]),
        };
        let expected_length = 1 + v_num + c_len + upper_t_len;
        Self::assert_tokens(expected_length, tokens.len(), line_number);
        Edge {
            edge_type: String::from(type_str),
            vertices: tokens[1..1+v_num].iter()
                .map(|s| Self::parse_val(s, line_number)).collect(),
            restriction:tokens[1+v_num..1+v_num+c_len].iter()
                .map(|s| Self::parse_val(s, line_number)).collect(),
            information_matrix: index_mapping.iter()
                .map(|i| Self::parse_val(tokens[1+v_num+c_len + *i as usize], line_number)).collect(),
        }
    }

    fn parse_fix(tokens: &[&str], line_number: usize) -> usize {
        let expected_length = 2;
        Self::assert_tokens(expected_length, tokens.len(), line_number);
        Self::parse_val(tokens[1], line_number)
    }

    fn assert_tokens(expected: usize, actual: usize, line_number: usize) {
        if actual != expected {
            panic!("Wrong number of tokens in line {}: Expected: {}; Actual: {}", line_number, expected, actual);
        }
    }

    fn parse_val<T: std::str::FromStr>(s: &str, line_number: usize) -> T {
        match s.parse() {
            Ok(val) => val,
            Err(str) => panic!("Could not parse the following value to the correct data type in line {}: {}", line_number, s),
        }
    }

    fn vertex_to_string(v: &Vertex, fixed_vertices: &HashSet<usize>) -> String {
        let mut tokens: Vec<String> = vec![];
        match v.vertex_type.as_str() {
            "POSE2D_ANGLE" => tokens.push(String::from("VERTEX_SE2")),
            "LANDMARK2D_ANGLE" => tokens.push(String::from("VERTEX_XY")),
            other_type => panic!(format!("Vertex type unsupported to be composed to G2O format: {}", other_type)),
        }
        tokens.push(v.id.to_string());
        Self::append_f64_slice_to_string_vec(&mut tokens, &v.content);
        let mut vertex_string = tokens.join(" ");
        if fixed_vertices.contains(&v.id) {
            vertex_string.push_str(&format!("\nFIX {}", v.id));
        }
        vertex_string
    }

    fn edge_to_string(e: &Edge) -> String {
        let mut tokens: Vec<String> = vec![];
        match e.edge_type.as_str() {
            "PRIOR2D_ANGLE" => tokens.push(String::from("EDGE_PRIOR_SE2")),
            "ODOMETRY2D_ANGLE" => tokens.push(String::from("EDGE_SE2")),
            "OBSERVATION2D_ANGLE" => tokens.push(String::from("EDGE_SE2_XY")),
            other_type => panic!(format!("Edge type unsupported to be composed to G2O format: {}", other_type)),
        }
        Self::append_usize_slice_to_string_vec(&mut tokens, e.vertices.as_slice());
        Self::append_f64_slice_to_string_vec(&mut tokens, &e.restriction);
        let upper_triangle = match e.edge_type.as_str() {
            "PRIOR2D_ANGLE" | "ODOMETRY2D_ANGLE" => vec![0, 1, 2, 4, 5, 8],
            "OBSERVATION2D_ANGLE" => vec![0, 1, 3],
            other_type => panic!(format!("Edge type unsupported to be composed to G2O format: {}", other_type)),
        };
        Self::append_f64_slice_elements_to_string_vec(&mut tokens, &e.information_matrix, &upper_triangle);
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

        let factor_graph =
            match G2oParser::parse_file("test_files/minimal_size_and_types.g2o") {
                Ok(x) => x,
                Err(str) => panic!(str),
            };
        dbg!("{:?}", &factor_graph);
        G2oParser::compose_file(&factor_graph, "test_files/minimal_size_and_types_copy.g2o");
    }

    #[test]
    #[ignore] // deprecated input file
    fn test_dumb_compose_model_to_string() {
        init();

        let model = JsonParser::parse_file_to_model("test_files/dumb.json").unwrap();
        // dbg!(&model);
        let g2o_string = G2oParser::compose_model_to_string(model).unwrap();
        info!("\n{}", &g2o_string);
    }

    // TODO Should compose_model_to_string() be tested with unit tests?
}