//! Conversion between factor graph structures and G2O files.

use crate::parser::model::{FactorGraphModel, Vertex, Edge};
use crate::parser::Parser;
use std::collections::BTreeSet;

/// Implements G2O specific functions for parsing and composing files.
///
/// More information on the G2O file format: https://github.com/RainerKuemmerle/g2o/wiki/File-Format
///
/// Currently supported G2O vertices: VERTEX_SE2, VERTEX_XY, VERTEX_SE3:QUAT
///
/// Currently supported G2O edges: EDGE_PRIOR_SE2, EDGE_SE2, EDGE_SE2_XY, EDGE_SE3_PRIOR (*), EDGE_SE3:QUAT
///
/// (*) When using this edge, the 2nd vertex parameter is expected to be the vertex with ID 0 as follows:
/// "PARAMS_SE3OFFSET 0 0 0 0 0 0 0 1".
/// Anything else will result in undefined and most likely undesired behavior.
/// The vertex "PARAMS_SE3OFFSET" is not supported in any other scenario.
///
/// Note: Currently panics instead of returning an Err() when parsing an invalid file.
pub struct G2oParser;

impl Parser for G2oParser {
    // TODO return Err() instead of panicking when parsing an invalid file
    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String> {
        let mut model = FactorGraphModel {
            vertices: vec![],
            edges: vec![],
            fixed_vertices: BTreeSet::new(),
        };
        let lines = s.split("\n");
        lines.enumerate()
            .for_each(|(i, line)| Self::parse_line(&mut model, line, i+1));
        Ok(model)
    }

    fn compose_model_to_string(model: FactorGraphModel) -> Result<String, String> {
        let mut str_vec: Vec<String> = vec![];
        if(model.edges.iter().any(|e| e.edge_type == "PRIOR3D_QUAT")) {
            str_vec.push(String::from("PARAMS_SE3OFFSET 0 0 0 0 0 0 0 1"));
        }
        str_vec.extend::<Vec<String>>(model.vertices.iter().map(|v| Self::vertex_to_string(v, &model.fixed_vertices)).collect());
        str_vec.extend::<Vec<String>>(model.edges.iter().map(Self::edge_to_string).collect());
        Ok(str_vec.join("\n"))
    }
}

impl G2oParser {
    fn parse_line(model: &mut FactorGraphModel, line: &str, line_number: usize) {
        let tokens: Vec<&str> = line.split_whitespace().collect();
        if tokens.len() == 0 || line.starts_with('#') {
            return;
        }
        match tokens[0] {
            "VERTEX_SE2" | "VERTEX_XY"
            | "VERTEX_SE3:QUAT"
            => model.vertices.push(Self::parse_vertex(&tokens, line_number)),
            "EDGE_PRIOR_SE2" | "EDGE_SE2" | "EDGE_SE2_XY"
            | "EDGE_SE3_PRIOR" | "EDGE_SE3:QUAT"
            => model.edges.push(Self::parse_edge(&tokens, line_number)),
            "FIX" => {model.fixed_vertices.extend(Self::parse_fix(&tokens, line_number));},
            "PARAMS_SE3OFFSET" => (), // line expected to equal "PARAMS_SE3OFFSET 0 0 0 0 0 0 0 1"
            _ => panic!("Unknown keyword at beginning of line {}: {}", line_number, tokens[0]),
        };
    }

    fn parse_vertex(tokens: &[&str], line_number: usize) -> Vertex {
        let (type_str, c_len) = match tokens[0] {
            "VERTEX_SE2" => ("POSE2D_ANGLE", 3),
            "VERTEX_XY" => ("LANDMARK2D_ANGLE", 2),
            "VERTEX_SE3:QUAT" => ("POSE3D_QUAT", 7),
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
        let (type_str, v_num, c_len, (index_mapping, upper_t_len)) = match tokens[0] {
            "EDGE_PRIOR_SE2" => ("PRIOR2D_ANGLE", 1, 3, Self::get_index_mapping_vec_and_upper_t_len(3)),
            "EDGE_SE2" => ("ODOMETRY2D_ANGLE", 2, 3, Self::get_index_mapping_vec_and_upper_t_len(3)),
            "EDGE_SE2_XY" => ("OBSERVATION2D_ANGLE", 2, 2, Self::get_index_mapping_vec_and_upper_t_len(2)),
            "EDGE_SE3_PRIOR" => ("PRIOR3D_QUAT", 2, 7, Self::get_index_mapping_vec_and_upper_t_len(6)),
            "EDGE_SE3:QUAT" => ("ODOMETRY3D_QUAT", 2, 7, Self::get_index_mapping_vec_and_upper_t_len(6)),
            _ => panic!("Unknown keyword at beginning of line {}: {}", line_number, tokens[0]),
        };
        let expected_length = 1 + v_num + c_len + upper_t_len;
        Self::assert_tokens(expected_length, tokens.len(), line_number);
        Edge {
            edge_type: String::from(type_str),
            vertices: match tokens[0] {
                "EDGE_SE3_PRIOR" => tokens[1..2].iter().
                    map(|s| Self::parse_val(s, line_number)).collect(),
                _ => tokens[1..1+v_num].iter().
                    map(|s| Self::parse_val(s, line_number)).collect(),
            },
            restriction: tokens[1+v_num..1+v_num+c_len].iter()
                .map(|s| Self::parse_val(s, line_number)).collect(),
            information_matrix: index_mapping.iter()
                .map(|i| Self::parse_val(tokens[1+v_num+c_len + *i], line_number)).collect(),
        }
    }

    fn get_index_mapping_vec_and_upper_t_len(dim: usize) -> (Vec<usize>, usize) {
        let mut full_matrix_vec: Vec<usize> = vec![0; dim * dim];
        let mut upper_t_len = 0;
        for i in 0..dim {
            for j in i..dim {
                full_matrix_vec[i*dim + j] = upper_t_len;
                full_matrix_vec[i + j*dim] = upper_t_len;
                upper_t_len += 1;
            }
        }
        let upper_t_len = dim * (dim+1) / 2;
        (full_matrix_vec, upper_t_len)
    }

    fn parse_fix(tokens: &[&str], line_number: usize) -> BTreeSet<usize> {
        if tokens.len() == 1 {
            panic!("Empty set of fixed vertices in line {}: Expected at least one vertex ID.", line_number);
        }
        tokens[1..].iter()
            .map(|s| Self::parse_val(s, line_number)).collect()
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

    fn vertex_to_string(v: &Vertex, fixed_vertices: &BTreeSet<usize>) -> String {
        let mut tokens: Vec<String> = vec![];
        match v.vertex_type.as_str() {
            "POSE2D_ANGLE" => tokens.push(String::from("VERTEX_SE2")),
            "LANDMARK2D_ANGLE" => tokens.push(String::from("VERTEX_XY")),
            "POSE3D_QUAT" => tokens.push(String::from("VERTEX_SE3:QUAT")),
            other_type => panic!(format!("Vertex type unsupported to be composed to G2O format: {}", other_type)),
        }
        tokens.push(v.id.to_string());
        Self::append_f64_slice_to_string_vec(&mut tokens, &v.content);
        let mut vertex_string = tokens.join(" ");
        // TODO see if compatible with original g2o, otherwise print set of fixed vertices in one line
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
            "PRIOR3D_QUAT" => tokens.push(String::from("EDGE_SE3_PRIOR")),
            "ODOMETRY3D_QUAT" => tokens.push(String::from("EDGE_SE3:QUAT")),
            other_type => panic!(format!("Edge type unsupported to be composed to G2O format: {}", other_type)),
        }
        Self::append_usize_slice_to_string_vec(&mut tokens, e.vertices.as_slice());
        if e.edge_type == "PRIOR3D_QUAT" {
            Self::append_usize_slice_to_string_vec(&mut tokens, &[0]); // the 2nd vertex index of PRIOR3D_QUAT should always be 0
        }
        Self::append_f64_slice_to_string_vec(&mut tokens, &e.restriction);
        let upper_triangle = match e.edge_type.as_str() {
            "PRIOR2D_ANGLE" | "ODOMETRY2D_ANGLE" => Self::get_upper_triangle_indices(3),
            "OBSERVATION2D_ANGLE" => Self::get_upper_triangle_indices(2),
            "PRIOR3D_QUAT" | "ODOMETRY3D_QUAT" => Self::get_upper_triangle_indices(6),
            other_type => panic!(format!("Edge type unsupported to be composed to G2O format: {}", other_type)),
        };
        Self::append_f64_slice_elements_to_string_vec(&mut tokens, &e.information_matrix, &upper_triangle);
        tokens.join(" ")
    }

    fn get_upper_triangle_indices(dim: usize) -> Vec<usize> {
        let mut indices: Vec<usize> = vec![];
        for i in 0..dim {
            for j in i..dim {
                indices.push(i*dim+j)
            }
        }
        indices
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
            match G2oParser::parse_file("data_files/minimal_size_and_types.g2o") {
                Ok(x) => x,
                Err(str) => panic!(str),
            };
        dbg!("{:?}", &factor_graph);
        G2oParser::compose_file(&factor_graph, "data_files/minimal_size_and_types_copy.g2o");
    }

    #[test]
    #[ignore] // deprecated input file
    fn test_dumb_compose_model_to_string() {
        init();

        let model = JsonParser::parse_file_to_model("data_files/dumb.json").unwrap();
        // dbg!(&model);
        let g2o_string = G2oParser::compose_model_to_string(model).unwrap();
        info!("\n{}", &g2o_string);
    }

    // TODO Should compose_model_to_string() be tested with unit tests?
}