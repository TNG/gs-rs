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
//! Conversion between factor graph structures and G2O files.

use crate::parser::model::{Edge, FactorGraphModel, Vertex};
use crate::parser::Parser;
use std::collections::BTreeSet;

/// Implements G2O specific functions for parsing and composing files.
///
/// More information on the G2O file format: https://github.com/RainerKuemmerle/g2o/wiki/File-Format
///
/// Currently supported G2O vertices:
/// VERTEX_SE2, VERTEX_XY, VERTEX_SE3:QUAT, VERTEX_TRACKXYZ
///
/// Currently supported G2O edges:
/// EDGE_PRIOR_SE2, EDGE_SE2, EDGE_SE2_XY, EDGE_SE3_PRIOR (*), EDGE_SE3:QUAT, EDGE_SE3_TRACKXYZ (*)
///
/// (*) When using one of these edges, the 2nd (EDGE_SE3_PRIOR) or 3rd (EDGE_SE3_TRACKXYZ)
/// vertex/offset parameter is expected to be the offset with ID 0 as follows:
/// "PARAMS_SE3OFFSET 0 0 0 0 0 0 0 1".
/// Anything else will result in undefined and most likely undesired behavior.
/// The offset "PARAMS_SE3OFFSET" is not supported in any other scenario.
///
/// Note: Currently panics instead of returning an Err() when parsing an invalid file.
pub struct G2oParser;

impl Parser for G2oParser {
    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String> {
        let mut model = FactorGraphModel {
            vertices: vec![],
            edges: vec![],
            fixed_vertices: BTreeSet::new(),
        };
        let lines = s.split("\n");
        lines
            .enumerate()
            .for_each(|(i, line)| Self::parse_line(&mut model, line, i + 1));
        Ok(model)
    }

    fn compose_model_to_string(model: FactorGraphModel) -> Result<String, String> {
        let mut str_vec: Vec<String> = vec![];
        if model
            .edges
            .iter()
            .any(|e| e.edge_type == "Position3D" || e.edge_type == "Observation3D")
        {
            str_vec.push(String::from("PARAMS_SE3OFFSET 0 0 0 0 0 0 0 1"));
        }
        str_vec.extend::<Vec<String>>(
            model
                .vertices
                .iter()
                .map(|v| Self::vertex_to_string(v, &model.fixed_vertices))
                .collect(),
        );
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
            "VERTEX_SE2" | "VERTEX_XY" | "VERTEX_SE3:QUAT" | "VERTEX_TRACKXYZ" => {
                model.vertices.push(Self::parse_vertex(&tokens, line_number))
            }
            "EDGE_PRIOR_SE2" | "EDGE_SE2" | "EDGE_SE2_XY" | "EDGE_SE3_PRIOR" | "EDGE_SE3:QUAT"
            | "EDGE_SE3_TRACKXYZ" => model.edges.push(Self::parse_edge(&tokens, line_number)),
            "FIX" => {
                model.fixed_vertices.extend(Self::parse_fix(&tokens, line_number));
            }
            "PARAMS_SE3OFFSET" => (), // line expected to equal "PARAMS_SE3OFFSET 0 0 0 0 0 0 0 1"
            _ => panic!("Unknown keyword at beginning of line {}: {}", line_number, tokens[0]),
        };
    }

    fn parse_vertex(tokens: &[&str], line_number: usize) -> Vertex {
        let (type_str, c_len) = match tokens[0] {
            "VERTEX_SE2" => ("Vehicle2D", 3),
            "VERTEX_XY" => ("Landmark2D", 2),
            "VERTEX_SE3:QUAT" => ("Vehicle3D", 7),
            "VERTEX_TRACKXYZ" => ("Landmark3D", 3),
            _ => panic!("Unknown keyword at beginning of line {}: {}", line_number, tokens[0]),
        };
        let expected_length = 2 + c_len;
        Self::assert_tokens(expected_length, tokens.len(), line_number);
        Vertex {
            id: Self::parse_val(tokens[1], line_number),
            vertex_type: String::from(type_str),
            content: tokens[2..].iter().map(|s| Self::parse_val(s, line_number)).collect(),
        }
    }

    fn parse_edge(tokens: &[&str], line_number: usize) -> Edge {
        let (type_str, v_num, c_len, (index_mapping, upper_t_len)) = match tokens[0] {
            "EDGE_PRIOR_SE2" => ("Position2D", 1, 3, Self::get_index_mapping_vec_and_upper_t_len(3)),
            "EDGE_SE2" => ("Odometry2D", 2, 3, Self::get_index_mapping_vec_and_upper_t_len(3)),
            "EDGE_SE2_XY" => ("Observation2D", 2, 2, Self::get_index_mapping_vec_and_upper_t_len(2)),
            "EDGE_SE3_PRIOR" => ("Position3D", 2, 7, Self::get_index_mapping_vec_and_upper_t_len(6)),
            "EDGE_SE3:QUAT" => ("Odometry3D", 2, 7, Self::get_index_mapping_vec_and_upper_t_len(6)),
            "EDGE_SE3_TRACKXYZ" => ("Observation3D", 3, 3, Self::get_index_mapping_vec_and_upper_t_len(3)),
            _ => panic!("Unknown keyword at beginning of line {}: {}", line_number, tokens[0]),
        };
        let expected_length = 1 + v_num + c_len + upper_t_len;
        Self::assert_tokens(expected_length, tokens.len(), line_number);
        Edge {
            edge_type: String::from(type_str),
            vertices: match tokens[0] {
                "EDGE_SE3_PRIOR" | "EDGE_SE3_TRACKXYZ" => tokens[1..v_num]
                    .iter()
                    .map(|s| Self::parse_val(s, line_number))
                    .collect(),
                _ => tokens[1..1 + v_num]
                    .iter()
                    .map(|s| Self::parse_val(s, line_number))
                    .collect(),
            },
            restriction: tokens[1 + v_num..1 + v_num + c_len]
                .iter()
                .map(|s| Self::parse_val(s, line_number))
                .collect(),
            information_matrix: index_mapping
                .iter()
                .map(|i| Self::parse_val(tokens[1 + v_num + c_len + *i], line_number))
                .collect(),
        }
    }

    fn get_index_mapping_vec_and_upper_t_len(dim: usize) -> (Vec<usize>, usize) {
        let mut full_matrix_vec: Vec<usize> = vec![0; dim * dim];
        let mut upper_t_len = 0;
        for i in 0..dim {
            for j in i..dim {
                full_matrix_vec[i * dim + j] = upper_t_len;
                full_matrix_vec[i + j * dim] = upper_t_len;
                upper_t_len += 1;
            }
        }
        let upper_t_len = dim * (dim + 1) / 2;
        (full_matrix_vec, upper_t_len)
    }

    fn parse_fix(tokens: &[&str], line_number: usize) -> BTreeSet<usize> {
        if tokens.len() == 1 {
            panic!(
                "Empty set of fixed vertices in line {}: Expected at least one vertex ID.",
                line_number
            );
        }
        tokens[1..].iter().map(|s| Self::parse_val(s, line_number)).collect()
    }

    fn assert_tokens(expected: usize, actual: usize, line_number: usize) {
        if actual != expected {
            panic!(
                "Wrong number of tokens in line {}: Expected: {}; Actual: {}",
                line_number, expected, actual
            );
        }
    }

    fn parse_val<T: std::str::FromStr>(s: &str, line_number: usize) -> T {
        match s.parse() {
            Ok(val) => val,
            Err(_str) => panic!(
                "Could not parse the following value to the correct data type in line {}: {}",
                line_number, s
            ),
        }
    }

    fn vertex_to_string(v: &Vertex, fixed_vertices: &BTreeSet<usize>) -> String {
        let mut tokens: Vec<String> = vec![];
        match v.vertex_type.as_str() {
            "Vehicle2D" => tokens.push(String::from("VERTEX_SE2")),
            "Landmark2D" => tokens.push(String::from("VERTEX_XY")),
            "Vehicle3D" => tokens.push(String::from("VERTEX_SE3:QUAT")),
            "Landmark3D" => tokens.push(String::from("VERTEX_TRACKXYZ")),
            other_type => panic!(format!(
                "Vertex type unsupported to be composed to G2O format: {}",
                other_type
            )),
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
            "Position2D" => tokens.push(String::from("EDGE_PRIOR_SE2")),
            "Odometry2D" => tokens.push(String::from("EDGE_SE2")),
            "Observation2D" => tokens.push(String::from("EDGE_SE2_XY")),
            "Position3D" => tokens.push(String::from("EDGE_SE3_PRIOR")),
            "Odometry3D" => tokens.push(String::from("EDGE_SE3:QUAT")),
            "Observation3D" => tokens.push(String::from("EDGE_SE3_TRACKXYZ")),
            other_type => panic!(format!(
                "Edge type unsupported to be composed to G2O format: {}",
                other_type
            )),
        }
        Self::append_usize_slice_to_string_vec(&mut tokens, e.vertices.as_slice());
        if e.edge_type == "Position3D" || e.edge_type == "Observation3D" {
            Self::append_usize_slice_to_string_vec(&mut tokens, &[0]); // the last vertex/offset index should be 0 for these edges
        }
        Self::append_f64_slice_to_string_vec(&mut tokens, &e.restriction);
        let upper_triangle = match e.edge_type.as_str() {
            "Position2D" | "Odometry2D" | "Observation3D" => Self::get_upper_triangle_indices(3),
            "Observation2D" => Self::get_upper_triangle_indices(2),
            "Position3D" | "Odometry3D" => Self::get_upper_triangle_indices(6),
            other_type => panic!(format!(
                "Edge type unsupported to be composed to G2O format: {}",
                other_type
            )),
        };
        Self::append_f64_slice_elements_to_string_vec(&mut tokens, &e.information_matrix, &upper_triangle);
        tokens.join(" ")
    }

    fn get_upper_triangle_indices(dim: usize) -> Vec<usize> {
        let mut indices: Vec<usize> = vec![];
        for i in 0..dim {
            for j in i..dim {
                indices.push(i * dim + j)
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
    use crate::parser::model::{Edge, Vertex};
    use log::LevelFilter;
    use std::collections::BTreeSet;
    use std::fs;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    fn get_2d_model() -> FactorGraphModel {
        let vertices = vec![
            Vertex {
                id: 0,
                vertex_type: String::from("Vehicle2D"),
                content: vec![1.0, 0.0, 1.57],
            },
            Vertex {
                id: 1,
                vertex_type: String::from("Vehicle2D"),
                content: vec![0.0, 1.0, 3.14],
            },
            Vertex {
                id: 2,
                vertex_type: String::from("Landmark2D"),
                content: vec![1.5, 2.0],
            },
        ];
        let edges = vec![
            Edge {
                edge_type: String::from("Odometry2D"),
                vertices: vec![0, 1],
                restriction: vec![1.0, 1.5, 1.57],
                information_matrix: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            },
            Edge {
                edge_type: String::from("Observation2D"),
                vertices: vec![0, 2],
                restriction: vec![0.0, -1.0],
                information_matrix: vec![1.0, 0.0, 0.0, 1.0],
            },
            Edge {
                edge_type: String::from("Position2D"),
                vertices: vec![1],
                restriction: vec![0.0, 1.0, 3.13],
                information_matrix: vec![10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 1.0],
            },
        ];
        let mut fixed_vertices = BTreeSet::new();
        fixed_vertices.insert(0);
        FactorGraphModel {
            vertices,
            edges,
            fixed_vertices,
        }
    }

    #[test]
    fn test_2d_type_parsing() {
        init();
        let parsed_model = G2oParser::parse_file_to_model("data_files/full_demos/all_2d_types.g2o").unwrap();
        dbg!("{:?}", &parsed_model);
        let expected_model = get_2d_model();
        assert_eq!(parsed_model, expected_model);
    }

    #[test]
    fn test_2d_type_composition() {
        let model = get_2d_model();
        let composed_string = G2oParser::compose_model_to_string(model).unwrap() + "\n";
        let expected_string = fs::read_to_string("data_files/full_demos/all_2d_types.g2o").unwrap();
        assert_eq!(&composed_string, &expected_string);
    }

    fn get_3d_model() -> FactorGraphModel {
        let vertices = vec![
            Vertex {
                id: 0,
                vertex_type: String::from("Vehicle3D"),
                content: vec![18.7381, 0.000000274428, 8.2287, 0.0, 0.0, 0.0, 1.0],
            },
            Vertex {
                id: 1,
                vertex_type: String::from("Vehicle3D"),
                content: vec![19.0477, 2.34636, 8.2319, -0.139007, 0.0806488, 0.14657, 0.976059],
            },
            Vertex {
                id: 2,
                vertex_type: String::from("Landmark3D"),
                content: vec![18.2645, 4.24943, 15.2057],
            },
        ];
        let edges = vec![
            Edge {
                edge_type: String::from("Odometry3D"),
                vertices: vec![0, 1],
                restriction: vec![0.309576, 2.34636, 0.00315914, -0.139007, 0.0806488, 0.14657, 0.976059],
                information_matrix: vec![
                    1.0,
                    0.000000000000000000962965,
                    0.000000000000000000962965,
                    0.0000000588441,
                    -0.0000000203096,
                    0.00000000340337,
                    0.000000000000000000962965,
                    1.0,
                    0.000000000000000000962965,
                    0.0000000588441,
                    -0.0000000203096,
                    0.00000000340337,
                    0.000000000000000000962965,
                    0.000000000000000000962965,
                    1.0,
                    0.0000000588441,
                    -0.0000000203096,
                    0.00000000340337,
                    0.0000000588441,
                    0.0000000588441,
                    0.0000000588441,
                    4108.72,
                    -34.2982,
                    884.091,
                    -0.0000000203096,
                    -0.0000000203096,
                    -0.0000000203096,
                    -34.2982,
                    3951.5,
                    40.2084,
                    0.00000000340337,
                    0.00000000340337,
                    0.00000000340337,
                    884.091,
                    40.2084,
                    4100.08,
                ],
            },
            Edge {
                edge_type: String::from("Position3D"),
                vertices: vec![1],
                restriction: vec![0.309576, 2.34636, 0.00315914, -0.139007, 0.0806488, 0.14657, 0.976059],
                information_matrix: vec![
                    1.0,
                    0.000000000000000000962965,
                    0.000000000000000000962965,
                    0.0000000588441,
                    -0.0000000203096,
                    0.00000000340337,
                    0.000000000000000000962965,
                    1.0,
                    0.000000000000000000962965,
                    0.0000000588441,
                    -0.0000000203096,
                    0.00000000340337,
                    0.000000000000000000962965,
                    0.000000000000000000962965,
                    1.0,
                    0.0000000588441,
                    -0.0000000203096,
                    0.00000000340337,
                    0.0000000588441,
                    0.0000000588441,
                    0.0000000588441,
                    4108.72,
                    -34.2982,
                    884.091,
                    -0.0000000203096,
                    -0.0000000203096,
                    -0.0000000203096,
                    -34.2982,
                    3951.5,
                    40.2084,
                    0.00000000340337,
                    0.00000000340337,
                    0.00000000340337,
                    884.091,
                    40.2084,
                    4100.08,
                ],
            },
            Edge {
                edge_type: String::from("Observation3D"),
                vertices: vec![1, 2],
                restriction: vec![-0.034127, 2.24359, -0.503123],
                information_matrix: vec![
                    3934.45, -9.14727, 63.005, -9.14727, 3998.72, 10.7561, 63.005, 10.7561, 3909.38,
                ],
            },
        ];
        let mut fixed_vertices = BTreeSet::new();
        fixed_vertices.insert(0);
        FactorGraphModel {
            vertices,
            edges,
            fixed_vertices,
        }
    }

    #[test]
    fn test_3d_type_parsing() {
        init();
        let parsed_model = G2oParser::parse_file_to_model("data_files/full_demos/all_3d_types.g2o").unwrap();
        dbg!("{:?}", &parsed_model);
        let expected_model = get_3d_model();
        assert_eq!(parsed_model, expected_model);
    }

    #[test]
    fn test_3d_type_composition() {
        let model = get_3d_model();
        let composed_string = G2oParser::compose_model_to_string(model).unwrap();
        let expected_string = fs::read_to_string("data_files/full_demos/all_3d_types.g2o").unwrap();
        assert_eq!(&composed_string, &expected_string);
    }
}
