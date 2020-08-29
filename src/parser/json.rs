// -----------------------------------------------------------------------------------------------------
//                                      gs-rs - Graph SLAM in Rust
// -----------------------------------------------------------------------------------------------------
//
// SPDX-FileCopyrightText:      © 2020 Samuel Valenzuela (samuel.valenzuela@tngtech.com)
//                              © 2020 Florian Rohm (florian.rohm@tngtech.com)
//                              © 2020 Daniel Pape (daniel.pape@tngtech.com)
// SPDX-License-Identifier:     MIT OR Apache-2.0
//
// This product includes software developed at TNG Technology Consulting GmbH (https://www.tngtech.com/).
//

//! Conversion between factor graph structures and JSON files.

use crate::parser::model::FactorGraphModel;
use crate::parser::Parser;

/// Implements JSON specific functions for parsing and composing files.
///
/// Uses the JSON representation of [FactorGraphModel](../model/struct.FactorGraphModel.html).
pub struct JsonParser;

impl Parser for JsonParser {
    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String> {
        match serde_json::from_str::<FactorGraphModel>(s) {
            Ok(model) => Ok(model),
            Err(e) => Err(format!("Parsing to FactorGraphModel unsuccessful: {}", e)),
        }
    }

    fn compose_model_to_string(model: FactorGraphModel) -> Result<String, String> {
        match serde_json::to_string_pretty(&model) {
            Ok(s) => Ok(s),
            Err(e) => Err(format!("Composing FactorGraphModel as JSON string unsuccessful: {}", e)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::model::{Edge, Vertex};
    use log::info;
    use log::LevelFilter;
    use std::collections::BTreeSet;
    use std::fs;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    #[should_panic]
    fn test_parse_missing_file() {
        init();
        let parsed_model: FactorGraphModel = JsonParser::parse_file_to_model("data_files/missing_file.json").unwrap();
        info!(
            "TEST FAILED! The missing file was able to be parsed: {:?}",
            parsed_model
        );
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
        let parsed_model = JsonParser::parse_file_to_model("data_files/full_demos/all_2d_types.json").unwrap();
        dbg!("{:?}", &parsed_model);
        let expected_model = get_2d_model();
        assert_eq!(parsed_model, expected_model);
    }

    #[test]
    fn test_2d_type_composition() {
        let model = get_2d_model();
        let composed_string = JsonParser::compose_model_to_string(model).unwrap() + "\n";
        let expected_string = fs::read_to_string("data_files/full_demos/all_2d_types.json").unwrap();
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
        #[rustfmt::skip]
        let edges = vec![
            Edge {
                edge_type: String::from("Odometry3D"),
                vertices: vec![0, 1],
                restriction: vec![0.309576, 2.34636, 0.00315914, -0.139007, 0.0806488, 0.14657, 0.976059],
                information_matrix: vec![1.0, 0.000000000000000000962965, 0.000000000000000000962965, 0.0000000588441, -0.0000000203096, 0.00000000340337, 0.000000000000000000962965, 1.0, 0.000000000000000000962965, 0.0000000588441, -0.0000000203096, 0.00000000340337, 0.000000000000000000962965, 0.000000000000000000962965, 1.0, 0.0000000588441, -0.0000000203096, 0.00000000340337, 0.0000000588441, 0.0000000588441, 0.0000000588441, 4108.72, -34.2982, 884.091, -0.0000000203096, -0.0000000203096, -0.0000000203096, -34.2982, 3951.5, 40.2084, 0.00000000340337, 0.00000000340337, 0.00000000340337, 884.091, 40.2084, 4100.08],
            },
            Edge {
                edge_type: String::from("Position3D"),
                vertices: vec![1],
                restriction: vec![0.309576, 2.34636, 0.00315914, -0.139007, 0.0806488, 0.14657, 0.976059],
                information_matrix: vec![1.0, 0.000000000000000000962965, 0.000000000000000000962965, 0.0000000588441, -0.0000000203096, 0.00000000340337, 0.000000000000000000962965, 1.0, 0.000000000000000000962965, 0.0000000588441, -0.0000000203096, 0.00000000340337, 0.000000000000000000962965, 0.000000000000000000962965, 1.0, 0.0000000588441, -0.0000000203096, 0.00000000340337, 0.0000000588441, 0.0000000588441, 0.0000000588441, 4108.72, -34.2982, 884.091, -0.0000000203096, -0.0000000203096, -0.0000000203096, -34.2982, 3951.5, 40.2084, 0.00000000340337, 0.00000000340337, 0.00000000340337, 884.091, 40.2084, 4100.08],
            },
            Edge {
                edge_type: String::from("Observation3D"),
                vertices: vec![1, 2],
                restriction: vec![-0.034127, 2.24359, -0.503123],
                information_matrix: vec![3934.45, -9.14727, 63.005, -9.14727, 3998.72, 10.7561, 63.005, 10.7561, 3909.38],
            }
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
        let parsed_model = JsonParser::parse_file_to_model("data_files/full_demos/all_3d_types.json").unwrap();
        dbg!("{:?}", &parsed_model);
        let expected_model = get_3d_model();
        assert_eq!(parsed_model, expected_model);
    }

    #[test]
    fn test_3d_type_composition() {
        let model = get_3d_model();
        let composed_string = JsonParser::compose_model_to_string(model).unwrap();
        let expected_string = fs::read_to_string("data_files/full_demos/all_3d_types.json").unwrap();
        assert_eq!(&composed_string, &expected_string);
    }
}
