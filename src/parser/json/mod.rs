//! Conversion between factor graph structures and JSON files.

use crate::parser::model::FactorGraphModel;
use crate::parser::Parser;

// TODO properly document Json file format
/// Implements JSON specific functions for parsing and composing files.
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
            Err(e) => Err(format!(
                "Composing FactorGraphModel as JSON string unsuccessful: {}",
                e
            )),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::model::{Edge, Vertex};
    use log::LevelFilter;
    use std::collections::BTreeSet;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    #[ignore] // TODO use valid file
    fn test_parse_valid_file_to_model() {
        init();

        let parsed_model =
            match JsonParser::parse_file_to_model("test_files/testTrajectory2DAngle.json") {
                Ok(x) => x,
                Err(str) => panic!(str),
            };
        dbg!("{:?}", &parsed_model);

        let vertices = vec![
            Vertex {
                id: 0,
                vertex_type: String::from("POSE2D_ANGLE"),
                content: vec![0.0, 1.0, 0.0],
            },
            Vertex {
                id: 1,
                vertex_type: String::from("POSE2D_ANGLE"),
                content: vec![1.0, 0.0, 0.0],
            },
        ];
        let edges = vec![
            Edge {
                edge_type: String::from("PRIOR2D_ANGLE"),
                vertices: vec![0],
                restriction: vec![0.0, 1.0, 0.0],
                information_matrix: vec![10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.000_001],
            },
            Edge {
                edge_type: String::from("PRIOR2D_ANGLE"),
                vertices: vec![1],
                restriction: vec![1.0, 0.0, 0.0],
                information_matrix: vec![10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.000_001],
            },
            Edge {
                edge_type: String::from("ODOMETRY2D_ANGLE"),
                vertices: vec![0, 1],
                restriction: vec![0.5, -0.5, 0.0],
                information_matrix: vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.000_001],
            },
        ];
        let mut fixed_vertices = BTreeSet::new();
        fixed_vertices.insert(0);
        let expected_model = FactorGraphModel {
            vertices,
            edges,
            fixed_vertices,
        };
        assert_eq!(parsed_model, expected_model);
    }

    #[test]
    #[should_panic]
    fn test_parse_invalid_file() {
        init();

        let parsed_model: FactorGraphModel =
            match JsonParser::parse_file_to_model("test_files/testBrokenTrajectory2DAngle.json") {
                Ok(x) => x,
                Err(str) => panic!(str),
            };
        info!(
            "TEST FAILED! The invalid file was able to be parsed: {:?}",
            parsed_model
        );
    }

    #[test]
    #[should_panic]
    fn test_parse_missing_file() {
        init();

        let parsed_model: FactorGraphModel =
            match JsonParser::parse_file_to_model("test_files/missing_file.json") {
                Ok(x) => x,
                Err(str) => panic!(str),
            };
        info!(
            "TEST FAILED! The missing file was able to be parsed: {:?}",
            parsed_model
        );
    }

    // TODO Should compose_model_to_string() be tested with unit tests?
}
