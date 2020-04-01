//! Conversion between factor graph structures and G2O files.

use crate::parser::model::FactorGraphModel;
use crate::parser::Parser;
use regex::Regex;
use crate::parser::json::JsonParser;

/// Implements G2O specific functions for parsing and composing files.
/// Uses the JsonParser as an intermediate step.
///
/// Current expectations:
/// 1) At least one vertex and edge exist.
/// 2) All vertices are listed before any edges.
pub struct G2oParser;

impl Parser for G2oParser {
    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String> {
        JsonParser::parse_string_to_model(&G2oParser::g2o_to_json(s))
    }

    fn compose_model_to_string(model: FactorGraphModel) -> Result<String, String> {
        unimplemented!()
    }
}

impl G2oParser {
    fn g2o_to_json(s: &str) -> String {
        let mut json_string = ["{\n  \"vertices\":\n  [\n", s, "  ]\n}"].concat();

        let fix_regex = Regex::new("FIX (.*)\n").unwrap();
        json_string = fix_regex.replace(&json_string, "").to_string();

        let pose_var2d_regex = Regex::new("VERTEX_SE2 (.*) (.*) (.*) (.*)").unwrap();
        json_string = pose_var2d_regex.replace_all(&json_string, "    { \"id\": $1, \"type\": \"POSE2D_ANGLE\", \"position\": [$2, $3], \"rotation\": [$4] },").to_string();

        let division_regex = Regex::new("},\nEDGE_SE2").unwrap();
        json_string = division_regex.replace(&json_string, "}\n  ],\n  \"edges\":\n  [\nEDGE_SE2").to_string();

        let odometry_factor2d_regex = Regex::new("EDGE_SE2 (.*) (.*) (.*) (.*) (.*) (.*) (.*) (.*) (.*) (.*) (.*)").unwrap();
        json_string = odometry_factor2d_regex.replace_all(&json_string, "    { \"type\": \"ODOMETRY2D_ANGLE\", \"vertices\": [$1, $2], \"restriction\": [$3, $4, $5], \"informationMatrix\": [$6, $7, $8, $7, $9, $10, $8, $10, $11] },").to_string();

        // TODO support more variable and factor types

        let end_regex = Regex::new("},\n  ]\n}").unwrap();
        json_string = end_regex.replace(&json_string, "}\n  ]\n}").to_string();

        json_string
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use log::LevelFilter;
    use std::fs;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn test_minimal_g2o_to_json() {
        init();

        let file_path = "test_files/minimal_size_and_types.g2o";
        let g2o_string = match fs::read_to_string(file_path) {
            Ok(s) => s,
            Err(_e) => panic!(format!("File could not be parsed: {}", file_path)),
        };
        info!("\n{}", &g2o_string);
        let json_string = G2oParser::g2o_to_json(&g2o_string);
        info!("\n{}", &json_string);
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
}