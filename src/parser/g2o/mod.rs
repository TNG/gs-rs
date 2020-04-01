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
        JsonParser::parse_string_to_model(&Self::g2o_to_json(s))
    }

    fn compose_model_to_string(model: FactorGraphModel) -> Result<String, String> {
        match JsonParser::compose_model_to_string(model) {
            Ok(s) => Ok(Self::json_to_g2o(&s)),
            Err(s) => Err(s),
        }
    }
}

impl G2oParser {
    fn g2o_to_json(g2o_string: &str) -> String {
        let mut json_string = ["{\n  \"vertices\":\n  [\n", g2o_string, "  ]\n}"].concat();

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

    fn json_to_g2o(json_string: &str) -> String {
        let mut g2o_string = json_string.to_owned();

        let start_end_regex = Regex::new("\\{\n  \"vertices\": \\[\n((\\S|\\s)*)\n  ]\n}").unwrap();
        g2o_string = start_end_regex.replace(&g2o_string, "$1").to_string();

        let division_regex = Regex::new("  ],\n  \"edges\": \\[\n").unwrap();
        g2o_string = division_regex.replace(&g2o_string, "").to_string();

        let pose_var2d_regex = Regex::new(" {4}\\{\n {6}\"id\": (.*),\n {6}\"type\": \"POSE2D_ANGLE\",\n {6}\"position\": \\[\n {8}(.*),\n {8}(.*)\n {6}],\n {6}\"rotation\": \\[\n {8}(.*)\n {6}]\n {4}},?").unwrap();
        g2o_string = pose_var2d_regex.replace_all(&g2o_string, "VERTEX_SE2 $1 $2 $3 $4").to_string();

        // TODO finish regex replacement
        let odometry_factor2d_regex = Regex::new(" {4}\\{").unwrap();
        g2o_string = odometry_factor2d_regex.replace_all(&g2o_string, "EDGE_SE2 ... TODO").to_string();

        g2o_string
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

    #[test]
    fn test_dumb_json_to_g2o() {
        init();

        let file_path = "test_files/dumb.json";
        let json_string = match fs::read_to_string(file_path) {
            Ok(s) => s,
            Err(_e) => panic!(format!("File could not be parsed: {}", file_path)),
        };
        // info!("\n{}", &json_string);
        let g2o_string = G2oParser::json_to_g2o(&json_string);
        info!("\n{}", &g2o_string);
    }

    // TODO Should compose_model_to_string() be tested with unit tests?
}