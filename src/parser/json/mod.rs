use crate::parser::Parser;
use crate::parser::model::{FactorGraphModel, Vertex, Edge};

pub struct JsonParser;

impl Parser for JsonParser {
    fn parse_string_to_model(s: &str) -> Result<FactorGraphModel, String> {
        match serde_json::from_str::<FactorGraphModel>(s) {
            Ok(model) => Ok(model),
            Err(e) => Err(format!("Parsing to FactorGraphModel unsuccessful: {}", e)),
        }
    }

    fn compose_model_to_string(model: FactorGraphModel) -> Result<String, String> {
        match serde_json::to_string(&model) {
            Ok(s) => Ok(s),
            Err(e) => Err(format!("Composing FactorGraphModel as JSON string unsuccessful: {}", e)),
        }
    }

}

#[cfg(test)]
mod tests {
    use log::LevelFilter;
    use super::*;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn test_parse_valid_file() {
        init();

        let parsed_model: FactorGraphModel = match JsonParser::parse_file_to_model("test_files/testTrajectory2DAngle.json") {
            Ok(x) => x,
            Err(str) => panic!(str),
        };
        dbg!("{:?}", &parsed_model);

        let vertices = vec![Vertex::new( 0, String::from("POSE2D_ANGLE"), [1.0, 0.0], [1.57] ),
                            Vertex::new( 1, String::from("POSE2D_ANGLE"), [0.0, 1.0], [3.14] )];
        let edges = vec![Edge::new( String::from("PRIOR2D_ANGLE"), vec![0], [1.0, 0.0, 1.58],
                                    [10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.5] ),
                         Edge::new( String::from("PRIOR2D_ANGLE"), vec![1], [0.0, 1.0, 3.13],
                                    [10.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.5] ),
                         Edge::new( String::from("ODOMETRY2D_ANGLE"), vec![0, 1], [1.0, 1.0, 1.57],
                                    [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.1] )];
        let expected_model = FactorGraphModel::new(vertices, edges);
        assert_eq!(parsed_model, expected_model);
    }

    #[test]
    #[should_panic]
    fn test_parse_invalid_file() {
        init();

        let parsed_model: FactorGraphModel = match JsonParser::parse_file_to_model("test_files/testBrokenTrajectory2DAngle.json") {
            Ok(x) => x,
            Err(str) => panic!(str),
        };
        info!("TEST FAILED! The invalid file was able to be parsed: {:?}", parsed_model);
    }

    #[test]
    #[should_panic]
    fn test_parse_missing_file() {
        init();

        let parsed_model: FactorGraphModel = match JsonParser::parse_file_to_model("test_files/missing_file.json") {
            Ok(x) => x,
            Err(str) => panic!(str),
        };
        info!("TEST FAILED! The missing file was able to be parsed: {:?}", parsed_model);
    }

    // TODO Should compose_model_to_string() be tested with unit tests?
}