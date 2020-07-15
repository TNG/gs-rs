use gs_rs::parser::g2o::G2oParser;
use gs_rs::parser::json::JsonParser;
use gs_rs::parser::Parser;

fn main() {
    // parse file in one format
    let factor_graph = G2oParser::parse_file("examples/io_files/MIT_2D.g2o").unwrap();

    // compose file in another format
    JsonParser::compose_file(&factor_graph, "examples/io_files/MIT_2D.json").unwrap();
}
