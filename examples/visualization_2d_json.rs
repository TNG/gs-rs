use gs_rs::parser::json::JsonParser;
use gs_rs::parser::Parser;
use gs_rs::visualizer::visualize;

fn main() {
    // parse json file containing 2D vertices (variables) and edges (factors) to internal factor graph representation
    let factor_graph = JsonParser::parse_file("examples/io_files/All_Types_2D.json").unwrap();

    // visualize the factor graph (does not work multiple times in a single execution of the program)
    visualize(&factor_graph);
}