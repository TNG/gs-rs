use gs_rs::parser::g2o::G2oParser;
use gs_rs::parser::Parser;
use gs_rs::visualizer::visualize;

fn main() {
    // parse g2o file containing 3D vertices (variables) and edges (factors) to internal factor graph representation
    let factor_graph = G2oParser::parse_file("examples/io_files/All_Types_3D.g2o").unwrap();

    // visualize the factor graph (does not work multiple times in a single execution of the program)
    visualize(&factor_graph);
}
