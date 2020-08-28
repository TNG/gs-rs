use gs_rs::optimizer::optimize;
use gs_rs::parser::g2o::G2oParser;
use gs_rs::parser::Parser;

fn main() {
    // parse g2o file containing 3D variables and odometries to internal factor graph representation
    let factor_graph = G2oParser::parse_file("examples/io_files/Sphere_3D.g2o").unwrap();

    // optimize the factor graph's variables with 10 iterations
    optimize(&factor_graph, 10);

    // compose g2o file containing optimized 3D variables and unchanged odometries
    G2oParser::compose_file(&factor_graph, "examples/io_files/Sphere_3D_optimized.g2o").unwrap();
}
