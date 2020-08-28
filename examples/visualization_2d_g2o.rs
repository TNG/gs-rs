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


use gs_rs::parser::g2o::G2oParser;
use gs_rs::parser::Parser;
use gs_rs::visualizer::visualize;

fn main() {
    // parse g2o file containing 2D vertices (variables) and edges (factors) to internal factor graph representation
    let factor_graph = G2oParser::parse_file("examples/io_files/All_Types_2D.g2o").unwrap();

    // visualize the factor graph (does not work multiple times in a single execution of the program)
    visualize(&factor_graph);
}
