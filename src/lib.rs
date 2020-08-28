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


pub mod factor_graph;
pub mod optimizer;
pub mod parser;
pub mod visualizer;

use parser::g2o::G2oParser;
use parser::Parser;

/// Example for the usage of the library
pub fn optimize(in_file: &str, out_file: &str, iterations: usize) {
    let factor_graph = G2oParser::parse_file(in_file).unwrap();

    optimizer::optimize(&factor_graph, iterations);

    G2oParser::compose_file(&factor_graph, out_file).unwrap();
}
