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
use gs_rs::parser::json::JsonParser;
use gs_rs::parser::Parser;

fn main() {
    // parse file in one format
    let factor_graph = G2oParser::parse_file("examples/io_files/MIT_2D.g2o").unwrap();

    // compose file in another format
    JsonParser::compose_file(&factor_graph, "examples/io_files/MIT_2D.json").unwrap();
}
