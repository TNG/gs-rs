// as of rust 2018 edition, extern crate is not needed in most cases.
// one special case is in here with the log and approx crates:
// as far as I understand, the macro_use imports all macros declared with #[macro_export] in all descending modules
// with  ```use log::info``` it is only imported in the current module and hence encapsulated.
// one caveat: if one wants to exchange the logging macro from log::info to let's say fancy_log::info one would need to go
// in every file and change it. Even on midsized projects this search and replace would be worth it imho.
// For really big changes one could think of an adapter module, encapsulating this external dependency (never needed this, yet...)


use crate::parser::g2o::G2oParser;
use crate::parser::Parser;

pub mod factor_graph;
pub mod optimizer;
pub mod parser;
pub mod visualizer;

/// Example for the usage of the library
pub fn optimize(in_file: &str, out_file: &str, iterations: usize) {
    let factor_graph = G2oParser::parse_file(in_file).unwrap();

    optimizer::optimize(&factor_graph, iterations);

    G2oParser::compose_file(&factor_graph, out_file).unwrap();
}
