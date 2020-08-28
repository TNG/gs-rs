// gs-rs - Graph SLAM in Rust
// --------------
//
// © 2020 Samuel Valenzuela (samuel.valenzuela@tngtech.com)
// © 2020 Florian Rohm (florian.rohm@tngtech.com)
// © 2020 Daniel Pape (daniel.pape@tngtech.com)
//
// This product includes software developed at
// TNG Technology Consulting GmbH (https://www.tngtech.com/).
//
// gs-rs is licensed under the Apache License, Version 2.0 (LICENSE-APACHE.md or
// http://www.apache.org/licenses/LICENSE-2.0) or the MIT license (LICENSE-MIT.md
// or http://opensource.org/licenses/MIT), at your option.

//! Solvers for systems of linear equations (linear systems).

#![allow(non_snake_case)]

use nalgebra::{DMatrix, DVector};

pub mod sparse_cholesky;

/// Trait which all solvers should implement.
pub trait Solver {
    /// Solves the linear system defined by H*x = b.
    /// H is expected column-by-column.
    fn solve(H: DMatrix<f64>, b: &DVector<f64>) -> Result<Vec<f64>, String>;
}
