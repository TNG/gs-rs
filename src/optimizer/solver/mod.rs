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
