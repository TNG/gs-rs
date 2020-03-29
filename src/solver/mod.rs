#![allow(non_snake_case)]

use nalgebra::{DMatrix, DVector};

pub mod cholesky;
pub mod lu;
pub mod sparse_cholesky;

pub trait Solver {
    /// Solves the linear equation system defined by
    /// H*x = b
    /// H is expected column-by-column
    fn solve(b: &DVector<f64>, H: DMatrix<f64>) -> Result<Vec<f64>, String>;
}
