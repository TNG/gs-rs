#![allow(non_snake_case)]

use nalgebra::{DVector, DMatrix};

pub mod sparse_cholesky;
pub mod cholesky;
pub mod lu;

pub trait Solver {
    /// Solves the linear equation system defined by
    /// H*x = b
    /// H is expected column-by-column
    fn solve(b: &DVector<f64>, H: DMatrix<f64>) -> Result<Vec<f64>, String>;
}
