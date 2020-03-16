#![allow(non_snake_case)]
pub mod cholesky;
pub mod lu;

pub trait Solver {
    /// Solves the linear equation system defined by
    /// H*x = b
    /// H is expected column-by-column
    fn solve(b: &[f64], H: &[f64]) -> Result<Vec<f64>, String>;
}
