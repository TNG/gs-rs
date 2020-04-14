use nalgebra::{DMatrix, DVector};
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var_i: &Box<dyn Variable>, var_j: &Box<dyn Variable>) {
    unimplemented!() // TODO
}
