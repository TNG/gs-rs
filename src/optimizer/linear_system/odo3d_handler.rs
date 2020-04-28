use nalgebra::{DMatrix, DVector, Vector2, Matrix3x6, Matrix6x3, Matrix, Dynamic, U1, U6, Vector, SliceStorage, Matrix3, Vector3, Rotation3, RowVector3, Rotation2};
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;
use std::f64::consts::PI;

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var_i: &Box<dyn Variable>, var_j: &Box<dyn Variable>) {
    unimplemented!() // TODO take a look at g2o and implement
}