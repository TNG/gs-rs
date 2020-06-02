use nalgebra::{DMatrix, DVector, Matrix6};
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var: &Box<dyn Variable>) {
    if var.is_fixed() {
        return;
    }
    // get isometries
    // let (jacobi, jacobi_T) = calc_jacobians(rot_m);
    // let right_mult = &factor.information_matrix.content * jacobi;

    // let H_update = jacobi_T * &right_mult;
    // update_H_submatrix(H, &H_update, var);
    // update b
}

// fn calc_jacobians(rot_m: f64) -> (Matrix6<f64>, Matrix6<f64>) {
//     let jacobian = *Rotation3::from_axis_angle(&Vector3::z_axis(), -rot_m).matrix();
//     (jacobian, jacobian.transpose())
// }