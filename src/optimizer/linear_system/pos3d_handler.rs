use nalgebra::{DMatrix, DVector, Matrix3, Matrix6, Isometry3};
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;
use crate::optimizer::linear_system::iso3d_gradients::*;

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var: &Box<dyn Variable>) {
    if var.is_fixed() {
        return;
    }
    let iso_v = get_isometry(&var.get_content());
    let iso_m = get_isometry(&factor.constraint);
    let (jacobi, jacobi_T) = calc_jacobians(&iso_v, &iso_m);
    let right_mult = &factor.information_matrix.content * jacobi;

    print!("Prior Jacobian:{}", jacobi);

    // let H_update = jacobi_T * &right_mult;
    // update_H_submatrix(H, &H_update, var);
    // update b
}

fn calc_jacobians(iso_v: &Isometry3<f64>, iso_m: &Isometry3<f64>) -> (Matrix6<f64>, Matrix6<f64>) {
    let Err_iso = iso_m.inverse() * iso_v;
    let Err_rot = Err_iso.rotation.to_rotation_matrix();
    let dq_dR = calc_dq_dR(&Err_rot.matrix()); // variable name taken over from g2o

    let mut jacobian = Matrix6::from_vec(vec![0.0; 72]);
    jacobian.index_mut((..3, ..3)).copy_from(&Err_rot.matrix());
    jacobian.index_mut((3.., 3..)).copy_from(&(dq_dR * skew_matr_and_mult_parts(&Matrix3::<f64>::identity(), &Err_rot.matrix())));

    (jacobian, jacobian.transpose())
}