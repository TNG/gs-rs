use nalgebra::{DMatrix, DVector, Matrix3, Vector3, MatrixMN, U3, U9, Translation3, Isometry3};
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;
use crate::optimizer::linear_system::iso3d_gradients::*;

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var_i: &Box<dyn Variable>, var_j: &Box<dyn Variable>) {
    let iso_i = get_isometry(&var_i.get_content());
    let pos_j = get_pos(&var_j.get_content());
    // let pos_ij = get_pos(&factor.constraint);
    let (jacobi, jacobi_T) = calc_jacobians(&iso_i, &pos_j);
    print!("Observation Jacobian_i:{}", jacobi.index((..,..6)));
    print!("Observation Jacobian_j:{}", jacobi.index((..,6..)));
    // let right_mult = &factor.information_matrix.content * jacobi;
    //
    // let H_updates = jacobi_T * &right_mult;
    // update_H_submatrix(H, &H_updates.index((..3, ..3)), var_i, var_i);
    // update_H_submatrix(H, &H_updates.index((..3, 3..)), var_i, var_j);
    // update_H_submatrix(H, &H_updates.index((3.., ..3)), var_j, var_i);
    // update_H_submatrix(H, &H_updates.index((3.., 3..)), var_j, var_j);
    //
    // let err_pos = Rotation2::new(-rot_i) * (pos_j - pos_i) - pos_ij;
    // let err_vec = err_pos.data.to_vec();
    // let b_updates = (RowVector2::from_vec(err_vec) * &right_mult).transpose();
    // update_b_subvector(b, &b_updates.index((..3, ..)), var_i);
    // update_b_subvector(b, &b_updates.index((3.., ..)), var_j);
}

fn calc_jacobians(iso_i: &Isometry3<f64>, pos_j: &Vector3<f64>/*, iso_ij: &Isometry3<f64>*/) -> (MatrixMN<f64, U3, U9>, MatrixMN<f64, U9, U3>) {
    let local_j = (iso_i.inverse() * Translation3::from(*pos_j)).translation;
    let rot_i_inv = iso_i.inverse().rotation.to_rotation_matrix();

    let mut jacobian = MatrixMN::<f64, U3, U9>::from_vec(vec![0.0; 27]);
    jacobian.index_mut((.., 0..3)).copy_from(&-Matrix3::<f64>::identity());
    jacobian.index_mut((.., 3..6)).copy_from(&skew_trans(&local_j).transpose());
    jacobian.index_mut((.., 6..9)).copy_from(rot_i_inv.matrix());
    (jacobian, jacobian.transpose())
}

fn get_pos(pos_vec: &[f64]) -> Vector3<f64> {
    Vector3::new(pos_vec[0], pos_vec[1], pos_vec[2])
}