#![allow(non_snake_case)]

use nalgebra::{DMatrix, DVector, Vector2, Matrix2x5, Matrix5x2, Matrix, Dynamic, U1, U5, SliceStorage, Vector, RowVector2, Rotation2};
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var_i: &Box<dyn Variable>, var_j: &Box<dyn Variable>) {
    let (pos_i, rot_i) = get_pos_and_rot(&var_i.get_content());
    let pos_j = get_pos(&var_j.get_content());
    let pos_ij = get_pos(&factor.constraint);
    let (jacobi, jacobi_T) = calc_jacobians(&pos_i, rot_i, &pos_j);
    let right_mult = &factor.information_matrix.content * jacobi;

    let H_updates = jacobi_T * &right_mult;
    update_H_submatrix(H, &H_updates.index((..3, ..3)), var_i, var_i);
    update_H_submatrix(H, &H_updates.index((..3, 3..)), var_i, var_j);
    update_H_submatrix(H, &H_updates.index((3.., ..3)), var_j, var_i);
    update_H_submatrix(H, &H_updates.index((3.., 3..)), var_j, var_j);

    let err_pos = Rotation2::new(-rot_i) * (pos_j - pos_i) - pos_ij;
    let err_vec = err_pos.data.to_vec();
    let b_updates = (RowVector2::from_vec(err_vec) * &right_mult).transpose();
    update_b_subvector(b, &b_updates.index((..3, ..)), var_i);
    update_b_subvector(b, &b_updates.index((3.., ..)), var_j);
}

fn calc_jacobians(pos_i: &Vector2<f64>, rot_i: f64, pos_j: &Vector2<f64>) -> (Matrix2x5<f64>, Matrix5x2<f64>) {
    let delta_pos_vec = pos_j - pos_i;
    let delta_pos = delta_pos_vec.data.as_slice();
    let sin_i = rot_i.sin();
    let cos_i = rot_i.cos();
    let mid_col_top = -sin_i * delta_pos[0] + cos_i * delta_pos[1];
    let mid_col_bot = -cos_i * delta_pos[0] - sin_i * delta_pos[1];
    let jacobian = Matrix2x5::from_vec(vec![     -cos_i,       sin_i,    // transposed matrix is displayed
                                                 -sin_i,      -cos_i,
                                            mid_col_top, mid_col_bot,
                                                  cos_i,      -sin_i,
                                                  sin_i,       cos_i,]);
    (jacobian, jacobian.transpose())
}

fn update_H_submatrix(H: &mut DMatrix<f64>, added_matrix: &Matrix<f64, Dynamic, Dynamic, SliceStorage<f64,Dynamic,Dynamic,U1,U5>>, var_row: &Box<dyn Variable>, var_col: &Box<dyn Variable>) {
    if var_row.is_fixed() || var_col.is_fixed() {
        return;
    }
    let row_range = var_row.get_range().unwrap();
    let col_range = var_col.get_range().unwrap();
    let updated_submatrix = &(H.index((row_range.clone(), col_range.clone())) + added_matrix);
    H.index_mut((row_range, col_range)).copy_from(updated_submatrix);
}

fn update_b_subvector(b: &mut DVector<f64>, added_vector: &Vector<f64, Dynamic, SliceStorage<f64,Dynamic,U1,U1,U5>>, var: &Box<dyn Variable>) {
    if var.is_fixed() {
        return;
    }
    let range = var.get_range().unwrap();
    let updated_subvector = &(b.index((range.clone(), ..)) + added_vector);
    b.index_mut((range, ..)).copy_from(updated_subvector);
}

fn get_pos(pos_vec: &[f64]) -> Vector2<f64> {
    Vector2::new(pos_vec[0], pos_vec[1])
}

fn get_pos_and_rot(pose: &[f64]) -> (Vector2<f64>, f64) {
    (Vector2::new(pose[0], pose[1]), pose[2])
}