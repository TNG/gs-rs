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


#![allow(non_snake_case)]

use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::{FixedType, LandmarkVariable2D, VehicleVariable2D};
use nalgebra::{DMatrix, DVector, Dynamic, Matrix, Matrix2x5, Matrix5x2, Rotation2, RowVector2, SliceStorage, Vector, Vector2, U1, U5};

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var_i: &VehicleVariable2D, var_j: &LandmarkVariable2D) {
    let (pos_i, rot_i) = get_pos_and_rot(&*var_i.pose.borrow());
    let pos_j = get_pos(&*var_j.position.borrow());
    let pos_ij = get_pos(&factor.constraint);
    let (jacobi, jacobi_T) = calc_jacobians(&pos_i, rot_i, &pos_j);
    let right_mult = &factor.information_matrix.content * jacobi;

    let H_updates = jacobi_T * &right_mult;
    update_H_submatrix(H, &H_updates.index((..3, ..3)), &var_i.fixed_type, &var_i.fixed_type);
    update_H_submatrix(H, &H_updates.index((..3, 3..)), &var_i.fixed_type, &var_j.fixed_type);
    update_H_submatrix(H, &H_updates.index((3.., ..3)), &var_j.fixed_type, &var_i.fixed_type);
    update_H_submatrix(H, &H_updates.index((3.., 3..)), &var_j.fixed_type, &var_j.fixed_type);

    let err_pos = Rotation2::new(-rot_i) * (pos_j - pos_i) - pos_ij;
    let err_vec = err_pos.data.to_vec();
    let b_updates = (RowVector2::from_vec(err_vec) * &right_mult).transpose();
    update_b_subvector(b, &b_updates.index((..3, ..)), &var_i.fixed_type);
    update_b_subvector(b, &b_updates.index((3.., ..)), &var_j.fixed_type);
}

fn calc_jacobians(pos_i: &Vector2<f64>, rot_i: f64, pos_j: &Vector2<f64>) -> (Matrix2x5<f64>, Matrix5x2<f64>) {
    let delta_pos_vec = pos_j - pos_i;
    let delta_pos = delta_pos_vec.data.as_slice();
    let sin_i = rot_i.sin();
    let cos_i = rot_i.cos();
    let mid_col_top = -sin_i * delta_pos[0] + cos_i * delta_pos[1];
    let mid_col_bot = -cos_i * delta_pos[0] - sin_i * delta_pos[1];
    #[rustfmt::skip]
    let jacobian = Matrix2x5::from_vec(vec![     -cos_i,       sin_i,    // transposed matrix is displayed
                                                 -sin_i,      -cos_i,
                                            mid_col_top, mid_col_bot,
                                                  cos_i,      -sin_i,
                                                  sin_i,       cos_i,]);
    (jacobian, jacobian.transpose())
}

fn update_H_submatrix(
    H: &mut DMatrix<f64>,
    added_matrix: &Matrix<f64, Dynamic, Dynamic, SliceStorage<f64, Dynamic, Dynamic, U1, U5>>,
    var_row: &FixedType,
    var_col: &FixedType,
) {
    if let (FixedType::NonFixed(row_range), FixedType::NonFixed(col_range)) = (var_row, var_col) {
        let updated_submatrix = &(H.index((row_range.to_owned(), col_range.to_owned())) + added_matrix);
        H.index_mut((row_range.to_owned(), col_range.to_owned())).copy_from(updated_submatrix);
    }
}

fn update_b_subvector(b: &mut DVector<f64>, added_vector: &Vector<f64, Dynamic, SliceStorage<f64, Dynamic, U1, U1, U5>>, var: &FixedType) {
    if let FixedType::NonFixed(range) = var {
        let range = range.to_owned();
        let updated_subvector = &(b.index((range.clone(), ..)) + added_vector);
        b.index_mut((range, ..)).copy_from(updated_subvector);
    }
}

fn get_pos(pos_vec: &[f64]) -> Vector2<f64> {
    Vector2::new(pos_vec[0], pos_vec[1])
}

fn get_pos_and_rot(pose: &[f64]) -> (Vector2<f64>, f64) {
    (Vector2::new(pose[0], pose[1]), pose[2])
}
