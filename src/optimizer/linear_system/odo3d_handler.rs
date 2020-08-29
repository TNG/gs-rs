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
use crate::factor_graph::variable::{FixedType, VehicleVariable3D};
use crate::optimizer::linear_system::iso3d_gradients::{calc_dq_dR, get_isometry, skew_matr_T_and_mult_parts, skew_matr_and_mult_parts, skew_trans};
use nalgebra::{DMatrix, DVector, Dynamic, Isometry3, Matrix, Matrix3, Matrix6, MatrixMN, RowVector6, SliceStorage, Vector, U1, U12, U6};

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var_i: &VehicleVariable3D, var_j: &VehicleVariable3D) {
    let iso_i = get_isometry(&*var_i.pose.borrow());
    let iso_j = get_isometry(&*var_j.pose.borrow());
    let iso_ij = get_isometry(&factor.constraint);
    let (jacobi, jacobi_T) = calc_jacobians(&iso_i, &iso_j, &iso_ij);
    let right_mult = &factor.information_matrix.content * jacobi;

    let H_updates = jacobi_T * &right_mult;
    update_H_submatrix(H, &H_updates.index((..6, ..6)), &var_i.fixed_type, &var_i.fixed_type);
    update_H_submatrix(H, &H_updates.index((..6, 6..)), &var_i.fixed_type, &var_j.fixed_type);
    update_H_submatrix(H, &H_updates.index((6.., ..6)), &var_j.fixed_type, &var_i.fixed_type);
    update_H_submatrix(H, &H_updates.index((6.., 6..)), &var_j.fixed_type, &var_j.fixed_type);

    let err = iso_ij.inverse() * iso_i.inverse() * iso_j;
    let mut err_vec = err.translation.vector.data.to_vec();
    err_vec.extend_from_slice(&err.rotation.quaternion().coords.data.to_vec()[..3]);
    let b_updates = (RowVector6::from_vec(err_vec) * &right_mult).transpose();
    update_b_subvector(b, &b_updates.index((..6, ..)), &var_i.fixed_type);
    update_b_subvector(b, &b_updates.index((6.., ..)), &var_j.fixed_type);
}

fn calc_jacobians(iso_i: &Isometry3<f64>, iso_j: &Isometry3<f64>, iso_ij: &Isometry3<f64>) -> (MatrixMN<f64, U6, U12>, MatrixMN<f64, U12, U6>) {
    let A_ij = iso_ij.inverse();
    let B_ij = iso_i.inverse() * iso_j;
    let Err_ij = A_ij * B_ij;
    let A_rot = A_ij.rotation.to_rotation_matrix();
    let B_rot = B_ij.rotation.to_rotation_matrix();
    let Err_rot = Err_ij.rotation.to_rotation_matrix();
    let dq_dR = calc_dq_dR(&Err_rot.matrix()); // variable name taken over from g2o

    let mut jacobian_i = Matrix6::from_vec(vec![0.0; 36]);
    let mut jacobian_j = Matrix6::from_vec(vec![0.0; 36]);
    jacobian_i.index_mut((..3, ..3)).copy_from(&(-1.0 * A_rot.matrix()));
    jacobian_j.index_mut((..3, ..3)).copy_from(Err_rot.matrix());
    jacobian_i
        .index_mut((..3, 3..))
        .copy_from(&(A_rot.matrix() * skew_trans(&B_ij.translation).transpose()));
    jacobian_i
        .index_mut((3.., 3..))
        .copy_from(&(dq_dR * skew_matr_T_and_mult_parts(&B_rot.matrix(), &A_rot.matrix())));
    jacobian_j
        .index_mut((3.., 3..))
        .copy_from(&(dq_dR * skew_matr_and_mult_parts(&Matrix3::<f64>::identity(), &Err_rot.matrix())));

    let mut jacobian = MatrixMN::<f64, U6, U12>::from_vec(vec![0.0; 72]);
    jacobian.index_mut((.., ..6)).copy_from(&jacobian_i);
    jacobian.index_mut((.., 6..)).copy_from(&jacobian_j);
    (jacobian, jacobian.transpose())
}

fn update_H_submatrix(
    H: &mut DMatrix<f64>,
    added_matrix: &Matrix<f64, Dynamic, Dynamic, SliceStorage<f64, Dynamic, Dynamic, U1, U12>>,
    row_type: &FixedType,
    col_type: &FixedType,
) {
    if let (FixedType::NonFixed(row_range), FixedType::NonFixed(col_range)) = (row_type, col_type) {
        let (row_range, col_range) = (row_range.to_owned(), col_range.to_owned());
        let updated_submatrix = &(H.index((row_range.clone(), col_range.clone())) + added_matrix);
        H.index_mut((row_range, col_range)).copy_from(updated_submatrix);
    }
}

fn update_b_subvector(b: &mut DVector<f64>, added_vector: &Vector<f64, Dynamic, SliceStorage<f64, Dynamic, U1, U1, U12>>, var: &FixedType) {
    if let FixedType::NonFixed(range) = var {
        let range = range.to_owned();

        let updated_subvector = &(b.index((range.clone(), ..)) + added_vector);
        b.index_mut((range, ..)).copy_from(updated_subvector);
    }
}
