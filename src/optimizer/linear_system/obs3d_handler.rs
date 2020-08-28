// gs-rs - Graph SLAM in Rust
// --------------
//
// © 2020 Samuel Valenzuela (samuel.valenzuela@tngtech.com)
// © 2020 Florian Rohm (florian.rohm@tngtech.com)
// © 2020 Daniel Pape (daniel.pape@tngtech.com)
//
// This product includes software developed at
// TNG Technology Consulting GmbH (https://www.tngtech.com/).
//
// gs-rs is licensed under the Apache License, Version 2.0 (LICENSE-APACHE.md or
// http://www.apache.org/licenses/LICENSE-2.0) or the MIT license (LICENSE-MIT.md
// or http://opensource.org/licenses/MIT), at your option.

#![allow(non_snake_case)]

use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::{FixedType, LandmarkVariable3D, VehicleVariable3D};
use crate::optimizer::linear_system::iso3d_gradients::{get_isometry, skew_trans};
use nalgebra::{
    DMatrix, DVector, Dynamic, Isometry3, Matrix, Matrix3, MatrixMN, RowVector3, SliceStorage, Translation3, Vector,
    Vector3, U1, U3, U9,
};

pub fn update_H_b(
    H: &mut DMatrix<f64>,
    b: &mut DVector<f64>,
    factor: &Factor,
    var_i: &VehicleVariable3D,
    var_j: &LandmarkVariable3D,
) {
    let iso_i = get_isometry(&*var_i.pose.borrow());
    let trans_j = get_trans(&*var_j.position.borrow());
    let local_j = (iso_i.inverse() * trans_j).translation;
    let pos_ij = get_pos(&factor.constraint);
    let (jacobi, jacobi_T) = calc_jacobians(&iso_i, &local_j);
    let right_mult = &factor.information_matrix.content * jacobi;

    let H_updates = jacobi_T * &right_mult;
    update_H_submatrix(H, &H_updates.index((..6, ..6)), &var_i.fixed_type, &var_i.fixed_type);
    update_H_submatrix(H, &H_updates.index((..6, 6..)), &var_i.fixed_type, &var_j.fixed_type);
    update_H_submatrix(H, &H_updates.index((6.., ..6)), &var_j.fixed_type, &var_i.fixed_type);
    update_H_submatrix(H, &H_updates.index((6.., 6..)), &var_j.fixed_type, &var_j.fixed_type);

    let err_pos = local_j.vector - pos_ij;
    let err_vec = err_pos.data.to_vec();
    let b_updates = (RowVector3::from_vec(err_vec) * &right_mult).transpose();
    update_b_subvector(b, &b_updates.index((..6, ..)), &var_i.fixed_type);
    update_b_subvector(b, &b_updates.index((6.., ..)), &var_j.fixed_type);
}

fn calc_jacobians(
    iso_i: &Isometry3<f64>,
    local_j: &Translation3<f64>,
) -> (MatrixMN<f64, U3, U9>, MatrixMN<f64, U9, U3>) {
    let rot_i_inv = iso_i.inverse().rotation.to_rotation_matrix();
    let mut jacobian = MatrixMN::<f64, U3, U9>::from_vec(vec![0.0; 27]);
    jacobian.index_mut((.., 0..3)).copy_from(&-Matrix3::<f64>::identity());
    jacobian
        .index_mut((.., 3..6))
        .copy_from(&skew_trans(&local_j).transpose());
    jacobian.index_mut((.., 6..9)).copy_from(rot_i_inv.matrix());
    (jacobian, jacobian.transpose())
}

fn update_H_submatrix(
    H: &mut DMatrix<f64>,
    added_matrix: &Matrix<f64, Dynamic, Dynamic, SliceStorage<f64, Dynamic, Dynamic, U1, U9>>,
    row_type: &FixedType,
    col_type: &FixedType,
) {
    if let (FixedType::NonFixed(row_range), FixedType::NonFixed(col_range)) = (row_type, col_type) {
        let updated_submatrix = &(H.index((row_range.to_owned(), col_range.to_owned())) + added_matrix);
        H.index_mut((row_range.to_owned(), col_range.to_owned()))
            .copy_from(updated_submatrix);
    }
}

fn update_b_subvector(
    b: &mut DVector<f64>,
    added_vector: &Vector<f64, Dynamic, SliceStorage<f64, Dynamic, U1, U1, U9>>,
    fixed_type: &FixedType,
) {
    if let FixedType::NonFixed(range) = fixed_type {
        let range = range.to_owned();
        let updated_subvector = &(b.index((range.clone(), ..)) + added_vector);
        b.index_mut((range, ..)).copy_from(updated_subvector);
    }
}

fn get_trans(data: &[f64; 3]) -> Translation3<f64> {
    Translation3::new(data[0], data[1], data[2])
}

fn get_pos(data: &[f64]) -> Vector3<f64> {
    Vector3::new(data[0], data[1], data[2])
}
