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
use crate::optimizer::linear_system::iso3d_gradients::{calc_dq_dR, get_isometry, skew_matr_and_mult_parts};
use nalgebra::{ArrayStorage, DMatrix, DVector, Isometry3, Matrix, Matrix3, Matrix6, RowVector6, Vector, U1, U6};
use std::ops::Range;

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var: &VehicleVariable3D) {
    let range = if let FixedType::NonFixed(range) = &var.fixed_type {
        range
    } else {
        return;
    };

    let iso_v = get_isometry(&*var.pose.borrow());
    let iso_m = get_isometry(&factor.constraint);
    let (jacobi, jacobi_T) = calc_jacobians(&iso_v, &iso_m);
    let right_mult = &factor.information_matrix.content * jacobi;

    let H_update = jacobi_T * &right_mult;
    update_H_submatrix(H, &H_update, &range);

    let err = iso_m.inverse() * iso_v;
    let mut err_vec = err.translation.vector.data.to_vec();
    err_vec.extend_from_slice(&err.rotation.quaternion().coords.data.to_vec()[..3]);
    let b_update = (RowVector6::from_vec(err_vec) * &right_mult).transpose();
    update_b_subvector(b, &b_update, &range);
}

fn calc_jacobians(iso_v: &Isometry3<f64>, iso_m: &Isometry3<f64>) -> (Matrix6<f64>, Matrix6<f64>) {
    let Err_iso = iso_m.inverse() * iso_v;
    let Err_rot = Err_iso.rotation.to_rotation_matrix();
    let dq_dR = calc_dq_dR(&Err_rot.matrix()); // variable name taken over from g2o

    let mut jacobian = Matrix6::from_vec(vec![0.0; 72]);
    jacobian.index_mut((..3, ..3)).copy_from(&Err_rot.matrix());
    jacobian
        .index_mut((3.., 3..))
        .copy_from(&(dq_dR * skew_matr_and_mult_parts(&Matrix3::<f64>::identity(), &Err_rot.matrix())));

    (jacobian, jacobian.transpose())
}

fn update_H_submatrix(H: &mut DMatrix<f64>, added_matrix: &Matrix<f64, U6, U6, ArrayStorage<f64, U6, U6>>, range: &Range<usize>) {
    let updated_submatrix = &(H.index((range.to_owned(), range.to_owned())) + added_matrix);
    H.index_mut((range.to_owned(), range.to_owned())).copy_from(updated_submatrix);
}

fn update_b_subvector(b: &mut DVector<f64>, added_vector: &Vector<f64, U6, ArrayStorage<f64, U6, U1>>, range: &Range<usize>) {
    let updated_subvector = &(b.index((range.to_owned(), ..)) + added_vector);
    b.index_mut((range.to_owned(), ..)).copy_from(updated_subvector);
}
