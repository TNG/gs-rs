#![allow(non_snake_case)]

use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::{FixedType, VehicleVariable2D};
use nalgebra::{
    DMatrix, DVector, Dynamic, Matrix, Matrix3, Matrix3x6, Matrix6x3, Rotation2, Rotation3, RowVector3, SliceStorage,
    Vector, Vector2, Vector3, U1, U6,
};
use std::f64::consts::PI;

pub fn update_H_b(
    H: &mut DMatrix<f64>,
    b: &mut DVector<f64>,
    factor: &Factor,
    var_i: &VehicleVariable2D,
    var_j: &VehicleVariable2D,
) {
    let (pos_i, rot_i) = get_pos_and_rot(&*var_i.pose.borrow());
    let (pos_j, rot_j) = get_pos_and_rot(&*var_j.pose.borrow());
    let (pos_ij, rot_ij) = get_pos_and_rot(&factor.constraint);
    let (jacobi, jacobi_T) = calc_jacobians(&pos_i, rot_i, &pos_j, rot_ij);
    let right_mult = &factor.information_matrix.content * jacobi;

    let H_updates = jacobi_T * &right_mult;
    update_H_submatrix(H, &H_updates.index((..3, ..3)), &var_i.fixed_type, &var_i.fixed_type);
    update_H_submatrix(H, &H_updates.index((..3, 3..)), &var_i.fixed_type, &var_j.fixed_type);
    update_H_submatrix(H, &H_updates.index((3.., ..3)), &var_j.fixed_type, &var_i.fixed_type);
    update_H_submatrix(H, &H_updates.index((3.., 3..)), &var_j.fixed_type, &var_j.fixed_type);

    let err_pos = Rotation2::new(-rot_ij) * (Rotation2::new(-rot_i) * (pos_j - pos_i) - pos_ij);
    let mut err_rot = rot_j - rot_i - rot_ij;
    if err_rot >= PI {
        err_rot -= 2.0 * PI;
    } else if err_rot < -PI {
        err_rot += 2.0 * PI;
    }
    let mut err_vec = err_pos.data.to_vec();
    err_vec.push(err_rot);
    let b_updates = (RowVector3::from_vec(err_vec) * &right_mult).transpose();
    update_b_subvector(b, &b_updates.index((..3, ..)), &var_i.fixed_type);
    update_b_subvector(b, &b_updates.index((3.., ..)), &var_j.fixed_type);
}

fn calc_jacobians(
    pos_i: &Vector2<f64>,
    rot_i: f64,
    pos_j: &Vector2<f64>,
    rot_ij: f64,
) -> (Matrix3x6<f64>, Matrix6x3<f64>) {
    let rot_obj = Rotation3::from_axis_angle(&Vector3::z_axis(), -rot_ij);
    let R_ij_T = rot_obj.matrix();
    let delta_pos_vec = pos_j - pos_i;
    let delta_pos = delta_pos_vec.data.as_slice();
    let sin_i = rot_i.sin();
    let cos_i = rot_i.cos();
    let last_column_top = -sin_i * delta_pos[0] + cos_i * delta_pos[1];
    let last_column_mid = -cos_i * delta_pos[0] - sin_i * delta_pos[1];
    #[rustfmt::skip]
    let jacobian_i = R_ij_T * &Matrix3::from_vec(vec![         -cos_i,           sin_i,  0.0,    // transposed matrix is displayed
                                                               -sin_i,          -cos_i,  0.0,
                                                      last_column_top, last_column_mid, -1.0,]);
    let jacobian_j = R_ij_T * Rotation3::from_axis_angle(&Vector3::z_axis(), -rot_i).matrix();
    let mut jacobian = Matrix3x6::from_vec(vec![0.0; 18]);
    jacobian.index_mut((.., ..3)).copy_from(&jacobian_i);
    jacobian.index_mut((.., 3..)).copy_from(&jacobian_j);
    (jacobian, jacobian.transpose())
}

fn update_H_submatrix(
    H: &mut DMatrix<f64>,
    added_matrix: &Matrix<f64, Dynamic, Dynamic, SliceStorage<f64, Dynamic, Dynamic, U1, U6>>,
    row_type: &FixedType,
    col_type: &FixedType,
) {
    if let (FixedType::NonFixed(row_range), FixedType::NonFixed(col_range)) = (row_type, col_type) {
        let (row_range, col_range) = (row_range.to_owned(), col_range.to_owned());
        let updated_submatrix = &(H.index((row_range.clone(), col_range.clone())) + added_matrix);
        H.index_mut((row_range, col_range)).copy_from(updated_submatrix);
    }
}

fn update_b_subvector(
    b: &mut DVector<f64>,
    added_vector: &Vector<f64, Dynamic, SliceStorage<f64, Dynamic, U1, U1, U6>>,
    fixed_type: &FixedType,
) {
    if let FixedType::NonFixed(range) = fixed_type {
        let range = range.to_owned();

        let updated_subvector = &(b.index((range.clone(), ..)) + added_vector);
        b.index_mut((range, ..)).copy_from(updated_subvector);
    }
}

fn get_pos_and_rot(pose: &[f64]) -> (Vector2<f64>, f64) {
    (Vector2::new(pose[0], pose[1]), pose[2])
}
