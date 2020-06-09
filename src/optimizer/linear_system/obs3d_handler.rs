use nalgebra::{DMatrix, DVector, Matrix3, MatrixMN, U1, U3, U9, Translation3, Isometry3, RowVector3, Matrix, Dynamic, Vector, SliceStorage, Vector3};
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;
use crate::optimizer::linear_system::iso3d_gradients::*;

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var_i: &Box<dyn Variable>, var_j: &Box<dyn Variable>) {
    let iso_i = get_isometry(&var_i.get_content());
    let trans_j = get_trans(&var_j.get_content());
    let local_j = (iso_i.inverse() * trans_j).translation;
    let pos_ij = get_pos(&factor.constraint);
    let (jacobi, jacobi_T) = calc_jacobians(&iso_i, &local_j);
    let right_mult = &factor.information_matrix.content * jacobi;

    let H_updates = jacobi_T * &right_mult;
    update_H_submatrix(H, &H_updates.index((..6, ..6)), var_i, var_i);
    update_H_submatrix(H, &H_updates.index((..6, 6..)), var_i, var_j);
    update_H_submatrix(H, &H_updates.index((6.., ..6)), var_j, var_i);
    update_H_submatrix(H, &H_updates.index((6.., 6..)), var_j, var_j);

    let err_pos = local_j.vector - pos_ij;
    let err_vec = err_pos.data.to_vec();
    let b_updates = (RowVector3::from_vec(err_vec.clone()/*TODO REMOVE*/) * &right_mult).transpose();
    update_b_subvector(b, &b_updates.index((..6, ..)), var_i);
    update_b_subvector(b, &b_updates.index((6.., ..)), var_j);

    println!("Observation Error: {:?}", err_vec);
    print!("Observation Jacobian_i:{}", jacobi.index((..,..6)));
    print!("Observation Jacobian_j:{}", jacobi.index((..,6..)));
    print!("From b:{}", b_updates.index((..6,..)));
    print!("From A:{}", H_updates.index((..6,..6)));
    print!("To b:{}", b_updates.index((6..,..)));
    print!("To A:{}", H_updates.index((6..,6..)));
}

fn calc_jacobians(iso_i: &Isometry3<f64>, local_j: &Translation3<f64>) -> (MatrixMN<f64, U3, U9>, MatrixMN<f64, U9, U3>) {
    let rot_i_inv = iso_i.inverse().rotation.to_rotation_matrix();
    let mut jacobian = MatrixMN::<f64, U3, U9>::from_vec(vec![0.0; 27]);
    jacobian.index_mut((.., 0..3)).copy_from(&-Matrix3::<f64>::identity());
    jacobian.index_mut((.., 3..6)).copy_from(&skew_trans(&local_j).transpose());
    jacobian.index_mut((.., 6..9)).copy_from(rot_i_inv.matrix());
    (jacobian, jacobian.transpose())
}

fn update_H_submatrix(H: &mut DMatrix<f64>, added_matrix: &Matrix<f64, Dynamic, Dynamic, SliceStorage<f64,Dynamic,Dynamic,U1,U9>>, var_row: &Box<dyn Variable>, var_col: &Box<dyn Variable>) {
    if var_row.is_fixed() || var_col.is_fixed() {
        return;
    }
    let row_range = var_row.get_range().unwrap();
    let col_range = var_col.get_range().unwrap();
    let updated_submatrix = &(H.index((row_range.clone(), col_range.clone())) + added_matrix);
    H.index_mut((row_range, col_range)).copy_from(updated_submatrix);
}

fn update_b_subvector(b: &mut DVector<f64>, added_vector: &Vector<f64, Dynamic, SliceStorage<f64,Dynamic,U1,U1,U9>>, var: &Box<dyn Variable>) {
    if var.is_fixed() {
        return;
    }
    let range = var.get_range().unwrap();
    let updated_subvector = &(b.index((range.clone(), ..)) + added_vector);
    b.index_mut((range, ..)).copy_from(updated_subvector);
}

fn get_trans(data: &[f64]) -> Translation3<f64> {
    Translation3::new(data[0], data[1], data[2])
}

fn get_pos(data: &[f64]) -> Vector3<f64> {
    Vector3::new(data[0], data[1], data[2])
}