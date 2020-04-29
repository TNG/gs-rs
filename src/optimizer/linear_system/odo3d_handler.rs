use nalgebra::{DMatrix, DVector, Vector3, UnitQuaternion, Quaternion, MatrixMN, Isometry3, Translation3, U6, U12, U3, U9, Matrix6, Rotation3, Matrix3};
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;
use std::f64::consts::PI;
use std::char::DecodeUtf16;

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var_i: &Box<dyn Variable>, var_j: &Box<dyn Variable>) {
    let iso_i = get_isometry(&var_i.get_content());
    let iso_j = get_isometry(&var_j.get_content());
    let iso_ij = get_isometry(&factor.constraint);
    let (jacobi, jacobi_T) = calc_jacobians(&iso_i, &iso_j, &iso_ij);


    // TODO take a look at g2o and implement
}

fn calc_jacobians(iso_i: &Isometry3<f64>, iso_j: &Isometry3<f64>, iso_ij: &Isometry3<f64>) -> (MatrixMN<f64, U6, U12>, MatrixMN<f64, U12, U6>) {
    let A_ij = iso_ij.inverse();
    let B_ij = iso_i.inverse() * iso_j;
    let Err_ij = A_ij * B_ij;
    let A_rot = A_ij.rotation.to_rotation_matrix();
    let B_rot = B_ij.rotation.to_rotation_matrix();
    let Err_rot = Err_ij.rotation.to_rotation_matrix();

    let dq_dR = MatrixMN::<f64, U3, U9>::from_vec(vec![0.0; 27]); // TODO calculate dq_dR

    let mut jacobian_i = Matrix6::from_vec(vec![0.0; 36]);
    let mut jacobian_j = Matrix6::from_vec(vec![0.0; 36]);
    jacobian_i.index_mut((..3, ..3)).copy_from(&(-1.0 * A_rot.matrix()));
    jacobian_j.index_mut((..3, ..3)).copy_from(Err_rot.matrix());
    jacobian_j.index_mut((..3, 3..)).copy_from(&(A_rot.matrix() * skew_trans(&B_ij.translation).transpose()));
    jacobian_i.index_mut((3.., 3..)).copy_from(&(dq_dR * skew_matr_and_mult_parts(&B_rot.matrix(), &A_rot.matrix())));
    jacobian_j.index_mut((3.., 3..)).copy_from(&(dq_dR * skew_matr_and_mult_parts(&Matrix3::<f64>::identity(), &Err_rot.matrix())));

    let mut jacobian = MatrixMN::<f64, U6, U12>::from_vec(vec![0.0; 72]);
    jacobian.index_mut((.., ..6)).copy_from(&jacobian_i);
    jacobian.index_mut((.., 6..)).copy_from(&jacobian_j);
    (jacobian, jacobian.transpose())
}

fn skew_trans(trans: &Translation3<f64>) -> Matrix3<f64> {
    let data = trans.vector.data.as_slice();
    Matrix3::from_vec(vec![     0.0,  data[2], -data[1],   // transposed matrix is displayed
                           -data[2],      0.0,  data[0],
                            data[1], -data[0],      0.0,])
}

fn skew_matr_and_mult_parts(matr: &Matrix3<f64>, mult: &Matrix3<f64>) -> MatrixMN<f64, U9, U3> {
    let m = matr;
    let matr_x = mult * Matrix3::from_vec(vec![        0.0,         0.0,         0.0,    // transposed matrix is displayed
                                               -get(m,2,0), -get(m,2,1), -get(m,2,2),
                                                get(m,1,0),  get(m,1,1),  get(m,1,2),]);
    let matr_y = mult * Matrix3::from_vec(vec![ get(m,2,0),  get(m,2,1),  get(m,2,2),    // transposed matrix is displayed
                                                       0.0,         0.0,         0.0,
                                               -get(m,0,0), -get(m,0,1), -get(m,0,2),]);
    let matr_z = mult * Matrix3::from_vec(vec![-get(m,1,0), -get(m,1,1), -get(m,1,2),    // transposed matrix is displayed
                                                get(m,0,0),  get(m,0,1),  get(m,0,2),
                                                       0.0,         0.0,         0.0,]);
    let mut ret = MatrixMN::<f64, U9, U3>::from_vec(vec![0.0; 27]);
    ret.index_mut((0..3, ..)).copy_from(&matr_x);
    ret.index_mut((3..6, ..)).copy_from(&matr_y);
    ret.index_mut((6..9, ..)).copy_from(&matr_z);
    ret
}

fn get(m: &Matrix3<f64>, row: usize, col: usize) -> f64 {
    m.data.as_slice()[col*3 + row]
}

fn get_isometry(pose: &[f64]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(pose[0], pose[1], pose[2]),
        UnitQuaternion::from_quaternion(Quaternion::new(pose[6], pose[3], pose[4], pose[5])),
    )
}