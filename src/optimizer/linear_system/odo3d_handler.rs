use nalgebra::{DMatrix, DVector, Vector3, UnitQuaternion, Quaternion, MatrixMN, Isometry3, Translation3, U6, U12};
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

    // Change from MatrixMN to DMatrix?
    // let jacobian: MatrixMN<f64, U6, U12> = DMatrix::from_vec(6, 12, vec![0.0; 72]).into();
    // (jacobian, jacobian.transpose())
    unimplemented!()
}

// TODO remove if unused
fn get_pos_and_rot(pose: &[f64]) -> (Vector3<f64>, UnitQuaternion<f64>) {
    (
        Vector3::new(pose[0], pose[1], pose[2]),
        UnitQuaternion::from_quaternion(Quaternion::new(pose[6], pose[3], pose[4], pose[5])),
    )
}

fn get_isometry(pose: &[f64]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(pose[0], pose[1], pose[2]),
        UnitQuaternion::from_quaternion(Quaternion::new(pose[6], pose[3], pose[4], pose[5])),
    )
}