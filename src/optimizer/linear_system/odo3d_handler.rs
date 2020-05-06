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
    // iso_i, iso_j, iso_ij (i.e. all parameters) CORRECT
    let A_ij = iso_ij.inverse(); // CORRECT
    let B_ij = iso_i.inverse() * iso_j; // CORRECT

    // TODO check if solution is okay with this error calculation
    // INCORRECT... or is Rust just more precise?
    // first error translation correct, after that starts getting slightly worse; rotation incorrect from beginning on
    // g2o seems to be using this operator implementation: https://eigen.tuxfamily.org/dox/classEigen_1_1Transform.html#title46
    // Code should be found starting in line 1515: https://github.com/eigenteam/eigen-git-mirror/blob/master/Eigen/src/Geometry/Transform.h
    let Err_ij = A_ij * B_ij;

    let A_rot = A_ij.rotation.to_rotation_matrix();
    let B_rot = B_ij.rotation.to_rotation_matrix();
    let Err_rot = Err_ij.rotation.to_rotation_matrix();

    let dq_dR = calc_dq_dR(&Err_rot.matrix());

    let mut jacobian_i = Matrix6::from_vec(vec![0.0; 36]);
    let mut jacobian_j = Matrix6::from_vec(vec![0.0; 36]);
    jacobian_i.index_mut((..3, ..3)).copy_from(&(-1.0 * A_rot.matrix()));
    jacobian_j.index_mut((..3, ..3)).copy_from(Err_rot.matrix());
    jacobian_i.index_mut((..3, 3..)).copy_from(&(A_rot.matrix() * skew_trans(&B_ij.translation).transpose())); // TODO negate skew_trans? (g2o output is the exact negative) -> remove .transpose() (= *1.0 as skew-symmetric and diagonal = [0])
    jacobian_i.index_mut((3.., 3..)).copy_from(&(dq_dR * skew_matr_T_and_mult_parts(&B_rot.matrix(), &A_rot.matrix())));
    print!("ret:{}", skew_matr_and_mult_parts(&Matrix3::<f64>::identity(), &Err_rot.matrix()));
    jacobian_j.index_mut((3.., 3..)).copy_from(&(dq_dR * skew_matr_and_mult_parts(&Matrix3::<f64>::identity(), &Err_rot.matrix())));

    print!("Odometry Jacobian_i:{}", &jacobian_i);
    print!("Odometry Jacobian_j:{}", &jacobian_j);

    let mut jacobian = MatrixMN::<f64, U6, U12>::from_vec(vec![0.0; 72]);
    jacobian.index_mut((.., ..6)).copy_from(&jacobian_i);
    jacobian.index_mut((.., 6..)).copy_from(&jacobian_j);
    (jacobian, jacobian.transpose())
}

// code copied from g2o for the case [ sin >= 0 && trace > 0 ]
fn calc_dq_dR(matr: &Matrix3<f64>) -> MatrixMN<f64, U3, U9> {
    let m = matr;
    let trace = get(m,0,0) + get(m,1,1) + get(m,2,2);
    let sin = (trace + 1.0).sqrt() * 2.0;
    let factor = -0.03125 / (sin*sin*sin);
    let a1 = (get(m,2,1) - get(m,1,2)) * factor;
    let a2 = (get(m,0,2) - get(m,2,0)) * factor;
    let a3 = (get(m,1,0) - get(m,0,1)) * factor;
    let b = 1.0/sin;
    MatrixMN::<f64, U3, U9>::from_vec(vec![ a1,  a2,  a3,   // transposed matrix is displayed
                                           0.0, 0.0,   b,
                                           0.0,  -b, 0.0,
                                           0.0, 0.0,  -b,
                                            a1,  a2,  a3,
                                             b, 0.0, 0.0,
                                           0.0,   b, 0.0,
                                            -b, 0.0, 0.0,
                                            a1,  a2,  a3,])
}

// TODO remove commented out calc_dq_dR if it stays unused
// // formula implemented according to Chapter 10.3.2 in
// // https://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf
// //
// // However: g2o passes the Error rotation matrix which always has 1s in the diagonal
// //          (at least in the test output). This would mean that (trace - 1.0) / 2.0
// //          would always equal 1 and always the special case would have to be called,
// //          which does not equal the test output of g2o.
// //
// // TODO rename to ln_R_derivative_wrt_R() once everything is correct
// fn calc_dq_dR(matr: &Matrix3<f64>) -> MatrixMN<f64, U3, U9> {
//     let m = matr;
//     let trace = get(m,0,0) + get(m,1,1) + get(m,2,2);
//     let cos = (trace - 1.0) / 2.0;
//     // TODO return special case if cos > 0.999999...
//     if cos > 0.999999 {
//         println!("special case with cos: {}", cos);
//     }
//     let sin = (1.0 - cos*cos).sqrt();
//     let angle = sin.asin(); // TODO correct angle?
//     let factor = (angle*cos - sin) / (4.0*sin*sin*sin);
//     let a1 = (get(m,2,1) - get(m,1,2)) * factor;
//     let a2 = (get(m,0,2) - get(m,2,0)) * factor;
//     let a3 = (get(m,1,0) - get(m,0,1)) * factor;
//     let b = angle / (2.0*sin);
//     MatrixMN::<f64, U3, U9>::from_vec(vec![ a1,  a2,  a3,   // transposed matrix is displayed
//                                            0.0, 0.0,   b,
//                                            0.0,  -b, 0.0,
//                                            0.0, 0.0,  -b,
//                                             a1,  a2,  a3,
//                                              b, 0.0, 0.0,
//                                            0.0,   b, 0.0,
//                                             -b, 0.0, 0.0,
//                                             a1,  a2,  a3,])
// }

fn skew_trans(trans: &Translation3<f64>) -> Matrix3<f64> {
    let t = 2.0 * trans.vector;
    let data = t.data.as_slice();
    Matrix3::from_vec(vec![     0.0,  data[2], -data[1],   // transposed matrix is displayed
                           -data[2],      0.0,  data[0],
                            data[1], -data[0],      0.0,])
}

fn skew_matr_and_mult_parts(matr: &Matrix3<f64>, mult: &Matrix3<f64>) -> MatrixMN<f64, U9, U3> {
    let m = &(2.0 * matr);
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

// TODO more elegant solution if this works
fn skew_matr_T_and_mult_parts(matr: &Matrix3<f64>, mult: &Matrix3<f64>) -> MatrixMN<f64, U9, U3> {
    let m = &(2.0 * matr);
    let matr_x = mult * Matrix3::from_vec(vec![        0.0,         0.0,         0.0,    // transposed matrix is displayed
                                               -get(m,2,0), -get(m,2,1), -get(m,2,2),
                                                get(m,1,0),  get(m,1,1),  get(m,1,2),]).transpose();
    let matr_y = mult * Matrix3::from_vec(vec![ get(m,2,0),  get(m,2,1),  get(m,2,2),    // transposed matrix is displayed
                                                       0.0,         0.0,         0.0,
                                               -get(m,0,0), -get(m,0,1), -get(m,0,2),]).transpose();
    let matr_z = mult * Matrix3::from_vec(vec![-get(m,1,0), -get(m,1,1), -get(m,1,2),    // transposed matrix is displayed
                                                get(m,0,0),  get(m,0,1),  get(m,0,2),
                                                       0.0,         0.0,         0.0,]).transpose();
    let mut ret = MatrixMN::<f64, U9, U3>::from_vec(vec![0.0; 27]);
    ret.index_mut((0..3, ..)).copy_from(&matr_x);
    ret.index_mut((3..6, ..)).copy_from(&matr_y);
    ret.index_mut((6..9, ..)).copy_from(&matr_z);
    ret
}

fn get_isometry(pose: &[f64]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(pose[0], pose[1], pose[2]),
        UnitQuaternion::from_quaternion(Quaternion::new(pose[6], pose[3], pose[4], pose[5])),
    )
}

fn get(m: &Matrix3<f64>, row: usize, col: usize) -> f64 {
    m.data.as_slice()[row*3 + col]
}