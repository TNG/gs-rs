use nalgebra::{DMatrix, DVector, Vector3, UnitQuaternion, Quaternion, MatrixMN, Isometry3, Translation3, U6, U12, U3, U9, Matrix6, Rotation3, Matrix3, Matrix, Dynamic, SliceStorage, U1, RowVector6, Vector};
use crate::factor_graph::factor::Factor;
use crate::factor_graph::variable::Variable;
use std::f64::consts::PI;
use std::char::DecodeUtf16;

pub fn update_H_b(H: &mut DMatrix<f64>, b: &mut DVector<f64>, factor: &Factor, var_i: &Box<dyn Variable>, var_j: &Box<dyn Variable>) {
    let iso_i = get_isometry(&var_i.get_content());
    let iso_j = get_isometry(&var_j.get_content());
    let iso_ij = get_isometry(&factor.constraint);
    let (jacobi, jacobi_T) = calc_jacobians(&iso_i, &iso_j, &iso_ij);
    let right_mult = &factor.information_matrix.content * jacobi;

    let H_updates = jacobi_T * &right_mult;
    update_H_submatrix(H, &H_updates.index((..6, ..6)), var_i, var_i);
    update_H_submatrix(H, &H_updates.index((..6, 6..)), var_i, var_j);
    update_H_submatrix(H, &H_updates.index((6.., ..6)), var_j, var_i);
    update_H_submatrix(H, &H_updates.index((6.., 6..)), var_j, var_j);

    let err = iso_ij * iso_i.inverse() * iso_j;
    let mut err_vec = err.translation.vector.data.to_vec();
    err_vec.extend_from_slice(&err.rotation.quaternion().coords.data.to_vec()[..3]);
    let b_updates = (RowVector6::from_vec(err_vec) * &right_mult).transpose();
    update_b_subvector(b, &b_updates.index((..6, ..)), var_i);
    update_b_subvector(b, &b_updates.index((6.., ..)), var_j);

    print!("From b:{}", &b_updates.index((..6, ..))); // very incorrect
    print!("From A:{}", &H_updates.index((..6, ..6))); // rather incorrect
    print!("To b:{}", &b_updates.index((6.., ..))); // very incorrect
    print!("To A:{}", &H_updates.index((6.., 6..))); // fairly correct
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

    // comparing to g2o, the skew functions used in the following snippet return a transposed result
    // in the tests, skew_trans seems to work the same as in g2o
    jacobian_i.index_mut((..3, 3..)).copy_from(&(A_rot.matrix() * skew_trans(&B_ij.translation).transpose()));
    jacobian_i.index_mut((3.., 3..)).copy_from(&(dq_dR * skew_matr_T_and_mult_parts(&B_rot.matrix(), &A_rot.matrix())));
    jacobian_j.index_mut((3.., 3..)).copy_from(&(dq_dR * skew_matr_and_mult_parts(&Matrix3::<f64>::identity(), &Err_rot.matrix())));

    print!("ret:{}", dq_dR); // ALMOST CORRECT

    print!("Odometry Jacobian_i:{}", &jacobian_i); // top left: CORRECT        | top right: CORRECT | bottom left: CORRECT | bottom right: SLIGHTLY INCORRECT
    print!("Odometry Jacobian_j:{}", &jacobian_j); // top left: ALMOST CORRECT | top right: CORRECT | bottom left: CORRECT | bottom right: ALMOST CORRECT

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

/*// TODO remove commented out calc_dq_dR if it stays unused
// formula implemented according to Chapter 10.3.2 in
// https://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf
//
// However: g2o passes the Error rotation matrix which always has 1s in the diagonal
//          (at least in the test output). This would mean that (trace - 1.0) / 2.0
//          would always equal 1 and always the special case would have to be called,
//          which does not equal the test output of g2o.
//
// TODO rename to ln_R_derivative_wrt_R() once everything is correct
fn calc_dq_dR(matr: &Matrix3<f64>) -> MatrixMN<f64, U3, U9> {
    let m = matr;
    let trace = get(m,0,0) + get(m,1,1) + get(m,2,2);
    let cos = (trace - 1.0) / 2.0;
    // TODO return special case if cos > 0.999999...
    if cos > 0.999999 {
        println!("special case with cos: {}", cos);
    }
    let sin = (1.0 - cos*cos).sqrt();
    let angle = sin.asin(); // TODO correct angle?
    let factor = (angle*cos - sin) / (4.0*sin*sin*sin);
    let a1 = (get(m,2,1) - get(m,1,2)) * factor;
    let a2 = (get(m,0,2) - get(m,2,0)) * factor;
    let a3 = (get(m,1,0) - get(m,0,1)) * factor;
    let b = angle / (2.0*sin);
    MatrixMN::<f64, U3, U9>::from_vec(vec![ a1,  a2,  a3,   // transposed matrix is displayed
                                           0.0, 0.0,   b,
                                           0.0,  -b, 0.0,
                                           0.0, 0.0,  -b,
                                            a1,  a2,  a3,
                                             b, 0.0, 0.0,
                                           0.0,   b, 0.0,
                                            -b, 0.0, 0.0,
                                            a1,  a2,  a3,])
}*/

fn skew_trans(trans: &Translation3<f64>) -> Matrix3<f64> {
    let t = 2.0 * trans.vector;
    let data = t.data.as_slice();
    Matrix3::from_vec(vec![     0.0,  data[2], -data[1],   // transposed matrix is displayed
                           -data[2],      0.0,  data[0],
                            data[1], -data[0],      0.0,])
}

fn skew_matr_and_mult_parts(matr: &Matrix3<f64>, mult: &Matrix3<f64>) -> MatrixMN<f64, U9, U3> {
    let m = matr;
    let top_part = mult * skew_trans(&Translation3::new(get(m,0,0), get(m,1,0), get(m,2,0)));
    let mid_part = mult * skew_trans(&Translation3::new(get(m,0,1), get(m,1,1), get(m,2,1)));
    let bot_part = mult * skew_trans(&Translation3::new(get(m,0,2), get(m,1,2), get(m,2,2)));
    let mut ret = MatrixMN::<f64, U9, U3>::from_vec(vec![0.0; 27]);
    ret.index_mut((0..3, ..)).copy_from(&top_part);
    ret.index_mut((3..6, ..)).copy_from(&mid_part);
    ret.index_mut((6..9, ..)).copy_from(&bot_part);
    ret
}

// TODO more elegant solution if this works
fn skew_matr_T_and_mult_parts(matr: &Matrix3<f64>, mult: &Matrix3<f64>) -> MatrixMN<f64, U9, U3> {
    let m = matr;
    let top_part = mult * skew_trans(&Translation3::new(get(m,0,0), get(m,1,0), get(m,2,0))).transpose();
    let mid_part = mult * skew_trans(&Translation3::new(get(m,0,1), get(m,1,1), get(m,2,1))).transpose();
    let bot_part = mult * skew_trans(&Translation3::new(get(m,0,2), get(m,1,2), get(m,2,2))).transpose();
    let mut ret = MatrixMN::<f64, U9, U3>::from_vec(vec![0.0; 27]);
    ret.index_mut((0..3, ..)).copy_from(&top_part);
    ret.index_mut((3..6, ..)).copy_from(&mid_part);
    ret.index_mut((6..9, ..)).copy_from(&bot_part);
    ret
}

fn get_isometry(pose: &[f64]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(pose[0], pose[1], pose[2]),
        UnitQuaternion::from_quaternion(Quaternion::new(pose[6], pose[3], pose[4], pose[5])),
    )
}

fn get(m: &Matrix3<f64>, row: usize, col: usize) -> f64 {
    m.data.as_slice()[row + col*3]
}

fn update_H_submatrix(H: &mut DMatrix<f64>, added_matrix: &Matrix<f64, Dynamic, Dynamic, SliceStorage<f64,Dynamic,Dynamic,U1,U12>>, var_row: &Box<dyn Variable>, var_col: &Box<dyn Variable>) {
    if var_row.is_fixed() || var_col.is_fixed() {
        return;
    }
    let row_range = var_row.get_range().unwrap();
    let col_range = var_col.get_range().unwrap();
    let updated_submatrix = &(H.index((row_range.clone(), col_range.clone())) + added_matrix);
    H.index_mut((row_range, col_range)).copy_from(updated_submatrix);
}

fn update_b_subvector(b: &mut DVector<f64>, added_vector: &Vector<f64, Dynamic, SliceStorage<f64,Dynamic,U1,U1,U12>>, var: &Box<dyn Variable>) {
    if var.is_fixed() {
        return;
    }
    let range = var.get_range().unwrap();
    let updated_subvector = &(b.index((range.clone(), ..)) + added_vector);
    b.index_mut((range, ..)).copy_from(updated_subvector);
}

#[cfg(test)]
mod tests {
    use super::*;

    use log::LevelFilter;
    use std::time::SystemTime;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    fn relative_eq_slice(actual: &[f64], expected: &[f64], epsilon: f64) {
        actual.iter().zip(expected.iter())
            .for_each(|(a, e)| assert!(relative_eq!(a, e, epsilon = epsilon)));
    }

    #[test]
    fn test_testing_helper_positive() {
        init();
        let actual   = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let expected = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        relative_eq_slice(&actual, &expected, 1e-10);
    }

    #[test]
    #[should_panic]
    fn test_testing_helper_negative() {
        init();
        let actual   = vec![1.01, 2.0, 3.0, 4.0, 5.0];
        let expected = vec![1.0,  2.0, 3.0, 4.0, 5.0];
        relative_eq_slice(&actual, &expected, 1e-10);
    }

    #[test]
    fn test_dq_dR() {
        init();
        // variable Re of last factor in first iteration of mini_3d.g2o
        let err_rot_matrix = Matrix3::from_vec(vec![         1.0,  -7.7835e-07, 5.74141e-08,    // transposed matrix is displayed
                                                     7.99504e-07,          1.0, 1.14998e-07,
                                                    -7.15592e-08, -1.46825e-07,         1.0,]);
        let actual = calc_dq_dR(&err_rot_matrix);
        let a1 = -8.18195e-09;
        let a2 =  4.03041e-09;
        let a3 =  4.93079e-08;
        let b =          0.25;
        let expected = MatrixMN::<f64, U3, U9>::from_vec(vec![ a1,  a2,  a3,    // transposed matrix is displayed
                                                              0.0, 0.0,   b,
                                                              0.0,  -b, 0.0,
                                                              0.0, 0.0,  -b,
                                                               a1,  a2,  a3,
                                                                b, 0.0, 0.0,
                                                              0.0,   b, 0.0,
                                                               -b, 0.0, 0.0,
                                                               a1,  a2,  a3,]);
        info!("Actual: {}", actual);
        info!("Expected: {}", expected);
        relative_eq_slice(actual.data.as_slice(), expected.data.as_slice(), 1e-7);
    }

    #[test]
    fn test_skew_trans() {
        init();
        // variable Tb of last factor in first iteration of mini_3d.g2o
        let b_trans = Translation3::new(-0.0199389, 2.43871, -0.14102);
        let actual = skew_trans(&b_trans).transpose();
        let expected = Matrix3::from_vec(vec![      0.0,   0.282041,   4.87743,    // transposed matrix is displayed
                                              -0.282041,        0.0, 0.0398779,
                                               -4.87743, -0.0398779,       0.0,]);
        info!("Actual: {}", actual);
        info!("Expected: {}", expected);
        relative_eq_slice(actual.data.as_slice(), expected.data.as_slice(), 1e-5 + 1e-10);
    }

    #[test]
    fn test_skew_matr_and_mult_parts() {
        init();
        // TODO remove _f64.round() which was used to have a cleaner output for debugging
        let actual = skew_matr_and_mult_parts(
            // variable Rb of last factor in first iteration of mini_3d.g2o
            &Matrix3::<f64>::from_vec(vec![  0.999284, -0.0244698, 0.0288424,    // transposed matrix is displayed
                                            0.0150356,   0.956688,  0.290726,
                                           -0.0347072,  -0.290084,  0.956372,]),
            // variable Ra of last factor in first iteration of mini_3d.g2o
            &Matrix3::<f64>::from_vec(vec![  0.999284, 0.0150348, -0.0347071,    // transposed matrix is displayed
                                           -0.0244691,  0.956688,  -0.290084,
                                            0.0288425,  0.290726,   0.956372,]),
        );
        let expected = MatrixMN::<f64, U9, U3>::from_vec(vec![ 5.23021e-08, 0.0694143, 0.0300711, -0.0694142,  2.85328e-07,   -1.99857, -0.0300695,    1.99857,  2.91287e-07,    // transposed matrix is displayed
                                                               3.41719e-07,  0.580168,   1.91338,  -0.580168,  4.58219e-07,  0.0489397,   -1.91338, -0.0489382, -1.44105e-07,
                                                              -1.52217e-06,  -1.91274,  0.581451,    1.91274, -1.52261e-06, -0.0576846,  -0.581452,  0.0576852, -3.31386e-08,]);
        info!("Actual: {}", actual);
        info!("Expected: {}", expected);
        relative_eq_slice(actual.data.as_slice(), expected.data.as_slice(), 1e-5);
    }

}
