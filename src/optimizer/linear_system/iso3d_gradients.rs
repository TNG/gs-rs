use nalgebra::{Matrix3, MatrixMN, Translation3, Isometry3, UnitQuaternion, Quaternion, U3, U9};

// code copied from g2o for the case [ sin >= 0 && trace > 0 ]
pub fn calc_dq_dR(matr: &Matrix3<f64>) -> MatrixMN<f64, U3, U9> {
    let m = matr;
    let trace = get(m,0,0) + get(m,1,1) + get(m,2,2);
    let sin = (trace + 1.0).sqrt() * 0.5;
    let factor = -0.03125 / (sin*sin*sin);
    let a1 = (get(m,2,1) - get(m,1,2)) * factor;
    let a2 = (get(m,0,2) - get(m,2,0)) * factor;
    let a3 = (get(m,1,0) - get(m,0,1)) * factor;
    let b = 0.25/sin;
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

pub fn skew_trans(trans: &Translation3<f64>) -> Matrix3<f64> {
    let t = 2.0 * trans.vector;
    let data = t.data.as_slice();
    // to match g2o output, skew seems to need to return skew_T
    Matrix3::from_vec(vec![     0.0, -data[2],  data[1],   // transposed matrix is displayed
                            data[2],      0.0, -data[0],
                           -data[1],  data[0],      0.0,])
}

pub fn skew_matr_and_mult_parts(matr: &Matrix3<f64>, mult: &Matrix3<f64>) -> MatrixMN<f64, U9, U3> {
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

pub fn skew_matr_T_and_mult_parts(matr: &Matrix3<f64>, mult: &Matrix3<f64>) -> MatrixMN<f64, U9, U3> {
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

pub fn get_isometry(pose: &[f64]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(pose[0], pose[1], pose[2]),
        UnitQuaternion::from_quaternion(Quaternion::new(pose[6], pose[3], pose[4], pose[5])),
    )
}

pub fn get_isometry_normalized(pose: &[f64]) -> Isometry3<f64> {
    let quaternion = Quaternion::new(1.0, pose[3], pose[4], pose[5]);
    let unit_quaternion = UnitQuaternion::new_normalize(quaternion);
    Isometry3::from_parts(
        Translation3::new(pose[0], pose[1], pose[2]),
        unit_quaternion,
    )
}

fn get(m: &Matrix3<f64>, row: usize, col: usize) -> f64 {
    m.data.as_slice()[row + col*3]
}

#[cfg(test)]
mod tests {
    use super::*;

    use log::LevelFilter;
    use log::info;
    use approx::relative_eq;

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
    fn test_testing_helper_pass() {
        init();
        let actual   = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let expected = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        relative_eq_slice(&actual, &expected, 1e-10);
    }

    #[test]
    #[should_panic]
    fn test_testing_helper_fail() {
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
        relative_eq_slice(actual.data.as_slice(), expected.data.as_slice(), 1e-10);
    }

    #[test]
    fn test_skew_trans() {
        init();
        // variable Tb of last factor in first iteration of mini_3d.g2o
        let b_trans = Translation3::new(-0.0199389, 2.43871, -0.14102);
        let actual = skew_trans(&b_trans).transpose();
        let expected = Matrix3::from_vec(vec![      0.0,  -0.282041,   -4.87743,    // transposed matrix is displayed
                                               0.282041,        0.0, -0.0398779,
                                                4.87743,  0.0398779,        0.0,]);
        info!("Actual: {}", actual);
        info!("Expected: {}", expected);
        relative_eq_slice(actual.data.as_slice(), expected.data.as_slice(), 1e-5 + 1e-10);
    }

    #[test]
    fn test_skew_matr_T_and_mult_parts() {
        init();
        let actual = skew_matr_T_and_mult_parts(
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
