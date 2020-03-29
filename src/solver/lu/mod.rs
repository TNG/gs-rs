#![allow(non_snake_case)]

use nalgebra::{DMatrix, DVector, LU};

use crate::solver::Solver;

pub struct LUSolver;

impl Solver for LUSolver {
    fn solve(b: &DVector<f64>, H: DMatrix<f64>) -> Result<Vec<f64>, String> {
        let lu_decomposition = LU::new(H);

        match lu_decomposition.solve(&b) {
            None => Err(String::from("H is not invertible.")),
            Some(lu) => Ok(lu.data.into()),
        }
    }
}

#[cfg(test)]
mod test {
    use log::LevelFilter;
    use nalgebra::{DMatrix, DVector};

    use crate::solver::lu::LUSolver;
    use crate::solver::Solver;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn solver_invertible_test() {
        init();
        #[allow(non_snake_case)]
            let invertible_H = vec![2.0, -1.0, 2.0,
                                    -1.0, 2.0, -1.0,
                                    0.0, -1.0, 2.0];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = LUSolver::solve(&DVector::from_vec(b), DMatrix::<f64>::from_vec(3, 3, invertible_H));
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str)
        };
        assert!(relative_eq!(x[0], 6.0, epsilon = 1e-10));
        assert!(relative_eq!(x[1], 6.0, epsilon = 1e-10));
        assert!(relative_eq!(x[2], 0.0, epsilon = 1e-10));
    }

    #[test]
    #[should_panic]
    fn solver_not_invertible_test() {
        init();
        #[allow(non_snake_case)]
            let not_invertible_H = vec![0.0, 2.0, -1.0,
                                        3.0, -2.0, 1.0,
                                        3.0, 2.0, -1.0];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = LUSolver::solve(&DVector::from_vec(b), DMatrix::<f64>::from_vec(3, 3, not_invertible_H.clone()));
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str)
        };
        info!("TEST FAILED! The solver returned {:?} for not invertible H = {:?}", x, not_invertible_H);
    }

    #[test]
    #[should_panic]
    fn solver_incompatible_dimension_test() {
        init();
        #[allow(non_snake_case)]
            let positive_definite_H = vec![2.0, -1.0, 0.0,
                                           -1.0, 2.0, -1.0,
                                           0.0, -1.0, 2.0];
        let b = vec![6.0, 6.0, 6.0, 6.0];

        let solve_output = LUSolver::solve(&DVector::from_vec(b.clone()), DMatrix::<f64>::from_vec(3, 3, positive_definite_H.clone()));
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str)
        };
        info!("TEST FAILED! The solver returned {:?} for incompatible dimensions: H = {:?}; b = {:?}", x, positive_definite_H, b);
    }
}