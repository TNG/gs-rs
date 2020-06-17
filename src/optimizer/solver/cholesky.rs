//! Solver for linear systems using the Cholesky decomposition.

#![allow(non_snake_case)]

use crate::optimizer::solver::Solver;
use nalgebra::{Cholesky, DMatrix, DVector};

/// Implements the solver using the Cholesky decomposition.
pub struct CholeskySolver;

impl Solver for CholeskySolver {
    fn solve(H: DMatrix<f64>, b: &DVector<f64>) -> Result<Vec<f64>, String> {
        match Cholesky::new(H) {
            None => Err(String::from("H is not positive-definite")),
            Some(cholesky) => Ok(cholesky.solve(&b).data.into()),
        }
    }
}

#[cfg(test)]
mod test {
    use log::LevelFilter;

    use crate::optimizer::solver::cholesky::CholeskySolver;
    use crate::optimizer::solver::Solver;
    use nalgebra::{DMatrix, DVector};

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn solver_positive_definite_test() {
        init();
        #[allow(non_snake_case)]
        let positive_definite_H = vec![
            2.0, -1.0, 0.0, // transposed H is displayed
            -1.0, 2.0, -1.0, 0.0, -1.0, 2.0,
        ];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = CholeskySolver::solve(
            DMatrix::<f64>::from_vec(3, 3, positive_definite_H),
            &DVector::from_vec(b),
        );
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str),
        };
        assert!(relative_eq!(x[0], 9.0, epsilon = 1e-10));
        assert!(relative_eq!(x[1], 12.0, epsilon = 1e-10));
        assert!(relative_eq!(x[2], 9.0, epsilon = 1e-10));
    }

    #[test]
    fn solver_positive_definite_array_test() {
        init();
        #[allow(non_snake_case)]
        let positive_definite_H = vec![
            2.0, -1.0, 0.0, // transposed H is displayed
            -1.0, 2.0, -1.0, 0.0, -1.0, 2.0,
        ];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = CholeskySolver::solve(
            DMatrix::<f64>::from_vec(3, 3, positive_definite_H),
            &DVector::from_vec(b),
        );
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str),
        };
        assert!(relative_eq!(x[0], 9.0, epsilon = 1e-10));
        assert!(relative_eq!(x[1], 12.0, epsilon = 1e-10));
        assert!(relative_eq!(x[2], 9.0, epsilon = 1e-10));
    }

    #[test]
    #[should_panic]
    fn solver_not_positive_definite_test() {
        init();
        #[allow(non_snake_case)]
        let not_positive_definite_H = vec![
            1.0, 2.0, 4.0, // transposed H is displayed
            2.0, 3.0, 5.0, 4.0, 5.0, 6.0,
        ];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = CholeskySolver::solve(
            DMatrix::<f64>::from_vec(3, 3, not_positive_definite_H.clone()),
            &DVector::from_vec(b),
        );
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str),
        };
        info!(
            "TEST FAILED! The solver returned {:?} for not positive-definite H = {:?}",
            x, not_positive_definite_H
        );
    }

    #[test]
    #[ignore]
    #[should_panic]
    fn solver_not_symmetric_test() {
        init();
        #[allow(non_snake_case)]
        let not_symmetric_H = vec![
            2.0, -1.0, 2.0, // transposed H is displayed
            -1.0, 2.0, -1.0, 0.0, -1.0, 2.0,
        ];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = CholeskySolver::solve(
            DMatrix::<f64>::from_vec(3, 3, not_symmetric_H.clone()),
            &DVector::from_vec(b),
        );
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str),
        };
        info!(
            "TEST FAILED! The solver returned {:?} for not symmetric H = {:?}",
            x, not_symmetric_H
        );
    }

    #[test]
    #[should_panic]
    fn solver_incompatible_dimension_test() {
        init();
        #[allow(non_snake_case)]
        let positive_definite_H = vec![
            2.0, -1.0, 0.0, // transposed H is displayed
            -1.0, 2.0, -1.0, 0.0, -1.0, 2.0,
        ];
        let b = vec![6.0, 6.0, 6.0, 6.0];
        let solve_output = CholeskySolver::solve(
            DMatrix::<f64>::from_vec(3, 3, positive_definite_H.clone()),
            &DVector::from_vec(b.clone()),
        );
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str),
        };
        info!(
            "TEST FAILED! The solver returned {:?} for incompatible dimensions: H = {:?}; b = {:?}",
            x, positive_definite_H, b
        );
    }
}
