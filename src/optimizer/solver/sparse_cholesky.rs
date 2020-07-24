//! Solver for linear systems using the Cholesky decomposition on a sparse matrix.

#![allow(non_snake_case)]

use crate::optimizer::solver::Solver;
use nalgebra::{CsCholesky, CsMatrix, DMatrix, DVector};

/// Implements the solver using the Cholesky decomposition on a sparse matrix.
pub struct SparseCholeskySolver;

impl Solver for SparseCholeskySolver {
    /// Assumes that H is symmetric. Might return wrong result if this is not the case.
    fn solve(H: DMatrix<f64>, b: &DVector<f64>) -> Result<Vec<f64>, String> {
        // TODO @Daniel (TODO created by you): pass matrix as sparse already
        let sparse = CsCholesky::new(&CsMatrix::from(H));
        match sparse.l() {
            None => Err(String::from("H is not positive-definite")),
            Some(l) => Ok(
                l.tr_solve_lower_triangular(&l.solve_lower_triangular(&b).unwrap())
                .unwrap().data.into()
            ),
        }
    }
}

#[cfg(test)]
mod test {
    use log::LevelFilter;
    use log::info;
    use approx::relative_eq;
    use nalgebra::{DMatrix, DVector};


    use crate::optimizer::solver::sparse_cholesky::SparseCholeskySolver;
    use crate::optimizer::solver::Solver;

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
        let positive_definite_H = vec![ 2.0, -1.0,  0.0,   // transposed H is displayed
                                       -1.0,  2.0, -1.0,
                                        0.0, -1.0,  2.0,];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = SparseCholeskySolver::solve(
            DMatrix::<f64>::from_vec(3, 3, positive_definite_H.clone()),
            &DVector::from_vec(b.clone()),
        );
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str),
        };
        info!("H = {:?}; b = {:?}  |  x = {:?}", positive_definite_H, b, x);
        assert!(relative_eq!(x[0], 9.0, epsilon = 1e-10));
        assert!(relative_eq!(x[1], 12.0, epsilon = 1e-10));
        assert!(relative_eq!(x[2], 9.0, epsilon = 1e-10));
    }

    #[test]
    #[should_panic]
    fn solver_not_positive_definite_test() {
        init();
        #[allow(non_snake_case)]
        let not_positive_definite_H = vec![1.0, 2.0, 4.0,   // transposed H is displayed
                                           2.0, 3.0, 5.0,
                                           4.0, 5.0, 6.0,];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = SparseCholeskySolver::solve(
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
    #[ignore] // currently no check if symmetric
    #[should_panic]
    fn solver_not_symmetric_test() {
        init();
        #[allow(non_snake_case)]
        let not_symmetric_H = vec![ 2.0, -1.0,  2.0,   // transposed H is displayed
                                   -1.0,  2.0, -1.0,
                                    0.0, -1.0,  2.0,];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = SparseCholeskySolver::solve(
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
        let positive_definite_H = vec![ 2.0, -1.0,  0.0,   // transposed H is displayed
                                       -1.0,  2.0, -1.0,
                                        0.0, -1.0,  2.0,];
        let b = vec![6.0, 6.0, 6.0, 6.0];
        let solve_output = SparseCholeskySolver::solve(
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
