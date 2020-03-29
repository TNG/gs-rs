#![allow(non_snake_case)]

use nalgebra::{CsCholesky, CsMatrix, DMatrix, DVector};

use crate::solver::Solver;

pub struct SparseCholeskySolver;

impl Solver for SparseCholeskySolver {
    fn solve(b: &DVector<f64>, H: DMatrix<f64>) -> Result<Vec<f64>, String> {
        // TODO pass matrix as lower triangular or sparse already
        /* if H.transpose() != H {
             return Err(String::from("H is not symmetric"));
         }
 */
        let sparse = CsCholesky::new(&CsMatrix::from(H));
        match sparse.l() {
            None => Err(String::from("H is not positive-definite")),
            Some(l) => Ok(l.tr_solve_lower_triangular(&l.solve_lower_triangular(&b).unwrap()).unwrap().data.into()),
        }
    }
}

#[cfg(test)]
mod test {
    use log::LevelFilter;
    use nalgebra::{DMatrix, DVector};

    use crate::solver::Solver;
    use crate::solver::sparse_cholesky::SparseCholeskySolver;

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
            let positive_definite_H = vec![2.0, -1.0, 0.0,
                                           -1.0, 2.0, -1.0,
                                           0.0, -1.0, 2.0];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = SparseCholeskySolver::solve(&DVector::from_vec(b.clone()), DMatrix::<f64>::from_vec(3, 3, positive_definite_H.clone()));
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str)
        };
        info!("H = {:?}; b = {:?}  |  x = {:?}", positive_definite_H, b, x);
        assert!(relative_eq!(x[0],  9.0, epsilon = 1e-10));
        assert!(relative_eq!(x[1], 12.0, epsilon = 1e-10));
        assert!(relative_eq!(x[2],  9.0, epsilon = 1e-10));
    }

    #[test]
    fn solver_positive_definite_array_test() {
        init();
        #[allow(non_snake_case)]
            let positive_definite_H = vec![2.0, -1.0, 0.0,
                                           -1.0, 2.0, -1.0,
                                           0.0, -1.0, 2.0];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = SparseCholeskySolver::solve(&DVector::from_vec(b.clone()), DMatrix::<f64>::from_vec(3, 3, positive_definite_H.clone()));
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str)
        };
        info!("H = {:?}; b = {:?}  |  x = {:?}", positive_definite_H, b, x);
        assert!(relative_eq!(x[0],  9.0, epsilon = 1e-10));
        assert!(relative_eq!(x[1], 12.0, epsilon = 1e-10));
        assert!(relative_eq!(x[2],  9.0, epsilon = 1e-10));
    }

    #[test]
    #[should_panic]
    fn solver_not_positive_definite_test() {
        init();
        #[allow(non_snake_case)]
            let not_positive_definite_H = vec![1.0, 2.0, 4.0,
                                               2.0, 3.0, 5.0,
                                               4.0, 5.0, 6.0];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = SparseCholeskySolver::solve(&DVector::from_vec(b), DMatrix::<f64>::from_vec(3, 3, not_positive_definite_H.clone()));
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str)
        };
        info!("TEST FAILED! The solver returned {:?} for not positive-definite H = {:?}", x, not_positive_definite_H);
    }

    #[test]
    #[ignore]
    #[should_panic]
    fn solver_not_symmetric_test() {
        init();
        #[allow(non_snake_case)]
            let not_symmetric_H = vec![2.0, -1.0, 2.0,
                                       -1.0, 2.0, -1.0,
                                       0.0, -1.0, 2.0];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = SparseCholeskySolver::solve(&DVector::from_vec(b), DMatrix::<f64>::from_vec(3, 3, not_symmetric_H.clone()));
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str)
        };
        info!("TEST FAILED! The solver returned {:?} for not symmetric H = {:?}", x, not_symmetric_H);
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
        let solve_output = SparseCholeskySolver::solve(&DVector::from_vec(b.clone()), DMatrix::<f64>::from_vec(3, 3, positive_definite_H.clone()));
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str)
        };
        info!("TEST FAILED! The solver returned {:?} for incompatible dimensions: H = {:?}; b = {:?}", x, positive_definite_H, b);
    }
}