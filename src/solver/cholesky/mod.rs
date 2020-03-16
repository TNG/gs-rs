#![allow(non_snake_case)]

use nalgebra::{Cholesky, DMatrix};
use crate::solver::Solver;

pub struct CholeskySolver;

impl Solver for CholeskySolver {
    fn solve(b: &[f64], H: &[f64]) -> Result<Vec<f64>, String> {
        let dim = b.len();

        if H.len() != dim * dim {
            return Err(format!("Incompatible dimensions! The length of H is not the length of b squared: H.len() = {:?}; b.len() = {:?}", H.len(), dim));
        }

        for i in 0..dim {
            for j in 0..i {
                if H[dim * i + j] != H[dim * j + i] {
                    return Err(format!("H is not symmetric: H = {:?}", H));
                }
            }
        }

        let matrix_H = DMatrix::from_vec(dim, dim, H.to_vec());
        let vector_b = DMatrix::from_vec(dim, 1, b.to_vec());

        match Cholesky::new(matrix_H) {
            None => Err(format!("H is not positive-definite: H = {:?}", H)),
            Some(cholesky) => Ok(cholesky.solve(&vector_b).data.into()),
        }
    }
}

#[cfg(test)]
mod test {
    use log::LevelFilter;

    use crate::solver::cholesky::{CholeskySolver};
    use crate::solver::Solver;

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
        let positive_definite_H = vec![ 2.0, -1.0,  0.0,  // transposed H is displayed
                                       -1.0,  2.0, -1.0,
                                        0.0, -1.0,  2.0];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = CholeskySolver::solve(&b, &positive_definite_H);
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
        let positive_definite_H = [ 2.0, -1.0,  0.0,  // transposed H is displayed
                                   -1.0,  2.0, -1.0,
                                    0.0, -1.0,  2.0];
        let b = [6.0, 6.0, 6.0];
        let solve_output = CholeskySolver::solve(&b, &positive_definite_H);
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
        let not_positive_definite_H = vec![1.0, 2.0, 4.0,  // transposed H is displayed
                                           2.0, 3.0, 5.0,
                                           4.0, 5.0, 6.0];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = CholeskySolver::solve(&b, &not_positive_definite_H);
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str)
        };
        info!("TEST FAILED! The solver returned {:?} for not positive definite H = {:?}", x, not_positive_definite_H);
    }

    #[test]
    #[should_panic]
    fn solver_not_symmetric_test() {
        init();
        #[allow(non_snake_case)]
        let not_symmetric_H = vec![ 2.0, -1.0,  2.0,  // transposed H is displayed
                                   -1.0,  2.0, -1.0,
                                    0.0, -1.0,  2.0];
        let b = vec![6.0, 6.0, 6.0];
        let solve_output = CholeskySolver::solve(&b, &not_symmetric_H);
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
        let positive_definite_H = vec![ 2.0, -1.0,  0.0,  // transposed H is displayed
                                       -1.0,  2.0, -1.0,
                                        0.0, -1.0,  2.0];
        let b = vec![6.0, 6.0, 6.0, 6.0];
        let solve_output = CholeskySolver::solve(&b, &positive_definite_H);
        let x = match solve_output {
            Ok(sol) => sol,
            Err(str) => panic!(str)
        };
        info!("TEST FAILED! The solver returned {:?} for incompatible dimensions: H = {:?}; b = {:?}", x, positive_definite_H, b);
    }
}