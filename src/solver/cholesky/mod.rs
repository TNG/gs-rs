#[cfg(test)]
mod test {

    use log::LevelFilter;
    use nalgebra::{Matrix3, Matrix3x1};

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn cholesky_solver_smoke_test() {
        init();
        let A = Matrix3::new(2., -1., 0.,
                             -1., 2., -1.,
                             0., -1., 2.);

        let b = Matrix3x1::new(6.0, 6.0, 6.0);

        let update = A.cholesky().unwrap().solve(&b);

        relative_eq!(update[0], 9.0, epsilon =  1e-10);
        relative_eq!(update[1], 12.0, epsilon =  1e-10);
        relative_eq!(update[2], 9.0, epsilon = 1e-10);
    }
}