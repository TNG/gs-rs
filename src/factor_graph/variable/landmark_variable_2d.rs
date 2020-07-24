use super::*;

#[cfg(test)]
mod tests {
    use super::*;
    use log::info;
    use log::LevelFilter;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn test_new_fixed() {
        init();

        let test_variable = LandmarkVariable2D::new(1, 3.0, 5.0, true, None);
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.0.position, &[3.0, 5.0]);
        assert_eq!(test_variable.0.kind, FixedType::Fixed);
        assert_eq!(test_variable.0.id, 1);
    }

    #[test]
    fn test_new_dynamic() {
        init();

        let test_variable = LandmarkVariable2D::new(1, 3.0, 5.0, false, Some(0..2));
        info!("{:?}", &test_variable);
        assert_eq!(test_variable.0.position, &[3.0, 5.0]);
        assert_eq!(test_variable.0.id, 1);
        assert_eq!(test_variable.0.kind, FixedType::NonFixed(Some(0..2)));
    }
}
