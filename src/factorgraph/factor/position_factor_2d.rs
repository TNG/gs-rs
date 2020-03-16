use uuid::Uuid;
use crate::factorgraph::factor::{Factor, FactorType};

#[derive(Debug)]
pub struct PositionFactor2D {
    id: Uuid,
    /// [x, y, phi], phi defaulting to 0
    content: [f64; 3],
    /// defaulting to identity matrix
    information_matrix: [f64; 6]
}

impl PositionFactor2D {
    pub fn from_xy(x: f64, y: f64) -> Self {
        PositionFactor2D {
            id: Uuid::new_v4(),
            content: [x, y, 0.0],
            information_matrix: [1.0,
                                 0.0, 1.0,
                                 0.0, 0.0, 1.0],
        }
    }

    pub fn from_xy_and_information_matrix(x: f64, y: f64, information_matrix: [f64; 6]) -> Self {
        PositionFactor2D {
            id: Uuid::new_v4(),
            content: [x, y, 0.0],
            information_matrix,
        }
    }
}

impl Factor<'_> for PositionFactor2D {
    fn get_id(&self) -> Uuid {
        self.id
    }

    fn get_type(&self) -> FactorType {
        FactorType::Position2D
    }

    fn get_content(&self) -> &[f64] {
        &self.content
    }

    fn get_information_matrix(&self) -> &[f64] {
        &self.information_matrix
    }
}

#[cfg(test)]
mod tests {
    use log::LevelFilter;
    use super::*;

    fn init() {
        let _ = env_logger::builder()
            .is_test(true)
            .filter_level(LevelFilter::Debug)
            .try_init();
    }

    #[test]
    fn test_from_xy() {
        init();

        let test_factor = PositionFactor2D::from_xy(3.0, 5.0);
        info!("{:?}", &test_factor);
        assert_eq!(test_factor.get_content(), &[3.0, 5.0, 0.0]);
        assert_eq!(test_factor.get_type(), FactorType::Position2D);
        assert_eq!(test_factor.get_information_matrix(), &[1.0, 0.0, 1.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn test_from_xy_and_information_matrix() {
        init();

        let test_factor = PositionFactor2D::from_xy_and_information_matrix(3.0, 5.0, [1.0, 0.0, 1.0, 0.0, 0.0, 0.1]);
        info!("{:?}", &test_factor);
        assert_eq!(test_factor.get_content(), &[3.0, 5.0, 0.0]);
        assert_eq!(test_factor.get_type(), FactorType::Position2D);
        assert_eq!(test_factor.get_information_matrix(), &[1.0, 0.0, 1.0, 0.0, 0.0, 0.1]);
    }

}