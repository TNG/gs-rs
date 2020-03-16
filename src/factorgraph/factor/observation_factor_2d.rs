use uuid::Uuid;
use crate::factorgraph::factor::{Factor, FactorType};

#[derive(Debug)]
pub struct ObservationFactor2D {
    id: Uuid,
    /// [x, y], observed position in relation to current pose
    content: [f64; 2],
    /// defaulting to identity matrix
    information_matrix: [f64; 3],
}

impl ObservationFactor2D {
    pub fn from_position(x: f64, y: f64) -> Self {
        ObservationFactor2D {
            id: Uuid::new_v4(),
            content: [x, y],
            information_matrix: [1.0,
                                 0.0, 1.0],
        }
    }

    pub fn from_position_and_information_matrix(x: f64, y: f64, information_matrix: [f64; 3]) -> Self {
        ObservationFactor2D {
            id: Uuid::new_v4(),
            content: [x, y],
            information_matrix,
        }
    }
}

impl Factor<'_> for ObservationFactor2D {
    fn get_id(&self) -> Uuid {
        self.id
    }

    fn get_type(&self) -> FactorType {
        FactorType::Observation2D
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
    fn test_from_position() {
        init();

        let test_factor = ObservationFactor2D::from_position(3.0, 5.0);
        info!("{:?}", &test_factor);
        assert_eq!(test_factor.get_content(), &[3.0, 5.0]);
        assert_eq!(test_factor.get_type(), FactorType::Observation2D);
        assert_eq!(test_factor.get_information_matrix(), &[1.0, 0.0, 1.0]);
    }

    #[test]
    fn test_from_position_and_information_matrix() {
        init();

        let test_factor = ObservationFactor2D::from_position_and_information_matrix(3.0, 5.0, [2.0, 0.0, 2.0]);
        info!("{:?}", &test_factor);
        assert_eq!(test_factor.get_content(), &[3.0, 5.0]);
        assert_eq!(test_factor.get_type(), FactorType::Observation2D);
        assert_eq!(test_factor.get_information_matrix(), &[2.0, 0.0, 2.0]);
    }

}