use uuid::Uuid;
use crate::factor_graph::factor::{Factor, FactorType};

#[derive(Debug, Clone)]
pub struct OdometryFactor2D {
    id: Uuid,
    /// [x, y, phi], with relation to previous pose
    restriction: [f64; 3],
    /// defaulting to identity matrix
    information_matrix: [f64; 6],
}

impl OdometryFactor2D {
    pub fn from_relative_pose(x: f64, y: f64, phi: f64) -> Self {
        OdometryFactor2D {
            id: Uuid::new_v4(),
            restriction: [x, y, phi],
            information_matrix: [1.0,
                                 0.0, 1.0,
                                 0.0, 0.0, 1.0],
        }
    }

    pub fn from_relative_pose_and_information_matrix(x: f64, y: f64, phi: f64, information_matrix: [f64; 6]) -> Self {
        OdometryFactor2D {
            id: Uuid::new_v4(),
            restriction: [x, y, phi],
            information_matrix,
        }
    }
}

impl Factor<'_> for OdometryFactor2D {
    fn get_id(&self) -> Uuid {
        self.id
    }

    fn get_type(&self) -> FactorType {
        FactorType::Odometry2D
    }

    fn get_restriction(&self) -> &[f64] {
        &self.restriction
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
    fn test_from_relative_pose() {
        init();

        let test_factor = OdometryFactor2D::from_relative_pose(3.0, 5.0, 0.1);
        info!("{:?}", &test_factor);
        assert_eq!(test_factor.get_restriction(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_factor.get_type(), FactorType::Odometry2D);
        assert_eq!(test_factor.get_information_matrix(), &[1.0, 0.0, 1.0, 0.0, 0.0, 1.0]);
    }

    #[test]
    fn test_from_relative_pose_and_information_matrix() {
        init();

        let test_factor = OdometryFactor2D::from_relative_pose_and_information_matrix(3.0, 5.0, 0.1, [1.0, 0.0, 1.0, 0.0, 0.0, 0.1]);
        info!("{:?}", &test_factor);
        assert_eq!(test_factor.get_restriction(), &[3.0, 5.0, 0.1]);
        assert_eq!(test_factor.get_type(), FactorType::Odometry2D);
        assert_eq!(test_factor.get_information_matrix(), &[1.0, 0.0, 1.0, 0.0, 0.0, 0.1]);
    }
}